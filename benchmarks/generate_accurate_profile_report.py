#!/usr/bin/env python3
"""Generate a Markdown profiling report for Latest_ACCURATE_Test."""

from __future__ import annotations

import csv
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from report_common import parse_timing_log


REPO_ROOT = Path(__file__).resolve().parents[1]
REPORT_ROOT = REPO_ROOT / "results/ov2slam_benchmark/Latest_ACCURATE_Test"
REPORT_PATH = REPORT_ROOT / "OV2SLAM_accurate_profile_report.md"
APE_SUMMARY_PLOT = REPORT_ROOT / "accurate_ape_rmse_by_sequence.png"
FE_SUMMARY_PLOT = REPORT_ROOT / "accurate_fe_mean_std_by_sequence.png"

SEQUENCE_ORDER = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
]

TIMER_KEYS = [
    "0.Full-Front_End",
    "1.FE_Track-Mono",
    "1.FE_createKeyframe",
    "0.Keyframe-Processing_Mapper",
    "1.BA_localBA",
    "0.LC_ProcessKF",
]

ROLE_ORDER_HINT = [
    "ov2_est",
    "ov2_lc",
    "ov2_main",
    "ov2_map",
    "ov2_feed",
    "ov2slam_node_main",
    "ov2_merge",
    "ov2_vizfr",
    "ov2_vizkf",
]


@dataclass
class EvoMetrics:
    gt_count: int
    est_count: int
    matched_found: int
    matched_max: int
    ape_mean: float
    ape_rmse: float
    ape_std: float


@dataclass
class CpuMetrics:
    mean_pct: float
    std_pct: float
    max_pct: float


@dataclass
class RoleMetrics:
    name: str
    mean_pct: float
    std_pct: float
    unique_tids: int


@dataclass
class SequenceData:
    name: str
    seq_dir: Path
    evo: EvoMetrics
    cpu: CpuMetrics
    timers: dict[str, tuple[float, float]]
    roles: list[RoleMetrics]

    @property
    def fe_mean_ms(self) -> float:
        return self.timers["0.Full-Front_End"][0]

    @property
    def fe_std_ms(self) -> float:
        return self.timers["0.Full-Front_End"][1]

    @property
    def fe_hz(self) -> float:
        return 1000.0 / self.fe_mean_ms if self.fe_mean_ms > 0 else 0.0

    @property
    def matched_pct(self) -> float:
        return 100.0 * self.evo.matched_found / self.evo.matched_max if self.evo.matched_max else 0.0

    @property
    def top_roles(self) -> list[RoleMetrics]:
        return [role for role in self.roles if role.mean_pct > 0][:3]


def parse_summary_header(summary_path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    if not summary_path.exists():
        return data
    for line in summary_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip()
    return data


def parse_evo_metrics(path: Path) -> EvoMetrics:
    text = path.read_text(encoding="utf-8", errors="ignore")

    def extract(pattern: str, cast=float) -> float | int:
        match = re.search(pattern, text, re.MULTILINE)
        if not match:
            raise ValueError(f"Could not parse {pattern!r} from {path}")
        return cast(match.group(1))

    gt_count = extract(r"Loaded\s+(\d+)\s+stamps and poses from: .*?(\.txt|gt\.tum)", int)
    est_count = extract(r"Loaded\s+(\d+)\s+stamps and poses from: .*ov2slam_kfs_traj\.txt", int)
    matched_found = extract(r"Found\s+(\d+)\s+of max\.\s+\d+\s+possible matching timestamps", int)
    matched_max = extract(r"Found\s+\d+\s+of max\.\s+(\d+)\s+possible matching timestamps", int)
    ape_mean = extract(r"^\s*mean\s+([0-9.]+)", float)
    ape_rmse = extract(r"^\s*rmse\s+([0-9.]+)", float)
    ape_std = extract(r"^\s*std\s+([0-9.]+)", float)
    return EvoMetrics(
        gt_count=gt_count,
        est_count=est_count,
        matched_found=matched_found,
        matched_max=matched_max,
        ape_mean=ape_mean,
        ape_rmse=ape_rmse,
        ape_std=ape_std,
    )


def parse_timing_csv(path: Path) -> dict[str, tuple[float, float]]:
    timings: dict[str, tuple[float, float]] = {}
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                timings[row["timer"]] = (float(row["mean_ms"]), float(row["std_ms"]))
            except Exception:
                continue
    missing = [key for key in TIMER_KEYS if key not in timings]
    if missing:
        raise ValueError(f"Missing timers in {path}: {missing}")
    return timings


def parse_timers_any(seq_dir: Path) -> dict[str, tuple[float, float]]:
    log_path = seq_dir / "ov2slam.log"
    if log_path.exists():
        timings = parse_timing_log(log_path)
        missing = [key for key in TIMER_KEYS if key not in timings]
        if not missing:
            return timings
    csv_path = seq_dir / "ov2slam_timings.csv"
    if csv_path.exists():
        return parse_timing_csv(csv_path)
    raise FileNotFoundError(f"Missing both ov2slam.log and ov2slam_timings.csv for {seq_dir}")


def parse_process_cpu_csv(path: Path) -> CpuMetrics:
    values: list[float] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                values.append(float(row["process_cpu_percent"]))
            except Exception:
                continue
    if not values:
        raise ValueError(f"No process CPU samples in {path}")
    mu = sum(values) / len(values)
    variance = sum((value - mu) ** 2 for value in values) / len(values)
    return CpuMetrics(mean_pct=mu, std_pct=math.sqrt(variance), max_pct=max(values))


def parse_role_csv(path: Path) -> list[RoleMetrics]:
    roles: list[RoleMetrics] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                roles.append(
                    RoleMetrics(
                        name=row["role"],
                        mean_pct=float(row["mean_cpu_all"]),
                        std_pct=float(row["std_cpu_all"]),
                        unique_tids=int(row["unique_tid_count"]),
                    )
                )
            except Exception:
                continue

    hint_order = {name: idx for idx, name in enumerate(ROLE_ORDER_HINT)}
    roles.sort(key=lambda role: (-role.mean_pct, hint_order.get(role.name, 999), role.name))
    return roles


def collect_sequences(report_root: Path) -> list[SequenceData]:
    sequences: list[SequenceData] = []
    for name in SEQUENCE_ORDER:
        seq_dir = report_root / name
        if not seq_dir.is_dir():
            continue
        sequences.append(
            SequenceData(
                name=name,
                seq_dir=seq_dir,
                evo=parse_evo_metrics(seq_dir / f"{name}_evo.txt"),
                cpu=parse_process_cpu_csv(seq_dir / "ov2slam_process_cpu.csv"),
                timers=parse_timers_any(seq_dir),
                roles=parse_role_csv(seq_dir / "ov2slam_named_thread_roles_aggregated.csv"),
            )
        )
    return sequences


def format_pct(value: float) -> str:
    return f"{value:.1f}%"


def format_role_triplet(roles: list[RoleMetrics]) -> str:
    if not roles:
        return "n/a"
    return ", ".join(f"`{role.name}` {role.mean_pct:.1f}%" for role in roles)


def relative_to_report(path: Path) -> str:
    return path.relative_to(REPORT_ROOT).as_posix()


def generate_summary_plots(sequences: list[SequenceData]) -> None:
    names = [seq.name for seq in sequences]

    fig, ax = plt.subplots(figsize=(11, 4.8), dpi=170)
    rmse_values = [seq.evo.ape_rmse for seq in sequences]
    x = list(range(len(names)))
    bars = ax.bar(x, rmse_values, color="#4f81bd")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=35, ha="right")
    ax.set_ylabel("APE RMSE (m)")
    ax.set_title("Accurate OV2SLAM APE RMSE by sequence")
    ax.grid(True, axis="y", alpha=0.35)
    ax.bar_label(bars, labels=[f"{value:.3f}" for value in rmse_values], padding=3, fontsize=8, rotation=90)
    fig.tight_layout()
    fig.savefig(APE_SUMMARY_PLOT, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4.8), dpi=170)
    fe_mean = [seq.fe_mean_ms for seq in sequences]
    fe_std = [seq.fe_std_ms for seq in sequences]
    bars = ax.bar(x, fe_mean, yerr=fe_std, capsize=4, color="#c0504d", ecolor="#2f2f2f")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=35, ha="right")
    ax.set_ylabel("Front-end time (ms)")
    ax.set_title("Accurate OV2SLAM full front-end time by sequence (mean ± std)")
    ax.grid(True, axis="y", alpha=0.35)
    ax.bar_label(
        bars,
        labels=[f"{m:.1f}±{s:.1f}" for m, s in zip(fe_mean, fe_std)],
        padding=3,
        fontsize=8,
        rotation=90,
    )
    fig.tight_layout()
    fig.savefig(FE_SUMMARY_PLOT, bbox_inches="tight")
    plt.close(fig)


def aggregate_role_means(sequences: list[SequenceData]) -> list[tuple[str, float, float, int]]:
    buckets: dict[str, list[float]] = {}
    for seq in sequences:
        for role in seq.roles:
            buckets.setdefault(role.name, []).append(role.mean_pct)
    rows = []
    for name, values in buckets.items():
        mu = sum(values) / len(values)
        variance = sum((value - mu) ** 2 for value in values) / len(values)
        rows.append((name, mu, math.sqrt(variance), len(values)))
    hint_order = {name: idx for idx, name in enumerate(ROLE_ORDER_HINT)}
    rows.sort(key=lambda row: (-row[1], hint_order.get(row[0], 999), row[0]))
    return rows


def build_report(sequences: list[SequenceData], header: dict[str, str]) -> str:
    easiest = min(sequences, key=lambda seq: seq.evo.ape_rmse)
    hardest = max(sequences, key=lambda seq: seq.evo.ape_rmse)
    fastest = min(sequences, key=lambda seq: seq.fe_mean_ms)
    slowest = max(sequences, key=lambda seq: seq.fe_mean_ms)
    role_summary = aggregate_role_means(sequences)
    top_role_line = ", ".join(f"`{name}` {mu:.1f}%" for name, mu, _std, _count in role_summary[:4])

    camera_mode = header.get("Camera mode", "stereo")
    speed_mode = header.get("Speed mode", "accurate")
    traj_policy = header.get("Traj policy", "paper")
    cpu_set = header.get("OV2SLAM CPUs", "0,1,2,3")
    bag_rate = header.get("Bag rate", "1.0x")
    params_file = header.get("Params file", "/workspace/src/ov2slam_ros/parameters_files/accurate/euroc/euroc_stereo.yaml")
    date_line = header.get("Date", "")

    lines: list[str] = []
    lines.append("# OV2SLAM Accurate Profiling Report")
    lines.append("")
    lines.append(f"_Generated from `Latest_ACCURATE_Test` on {date_line or 'the available benchmark outputs'}._")
    lines.append("")
    lines.append("## Executive Summary")
    lines.append("")
    lines.append(
        f"This report profiles **OV2SLAM accurate stereo mode** on the **10 EuRoC sequences** present in `Latest_ACCURATE_Test`. "
        f"The runs were executed at **{bag_rate} playback**, with **trajectory policy `{traj_policy}`**, using the parameter file "
        f"`{params_file}`, and OV2SLAM pinned to CPUs **{cpu_set}**. The report is built directly from each sequence folder rather than "
        "from the top-level summary table, because the per-sequence EVO, timing, CPU, and plotting artifacts are the authoritative sources."
    )
    lines.append("")
    lines.append("| Sequence | APE RMSE (m) | Matched timestamps | FE mean (ms) | FE std (ms) | FE Hz | Process CPU mean | Process CPU max |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for seq in sequences:
        lines.append(
            f"| {seq.name} | {seq.evo.ape_rmse:.6f} | {seq.evo.matched_found}/{seq.evo.matched_max} ({seq.matched_pct:.1f}%) | "
            f"{seq.fe_mean_ms:.4f} | {seq.fe_std_ms:.4f} | {seq.fe_hz:.1f} | {seq.cpu.mean_pct:.3f}% | {seq.cpu.max_pct:.3f}% |"
        )
    lines.append("")
    lines.append("Meeting-ready observations:")
    lines.append(
        f"- APE RMSE spans **{easiest.evo.ape_rmse:.3f} m** to **{hardest.evo.ape_rmse:.3f} m**, with `{easiest.name}` currently the easiest sequence and `{hardest.name}` the hardest."
    )
    lines.append(
        f"- Full front-end time spans **{fastest.fe_mean_ms:.2f} ms** to **{slowest.fe_mean_ms:.2f} ms**, or roughly **{slowest.fe_hz:.1f} Hz** to **{fastest.fe_hz:.1f} Hz** of estimated front-end capacity."
    )
    lines.append(f"- Across the sequence set, the dominant logical thread roles are {top_role_line}.")
    lines.append("- Treat these as **preliminary single-run results**: they are strong enough for an executive baseline, but not yet a repeated-run statistical benchmark.")
    lines.append("")
    lines.append("![](accurate_ape_rmse_by_sequence.png)")
    lines.append("")
    lines.append("![](accurate_fe_mean_std_by_sequence.png)")
    lines.append("")
    lines.append("## Evaluation and Profiling Method")
    lines.append("")
    lines.append(
        f"- **Ground truth source:** each sequence uses the normalized `gt.tum` file saved in its result folder; the benchmark metadata identifies the source dataset as EuRoC and confirms the GT normalization step."
    )
    lines.append(
        "- **Evaluated estimate:** `ov2slam_kfs_traj.txt` is the trajectory evaluated for this accurate stereo report, matching the benchmark’s current paper-style stereo choice."
    )
    lines.append(
        "- **Alignment rule:** EVO compares trajectories with `evo_ape tum --align` and **does not apply `--correct_scale`**, because these are stereo runs with metric scale."
    )
    lines.append(
        "- **Matched timestamps:** the `found / max (%)` values in this report come directly from the `Synchronizing trajectories...` section in each `*_evo.txt` file."
    )
    lines.append(
        "- **APE and RPE visuals:** the report reuses the saved `*_ape_xy.png` and `*_rpe_trans.png` files already generated during benchmarking."
    )
    lines.append(
        "- **Front-end and backend timing:** timing values are taken from the final `Time Logs Summary` block in `ov2slam.log`; `ov2slam_timings.csv` is only used as a fallback if the log summary is unavailable."
    )
    lines.append(
        "- **Process CPU:** process-level CPU statistics are computed from the sampled `process_cpu_percent` values in `ov2slam_process_cpu.csv`."
    )
    lines.append(
        "- **Thread-role CPU:** thread-role statistics come from `ov2slam_named_thread_roles_aggregated.csv`, where samples are grouped by logical role such as `ov2_est`, `ov2_lc`, `ov2_main`, and `ov2_map`."
    )
    lines.append(
        "- **CPU averaging note:** CPU means and standard deviations are averages over the monitor samples across time, not instantaneous values copied from a single frame."
    )
    lines.append(
        "- **Thread-role aggregation note:** the role values are logical-role aggregates over all matching TIDs, so repeated `ov2_main` worker threads are merged before summary."
    )
    lines.append("")
    lines.append("## Cross-Sequence Results")
    lines.append("")
    lines.append("| Sequence | APE mean (m) | APE RMSE (m) | APE std (m) | Matched pairs | FE mean±std (ms) | Process CPU mean±std (max) | Top 3 thread roles |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |")
    for seq in sequences:
        lines.append(
            f"| {seq.name} | {seq.evo.ape_mean:.6f} | {seq.evo.ape_rmse:.6f} | {seq.evo.ape_std:.6f} | "
            f"{seq.evo.matched_found}/{seq.evo.matched_max} ({seq.matched_pct:.1f}%) | "
            f"{seq.fe_mean_ms:.3f} ± {seq.fe_std_ms:.3f} | "
            f"{seq.cpu.mean_pct:.2f}% ± {seq.cpu.std_pct:.2f}% (max {seq.cpu.max_pct:.2f}%) | "
            f"{format_role_triplet(seq.top_roles)} |"
        )
    lines.append("")
    lines.append(
        "Interpretation starter: accurate OV2SLAM stays in a relatively tight accuracy band on most EuRoC sequences, while the front-end remains below the 22 ms mark across this dataset. "
        "The main variation appears in backend pressure and sequence difficulty rather than in any obvious front-end collapse."
    )
    lines.append("")
    lines.append("## Profiling Breakdown")
    lines.append("")
    lines.append("### Timing Breakdown")
    lines.append("")
    lines.append("| Sequence | Full FE (ms) | FE Track (ms) | FE KF create (ms) | Mapper KF proc (ms) | Local BA (ms) | LC ProcessKF (ms) |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for seq in sequences:
        lines.append(
            f"| {seq.name} | {seq.timers['0.Full-Front_End'][0]:.3f} ± {seq.timers['0.Full-Front_End'][1]:.3f} | "
            f"{seq.timers['1.FE_Track-Mono'][0]:.3f} | {seq.timers['1.FE_createKeyframe'][0]:.3f} | "
            f"{seq.timers['0.Keyframe-Processing_Mapper'][0]:.3f} | {seq.timers['1.BA_localBA'][0]:.3f} | "
            f"{seq.timers['0.LC_ProcessKF'][0]:.3f} |"
        )
    lines.append("")
    lines.append("Discussion starters:")
    lines.append(
        f"- The accurate front-end stays between **{fastest.fe_mean_ms:.2f} ms** and **{slowest.fe_mean_ms:.2f} ms**, so the front-end is not the main source of cross-sequence instability here."
    )
    lines.append("- Local BA and loop-closure processing are much heavier than front-end tracking, which is consistent with the CPU plots showing strong backend activity.")
    lines.append("- If you need a meeting takeaway, the phrasing can be: “timing pressure is dominated by optimization stages rather than image-to-image tracking.”")
    lines.append("")
    lines.append("### CPU and Thread-Role Utilization")
    lines.append("")
    lines.append("| Sequence | Process CPU mean±std (max) | Top role 1 | Top role 2 | Top role 3 |")
    lines.append("| --- | --- | --- | --- | --- |")
    for seq in sequences:
        roles = seq.top_roles + [RoleMetrics("n/a", 0.0, 0.0, 0)] * (3 - len(seq.top_roles))
        lines.append(
            f"| {seq.name} | {seq.cpu.mean_pct:.2f}% ± {seq.cpu.std_pct:.2f}% (max {seq.cpu.max_pct:.2f}%) | "
            f"`{roles[0].name}` {roles[0].mean_pct:.1f}% ± {roles[0].std_pct:.1f}% | "
            f"`{roles[1].name}` {roles[1].mean_pct:.1f}% ± {roles[1].std_pct:.1f}% | "
            f"`{roles[2].name}` {roles[2].mean_pct:.1f}% ± {roles[2].std_pct:.1f}% |"
        )
    lines.append("")
    lines.append("| Logical thread role | Mean of per-sequence means | Std across sequences | Present in sequences |")
    lines.append("| --- | ---: | ---: | ---: |")
    for name, mu, std, count in role_summary:
        lines.append(f"| `{name}` | {mu:.3f}% | {std:.3f}% | {count} |")
    lines.append("")
    lines.append("Discussion starters:")
    lines.append(f"- The dominant logical roles on average are {top_role_line}.")
    lines.append("- The estimator and loop-closure workers consistently dominate sustained CPU consumption, while feeder and main-node bookkeeping remain lightweight.")
    lines.append("- A concise meeting phrasing is: “the optimization workers, not the ROS wrapper, carry most of the runtime load in accurate mode.”")
    lines.append("")
    lines.append("## Per-Sequence Appendix")
    lines.append("")
    for seq in sequences:
        lines.append(f"### {seq.name}")
        lines.append("")
        lines.append("| Metric | Value |")
        lines.append("| --- | --- |")
        lines.append(f"| APE mean / RMSE / std | {seq.evo.ape_mean:.6f} / {seq.evo.ape_rmse:.6f} / {seq.evo.ape_std:.6f} m |")
        lines.append(f"| Matched timestamps | {seq.evo.matched_found}/{seq.evo.matched_max} ({seq.matched_pct:.1f}%) |")
        lines.append(f"| FE mean / std / Hz | {seq.fe_mean_ms:.4f} ms / {seq.fe_std_ms:.4f} ms / {seq.fe_hz:.1f} Hz |")
        lines.append(f"| Process CPU mean / std / max | {seq.cpu.mean_pct:.3f}% / {seq.cpu.std_pct:.3f}% / {seq.cpu.max_pct:.3f}% |")
        lines.append(f"| Top thread roles | {format_role_triplet(seq.top_roles)} |")
        lines.append("")
        lines.append(f"![]({relative_to_report(seq.seq_dir / f'{seq.name}_ape_xy.png')})")
        lines.append("")
        lines.append(f"![]({relative_to_report(seq.seq_dir / f'{seq.name}_rpe_trans.png')})")
        lines.append("")
        lines.append(f"![]({relative_to_report(seq.seq_dir / 'ov2slam_cpu_usage.png')})")
        lines.append("")
        lines.append(f"![]({relative_to_report(seq.seq_dir / 'ov2slam_named_thread_roles_aggregated.png')})")
        lines.append("")
        lines.append(
            f"Links: [log]({relative_to_report(seq.seq_dir / 'ov2slam.log')}), "
            f"[timings]({relative_to_report(seq.seq_dir / 'ov2slam_timings.csv')}), "
            f"[thread roles]({relative_to_report(seq.seq_dir / 'ov2slam_named_thread_roles_aggregated.csv')})"
        )
        lines.append("")

    return "\n".join(lines) + "\n"


def main() -> int:
    if not REPORT_ROOT.is_dir():
        raise SystemExit(f"Missing report root: {REPORT_ROOT}")

    sequences = collect_sequences(REPORT_ROOT)
    if len(sequences) != len(SEQUENCE_ORDER):
        missing = [name for name in SEQUENCE_ORDER if name not in {seq.name for seq in sequences}]
        raise SystemExit(f"Expected {len(SEQUENCE_ORDER)} sequences, missing: {missing}")

    header = parse_summary_header(REPORT_ROOT / "summary.txt")
    generate_summary_plots(sequences)
    REPORT_PATH.write_text(build_report(sequences, header), encoding="utf-8")
    print(f"Wrote report: {REPORT_PATH}")
    print(f"Wrote plot  : {APE_SUMMARY_PLOT}")
    print(f"Wrote plot  : {FE_SUMMARY_PLOT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
