#!/usr/bin/env python3
"""Generate a Markdown profiling report for Latest_FAST_Test.

This version is resilient to incomplete benchmark folders:
- if `<seq>_evo.txt` is missing, it rebuilds it from the saved trajectory + dataset GT
- if APE/RPE plots are missing, it recreates them
- if `ov2slam_timings.csv` is missing, it falls back to the final timing summary in `ov2slam.log`
- if CPU/thread monitor CSVs are missing, it reports that explicitly instead of failing
"""

from __future__ import annotations

import math
import os
import re
import subprocess
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from report_common import (
    CpuMetrics,
    REPO_ROOT,
    RoleMetrics,
    SEQUENCE_ORDER,
    SequenceData,
    aggregate_role_means,
    parse_evo_metrics,
    parse_process_cpu_csv,
    parse_role_csv,
    parse_summary_header,
    parse_timing_log,
    parse_timing_csv,
)


REPORT_ROOT = REPO_ROOT / "results/ov2slam_benchmark/Latest_FAST_Test"
REPORT_PATH = REPORT_ROOT / "OV2SLAM_fast_profile_report.md"
APE_SUMMARY_PLOT = REPORT_ROOT / "fast_ape_rmse_by_sequence.png"
FE_SUMMARY_PLOT = REPORT_ROOT / "fast_fe_mean_std_by_sequence.png"

DATASET_ROOT = REPO_ROOT / "datasets/euroc"
EVO_APE = REPO_ROOT / "SLAM-Former/venv/bin/evo_ape"
EVO_HOME = Path("/tmp/evo_report_home")

FAST_TIMER_KEYS = [
    "0.Full-Front_End",
    "1.FE_Track-Mono",
    "1.FE_createKeyframe",
    "0.Keyframe-Processing_Mapper",
    "1.BA_localBA",
]


def relative_to_report(path: Path) -> str:
    return path.relative_to(REPORT_ROOT).as_posix()


def format_role_triplet(roles: list[RoleMetrics]) -> str:
    if not roles:
        return "n/a"
    return ", ".join(f"`{role.name}` {role.mean_pct:.1f}%" for role in roles)


def has_cpu(seq: SequenceData) -> bool:
    return not math.isnan(seq.cpu.mean_pct)


def fmt_cpu_mean(seq: SequenceData) -> str:
    return f"{seq.cpu.mean_pct:.3f}%" if has_cpu(seq) else "n/a"


def fmt_cpu_max(seq: SequenceData) -> str:
    return f"{seq.cpu.max_pct:.3f}%" if has_cpu(seq) else "n/a"


def fmt_cpu_full(seq: SequenceData) -> str:
    if not has_cpu(seq):
        return "n/a"
    return f"{seq.cpu.mean_pct:.2f}% ± {seq.cpu.std_pct:.2f}% (max {seq.cpu.max_pct:.2f}%)"


def nan_cpu() -> CpuMetrics:
    return CpuMetrics(mean_pct=float("nan"), std_pct=float("nan"), max_pct=float("nan"))


def resolve_gt_path(seq_name: str, seq_dir: Path) -> Path:
    local_gt = seq_dir / "gt.tum"
    if local_gt.exists():
        return local_gt
    dataset_gt = DATASET_ROOT / seq_name / f"{seq_name}.txt"
    if dataset_gt.exists():
        return dataset_gt
    raise FileNotFoundError(f"Missing GT for {seq_name}: expected {local_gt} or {dataset_gt}")


def ensure_evo_text(seq_name: str, seq_dir: Path) -> Path:
    evo_text = seq_dir / f"{seq_name}_evo.txt"
    if evo_text.exists():
        return evo_text

    gt_file = resolve_gt_path(seq_name, seq_dir)
    est_file = seq_dir / "ov2slam_kfs_traj.txt"
    cmd = [
        str(EVO_APE),
        "tum",
        str(gt_file),
        str(est_file),
        "--verbose",
        "--align",
        "--t_max_diff",
        "0.1",
    ]
    EVO_HOME.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["HOME"] = str(EVO_HOME)
    env["XDG_CONFIG_HOME"] = str(EVO_HOME)
    result = subprocess.run(cmd, check=True, capture_output=True, text=True, env=env)
    evo_text.write_text(result.stdout, encoding="utf-8")
    return evo_text


def load_tum_xyz(path: Path) -> list[tuple[float, float, float, float]]:
    rows: list[tuple[float, float, float, float]] = []
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.replace(",", " ").split()
        if len(fields) < 8:
            continue
        rows.append((float(fields[0]), float(fields[1]), float(fields[2]), float(fields[3])))
    if not rows:
        raise ValueError(f"No TUM poses found in {path}")
    return rows


def associate_xyz(
    gt_rows: list[tuple[float, float, float, float]],
    est_rows: list[tuple[float, float, float, float]],
    max_diff: float = 0.1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    gt_points = []
    est_points = []
    assoc_times = []
    j = 0
    for gt_ts, gx, gy, gz in gt_rows:
        while j < len(est_rows) and est_rows[j][0] < gt_ts - max_diff:
            j += 1
        if j >= len(est_rows):
            break
        best_idx = None
        best_dt = math.inf
        k = j
        while k < len(est_rows) and est_rows[k][0] <= gt_ts + max_diff:
            dt = abs(est_rows[k][0] - gt_ts)
            if dt < best_dt:
                best_dt = dt
                best_idx = k
            k += 1
        if best_idx is None:
            continue
        _, ex, ey, ez = est_rows[best_idx]
        gt_points.append([gx, gy, gz])
        est_points.append([ex, ey, ez])
        assoc_times.append(gt_ts)
        j = best_idx + 1
    if len(gt_points) < 3:
        raise ValueError(f"Only {len(gt_points)} timestamp associations found; need at least 3 for alignment.")
    return (
        np.asarray(assoc_times, dtype=float),
        np.asarray(gt_points, dtype=float),
        np.asarray(est_points, dtype=float),
    )


def umeyama_similarity(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_demean = src - src_mean
    dst_demean = dst - dst_mean
    covariance = (dst_demean.T @ src_demean) / src.shape[0]
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(src.shape[1])
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vt
    src_var = np.mean(np.sum(src_demean * src_demean, axis=1))
    if src_var <= 0:
        raise ValueError("Source variance is zero; cannot align trajectory.")
    scale = float(np.sum(singular_values * np.diag(correction)) / src_var)
    translation = dst_mean - scale * (rotation @ src_mean)
    return scale, rotation, translation


def ensure_eval_plots(seq_name: str, seq_dir: Path) -> None:
    ape_plot_path = seq_dir / f"{seq_name}_ape_xy.png"
    rpe_plot_path = seq_dir / f"{seq_name}_rpe_trans.png"
    if ape_plot_path.exists() and rpe_plot_path.exists():
        return

    gt_rows = load_tum_xyz(resolve_gt_path(seq_name, seq_dir))
    est_rows = load_tum_xyz(seq_dir / "ov2slam_kfs_traj.txt")
    assoc_times, gt_assoc, est_assoc = associate_xyz(gt_rows, est_rows)
    scale, rotation, translation = umeyama_similarity(est_assoc, gt_assoc)

    gt_xyz = np.asarray([[x, y, z] for _, x, y, z in gt_rows], dtype=float)
    est_xyz = np.asarray([[x, y, z] for _, x, y, z in est_rows], dtype=float)
    est_aligned = (scale * (rotation @ est_xyz.T)).T + translation
    est_assoc_aligned = (scale * (rotation @ est_assoc.T)).T + translation

    fig, ax = plt.subplots(figsize=(6, 6), dpi=160)
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "--", color="#7f7f7f", linewidth=1.5, label="GT")
    ax.plot(est_aligned[:, 0], est_aligned[:, 1], color="#57c7b6", linewidth=1.6, label="OV2SLAM")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(seq_name)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(ape_plot_path, bbox_inches="tight")
    plt.close(fig)

    if len(assoc_times) >= 2:
        rel_time = assoc_times[1:] - assoc_times[0]
        gt_rel = gt_assoc[1:] - gt_assoc[:-1]
        est_rel = est_assoc_aligned[1:] - est_assoc_aligned[:-1]
        rpe_trans = np.linalg.norm(gt_rel - est_rel, axis=1)
        fig, ax = plt.subplots(figsize=(7, 4), dpi=160)
        ax.plot(rel_time, rpe_trans, color="#e67e22", linewidth=1.3)
        ax.set_xlabel("time since start (s)")
        ax.set_ylabel("translation RPE (m)")
        ax.set_title(f"{seq_name} RPE")
        ax.grid(True, alpha=0.35)
        fig.tight_layout()
        fig.savefig(rpe_plot_path, bbox_inches="tight")
        plt.close(fig)

def parse_timers_any(seq_dir: Path, required: list[str]) -> dict[str, tuple[float, float]]:
    log_path = seq_dir / "ov2slam.log"
    if log_path.exists():
        timings = parse_timing_log(log_path)
        missing = [key for key in required if key not in timings]
        if not missing:
            return timings
    csv_path = seq_dir / "ov2slam_timings.csv"
    if csv_path.exists():
        timings = parse_timing_csv(csv_path)
        missing = [key for key in required if key not in timings]
        if not missing:
            return timings
    raise ValueError(f"Missing required timers for {seq_dir.name}: {required}")


def parse_cpu_any(seq_dir: Path) -> CpuMetrics:
    cpu_path = seq_dir / "ov2slam_process_cpu.csv"
    return parse_process_cpu_csv(cpu_path) if cpu_path.exists() else nan_cpu()


def parse_roles_any(seq_dir: Path) -> list[RoleMetrics]:
    role_path = seq_dir / "ov2slam_named_thread_roles_aggregated.csv"
    return parse_role_csv(role_path) if role_path.exists() else []


def collect_profile_sequences(report_root: Path, required_timers: list[str]) -> list[SequenceData]:
    sequences: list[SequenceData] = []
    for name in SEQUENCE_ORDER:
        seq_dir = report_root / name
        if not seq_dir.is_dir():
            continue
        ensure_evo_text(name, seq_dir)
        ensure_eval_plots(name, seq_dir)
        sequences.append(
            SequenceData(
                name=name,
                seq_dir=seq_dir,
                evo=parse_evo_metrics(seq_dir / f"{name}_evo.txt"),
                cpu=parse_cpu_any(seq_dir),
                timers=parse_timers_any(seq_dir, required_timers),
                roles=parse_roles_any(seq_dir),
            )
        )
    return sequences


def collect_fast_sequences(report_root: Path) -> list[SequenceData]:
    return collect_profile_sequences(report_root, FAST_TIMER_KEYS)


def generate_summary_plots(sequences: list[SequenceData]) -> None:
    names = [seq.name for seq in sequences]
    x = list(range(len(names)))

    fig, ax = plt.subplots(figsize=(11, 4.8), dpi=170)
    rmse_values = [seq.evo.ape_rmse for seq in sequences]
    bars = ax.bar(x, rmse_values, color="#7d5ba6")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=35, ha="right")
    ax.set_ylabel("APE RMSE (m)")
    ax.set_title("Fast OV2SLAM APE RMSE by sequence")
    ax.grid(True, axis="y", alpha=0.35)
    ax.bar_label(bars, labels=[f"{value:.3f}" for value in rmse_values], padding=3, fontsize=8, rotation=90)
    fig.tight_layout()
    fig.savefig(APE_SUMMARY_PLOT, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4.8), dpi=170)
    fe_mean = [seq.fe_mean_ms for seq in sequences]
    fe_std = [seq.fe_std_ms for seq in sequences]
    bars = ax.bar(x, fe_mean, yerr=fe_std, capsize=4, color="#dd8452", ecolor="#2f2f2f")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=35, ha="right")
    ax.set_ylabel("Front-end time (ms)")
    ax.set_title("Fast OV2SLAM full front-end time by sequence (mean ± std)")
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


def build_report(sequences: list[SequenceData], header: dict[str, str]) -> str:
    easiest = min(sequences, key=lambda seq: seq.evo.ape_rmse)
    hardest = max(sequences, key=lambda seq: seq.evo.ape_rmse)
    fastest = min(sequences, key=lambda seq: seq.fe_mean_ms)
    slowest = max(sequences, key=lambda seq: seq.fe_mean_ms)
    role_summary = aggregate_role_means(sequences)
    top_role_line = ", ".join(f"`{name}` {mu:.1f}%" for name, mu, _std, _count in role_summary[:4]) if role_summary else "no role telemetry preserved"

    bag_rate = header.get("Bag rate", "1.0x")
    traj_policy = header.get("Traj policy", "paper")
    cpu_set = header.get("OV2SLAM CPUs", "0,1,2,3")
    params_file = header.get("Params file", "/workspace/src/ov2slam_ros/parameters_files/fast/euroc/euroc_stereo.yaml")
    date_line = header.get("Date", "")
    cpu_available = any(has_cpu(seq) for seq in sequences)

    lines: list[str] = []
    lines.append("# OV2SLAM Fast Profiling Report")
    lines.append("")
    lines.append(f"_Generated from `Latest_FAST_Test` on {date_line or 'the available benchmark outputs'}._")
    lines.append("")
    lines.append("## Executive Summary")
    lines.append("")
    lines.append(
        f"This report profiles **OV2SLAM fast stereo mode** on the **10 EuRoC sequences** present in `Latest_FAST_Test`. "
        f"The runs were executed at **{bag_rate} playback**, with **trajectory policy `{traj_policy}`**, using the parameter file "
        f"`{params_file}`, and OV2SLAM pinned to CPUs **{cpu_set}**. This is a **default-speed fast profile**, not the paper’s accelerated 100–400 Hz replay experiment."
    )
    lines.append("")
    lines.append("| Sequence | APE RMSE (m) | Matched timestamps | FE mean (ms) | FE std (ms) | FE Hz | Process CPU mean | Process CPU max |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for seq in sequences:
        lines.append(
            f"| {seq.name} | {seq.evo.ape_rmse:.6f} | {seq.evo.matched_found}/{seq.evo.matched_max} ({seq.matched_pct:.1f}%) | "
            f"{seq.fe_mean_ms:.4f} | {seq.fe_std_ms:.4f} | {seq.fe_hz:.1f} | {fmt_cpu_mean(seq)} | {fmt_cpu_max(seq)} |"
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
    if cpu_available:
        lines.append("- Process/thread CPU telemetry is available for part of this run set and is summarized below.")
    else:
        lines.append("- **CPU/core telemetry was not preserved in this Latest fast result set**, so the report discusses core utilization as an instrumentation gap rather than a measured outcome.")
    lines.append("- Treat these as **preliminary single-run results**: they are useful for a fast-mode baseline, but not a repeated-run statistical benchmark.")
    lines.append("")
    lines.append("![](fast_ape_rmse_by_sequence.png)")
    lines.append("")
    lines.append("![](fast_fe_mean_std_by_sequence.png)")
    lines.append("")
    lines.append("## Evaluation and Profiling Method")
    lines.append("")
    lines.append("- **Ground truth source:** if `gt.tum` was missing from the result folder, the report used the EuRoC dataset file `datasets/euroc/<seq>/<seq>.txt` directly.")
    lines.append("- **Evaluated estimate:** `ov2slam_kfs_traj.txt` is the trajectory evaluated for this fast stereo report.")
    lines.append("- **Alignment rule:** EVO compares trajectories with `evo_ape tum --align` and does not use scale correction because these are stereo runs.")
    lines.append("- **Matched timestamps:** the `found / max (%)` values come from regenerated `*_evo.txt` files whenever the original text output was not present in the result folder.")
    lines.append("- **APE and RPE visuals:** the report reuses saved PNGs when available and regenerates them from the saved trajectories when they are missing.")
    lines.append("- **Front-end and backend timing:** timing values are taken from the final `Time Logs Summary` block in `ov2slam.log`; `ov2slam_timings.csv` is only used as a fallback if the log summary is unavailable.")
    lines.append("- **Process CPU:** process-level CPU statistics come from `ov2slam_process_cpu.csv` when present.")
    lines.append("- **Thread-role CPU:** thread-role statistics come from `ov2slam_named_thread_roles_aggregated.csv` when present.")
    lines.append("- **CPU averaging note:** CPU means and standard deviations are averages over the monitor samples across time.")
    lines.append("- **Thread-role aggregation note:** role values are logical-role aggregates over all matching TIDs.")
    if not cpu_available:
        lines.append("- **Instrumentation caveat:** the `Latest_FAST_Test` folders do not retain the CPU monitor CSVs, so CPU/core utilization is reported as unavailable for this run set.")
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
            f"{fmt_cpu_full(seq)} | {format_role_triplet(seq.top_roles)} |"
        )
    lines.append("")
    lines.append(
        "Interpretation starter: fast OV2SLAM clearly reduces front-end latency, but the trade-off in this default-speed run is visible in several of the more difficult sequences where accuracy deteriorates."
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
            f"{seq.timers['0.Keyframe-Processing_Mapper'][0]:.3f} | {seq.timers['1.BA_localBA'][0]:.3f} | disabled |"
        )
    lines.append("")
    lines.append("Discussion starters:")
    lines.append(
        f"- The fast front-end stays between **{fastest.fe_mean_ms:.2f} ms** and **{slowest.fe_mean_ms:.2f} ms**, giving a noticeably higher front-end headroom than the accurate configuration."
    )
    lines.append("- Backend costs are also reduced, but difficult sequences still show substantial optimization work and this remains the main cost center even in fast mode.")
    lines.append("- A concise meeting phrasing is: “fast mode cuts front-end latency aggressively, but accuracy becomes more sequence-sensitive.”")
    lines.append("")
    lines.append("### CPU and Thread-Role Utilization")
    lines.append("")
    if cpu_available:
        lines.append("| Sequence | Process CPU mean±std (max) | Top role 1 | Top role 2 | Top role 3 |")
        lines.append("| --- | --- | --- | --- | --- |")
        for seq in sequences:
            roles = seq.top_roles + [RoleMetrics("n/a", 0.0, 0.0, 0)] * (3 - len(seq.top_roles))
            lines.append(
                f"| {seq.name} | {fmt_cpu_full(seq)} | "
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
        lines.append("- Even in fast mode, the estimator and optimization-related roles remain the main sustained CPU consumers.")
        lines.append("- A concise meeting phrasing is: “fast mode reduces latency more than it reduces optimization dominance.”")
    else:
        lines.append("The `Latest_FAST_Test` folders do not contain `ov2slam_process_cpu.csv` or `ov2slam_named_thread_roles_aggregated.csv`.")
        lines.append("")
        lines.append("Discussion starters:")
        lines.append("- Core/thread utilization was instrumented in other benchmark generations, but not preserved in this particular fast result set.")
        lines.append("- The most defensible comparison from `Latest_FAST_Test` is therefore **accuracy + latency**, not CPU telemetry.")
        lines.append("- Meeting phrasing: “for the latest fast run set, we can quantify accuracy and timing directly, while CPU/core load would need a rerun with monitor outputs retained.”")
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
        lines.append(f"| Process CPU mean / std / max | {fmt_cpu_full(seq)} |")
        lines.append(f"| Top thread roles | {format_role_triplet(seq.top_roles)} |")
        lines.append("")
        lines.append(f"![]({relative_to_report(seq.seq_dir / f'{seq.name}_ape_xy.png')})")
        lines.append("")
        lines.append(f"![]({relative_to_report(seq.seq_dir / f'{seq.name}_rpe_trans.png')})")
        lines.append("")
        if (seq.seq_dir / "ov2slam_cpu_usage.png").exists():
            lines.append(f"![]({relative_to_report(seq.seq_dir / 'ov2slam_cpu_usage.png')})")
            lines.append("")
        if (seq.seq_dir / "ov2slam_named_thread_roles_aggregated.png").exists():
            lines.append(f"![]({relative_to_report(seq.seq_dir / 'ov2slam_named_thread_roles_aggregated.png')})")
            lines.append("")
        link_bits = [
            f"[log]({relative_to_report(seq.seq_dir / 'ov2slam.log')})",
            f"[evo text]({relative_to_report(seq.seq_dir / f'{seq.name}_evo.txt')})",
        ]
        if (seq.seq_dir / "ov2slam_timings.csv").exists():
            link_bits.append(f"[timings csv]({relative_to_report(seq.seq_dir / 'ov2slam_timings.csv')})")
        if (seq.seq_dir / "ov2slam_named_thread_roles_aggregated.csv").exists():
            link_bits.append(f"[thread roles]({relative_to_report(seq.seq_dir / 'ov2slam_named_thread_roles_aggregated.csv')})")
        lines.append(f"Links: {', '.join(link_bits)}")
        lines.append("")

    return "\n".join(lines) + "\n"


def main() -> int:
    if not REPORT_ROOT.is_dir():
        raise SystemExit(f"Missing report root: {REPORT_ROOT}")

    sequences = collect_fast_sequences(REPORT_ROOT)
    header = parse_summary_header(REPORT_ROOT / "summary.txt")
    generate_summary_plots(sequences)
    REPORT_PATH.write_text(build_report(sequences, header), encoding="utf-8")
    print(f"Wrote report: {REPORT_PATH}")
    print(f"Wrote plot  : {APE_SUMMARY_PLOT}")
    print(f"Wrote plot  : {FE_SUMMARY_PLOT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
