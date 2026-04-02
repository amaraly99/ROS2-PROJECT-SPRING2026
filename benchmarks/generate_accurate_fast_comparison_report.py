#!/usr/bin/env python3
"""Generate a combined comparison report for Latest_ACCURATE_Test vs Latest_FAST_Test."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from report_common import REPO_ROOT, RoleMetrics, SequenceData, aggregate_role_means
from generate_fast_profile_report import collect_profile_sequences, has_cpu


RESULTS_ROOT = REPO_ROOT / "results/ov2slam_benchmark"
ACCURATE_ROOT = RESULTS_ROOT / "Latest_ACCURATE_Test"
FAST_ROOT = RESULTS_ROOT / "Latest_FAST_Test"
REPORT_ROOT = REPO_ROOT / "benchmarks"
REPORT_PATH = REPORT_ROOT / "OV2SLAM_accurate_vs_fast_comparison_report.md"
APE_COMPARE_PLOT = REPORT_ROOT / "accurate_vs_fast_ape_rmse_by_sequence.png"
RMSE_DELTA_PLOT = REPORT_ROOT / "accurate_vs_fast_rmse_delta_by_sequence.png"
FE_COMPARE_PLOT = REPORT_ROOT / "accurate_vs_fast_fe_mean_by_sequence.png"
CPU_COMPARE_PLOT = REPORT_ROOT / "accurate_vs_fast_process_cpu_mean_by_sequence.png"
ROLE_COMPARE_PLOT = REPORT_ROOT / "accurate_vs_fast_role_means.png"

ACCURATE_TIMER_KEYS = [
    "0.Full-Front_End",
    "1.FE_Track-Mono",
    "1.FE_createKeyframe",
    "0.Keyframe-Processing_Mapper",
    "1.BA_localBA",
    "0.LC_ProcessKF",
]

FAST_TIMER_KEYS = [
    "0.Full-Front_End",
    "1.FE_Track-Mono",
    "1.FE_createKeyframe",
    "0.Keyframe-Processing_Mapper",
    "1.BA_localBA",
]


def relative_to_report(path: Path) -> str:
    return path.relative_to(REPO_ROOT).as_posix()


def map_by_name(sequences: list[SequenceData]) -> dict[str, SequenceData]:
    return {seq.name: seq for seq in sequences}


def mean_value(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def role_map(rows: list[tuple[str, float, float, int]]) -> dict[str, tuple[float, float, int]]:
    return {name: (mu, std, count) for name, mu, std, count in rows}


def fmt_cpu(seq: SequenceData) -> str:
    if not has_cpu(seq):
        return "n/a"
    return f"{seq.cpu.mean_pct:.1f}% ± {seq.cpu.std_pct:.1f}% (max {seq.cpu.max_pct:.1f}%)"


def generate_plots(accurate: list[SequenceData], fast: list[SequenceData]) -> None:
    names = [seq.name for seq in accurate]
    x = list(range(len(names)))
    width = 0.38

    fig, ax = plt.subplots(figsize=(11.5, 4.8), dpi=170)
    acc_rmse = [seq.evo.ape_rmse for seq in accurate]
    fast_rmse = [seq.evo.ape_rmse for seq in fast]
    bars1 = ax.bar([v - width / 2 for v in x], acc_rmse, width=width, label="Accurate", color="#4f81bd")
    bars2 = ax.bar([v + width / 2 for v in x], fast_rmse, width=width, label="Fast", color="#7d5ba6")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=35, ha="right")
    ax.set_ylabel("APE RMSE (m)")
    ax.set_title("Accurate vs Fast: APE RMSE by sequence")
    ax.grid(True, axis="y", alpha=0.35)
    ax.legend()
    ax.bar_label(bars1, labels=[f"{v:.3f}" for v in acc_rmse], padding=2, fontsize=7, rotation=90)
    ax.bar_label(bars2, labels=[f"{v:.3f}" for v in fast_rmse], padding=2, fontsize=7, rotation=90)
    fig.tight_layout()
    fig.savefig(APE_COMPARE_PLOT, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11.5, 4.8), dpi=170)
    rmse_delta = [f.evo.ape_rmse - a.evo.ape_rmse for a, f in zip(accurate, fast)]
    bars = ax.bar(x, rmse_delta, color=["#c0504d" if v > 0 else "#55a868" for v in rmse_delta])
    ax.axhline(0.0, color="#2f2f2f", linewidth=1.0)
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=35, ha="right")
    ax.set_ylabel("Fast - Accurate RMSE (m)")
    ax.set_title("RMSE delta by sequence")
    ax.grid(True, axis="y", alpha=0.35)
    ax.bar_label(bars, labels=[f"{v:+.3f}" for v in rmse_delta], padding=3, fontsize=8, rotation=90)
    fig.tight_layout()
    fig.savefig(RMSE_DELTA_PLOT, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11.5, 4.8), dpi=170)
    acc_fe = [seq.fe_mean_ms for seq in accurate]
    acc_fe_std = [seq.fe_std_ms for seq in accurate]
    fast_fe = [seq.fe_mean_ms for seq in fast]
    fast_fe_std = [seq.fe_std_ms for seq in fast]
    bars1 = ax.bar(
        [v - width / 2 for v in x],
        acc_fe,
        width=width,
        yerr=acc_fe_std,
        capsize=4,
        label="Accurate",
        color="#4f81bd",
        ecolor="#2f2f2f",
    )
    bars2 = ax.bar(
        [v + width / 2 for v in x],
        fast_fe,
        width=width,
        yerr=fast_fe_std,
        capsize=4,
        label="Fast",
        color="#dd8452",
        ecolor="#2f2f2f",
    )
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=35, ha="right")
    ax.set_ylabel("Full front-end time (ms)")
    ax.set_title("Accurate vs Fast: full front-end latency by sequence")
    ax.grid(True, axis="y", alpha=0.35)
    ax.legend()
    ax.bar_label(bars1, labels=[f"{m:.1f}" for m in acc_fe], padding=2, fontsize=7, rotation=90)
    ax.bar_label(bars2, labels=[f"{m:.1f}" for m in fast_fe], padding=2, fontsize=7, rotation=90)
    fig.tight_layout()
    fig.savefig(FE_COMPARE_PLOT, bbox_inches="tight")
    plt.close(fig)

    if all(has_cpu(seq) for seq in accurate) and all(has_cpu(seq) for seq in fast):
        fig, ax = plt.subplots(figsize=(11.5, 4.8), dpi=170)
        acc_cpu = [seq.cpu.mean_pct for seq in accurate]
        acc_cpu_std = [seq.cpu.std_pct for seq in accurate]
        fast_cpu = [seq.cpu.mean_pct for seq in fast]
        fast_cpu_std = [seq.cpu.std_pct for seq in fast]
        bars1 = ax.bar(
            [v - width / 2 for v in x],
            acc_cpu,
            width=width,
            yerr=acc_cpu_std,
            capsize=4,
            label="Accurate",
            color="#4f81bd",
            ecolor="#2f2f2f",
        )
        bars2 = ax.bar(
            [v + width / 2 for v in x],
            fast_cpu,
            width=width,
            yerr=fast_cpu_std,
            capsize=4,
            label="Fast",
            color="#55a868",
            ecolor="#2f2f2f",
        )
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=35, ha="right")
        ax.set_ylabel("Process CPU mean (%)")
        ax.set_title("Accurate vs Fast: process CPU utilization by sequence")
        ax.grid(True, axis="y", alpha=0.35)
        ax.legend()
        ax.bar_label(bars1, labels=[f"{m:.0f}" for m in acc_cpu], padding=2, fontsize=7, rotation=90)
        ax.bar_label(bars2, labels=[f"{m:.0f}" for m in fast_cpu], padding=2, fontsize=7, rotation=90)
        fig.tight_layout()
        fig.savefig(CPU_COMPARE_PLOT, bbox_inches="tight")
        plt.close(fig)
    else:
        fig, ax = plt.subplots(figsize=(10.5, 3.6), dpi=170)
        ax.axis("off")
        ax.text(
            0.5,
            0.5,
            "Per-sequence CPU monitor CSVs are not available in the Latest_* folders.\nDirect process CPU comparison cannot be reconstructed from these result sets.",
            ha="center",
            va="center",
            fontsize=12,
        )
        fig.tight_layout()
        fig.savefig(CPU_COMPARE_PLOT, bbox_inches="tight")
        plt.close(fig)

    acc_roles = role_map(aggregate_role_means(accurate))
    fast_roles = role_map(aggregate_role_means(fast))
    if acc_roles and fast_roles:
        role_names = sorted(set(acc_roles) | set(fast_roles))
        x2 = list(range(len(role_names)))
        acc_role_means = [acc_roles.get(name, (0.0, 0.0, 0))[0] for name in role_names]
        fast_role_means = [fast_roles.get(name, (0.0, 0.0, 0))[0] for name in role_names]
        fig, ax = plt.subplots(figsize=(10.5, 4.8), dpi=170)
        bars1 = ax.bar([v - width / 2 for v in x2], acc_role_means, width=width, label="Accurate", color="#4f81bd")
        bars2 = ax.bar([v + width / 2 for v in x2], fast_role_means, width=width, label="Fast", color="#55a868")
        ax.set_xticks(x2)
        ax.set_xticklabels(role_names, rotation=35, ha="right")
        ax.set_ylabel("Mean CPU role usage (%)")
        ax.set_title("Accurate vs Fast: logical thread-role utilization")
        ax.grid(True, axis="y", alpha=0.35)
        ax.legend()
        ax.bar_label(bars1, labels=[f"{v:.1f}" for v in acc_role_means], padding=2, fontsize=7, rotation=90)
        ax.bar_label(bars2, labels=[f"{v:.1f}" for v in fast_role_means], padding=2, fontsize=7, rotation=90)
        fig.tight_layout()
        fig.savefig(ROLE_COMPARE_PLOT, bbox_inches="tight")
        plt.close(fig)
    else:
        fig, ax = plt.subplots(figsize=(10.5, 3.6), dpi=170)
        ax.axis("off")
        ax.text(
            0.5,
            0.5,
            "Named-thread role CSVs are not available in the Latest_* folders.\nDirect thread-role utilization comparison cannot be reconstructed here.",
            ha="center",
            va="center",
            fontsize=12,
        )
        fig.tight_layout()
        fig.savefig(ROLE_COMPARE_PLOT, bbox_inches="tight")
        plt.close(fig)


def build_report(accurate: list[SequenceData], fast: list[SequenceData]) -> str:
    acc_by_name = map_by_name(accurate)
    fast_by_name = map_by_name(fast)
    names = [seq.name for seq in accurate]

    rmse_deltas = [fast_by_name[name].evo.ape_rmse - acc_by_name[name].evo.ape_rmse for name in names]
    fe_deltas = [fast_by_name[name].fe_mean_ms - acc_by_name[name].fe_mean_ms for name in names]

    fast_better_rmse = sum(1 for value in rmse_deltas if value < 0)
    fast_worse_rmse = sum(1 for value in rmse_deltas if value > 0)
    avg_rmse_delta = mean_value(rmse_deltas)
    avg_fe_delta = mean_value(fe_deltas)
    avg_speedup = mean_value([acc_by_name[name].fe_mean_ms / fast_by_name[name].fe_mean_ms for name in names if fast_by_name[name].fe_mean_ms > 0])

    cpu_available = all(has_cpu(seq) for seq in accurate) and all(has_cpu(seq) for seq in fast)
    cpu_deltas = [fast_by_name[name].cpu.mean_pct - acc_by_name[name].cpu.mean_pct for name in names] if cpu_available else []
    avg_cpu_delta = mean_value(cpu_deltas) if cpu_deltas else float("nan")

    acc_roles = role_map(aggregate_role_means(accurate))
    fast_roles = role_map(aggregate_role_means(fast))
    role_names = sorted(set(acc_roles) | set(fast_roles))

    lines: list[str] = []
    lines.append("# OV2SLAM Accurate vs Fast Comparison Report")
    lines.append("")
    lines.append(
        "This report contrasts the `Latest_ACCURATE_Test` and `Latest_FAST_Test` result sets over the same 10 EuRoC sequences. "
        "The comparison is organized around **accuracy**, **latency**, and **core/thread utilization**, using the saved trajectories and logs rather than trusting the incomplete top-level summary files alone."
    )
    lines.append("")
    lines.append("## Executive Comparison")
    lines.append("")
    lines.append(
        f"- **Accuracy:** fast is better on **{fast_better_rmse}/10** sequences and worse on **{fast_worse_rmse}/10** sequences, with an average RMSE delta of **{avg_rmse_delta:+.3f} m** (`fast - accurate`)."
    )
    lines.append(
        f"- **Latency:** fast reduces full front-end time by **{abs(avg_fe_delta):.2f} ms on average** (`{avg_fe_delta:+.2f} ms` delta), equivalent to an average **{avg_speedup:.2f}x** front-end speedup."
    )
    if cpu_available:
        lines.append(
            f"- **Core utilization:** fast changes process CPU mean by **{avg_cpu_delta:+.1f}%** on average, and it materially reduces the dominant optimization-role footprint relative to accurate mode."
        )
    else:
        lines.append(
            "- **Core utilization:** the latest accurate and fast folders do not retain the process/thread CPU CSVs, so direct core-utilization deltas cannot be reconstructed from these result sets."
        )
    lines.append("")
    lines.append("![](accurate_vs_fast_ape_rmse_by_sequence.png)")
    lines.append("")
    lines.append("![](accurate_vs_fast_rmse_delta_by_sequence.png)")
    lines.append("")
    lines.append("![](accurate_vs_fast_fe_mean_by_sequence.png)")
    lines.append("")
    lines.append("![](accurate_vs_fast_process_cpu_mean_by_sequence.png)")
    lines.append("")
    lines.append("![](accurate_vs_fast_role_means.png)")
    lines.append("")
    lines.append("## Accuracy Comparison")
    lines.append("")
    lines.append("| Sequence | Accurate RMSE (m) | Fast RMSE (m) | RMSE delta (fast-accurate) | Accurate matched | Fast matched |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: |")
    for name in names:
        acc = acc_by_name[name]
        fst = fast_by_name[name]
        lines.append(
            f"| {name} | {acc.evo.ape_rmse:.6f} | {fst.evo.ape_rmse:.6f} | {fst.evo.ape_rmse - acc.evo.ape_rmse:+.6f} | "
            f"{acc.evo.matched_found}/{acc.evo.matched_max} ({acc.matched_pct:.1f}%) | "
            f"{fst.evo.matched_found}/{fst.evo.matched_max} ({fst.matched_pct:.1f}%) |"
        )
    lines.append("")
    lines.append("Difference notes:")
    lines.append("- Fast mode provides slightly worse APE on most sequences, with the clearest degradations appearing on the more difficult runs.")
    lines.append("- The explicit RMSE delta makes it easy to state whether fast mode is an acceptable trade-off sequence-by-sequence.")
    lines.append("- Meeting phrasing: “fast mode buys speed, but its accuracy penalty is sequence dependent and grows on harder trajectories.”")
    lines.append("")
    lines.append("## Latency Comparison")
    lines.append("")
    lines.append("| Sequence | Accurate FE mean±std (ms) | Fast FE mean±std (ms) | FE delta (ms) | FE speedup | Accurate Local BA (ms) | Fast Local BA (ms) |")
    lines.append("| --- | --- | --- | ---: | ---: | ---: | ---: |")
    for name in names:
        acc = acc_by_name[name]
        fst = fast_by_name[name]
        fe_delta = fst.fe_mean_ms - acc.fe_mean_ms
        speedup = acc.fe_mean_ms / fst.fe_mean_ms if fst.fe_mean_ms > 0 else 0.0
        lines.append(
            f"| {name} | {acc.fe_mean_ms:.3f} ± {acc.fe_std_ms:.3f} | {fst.fe_mean_ms:.3f} ± {fst.fe_std_ms:.3f} | "
            f"{fe_delta:+.3f} | {speedup:.2f}x | {acc.timers['1.BA_localBA'][0]:.3f} | {fst.timers['1.BA_localBA'][0]:.3f} |"
        )
    lines.append("")
    lines.append("Difference notes:")
    lines.append("- Fast mode consistently cuts front-end latency, often by a large margin, and this is the clearest advantage of the fast configuration.")
    lines.append("- Local BA cost also drops sharply in fast mode, which reinforces that the speedup is not only detector-side but also backend-facing.")
    lines.append("- Meeting phrasing: “the main value of fast mode is predictable latency reduction, not a minor optimization around the edges.”")
    lines.append("")
    lines.append("## Core and Thread Utilization Comparison")
    lines.append("")
    if cpu_available:
        lines.append("| Sequence | Accurate CPU mean±std (max) | Fast CPU mean±std (max) | CPU delta | Accurate top roles | Fast top roles |")
        lines.append("| --- | --- | --- | ---: | --- | --- |")
        for name in names:
            acc = acc_by_name[name]
            fst = fast_by_name[name]
            lines.append(
                f"| {name} | {fmt_cpu(acc)} | {fmt_cpu(fst)} | "
                f"{fst.cpu.mean_pct - acc.cpu.mean_pct:+.1f}% | "
                f"{', '.join(role.name for role in acc.top_roles) or 'n/a'} | "
                f"{', '.join(role.name for role in fst.top_roles) or 'n/a'} |"
            )
        lines.append("")
        lines.append("| Logical role | Accurate mean (%) | Fast mean (%) | Delta (fast-accurate) |")
        lines.append("| --- | ---: | ---: | ---: |")
        for role_name in role_names:
            acc_mean = acc_roles.get(role_name, (0.0, 0.0, 0))[0]
            fast_mean = fast_roles.get(role_name, (0.0, 0.0, 0))[0]
            lines.append(f"| `{role_name}` | {acc_mean:.3f} | {fast_mean:.3f} | {fast_mean - acc_mean:+.3f} |")
        lines.append("")
        lines.append("Difference notes:")
        lines.append("- Accurate mode drives much heavier sustained optimization load, especially through `ov2_est` and `ov2_lc`.")
        lines.append("- Fast mode lowers overall process CPU and collapses the loop-closure contribution, which is consistent with the fast parameter set disabling that expensive path.")
        lines.append("- Meeting phrasing: “fast mode does not just shorten latency; it also reduces total multi-core pressure by trimming optimization work.”")
    else:
        lines.append("The direct process/thread CPU comparison is unavailable from these latest result sets because the folders do not retain:")
        lines.append("")
        lines.append("- `ov2slam_process_cpu.csv`")
        lines.append("- `ov2slam_named_thread_roles_aggregated.csv`")
        lines.append("")
        lines.append("What can still be said:")
        lines.append("")
        lines.append("- Both result sets were run with OV2SLAM pinned to CPUs `0,1,2,3` according to the benchmark metadata.")
        lines.append("- The fast configuration disables loop closure, so a lower optimization-related multi-core load is expected qualitatively even though it is not quantified here.")
        lines.append("- Meeting phrasing: “the latest paired result sets support a direct accuracy/latency comparison, while core-utilization comparison needs a rerun with monitor CSVs retained.”")
    lines.append("")
    lines.append("## Linked Source Reports")
    lines.append("")
    lines.append(f"- [Accurate report]({relative_to_report(ACCURATE_ROOT / 'OV2SLAM_accurate_profile_report.md')})")
    lines.append(f"- [Fast report]({relative_to_report(FAST_ROOT / 'OV2SLAM_fast_profile_report.md')})")
    return "\n".join(lines) + "\n"


def main() -> int:
    accurate = collect_profile_sequences(ACCURATE_ROOT, ACCURATE_TIMER_KEYS)
    fast = collect_profile_sequences(FAST_ROOT, FAST_TIMER_KEYS)
    if {seq.name for seq in accurate} != {seq.name for seq in fast}:
        raise SystemExit("Accurate/Fast sequence sets do not match.")
    accurate.sort(key=lambda seq: seq.name)
    fast.sort(key=lambda seq: seq.name)
    generate_plots(accurate, fast)
    REPORT_PATH.write_text(build_report(accurate, fast), encoding="utf-8")
    print(f"Wrote report: {REPORT_PATH}")
    for path in (APE_COMPARE_PLOT, RMSE_DELTA_PLOT, FE_COMPARE_PLOT, CPU_COMPARE_PLOT, ROLE_COMPARE_PLOT):
        print(f"Wrote plot  : {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
