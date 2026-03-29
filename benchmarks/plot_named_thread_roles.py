#!/usr/bin/env python3
"""Aggregate OV2SLAM named-thread CPU data by logical thread role."""

from __future__ import annotations

import csv
import json
import math
import os
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


NAMED_THREAD_NAMES = (
    "ov2_main",
    "ov2_feed",
    "ov2_map",
    "ov2_est",
    "ov2_lc",
    "ov2_merge",
    "ov2_vizkf",
    "ov2_vizfr",
)

# Edit these paths directly when you want to analyze a different sequence.
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SEQUENCE_DIR = REPO_ROOT / "results/ov2slam_benchmark/Latest_ACCURATE_Test/MH_04_difficult"
SEQUENCE_DIR = Path(os.environ.get("THREAD_ROLE_SEQUENCE_DIR", str(DEFAULT_SEQUENCE_DIR)))
THREAD_CSV = Path(os.environ.get("THREAD_ROLE_THREAD_CSV", str(SEQUENCE_DIR / "ov2slam_thread_cpu.csv")))
PID_INFO_JSON = Path(os.environ.get("THREAD_ROLE_PID_INFO_JSON", str(SEQUENCE_DIR / "ov2slam_pid_info.json")))

# The results directory is often not writable from this session, so write here.
OUTPUT_PNG = Path(
    os.environ.get(
        "THREAD_ROLE_OUTPUT_PNG",
        str(REPO_ROOT / "mh04_named_thread_roles_aggregated.png"),
    )
)
SUMMARY_CSV = Path(
    os.environ.get(
        "THREAD_ROLE_SUMMARY_CSV",
        str(REPO_ROOT / "mh04_named_thread_roles_aggregated.csv"),
    )
)
FIGURE_TITLE = os.environ.get("THREAD_ROLE_FIGURE_TITLE", "MH_04_difficult OV2SLAM named threads")


def load_pid(pid_info_path: Path) -> int | None:
    try:
        data = json.loads(pid_info_path.read_text())
    except Exception:
        return None
    value = data.get("ov2slam_pid")
    return int(value) if value is not None else None


def role_for_row(comm: str, tid: int, ov2slam_pid: int | None) -> str | None:
    if ov2slam_pid is not None and tid == ov2slam_pid:
        return "ov2slam_node_main"
    if comm in NAMED_THREAD_NAMES:
        return comm
    return None


def mean_and_std(values: list[float]) -> tuple[float, float]:
    if not values:
        return 0.0, 0.0
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return mean, math.sqrt(variance)


def main() -> int:
    ov2slam_pid = load_pid(PID_INFO_JSON)

    overall_values: dict[str, list[float]] = defaultdict(list)
    per_core_values: dict[str, dict[int, list[float]]] = defaultdict(lambda: defaultdict(list))
    unique_tids: dict[str, set[int]] = defaultdict(set)

    with THREAD_CSV.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                tid = int(row["tid"])
                comm = row["comm"]
                last_cpu = int(row["last_cpu"])
                cpu_pct = float(row["thread_cpu_percent"])
            except Exception:
                continue

            role = role_for_row(comm, tid, ov2slam_pid)
            if role is None:
                continue

            overall_values[role].append(cpu_pct)
            per_core_values[role][last_cpu].append(cpu_pct)
            unique_tids[role].add(tid)

    roles = sorted(overall_values.keys())
    if not roles:
        raise SystemExit("No OV2SLAM named-thread rows found in the provided CSV.")

    cores = sorted({core for role in roles for core in per_core_values[role].keys()})
    overall_stats = {role: mean_and_std(values) for role, values in overall_values.items()}
    per_core_stats = {
        role: {
            core: mean_and_std(values)
            for core, values in per_core_values[role].items()
        }
        for role in roles
    }

    SUMMARY_CSV.parent.mkdir(parents=True, exist_ok=True)
    with SUMMARY_CSV.open("w", newline="") as f:
        fieldnames = ["role", "unique_tid_count", "sample_count", "mean_cpu_all", "std_cpu_all"]
        fieldnames += [f"mean_cpu_core_{core}" for core in cores]
        fieldnames += [f"std_cpu_core_{core}" for core in cores]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for role in roles:
            overall_mean, overall_std = overall_stats[role]
            row = {
                "role": role,
                "unique_tid_count": len(unique_tids[role]),
                "sample_count": len(overall_values[role]),
                "mean_cpu_all": f"{overall_mean:.3f}",
                "std_cpu_all": f"{overall_std:.3f}",
            }
            for core in cores:
                core_mean, core_std = per_core_stats[role].get(core, (0.0, 0.0))
                row[f"mean_cpu_core_{core}"] = f"{core_mean:.3f}"
                row[f"std_cpu_core_{core}"] = f"{core_std:.3f}"
            writer.writerow(row)

    fig, axes = plt.subplots(2, 1, figsize=(12, 10), dpi=160)

    # Subplot 1: overall average by logical role.
    x = list(range(len(roles)))
    overall_bar_values = [overall_stats[role][0] for role in roles]
    overall_bar_std = [overall_stats[role][1] for role in roles]
    bars = axes[0].bar(
        x,
        overall_bar_values,
        yerr=overall_bar_std,
        capsize=4,
        color="#4472c4",
        ecolor="#2f2f2f",
    )
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(roles, rotation=25, ha="right")
    axes[0].set_ylabel("mean CPU usage (%)")
    axes[0].set_ylim(0, 400)
    axes[0].set_title(f"{FIGURE_TITLE}: average by thread role (mean ± std)")
    axes[0].grid(True, axis="y", alpha=0.35)
    axes[0].bar_label(
        bars,
        labels=[f"{mean:.1f}±{std:.1f}" for mean, std in zip(overall_bar_values, overall_bar_std)],
        padding=3,
        fontsize=8,
    )

    # Subplot 2: per-core average by logical role.
    width = 0.8 / max(1, len(cores))
    palette = plt.get_cmap("tab10")
    for idx, core in enumerate(cores):
        core_values = [per_core_stats[role].get(core, (0.0, 0.0))[0] for role in roles]
        core_std = [per_core_stats[role].get(core, (0.0, 0.0))[1] for role in roles]
        pos = [value + (idx - (len(cores) - 1) / 2) * width for value in x]
        bars = axes[1].bar(
            pos,
            core_values,
            width=width,
            yerr=core_std,
            capsize=3,
            label=f"CPU {core}",
            color=palette(idx % 10),
            ecolor="#2f2f2f",
        )
        axes[1].bar_label(
            bars,
            labels=[f"{mean:.1f}±{std:.1f}" for mean, std in zip(core_values, core_std)],
            padding=2,
            fontsize=7,
            rotation=90,
        )

    axes[1].set_xticks(x)
    axes[1].set_xticklabels(roles, rotation=25, ha="right")
    axes[1].set_ylabel("mean CPU usage (%)")
    axes[1].set_ylim(0, 400)
    axes[1].set_title(f"{FIGURE_TITLE}: per-core average by thread role (mean ± std)")
    axes[1].grid(True, axis="y", alpha=0.35)
    axes[1].legend(loc="upper right", ncol=max(1, min(4, len(cores))))

    fig.tight_layout()
    OUTPUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT_PNG, bbox_inches="tight")
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
