#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# compare_slam_hil.py — aggregate N tagged HIL runs per SLAM backend into
# cross-run comparison figures + a summary table, for a paper/report.
#
#   benchmarks/.evo_venv/bin/python benchmarks/compare_slam_hil.py \
#       --group "ORB-SLAM2:bags/run_orbslam2_eval_preflight_code_julien_*" \
#       --group "OV2SLAM (accurate):bags/run_ov2slam_oracle_accurate_preflight_code_julien_*" \
#       --out docs/reports/2026-07-05_ov2slam_vs_orbslam2/figures
#
# Each --group is "Label:glob" — glob expands to N already-evaluated run dirs
# (must already have slam_metrics.csv from eval_slam_hil.py / slam_eval).
#
# Writes into --out:
#   cmp_frontend_latency.png   frontend latency, mean±std across runs, per group
#   cmp_ate_rmse.png           ATE RMSE (m), mean±std across runs, per group
#   cmp_cpu_total.png          total CPU%, mean±std across runs, per group
#   cmp_thread_<group>.png     per-thread CPU, mean±std across runs, one per group
#   cmp_trajectories_grid.png  all runs' fig_slam_trajectory.png, one row per group
#   summary.csv                per-run rows + per-group mean/std rows
# ─────────────────────────────────────────────────────────────────────────────
from __future__ import annotations

import argparse
import csv
import glob as globmod
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

_GROUP_COLORS = ["#6a4c93", "#2f6db3", "#7fb069", "#c44e52"]


def read_metrics(rundir: Path) -> dict:
    f = rundir / "slam_metrics.csv"
    if not f.exists():
        return {}
    out = {}
    with open(f, newline="") as fh:
        for row in csv.reader(fh):
            if len(row) != 2 or row[0] == "metric":
                continue
            k, v = row
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def read_thread_cpu(rundir: Path) -> dict:
    f = rundir / "slam_thread_cpu.csv"
    if not f.exists():
        return {}
    by_thread: dict = {}
    with open(f, newline="") as fh:
        for row in csv.DictReader(fh):
            name = row.get("thread_name", "").strip()
            if not name:
                continue
            try:
                by_thread.setdefault(name, []).append(float(row["cpu_percent"]))
            except (KeyError, ValueError):
                continue
    return {n: float(np.mean(v)) for n, v in by_thread.items()}


def parse_groups(args_groups: list[str]) -> dict[str, list[Path]]:
    groups = {}
    for spec in args_groups:
        label, pattern = spec.split(":", 1)
        dirs = sorted(Path(p) for p in globmod.glob(pattern) if Path(p).is_dir())
        if not dirs:
            raise SystemExit(f"no run dirs matched for group '{label}': {pattern}")
        groups[label] = dirs
    return groups


def grouped_bar(groups_data: dict[str, tuple[float, float]], ylabel: str, title: str,
                 filename: str, outdir: Path, ylim=None):
    labels = list(groups_data.keys())
    means = [groups_data[l][0] for l in labels]
    stds = [groups_data[l][1] for l in labels]
    colors = _GROUP_COLORS[: len(labels)]
    fig, ax = plt.subplots(figsize=(1.8 * len(labels) + 2, 5.5))
    x = np.arange(len(labels))
    ax.bar(x, means, yerr=stds, width=0.5, capsize=6, color=colors,
           edgecolor="black", linewidth=0.4, alpha=0.9)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if ylim:
        ax.set_ylim(*ylim)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(outdir / filename, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {filename}")


def thread_plot(label: str, rundirs: list[Path], outdir: Path, color: str, safe_name: str):
    per_run = [read_thread_cpu(d) for d in rundirs]
    all_names = sorted({n for r in per_run for n in r})
    if not all_names:
        return
    means, stds = [], []
    for n in all_names:
        vals = [r[n] for r in per_run if n in r]
        means.append(float(np.mean(vals)) if vals else 0.0)
        stds.append(float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0)
    order = np.argsort(means)[::-1]
    names_sorted = [all_names[i] for i in order]
    means_sorted = [means[i] for i in order]
    stds_sorted = [stds[i] for i in order]
    fig, ax = plt.subplots(figsize=(max(6, len(names_sorted) * 1.1 + 1), 5))
    x = np.arange(len(names_sorted))
    ax.bar(x, means_sorted, yerr=stds_sorted, capsize=4, color=color,
           edgecolor="black", linewidth=0.4)
    ax.set_xticks(x)
    ax.set_xticklabels(names_sorted, rotation=25, ha="right")
    ax.set_ylabel("CPU usage (% of one core)")
    ax.set_title(f"{label} — per-thread CPU (mean±std across {len(rundirs)} runs)")
    ax.set_ylim(0, max(400.0, max(means_sorted) * 1.2 if means_sorted else 400.0))
    ax.axhline(100, color="grey", linestyle="--", linewidth=1.0)
    ax.axhline(200, color="grey", linestyle="--", linewidth=1.0)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fname = f"cmp_thread_{safe_name}.png"
    fig.savefig(outdir / fname, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {fname}")


def trajectory_grid(groups: dict[str, list[Path]], outdir: Path):
    n_cols = max(len(v) for v in groups.values())
    n_rows = len(groups)
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(4.2 * n_cols, 4.6 * n_rows))
    if n_rows == 1:
        axes = np.array([axes])
    if n_cols == 1:
        axes = axes.reshape(n_rows, 1)
    for row, (label, dirs) in enumerate(groups.items()):
        for col in range(n_cols):
            ax = axes[row, col]
            if col < len(dirs):
                imgpath = dirs[col] / "fig_slam_trajectory.png"
                if imgpath.exists():
                    ax.imshow(mpimg.imread(imgpath))
                    ax.set_title(f"{label} — run {col+1}", fontsize=9)
                else:
                    ax.text(0.5, 0.5, "missing", ha="center", va="center")
            ax.axis("off")
    fig.tight_layout()
    fig.savefig(outdir / "cmp_trajectories_grid.png", dpi=180, bbox_inches="tight")
    plt.close(fig)
    print("  wrote cmp_trajectories_grid.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group", action="append", required=True, help="Label:glob_pattern")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)

    groups = parse_groups(args.group)

    rows = []
    per_group_metrics: dict[str, dict[str, list[float]]] = {}
    for label, dirs in groups.items():
        per_group_metrics[label] = {}
        for d in dirs:
            m = read_metrics(d)
            rows.append({"group": label, "run": d.name, **m})
            for k, v in m.items():
                if isinstance(v, float):
                    per_group_metrics[label].setdefault(k, []).append(v)

    def agg(metric):
        return {label: (float(np.mean(vals)), float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0)
                for label, vals in ((l, per_group_metrics[l].get(metric, [])) for l in groups)
                if vals}

    grouped_bar(agg("FE_full_tracking_ms_mean"), "Front-end latency (ms)",
                "Front-end latency — mean±std across runs", "cmp_frontend_latency.png", outdir)
    # Mission ATE (excludes the init_gate's pre-mission warm-up window) is the
    # number that reflects real tracking quality -- fall back to whole-bag
    # only if a group has no FSM marker at all (e.g. a non-gated run).
    ate_agg = agg("ATE_RMSE_mission_m")
    if not ate_agg:
        ate_agg = agg("ATE_RMSE_full_m")
    grouped_bar(ate_agg, "ATE RMSE (m)",
                "Mission ATE RMSE — mean±std across runs", "cmp_ate_rmse.png", outdir)
    grouped_bar(agg("CPU_percent_mean"), "CPU usage (% of one core)",
                "Total CPU usage — mean±std across runs", "cmp_cpu_total.png", outdir)

    for i, (label, dirs) in enumerate(groups.items()):
        safe = label.lower().replace(" ", "_").replace("(", "").replace(")", "")
        thread_plot(label, dirs, outdir, _GROUP_COLORS[i % len(_GROUP_COLORS)], safe)

    trajectory_grid(groups, outdir)

    # Print per-run + aggregated tables for the metrics a report table would need.
    print("\n=== per-run summary (ATE mission/full in m, FE in ms) ===")
    for label, dirs in groups.items():
        for d in dirs:
            m = read_metrics(d)
            print(f"  {label:20s} {d.name:60s} "
                  f"ATE_mission={m.get('ATE_RMSE_mission_m', float('nan')):.4f} "
                  f"ATE_full={m.get('ATE_RMSE_full_m', float('nan')):.4f} "
                  f"GT%={m.get('slam_coverage_after_fsm_pct', float('nan')):.2f} "
                  f"FE_mean={m.get('FE_full_tracking_ms_mean', float('nan')):.3f} "
                  f"FE_std={m.get('FE_full_tracking_ms_std', float('nan')):.3f}")
    print("\n=== per-group aggregate (mean ± std across runs) ===")
    for metric in ("ATE_RMSE_mission_m", "ATE_RMSE_full_m", "slam_coverage_after_fsm_pct",
                   "FE_full_tracking_ms_mean", "CPU_percent_mean"):
        for label, (mean, std) in agg(metric).items():
            print(f"  {label:20s} {metric:28s} {mean:.4f} ± {std:.4f}")

    fieldnames = sorted({k for r in rows for k in r})
    with open(outdir / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    print(f"  wrote summary.csv ({len(rows)} runs)")


if __name__ == "__main__":
    main()
