#!/usr/bin/env python3
"""
Per-run and aggregate post-processing for ORB-SLAM2 benchmark outputs.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import statistics
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REQUIRED_LONG_LIVED_ROLES = {
    "ORBFrontEnd": 5,
    "ORBLocalMap": 5,
    "ORBLoopClose": 5,
}


def safe_mean(values):
    clean = [value for value in values if value is not None and not (isinstance(value, float) and math.isnan(value))]
    if not clean:
        return None
    return statistics.fmean(clean)


def safe_std(values):
    clean = [value for value in values if value is not None and not (isinstance(value, float) and math.isnan(value))]
    if len(clean) < 2:
        return 0.0
    return statistics.pstdev(clean)


def read_csv_rows(path: Path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def parse_float(row: dict, key: str):
    try:
        return float(row[key])
    except Exception:
        return None


def normalize_metric_key(category: str) -> str:
    out = []
    for char in category:
        if char.isalnum():
            out.append(char.lower())
        else:
            out.append("_")
    key = "".join(out)
    while "__" in key:
        key = key.replace("__", "_")
    return key.strip("_")


def summarize_timing(run_dir: Path):
    timing_csv = run_dir / "timing_events.csv"
    if not timing_csv.exists():
        raise FileNotFoundError(f"Missing timing events CSV: {timing_csv}")

    grouped = defaultdict(list)
    rows = read_csv_rows(timing_csv)
    for row in rows:
        category = (row.get("category") or "").strip()
        thread_role = (row.get("thread_role") or "").strip() or "unknown"
        duration_ms = parse_float(row, "duration_ms")
        if not category or duration_ms is None:
            continue
        grouped[(category, thread_role)].append(duration_ms)

    summary_rows = []
    category_means = {}
    for (category, thread_role), durations in sorted(grouped.items()):
        avg_ms = safe_mean(durations)
        summary_rows.append(
            {
                "category": category,
                "thread_role": thread_role,
                "avg_ms": f"{avg_ms:.6f}",
                "std_ms": f"{safe_std(durations):.6f}",
                "min_ms": f"{min(durations):.6f}",
                "max_ms": f"{max(durations):.6f}",
                "n_samples": len(durations),
            }
        )
        category_means[category] = avg_ms

    summary_csv = run_dir / "timing_summary.csv"
    with summary_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["category", "thread_role", "avg_ms", "std_ms", "min_ms", "max_ms", "n_samples"],
        )
        writer.writeheader()
        writer.writerows(summary_rows)

    return category_means


def summarize_cpu(run_dir: Path):
    process_csv = run_dir / "orbslam_process_cpu.csv"
    thread_csv = run_dir / "orbslam_thread_cpu.csv"
    thread_summary_csv = run_dir / "orbslam_thread_cpu_summary.csv"
    info_json = run_dir / "orbslam_pid_info.json"
    if not process_csv.exists() or not thread_csv.exists() or not thread_summary_csv.exists():
        raise FileNotFoundError("CPU monitoring artifacts are missing for this run.")

    process_rows = []
    for row in read_csv_rows(process_csv):
        elapsed = parse_float(row, "elapsed_s")
        cpu_pct = parse_float(row, "process_cpu_percent")
        if elapsed is None or cpu_pct is None:
            continue
        process_rows.append((elapsed, cpu_pct))

    if not process_rows:
        raise RuntimeError("No process CPU samples were recorded.")

    times = [row[0] for row in process_rows]
    cpu_values = [row[1] for row in process_rows]
    cpu_summary = {
        "avg_pct": safe_mean(cpu_values),
        "std_pct": safe_std(cpu_values),
        "min_pct": min(cpu_values),
        "max_pct": max(cpu_values),
        "n_samples": len(cpu_values),
    }

    cpu_summary_path = run_dir / "cpu_summary.json"
    cpu_summary_path.write_text(json.dumps(cpu_summary, indent=2) + "\n")

    info = {}
    if info_json.exists():
        try:
            info = json.loads(info_json.read_text())
        except Exception:
            info = {}

    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=160)
    ax.plot(times, cpu_values, color="#2c7fb8", linewidth=1.5)
    ax.set_xlabel("time since monitor start (s)")
    ax.set_ylabel("ORB-SLAM2 CPU usage (%)")
    ax.set_ylim(0, 400)
    title = run_dir.parent.name
    if info.get("orbslam_pid") is not None:
        title = f"{run_dir.parent.name} ORB-SLAM2 PID {info['orbslam_pid']}"
    ax.set_title(title)
    ax.grid(True, alpha=0.35)
    fig.tight_layout()
    fig.savefig(run_dir / "orbslam_cpu_usage.png", bbox_inches="tight")
    plt.close(fig)

    thread_rows = []
    for row in read_csv_rows(thread_summary_csv):
        try:
            thread_rows.append(
                (
                    row["comm"],
                    int(row["tid"]),
                    int(row["samples"]),
                    float(row["mean_cpu_percent"]),
                    float(row["max_cpu_percent"]),
                    row["dominant_core"],
                )
            )
        except Exception:
            continue

    thread_rows.sort(key=lambda item: (-item[3], item[1]))
    meaningful_threads = [row for row in thread_rows if row[2] >= 5 or row[3] >= 0.1 or row[4] >= 1.0]
    if not meaningful_threads:
        meaningful_threads = thread_rows
    meaningful_threads = meaningful_threads[:20]

    if meaningful_threads:
        labels = [f"{comm} [{tid}]" for comm, tid, _samples, _mean, _max, _core in meaningful_threads]
        values = [mean_cpu for _comm, _tid, _samples, mean_cpu, _max, _core in meaningful_threads]
        fig, ax = plt.subplots(figsize=(max(10, 0.6 * len(labels) + 3), 5.5), dpi=160)
        x = list(range(len(labels)))
        bars = ax.bar(x, values, color="#4C72B0")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45, ha="right")
        ax.set_ylabel("mean CPU usage (%)")
        ax.set_ylim(0, 400)
        ax.set_title(f"{run_dir.parent.name} top thread IDs by mean CPU usage")
        ax.grid(True, axis="y", alpha=0.35)
        ax.bar_label(bars, labels=[f"{value:.1f}" for value in values], padding=3, fontsize=8, rotation=90)
        fig.tight_layout()
        fig.savefig(run_dir / "orbslam_threads_cpu_bar.png", bbox_inches="tight")
        plt.close(fig)

    return cpu_summary


def validate_thread_roles(run_dir: Path):
    thread_csv = run_dir / "orbslam_thread_cpu.csv"
    role_sample_counts = defaultdict(int)
    role_tids = defaultdict(set)

    for row in read_csv_rows(thread_csv):
        comm = (row.get("comm") or "").strip()
        if not comm:
            continue
        role_sample_counts[comm] += 1
        try:
            role_tids[comm].add(int(row["tid"]))
        except Exception:
            pass

    missing_roles = []
    for role, min_samples in REQUIRED_LONG_LIVED_ROLES.items():
        if role_sample_counts.get(role, 0) < min_samples:
            missing_roles.append(role)

    report = {
        "required_long_lived_roles": REQUIRED_LONG_LIVED_ROLES,
        "observed_role_sample_counts": dict(sorted(role_sample_counts.items())),
        "observed_role_tid_counts": {role: len(tids) for role, tids in sorted(role_tids.items())},
        "missing_required_roles": missing_roles,
    }
    report_path = run_dir / "thread_identity_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")

    if missing_roles:
        raise RuntimeError(
            "Missing required long-lived ORB-SLAM2 thread roles: "
            + ", ".join(missing_roles)
        )

    return report


def plot_metric_per_sequence(results_root: Path, metric_name: str, series: list[tuple[str, float, float]], ylabel: str, title: str, output_name: str):
    if not series:
        return
    labels = [item[0] for item in series]
    means = [item[1] for item in series]
    stds = [item[2] for item in series]
    fig, ax = plt.subplots(figsize=(max(6, len(labels) * 1.25), 5), dpi=160)
    x = list(range(len(labels)))
    bars = ax.bar(x, means, yerr=stds, capsize=4, color="#4C72B0")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.35)
    ax.bar_label(bars, labels=[f"{value:.3f}" for value in means], padding=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(results_root / output_name, bbox_inches="tight")
    plt.close(fig)


def selected_runs_for_plots(run_entries: list[dict]):
    clean = [entry for entry in run_entries if entry.get("rmse") is not None and not entry.get("needs_rerun")]
    if clean:
        return clean
    return [entry for entry in run_entries if entry.get("rmse") is not None]


def load_run_entry(run_dir: Path):
    ape_stats = run_dir / "ape_stats.json"
    if not ape_stats.exists():
        return None
    stats = json.loads(ape_stats.read_text())
    timing_rows = read_csv_rows(run_dir / "timing_summary.csv") if (run_dir / "timing_summary.csv").exists() else []
    timing_means = {}
    for row in timing_rows:
        category = row.get("category", "").strip()
        avg_ms = parse_float(row, "avg_ms")
        if category and avg_ms is not None:
            timing_means[category] = avg_ms
    cpu_summary = {}
    cpu_summary_path = run_dir / "cpu_summary.json"
    if cpu_summary_path.exists():
        cpu_summary = json.loads(cpu_summary_path.read_text())
    return {
        "run_dir": run_dir,
        "run_name": run_dir.name,
        "rmse": stats.get("rmse"),
        "needs_rerun": bool(stats.get("needs_rerun")),
        "frontend_full_tracking_ms": timing_means.get("frontend/full_tracking"),
        "cpu_total_avg_pct": cpu_summary.get("avg_pct"),
        "ape_stats": stats,
    }


def aggregate_results(results_root: Path):
    sequence_map = {}
    for seq_dir in sorted([path for path in results_root.iterdir() if path.is_dir()]):
        run_entries = []
        for run_dir in sorted([path for path in seq_dir.iterdir() if path.is_dir() and path.name.startswith("run_")]):
            entry = load_run_entry(run_dir)
            if entry:
                run_entries.append(entry)
        if run_entries:
            sequence_map[seq_dir.name] = run_entries

    rmse_series = []
    frontend_series = []
    cpu_series = []
    thread_role_values = defaultdict(list)

    for sequence_name, run_entries in sorted(sequence_map.items()):
        selected = selected_runs_for_plots(run_entries)
        rmse_values = [entry["rmse"] for entry in selected if entry["rmse"] is not None]
        frontend_values = [entry["frontend_full_tracking_ms"] for entry in selected if entry["frontend_full_tracking_ms"] is not None]
        cpu_values = [entry["cpu_total_avg_pct"] for entry in selected if entry["cpu_total_avg_pct"] is not None]
        if rmse_values:
            rmse_series.append((sequence_name, safe_mean(rmse_values), safe_std(rmse_values)))
        if frontend_values:
            frontend_series.append((sequence_name, safe_mean(frontend_values), safe_std(frontend_values)))
        if cpu_values:
            cpu_series.append((sequence_name, safe_mean(cpu_values), safe_std(cpu_values)))

        for entry in selected:
            thread_csv = entry["run_dir"] / "orbslam_thread_cpu.csv"
            if not thread_csv.exists():
                continue
            for row in read_csv_rows(thread_csv):
                comm = (row.get("comm") or "").strip()
                cpu_pct = parse_float(row, "thread_cpu_percent")
                if not comm or cpu_pct is None:
                    continue
                thread_role_values[comm].append(cpu_pct)

    plot_metric_per_sequence(
        results_root,
        "rmse",
        rmse_series,
        "APE RMSE (m)",
        "ORB-SLAM2 RMSE per Sequence",
        "rmse_per_sequence.png",
    )
    plot_metric_per_sequence(
        results_root,
        "frontend_full_tracking",
        frontend_series,
        "Frontend full tracking (ms)",
        "ORB-SLAM2 Frontend Timing per Sequence",
        "frontend_timing_per_sequence.png",
    )

    if cpu_series:
        labels = [item[0] for item in cpu_series]
        means = [item[1] for item in cpu_series]
        stds = [item[2] for item in cpu_series]
        fig, ax = plt.subplots(figsize=(max(6, len(labels) * 1.25), 5), dpi=160)
        x = list(range(len(labels)))
        bars = ax.bar(x, means, yerr=stds, capsize=4, color="#55A868")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=15, ha="right")
        ax.set_ylabel("Average CPU usage (%)")
        ax.set_ylim(0, 400)
        ax.set_title("ORB-SLAM2 Total CPU Usage per Sequence")
        ax.grid(True, axis="y", alpha=0.35)
        ax.bar_label(bars, labels=[f"{value:.1f}%" for value in means], padding=3, fontsize=8)
        fig.tight_layout()
        fig.savefig(results_root / "cpu_per_sequence.png", bbox_inches="tight")
        plt.close(fig)

    if thread_role_values:
        ordered = sorted(thread_role_values.items(), key=lambda item: (-safe_mean(item[1]), item[0]))
        labels = [item[0] for item in ordered]
        means = [safe_mean(item[1]) for item in ordered]
        stds = [safe_std(item[1]) for item in ordered]
        fig, ax = plt.subplots(figsize=(8, max(4, len(labels) * 0.45)), dpi=160)
        y = list(range(len(labels)))
        ax.barh(y, means, xerr=stds, capsize=3, color="#C44E52")
        ax.set_yticks(y)
        ax.set_yticklabels(labels, fontsize=8)
        ax.set_xlabel("Average CPU usage (%)")
        ax.set_xlim(0, 400)
        ax.set_title("ORB-SLAM2 CPU Usage per Thread Role")
        ax.grid(True, axis="x", alpha=0.35)
        ax.invert_yaxis()
        fig.tight_layout()
        fig.savefig(results_root / "cpu_per_thread.png", bbox_inches="tight")
        plt.close(fig)

    for sequence_name, run_entries in sorted(sequence_map.items()):
        for entry in run_entries:
            run_label = entry["run_name"]
            for src_name, dst_prefix in (("traj_xy.png", "traj_xy"), ("ape_traj.png", "ape_traj")):
                src = entry["run_dir"] / src_name
                if not src.exists():
                    continue
                dst = results_root / f"{dst_prefix}_{sequence_name}_{run_label}.png"
                shutil.copy2(src, dst)

    write_polling_results(results_root, sequence_map)


def write_polling_results(results_root: Path, sequence_map: dict[str, list[dict]]):
    lines = [
        "# ORB-SLAM2 Benchmark Gallery",
        "",
        f"- Results folder: `{results_root}`",
        "",
    ]
    for sequence_name, run_entries in sorted(sequence_map.items()):
        lines.append(f"## {sequence_name}")
        lines.append("")
        lines.append("| Run | RMSE | Match Ratio | APE | Trajectory XY | Stats | Trajectory |")
        lines.append("|---|---:|---:|---|---|---|---|")
        for entry in sorted(run_entries, key=lambda item: item["run_name"]):
            run_dir = entry["run_dir"]
            stats = entry["ape_stats"]
            match_ratio = stats.get("match_ratio")
            match_str = f"{match_ratio * 100:.1f}%" if isinstance(match_ratio, (float, int)) else "-"
            ape_rel = (run_dir / "ape_traj.png").relative_to(results_root).as_posix() if (run_dir / "ape_traj.png").exists() else "-"
            traj_rel = (run_dir / "traj_xy.png").relative_to(results_root).as_posix() if (run_dir / "traj_xy.png").exists() else "-"
            stats_rel = (run_dir / "ape_stats.json").relative_to(results_root).as_posix() if (run_dir / "ape_stats.json").exists() else "-"
            tum_rel = (run_dir / "trajectory.tum").relative_to(results_root).as_posix() if (run_dir / "trajectory.tum").exists() else "-"
            ape_cell = f"[![]({ape_rel})]({ape_rel})" if ape_rel != "-" else "-"
            traj_cell = f"[![]({traj_rel})]({traj_rel})" if traj_rel != "-" else "-"
            stats_cell = f"[json]({stats_rel})" if stats_rel != "-" else "-"
            tum_cell = f"[tum]({tum_rel})" if tum_rel != "-" else "-"
            lines.append(
                f"| {entry['run_name']} | {entry['rmse']:.6f} | {match_str} | {ape_cell} | {traj_cell} | {stats_cell} | {tum_cell} |"
            )
        lines.append("")

    (results_root / "polling_results.md").write_text("\n".join(lines) + "\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Post-process ORB-SLAM2 benchmark runs")
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--aggregate", action="store_true")
    parser.add_argument("--results-root", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.run_dir:
        summarize_timing(args.run_dir)
        summarize_cpu(args.run_dir)
        validate_thread_roles(args.run_dir)
        return 0

    if args.aggregate:
        if not args.results_root:
            raise SystemExit("--results-root is required with --aggregate")
        aggregate_results(args.results_root)
        return 0

    raise SystemExit("Either --run-dir or --aggregate must be provided.")


if __name__ == "__main__":
    raise SystemExit(main())
