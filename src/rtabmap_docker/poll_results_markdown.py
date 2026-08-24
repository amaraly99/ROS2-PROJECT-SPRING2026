#!/usr/bin/env python3

import argparse
import json
import statistics
import time
from datetime import datetime
from pathlib import Path


RESULTS_ROOT = Path(__file__).resolve().parent / "results"
OUTPUT_NAME = "polling_results.md"
IMAGE_FILES = [
    ("APE Trajectory Map", "ape_traj_map.png"),
    ("APE Trajectory Raw", "ape_traj_raw.png"),
    ("Trajectory XY", "traj_xy_trajectories.png"),
    ("Trajectory XYZ", "traj_xy_xyz.png"),
    ("Trajectory Speeds", "traj_xy_speeds.png"),
    ("Trajectory RPY", "traj_xy_rpy.png"),
]
METRIC_KEYS = ["rmse", "mean", "median", "std", "min", "max", "sse"]


def latest_results_dir(results_root: Path) -> Path:
    candidates = [p for p in results_root.iterdir() if p.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"No timestamped result folders found in {results_root}")
    return max(candidates, key=lambda p: p.name)


def sequence_dirs(results_dir: Path):
    return sorted([p for p in results_dir.iterdir() if p.is_dir()], key=lambda p: p.name)


def run_dirs(sequence_dir: Path):
    return sorted([p for p in sequence_dir.iterdir() if p.is_dir() and p.name.startswith("run_")], key=lambda p: p.name)


def read_stats(path: Path):
    return json.loads(path.read_text())


def fmt(value):
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def rel(path: Path, base: Path):
    return path.relative_to(base).as_posix()


def image_cell(path: Path, base: Path):
    if not path.exists():
        return "-"
    rel_path = rel(path, base)
    return f"[![]({rel_path})]({rel_path})"


def file_link_cell(path: Path, base: Path, label: str):
    if not path.exists():
        return "-"
    rel_path = rel(path, base)
    return f"[{label}]({rel_path})"


def collect_data(results_dir: Path):
    sequences = {}
    all_runs = set()

    for seq_dir in sequence_dirs(results_dir):
        runs = {}
        for run_dir in run_dirs(seq_dir):
            stats_path = run_dir / "ape_stats.json"
            if not stats_path.exists():
                continue
            runs[run_dir.name] = {
                "stats": read_stats(stats_path),
                "dir": run_dir,
            }
            all_runs.add(run_dir.name)
        if runs:
            sequences[seq_dir.name] = runs

    return sequences, sorted(all_runs)


def build_runwise_results_md(results_dir: Path, sequences, run_names):
    lines = ["## Run-wise Evo APE Results", ""]
    for run_name in run_names:
        lines.append(f"### {run_name}")
        lines.append("")
        lines.append("| Sequence | RMSE | Mean | Median | Std | Min | Max | SSE | Stats | TUM |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---|---|")
        for sequence_name in sorted(sequences):
            run = sequences[sequence_name].get(run_name)
            if not run:
                lines.append(f"| {sequence_name} | - | - | - | - | - | - | - | - | - |")
                continue
            stats = run["stats"]
            run_dir = run["dir"]
            stats_link = file_link_cell(run_dir / "ape_stats.json", results_dir, "json")
            traj_link = file_link_cell(run_dir / "trajectory.tum", results_dir, "tum")
            lines.append(
                "| "
                + " | ".join(
                    [
                        sequence_name,
                        fmt(stats.get("rmse")),
                        fmt(stats.get("mean")),
                        fmt(stats.get("median")),
                        fmt(stats.get("std")),
                        fmt(stats.get("min")),
                        fmt(stats.get("max")),
                        fmt(stats.get("sse")),
                        stats_link,
                        traj_link,
                    ]
                )
                + " |"
            )
        lines.append("")
    return lines


def build_trajectory_tables_md(results_dir: Path, sequences, run_names):
    lines = ["## Run-wise Trajectories", ""]
    for title, image_name in IMAGE_FILES:
        lines.append(f"### {title}")
        lines.append("")
        header = "| Sequence | " + " | ".join(run_names) + " |"
        divider = "|---|" + "|".join(["---"] * len(run_names)) + "|"
        lines.append(header)
        lines.append(divider)
        for sequence_name in sorted(sequences):
            cells = [sequence_name]
            for run_name in run_names:
                run = sequences[sequence_name].get(run_name)
                if not run:
                    cells.append("-")
                    continue
                cells.append(image_cell(run["dir"] / image_name, results_dir))
            lines.append("| " + " | ".join(cells) + " |")
        lines.append("")
    return lines


def aggregate_metrics(values):
    return {
        "runs": len(values),
        "avg": statistics.fmean(values),
        "best": min(values),
        "worst": max(values),
    }


def build_aggregated_results_md(sequences):
    lines = ["## Run-aggregated Evo APE Results", ""]
    lines.append("| Sequence | Runs | RMSE Avg | RMSE Best | RMSE Worst | Mean Avg | Median Avg | Std Avg | Max Avg |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|")

    for sequence_name in sorted(sequences):
        stats_list = [run["stats"] for run in sequences[sequence_name].values()]
        rmse = aggregate_metrics([stats["rmse"] for stats in stats_list])
        mean_avg = statistics.fmean(stats["mean"] for stats in stats_list)
        median_avg = statistics.fmean(stats["median"] for stats in stats_list)
        std_avg = statistics.fmean(stats["std"] for stats in stats_list)
        max_avg = statistics.fmean(stats["max"] for stats in stats_list)
        lines.append(
            "| "
            + " | ".join(
                [
                    sequence_name,
                    str(rmse["runs"]),
                    fmt(rmse["avg"]),
                    fmt(rmse["best"]),
                    fmt(rmse["worst"]),
                    fmt(mean_avg),
                    fmt(median_avg),
                    fmt(std_avg),
                    fmt(max_avg),
                ]
            )
            + " |"
        )

    lines.append("")

    all_stats = [run["stats"] for runs in sequences.values() for run in runs.values()]
    if all_stats:
        overall_rmse = aggregate_metrics([stats["rmse"] for stats in all_stats])
        lines.append("### Overall")
        lines.append("")
        lines.append("| Total Runs | RMSE Avg | RMSE Best | RMSE Worst | Mean Avg | Median Avg | Std Avg |")
        lines.append("|---:|---:|---:|---:|---:|---:|---:|")
        lines.append(
            "| "
            + " | ".join(
                [
                    str(overall_rmse["runs"]),
                    fmt(overall_rmse["avg"]),
                    fmt(overall_rmse["best"]),
                    fmt(overall_rmse["worst"]),
                    fmt(statistics.fmean(stats["mean"] for stats in all_stats)),
                    fmt(statistics.fmean(stats["median"] for stats in all_stats)),
                    fmt(statistics.fmean(stats["std"] for stats in all_stats)),
                ]
            )
            + " |"
        )
        lines.append("")

    return lines


def build_markdown(results_dir: Path):
    sequences, run_names = collect_data(results_dir)
    if not sequences:
        raise FileNotFoundError(f"No run data with ape_stats.json found in {results_dir}")

    lines = [
        "# Polling Results",
        "",
        f"- Generated: {datetime.now().isoformat(timespec='seconds')}",
        f"- Results folder: `{results_dir}`",
        f"- Sequences: `{len(sequences)}`",
        f"- Runs: `{', '.join(run_names)}`",
        "",
    ]
    lines.extend(build_runwise_results_md(results_dir, sequences, run_names))
    lines.extend(build_trajectory_tables_md(results_dir, sequences, run_names))
    lines.extend(build_aggregated_results_md(sequences))
    return "\n".join(lines).rstrip() + "\n"


def write_markdown(results_dir: Path):
    content = build_markdown(results_dir)
    output_path = results_dir / OUTPUT_NAME
    try:
        output_path.write_text(content)
    except PermissionError as exc:
        raise PermissionError(
            f"Cannot write {output_path}. "
            "Make sure the results folder is writable by your Pi user."
        ) from exc
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description="Write polling_results.md for the latest timestamped results folder."
    )
    parser.add_argument("--results-root", default=str(RESULTS_ROOT))
    parser.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="Polling interval in seconds. Use 0 to run once.",
    )
    args = parser.parse_args()

    results_root = Path(args.results_root).resolve()
    if not results_root.is_dir():
        raise SystemExit(f"[ERROR] Missing results root: {results_root}")

    last_target = None
    while True:
        target = latest_results_dir(results_root)
        output_path = write_markdown(target)
        if target != last_target:
            print(f"[INFO] Updated {output_path}")
            last_target = target
        else:
            print(f"[INFO] Refreshed {output_path}")

        if args.interval <= 0:
            break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
