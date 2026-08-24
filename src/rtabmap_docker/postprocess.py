#!/usr/bin/env python3

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path
from statistics import mean, pstdev

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _detect_storage_id(bag_dir: Path) -> str:
    metadata = bag_dir / "metadata.yaml"
    if metadata.exists():
        text = metadata.read_text()
        if "storage_identifier: mcap" in text:
            return "mcap"
        if "storage_identifier: sqlite3" in text:
            return "sqlite3"
    return "mcap"


def _open_reader(bag_dir: Path) -> rosbag2_py.SequentialReader:
    storage_id = _detect_storage_id(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _write_metric_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _summary_rows(metric_values: dict[str, list[float]]) -> list[dict]:
    rows = []
    for metric, values in sorted(metric_values.items()):
        if not values:
            continue
        rows.append(
            {
                "metric": metric,
                "avg_ms": f"{mean(values):.3f}",
                "std_ms": f"{pstdev(values):.3f}",
                "min_ms": f"{min(values):.3f}",
                "max_ms": f"{max(values):.3f}",
                "n_samples": len(values),
            }
        )
    return rows


def run_timing_postprocess(run_dir: Path) -> int:
    bag_dir = run_dir / "timing_bag"
    if not bag_dir.exists():
        print(f"timing: bag directory missing: {bag_dir}")
        return 1

    reader = _open_reader(bag_dir)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    odom_rows: list[dict] = []
    slam_rows: list[dict] = []
    summary_values: dict[str, list[float]] = defaultdict(list)

    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(topic_types[topic]))

        if topic == "/odom_info":
            stamp = _stamp_to_float(msg.header.stamp)
            values = {
                "odom/time_estimation": float(msg.time_estimation) * 1000.0,
                "odom/local_bundle": float(msg.local_bundle_time) * 1000.0,
            }
            for metric, value_ms in values.items():
                odom_rows.append(
                    {"stamp": f"{stamp:.9f}", "metric": metric, "value_ms": f"{value_ms:.3f}"}
                )
                summary_values[metric].append(value_ms)

        elif topic == "/rtabmap/info":
            stamp = _stamp_to_float(msg.header.stamp)
            for key, value in zip(msg.stats_keys, msg.stats_values):
                if not key.startswith("Timing/"):
                    continue
                metric = f"slam/{key[7:].replace(' ', '_')}"
                value_ms = float(value)
                slam_rows.append(
                    {"stamp": f"{stamp:.9f}", "metric": metric, "value_ms": f"{value_ms:.3f}"}
                )
                summary_values[metric].append(value_ms)

    odom_path = run_dir / "odom_timing.csv"
    slam_path = run_dir / "slam_timing.csv"
    summary_path = run_dir / "timing_summary.csv"

    _write_metric_csv(odom_path, odom_rows, ["stamp", "metric", "value_ms"])
    _write_metric_csv(slam_path, slam_rows, ["stamp", "metric", "value_ms"])
    _write_metric_csv(
        summary_path,
        _summary_rows(summary_values),
        ["metric", "avg_ms", "std_ms", "min_ms", "max_ms", "n_samples"],
    )

    odom_frames = len({row["stamp"] for row in odom_rows})
    slam_nodes = len({row["stamp"] for row in slam_rows})

    print(f"timing: wrote {odom_path.name} ({odom_frames} frames)")
    print(f"timing: wrote {slam_path.name} ({slam_nodes} nodes)")
    print(f"timing: wrote {summary_path.name} ({len(summary_values)} metrics)")
    return 0


def _load_cpu_csv(path: Path) -> tuple[list[float], dict[str, list[float]]]:
    totals_by_sample: dict[float, float] = defaultdict(float)
    thread_values: dict[str, list[float]] = defaultdict(list)

    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                wall_ts = float(row["wall_ts"])
                cpu_pct = float(row["cpu_pct"])
            except (KeyError, ValueError):
                continue
            totals_by_sample[wall_ts] += cpu_pct
            label = f"{row.get('process_name', 'unknown')}:{row.get('thread_name', 'unknown')}"
            thread_values[label].append(cpu_pct)

    return list(totals_by_sample.values()), thread_values


def _save_bar_chart(labels: list[str], values: list[float], path: Path, title: str, ylabel: str) -> None:
    plt.figure(figsize=(max(8, len(labels) * 0.6), 5))
    plt.bar(range(len(labels)), values, color="#3a86ff")
    plt.xticks(range(len(labels)), labels, rotation=45, ha="right")
    plt.title(title)
    plt.ylabel(ylabel)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def aggregate_cpu(results_root: Path, cpu_csvs: list[Path]) -> int:
    seq_totals: dict[str, list[float]] = defaultdict(list)
    thread_totals: dict[str, list[float]] = defaultdict(list)

    for cpu_csv in cpu_csvs:
        if not cpu_csv.exists():
            continue
        totals, threads = _load_cpu_csv(cpu_csv)
        seq_name = cpu_csv.parent.parent.name
        seq_totals[seq_name].extend(totals)
        for label, values in threads.items():
            thread_totals[label].extend(values)

    if not seq_totals:
        print("cpu: no CSV data found")
        return 1

    seq_labels = sorted(seq_totals)
    seq_values = [mean(seq_totals[label]) if seq_totals[label] else math.nan for label in seq_labels]

    thread_items = sorted(
        ((label, mean(values)) for label, values in thread_totals.items() if values),
        key=lambda item: item[1],
        reverse=True,
    )[:20]
    thread_labels = [label for label, _ in thread_items]
    thread_values = [value for _, value in thread_items]

    seq_chart = results_root / "cpu_per_sequence.png"
    thread_chart = results_root / "cpu_per_thread.png"

    _save_bar_chart(seq_labels, seq_values, seq_chart, "Average CPU per Sequence", "CPU %")
    _save_bar_chart(thread_labels, thread_values, thread_chart, "Average CPU per Thread", "CPU %")

    print(f"CPU chart saved → {seq_chart}")
    print(f"CPU chart saved → {thread_chart}")
    return 0


def parse_args():
    p = argparse.ArgumentParser(description="Benchmark post-processing helpers.")
    p.add_argument("--run-dir", type=Path, default=None, help="Process one run directory.")
    p.add_argument("--aggregate-cpu", action="store_true", help="Build aggregate CPU charts.")
    p.add_argument("--results-root", type=Path, default=None, help="Root results directory for charts.")
    p.add_argument("--cpu-csvs", type=Path, nargs="*", default=None, help="CPU CSV files to aggregate.")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if args.run_dir:
        return run_timing_postprocess(args.run_dir)

    if args.aggregate_cpu:
        if not args.results_root or not args.cpu_csvs:
            print("cpu: --results-root and --cpu-csvs are required", file=sys.stderr)
            return 2
        return aggregate_cpu(args.results_root, args.cpu_csvs)

    print("Nothing to do. Use --run-dir or --aggregate-cpu.", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
