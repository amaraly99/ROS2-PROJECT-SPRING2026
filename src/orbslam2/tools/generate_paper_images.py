#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import shutil
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RESULTS_ROOT = ROOT / "results" / "20260530_232822"
OUT_DIR = ROOT / "paper_images"

SEQUENCES = [
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


LEGACY_DATA = {
    "OV2 Accurate": {
        "color": "#2f6db3",
        "rmse": [0.040, 0.049, 0.046, 0.069, 0.067, 0.055, 0.088, 0.118, 0.072, 0.054],
        "latency": [17.083, 17.088, 19.522, 15.578, 15.815, 19.733, 21.698, 21.225, 15.033, 20.488],
        "cpu": [153.4, 149.6, 160.9, 147.0, 147.5, 157.1, 164.9, 162.0, 141.7, 163.9],
    },
    "OV2 Fast": {
        "color": "#7fb069",
        "rmse": [0.054, 0.052, 0.079, 0.152, 0.142, 0.094, 0.095, 0.670, 0.086, 0.204],
        "latency": [4.895, 5.025, 5.088, 4.990, 5.036, 5.114, 5.698, 6.105, 4.908, 5.793],
        "cpu": [42.8, 43.6, 48.4, 42.7, 42.3, 46.1, 46.8, 46.6, 42.0, 51.9],
    },
    "ORB-SLAM3": {
        "color": "#f28e2b",
        "rmse": [0.043, 0.071, 0.247, 0.039, 0.091, 0.131, 0.331, 0.069, math.nan, 0.228],
        "latency": [50.106, 48.757, 51.713, 38.323, 46.074, 32.325, 37.758, 46.427, math.nan, 36.495],
        "cpu": [139.6, 134.5, 145.5, 149.6, 143.6, 87.4, 112.2, 132.0, math.nan, 118.8],
    },
    "RTAB-Map": {
        "color": "#c44e52",
        "rmse": [0.102, 0.190, 0.146, 0.224, 0.166, 0.096, 0.081, 0.133, 0.161, 0.244],
        "rmse_std": [0.063, 0.143, 0.038, 0.070, 0.0, 0.029, 0.0, 0.046, 0.052, 0.035],
        "latency": [101.7, 104.4, 97.2, 104.2, 88.3, 103.6, 93.0, 92.8, 100.3, 90.4],
        "cpu": [132.3, 134.6, 125.3, 130.0, 131.2, 131.1, 120.1, 120.3, 128.2, 123.0],
    },
}


def ensure_out_dir() -> None:
    OUT_DIR.mkdir(exist_ok=True)


def load_orbslam2_rows() -> list[dict[str, str]]:
    summary_csv = RESULTS_ROOT / "experiment_summary.csv"
    with summary_csv.open() as f:
        rows = list(csv.DictReader(f))
    rows.sort(key=lambda row: SEQUENCES.index(row["sequence"]))
    return rows


def load_orbslam2_series(rows: list[dict[str, str]]) -> dict[str, list[float]]:
    latency_std = []
    cpu_std = []
    for row in rows:
        summary_path = RESULTS_ROOT / row["sequence"] / "run_summary.json"
        payload = json.loads(summary_path.read_text())
        clean_runs = [run for run in payload["runs"] if not run.get("needs_rerun")]
        pool = clean_runs or payload["runs"]
        latency_std.append(float(np.std([run["frontend_full_tracking_avg_ms"] for run in pool])))
        cpu_std.append(float(np.std([run["cpu_total_avg_pct"] for run in pool])))
    return {
        "rmse": [float(row["rmse_avg_clean"]) for row in rows],
        "rmse_std": [float(row["rmse_std_clean"]) for row in rows],
        "latency": [float(row["frontend_full_tracking_avg_clean_ms"]) for row in rows],
        "latency_std": latency_std,
        "cpu": [float(row["cpu_total_avg_pct"]) for row in rows],
        "cpu_std": cpu_std,
    }


def load_orbslam2_thread_means() -> tuple[list[str], list[float], list[float]]:
    roles: dict[str, list[float]] = defaultdict(list)
    for seq_dir in sorted(RESULTS_ROOT.iterdir()):
        if not seq_dir.is_dir():
            continue
        for run_dir in sorted(seq_dir.glob("run_*")):
            if not run_dir.is_dir():
                continue
            summary_csv = run_dir / "orbslam_thread_cpu_summary.csv"
            if not summary_csv.exists():
                continue
            with summary_csv.open() as f:
                for row in csv.DictReader(f):
                    comm = row["comm"].strip()
                    if not comm.startswith("ORB"):
                        continue
                    roles[comm].append(float(row["mean_cpu_percent"]))
    order = ["ORBGBA", "ORBLocalMap", "ORBFrontEnd", "ORBLoopClose", "ORBExtractL", "ORBExtractR"]
    labels = []
    means = []
    stds = []
    for role in order:
        vals = roles.get(role)
        if not vals:
            continue
        labels.append(role)
        means.append(float(np.mean(vals)))
        stds.append(float(np.std(vals)))
    return labels, means, stds


def copy_best_trajectory_examples() -> list[tuple[str, str]]:
    copied = []
    for seq_dir in sorted(RESULTS_ROOT.iterdir()):
        if not seq_dir.is_dir():
            continue
        summary_path = seq_dir / "run_summary.json"
        if not summary_path.exists():
            continue
        payload = json.loads(summary_path.read_text())
        clean_runs = [run for run in payload["runs"] if not run.get("needs_rerun")]
        pool = clean_runs or payload["runs"]
        best = min(pool, key=lambda run: run["rmse"])
        src = seq_dir / best["artifacts"]["traj_xy_png"]
        dst = OUT_DIR / f"orbslam2_{seq_dir.name}_trajectory_xy.png"
        shutil.copyfile(src, dst)
        copied.append((seq_dir.name, dst.name))
    return copied


def save_figure(fig: plt.Figure, name: str) -> None:
    fig.tight_layout()
    fig.savefig(OUT_DIR / name, dpi=220, bbox_inches="tight")
    plt.close(fig)


def grouped_bars(
    filename: str,
    ylabel: str,
    title: str,
    data_map: dict[str, dict[str, list[float]]],
    key: str,
    std_keys: dict[str, str] | None = None,
    yline_values: list[tuple[float, str, str]] | None = None,
    ylim: tuple[float, float] | None = None,
) -> None:
    labels = list(data_map.keys())
    width = 0.16
    x = np.arange(len(SEQUENCES))
    fig, ax = plt.subplots(figsize=(16, 6))
    offsets = np.linspace(-2, 2, len(labels)) * width
    for offset, label in zip(offsets, labels):
        values = np.array(data_map[label][key], dtype=float)
        stds = None
        if std_keys and label in std_keys:
            stds = np.array(data_map[label][std_keys[label]], dtype=float)
        mask = ~np.isnan(values)
        ax.bar(
            x[mask] + offset,
            values[mask],
            width=width,
            label=label,
            color=data_map[label]["color"],
            alpha=0.9,
            yerr=stds[mask] if stds is not None else None,
            capsize=3 if stds is not None else 0,
            edgecolor="black",
            linewidth=0.4,
        )
    if yline_values:
        for value, text, style in yline_values:
            ax.axhline(value, color=style, linestyle="--", linewidth=1.0)
            ax.text(len(SEQUENCES) - 0.25, value + 0.01 * (ylim[1] if ylim else 1.0), text, color=style, ha="right", va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(SEQUENCES, rotation=35, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if ylim:
        ax.set_ylim(*ylim)
    ax.grid(axis="y", alpha=0.25)
    ax.legend(ncol=3, fontsize=9)
    save_figure(fig, filename)


def orbslam2_single_series_plot(rows: list[dict[str, str]], key: str, std_key: str, title: str, ylabel: str, filename: str, color: str, ylim: tuple[float, float] | None = None, ylines: list[tuple[float, str, str]] | None = None) -> None:
    x = np.arange(len(SEQUENCES))
    values = np.array([float(row[key]) for row in rows], dtype=float)
    stds = np.array([float(row[std_key]) for row in rows], dtype=float)
    fig, ax = plt.subplots(figsize=(13, 5))
    ax.bar(x, values, color=color, edgecolor="black", linewidth=0.4, yerr=stds, capsize=4)
    if ylines:
        for value, text, style in ylines:
            ax.axhline(value, color=style, linestyle="--", linewidth=1.0)
            ymax = ylim[1] if ylim else max(values) * 1.1
            ax.text(len(SEQUENCES) - 0.25, value + 0.01 * ymax, text, color=style, ha="right", va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(SEQUENCES, rotation=35, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if ylim:
        ax.set_ylim(*ylim)
    ax.grid(axis="y", alpha=0.25)
    save_figure(fig, filename)


def orbslam2_thread_plot() -> None:
    labels, means, stds = load_orbslam2_thread_means()
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar(x, means, yerr=stds, capsize=4, color="#7a5195", edgecolor="black", linewidth=0.4)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=25, ha="right")
    ax.set_ylabel("CPU usage (% of one core)")
    ax.set_title("ORB-SLAM2 logical thread-role CPU usage")
    ax.set_ylim(0, 400)
    ax.axhline(100, color="grey", linestyle="--", linewidth=1.0)
    ax.axhline(200, color="grey", linestyle="--", linewidth=1.0)
    ax.grid(axis="y", alpha=0.25)
    save_figure(fig, "orbslam2_cpu_per_thread.png")


def overall_average_plot(data_map: dict[str, dict[str, list[float]]]) -> None:
    labels = list(data_map.keys())
    rmse_means = [float(np.nanmean(data_map[label]["rmse"])) for label in labels]
    lat_means = [float(np.nanmean(data_map[label]["latency"])) for label in labels]
    cpu_means = [float(np.nanmean(data_map[label]["cpu"])) for label in labels]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    metrics = [
        ("Average RMSE (m)", rmse_means, (0, 0.20)),
        ("Average frontend time (ms)", lat_means, (0, 110)),
        ("Average CPU (% of one core)", cpu_means, (0, 220)),
    ]
    for ax, (title, values, ylim) in zip(axes, metrics):
        colors = [data_map[label]["color"] for label in labels]
        ax.bar(np.arange(len(labels)), values, color=colors, edgecolor="black", linewidth=0.4)
        ax.set_title(title)
        ax.set_xticks(np.arange(len(labels)))
        ax.set_xticklabels(labels, rotation=28, ha="right")
        ax.set_ylim(*ylim)
        ax.grid(axis="y", alpha=0.25)
    save_figure(fig, "comparison_overall_means.png")


def write_manifest(copied_trajs: list[tuple[str, str]], rows: list[dict[str, str]]) -> None:
    manifest = {
        "source_results_root": str(RESULTS_ROOT),
        "orbslam2_sequences": [
            {
                "sequence": row["sequence"],
                "rmse_avg_clean": float(row["rmse_avg_clean"]),
                "frontend_full_tracking_avg_clean_ms": float(row["frontend_full_tracking_avg_clean_ms"]),
                "cpu_total_avg_pct": float(row["cpu_total_avg_pct"]),
            }
            for row in rows
        ],
        "copied_trajectory_examples": [{"sequence": seq, "file": file} for seq, file in copied_trajs],
    }
    (OUT_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2))


def main() -> None:
    ensure_out_dir()
    rows = load_orbslam2_rows()
    orbslam2 = load_orbslam2_series(rows)
    for row, latency_std, cpu_std in zip(rows, orbslam2["latency_std"], orbslam2["cpu_std"]):
        row["orbslam2_latency_std_ms"] = str(latency_std)
        row["orbslam2_cpu_std_pct"] = str(cpu_std)

    data_map = {
        "OV2 Accurate": LEGACY_DATA["OV2 Accurate"],
        "OV2 Fast": LEGACY_DATA["OV2 Fast"],
        "ORB-SLAM3": LEGACY_DATA["ORB-SLAM3"],
        "ORB-SLAM2": {
            "color": "#6a4c93",
            **orbslam2,
        },
        "RTAB-Map": LEGACY_DATA["RTAB-Map"],
    }

    orbslam2_single_series_plot(
        rows,
        key="rmse_avg_clean",
        std_key="rmse_std_clean",
        title="ORB-SLAM2 mean APE RMSE per sequence",
        ylabel="RMSE (m)",
        filename="orbslam2_rmse_per_sequence.png",
        color="#6a4c93",
        ylim=(0, 0.16),
        ylines=[(0.05, "5 cm", "grey"), (0.10, "10 cm", "grey")],
    )
    orbslam2_single_series_plot(
        rows,
        key="frontend_full_tracking_avg_clean_ms",
        std_key="orbslam2_latency_std_ms",
        title="ORB-SLAM2 frontend full-tracking time per sequence",
        ylabel="Frontend time (ms)",
        filename="orbslam2_frontend_timing_per_sequence.png",
        color="#6a4c93",
        ylim=(0, 70),
        ylines=[(50.0, "20 Hz budget", "firebrick")],
    )
    orbslam2_single_series_plot(
        rows,
        key="cpu_total_avg_pct",
        std_key="orbslam2_cpu_std_pct",
        title="ORB-SLAM2 total CPU usage per sequence",
        ylabel="CPU usage (% of one core)",
        filename="orbslam2_cpu_per_sequence.png",
        color="#6a4c93",
        ylim=(0, 260),
        ylines=[(100.0, "1 core", "grey"), (200.0, "2 cores", "grey")],
    )
    orbslam2_thread_plot()

    grouped_bars(
        filename="comparison_rmse_all_algorithms.png",
        ylabel="RMSE (m)",
        title="Per-sequence RMSE comparison across all algorithms",
        data_map=data_map,
        key="rmse",
        std_keys={"ORB-SLAM2": "rmse_std", "RTAB-Map": "rmse_std"},
        yline_values=[(0.05, "5 cm", "grey"), (0.10, "10 cm", "grey")],
        ylim=(0, 0.72),
    )
    grouped_bars(
        filename="comparison_latency_all_algorithms.png",
        ylabel="Per-frame time (ms)",
        title="Per-sequence frontend / odometry comparison across all algorithms",
        data_map=data_map,
        key="latency",
        std_keys={"ORB-SLAM2": "latency_std"},
        yline_values=[(50.0, "20 Hz budget", "firebrick"), (100.0, "10 Hz", "grey")],
        ylim=(0, 120),
    )
    grouped_bars(
        filename="comparison_cpu_all_algorithms.png",
        ylabel="CPU usage (% of one core)",
        title="Per-sequence CPU comparison across all algorithms",
        data_map=data_map,
        key="cpu",
        std_keys={"ORB-SLAM2": "cpu_std"},
        yline_values=[(100.0, "1 core", "grey"), (200.0, "2 cores", "grey")],
        ylim=(0, 240),
    )
    overall_average_plot(data_map)

    copied = copy_best_trajectory_examples()
    write_manifest(copied, rows)


if __name__ == "__main__":
    main()
