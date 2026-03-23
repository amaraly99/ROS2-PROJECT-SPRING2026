#!/usr/bin/env python3
"""
Plot YOLO resolution tradeoff results.

Usage:
    python3 benchmarks/plot_resolution.py benchmarks/results/res_640x480_*.csv \
        benchmarks/results/res_416x312_*.csv benchmarks/results/res_320x240_*.csv

Generates: benchmarks/results/resolution_tradeoff.png
"""

import csv
import sys
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_csv(path):
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    # Skip 10-frame warmup
    return rows[10:]


def get_label(path):
    """Extract resolution label from filename like res_640x480_..."""
    base = os.path.basename(path)
    for res in ["640x480", "416x312", "320x240"]:
        if res in base:
            return res
    return base


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_resolution.py <csv1> [csv2] [csv3]")
        sys.exit(1)

    csv_paths = sys.argv[1:]
    out_dir = os.path.dirname(csv_paths[0]) or "."

    datasets = []
    for path in csv_paths:
        rows = load_csv(path)
        label = get_label(path)
        n_dets = [float(r["n_dets"]) for r in rows]
        inference = [float(r["inference_ms"]) for r in rows]
        preprocess = [float(r["preprocess_ms"]) for r in rows]
        total_pp = [float(r["total_pp_ms"]) for r in rows]
        total = [p + i + pp for p, i, pp in zip(preprocess, inference, total_pp)]
        datasets.append({
            "label": label,
            "n_dets": n_dets,
            "inference_ms": inference,
            "preprocess_ms": preprocess,
            "total_pp_ms": total_pp,
            "total_ms": total,
        })

    # Sort by resolution (descending)
    res_order = {"640x480": 0, "416x312": 1, "320x240": 2}
    datasets.sort(key=lambda d: res_order.get(d["label"], 99))

    labels = [d["label"] for d in datasets]
    colors = ["#2196F3", "#FF9800", "#F44336"]

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    fig.suptitle("YOLO Detection vs Effective Resolution\n"
                 "(HEF fixed at 640×640, NPU inference time constant)",
                 fontsize=12, fontweight="bold")

    # ── Plot 1: Detection count bar chart ─────────────────────
    ax = axes[0]
    means = [np.mean(d["n_dets"]) for d in datasets]
    stds = [np.std(d["n_dets"]) for d in datasets]
    bars = ax.bar(labels, means, yerr=stds, color=colors[:len(labels)],
                  capsize=5, edgecolor="black", linewidth=0.5)
    ax.set_ylabel("Avg Detections / Frame")
    ax.set_xlabel("Effective Resolution")
    ax.set_title("Detection Yield")
    for bar, m in zip(bars, means):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                f"{m:.1f}", ha="center", va="bottom", fontsize=9)

    # ── Plot 2: Detection count boxplot ───────────────────────
    ax = axes[1]
    bp = ax.boxplot([d["n_dets"] for d in datasets], labels=labels,
                    patch_artist=True, widths=0.5)
    for patch, color in zip(bp["boxes"], colors[:len(labels)]):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
    ax.set_ylabel("Detections / Frame")
    ax.set_xlabel("Effective Resolution")
    ax.set_title("Detection Distribution")

    # ── Plot 3: Stacked timing bar ────────────────────────────
    ax = axes[2]
    pre_means = [np.mean(d["preprocess_ms"]) for d in datasets]
    inf_means = [np.mean(d["inference_ms"]) for d in datasets]
    pp_means = [np.mean(d["total_pp_ms"]) for d in datasets]

    x = np.arange(len(labels))
    w = 0.5
    ax.bar(x, pre_means, w, label="Preprocess", color="#4CAF50")
    ax.bar(x, inf_means, w, bottom=pre_means, label="NPU Inference", color="#2196F3")
    ax.bar(x, pp_means, w,
           bottom=[p + i for p, i in zip(pre_means, inf_means)],
           label="CPU Postprocess", color="#FF9800")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Time (ms)")
    ax.set_xlabel("Effective Resolution")
    ax.set_title("Pipeline Timing Breakdown")
    ax.legend(fontsize=8, loc="upper right")

    plt.tight_layout()
    out_path = os.path.join(out_dir, "resolution_tradeoff.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Plot saved to: {out_path}")


if __name__ == "__main__":
    main()
