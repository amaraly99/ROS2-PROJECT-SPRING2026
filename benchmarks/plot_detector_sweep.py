#!/usr/bin/env python3
"""
plot_detector_sweep.py — paper figures + table from a detector-sweep folder.

Reads every *_summary.json in a results dir and emits:
  detector_comparison.csv        one row per config (the paper table)
  fig_cmd_vel_hz.png             primary metric: /cmd_vel publish rate ± std
  fig_detector_latency.png       detector-only stage breakdown (S4/S5/S6/S7)
  fig_time_to_reached.png        mission: time-to-REACHED ± std (reached runs)

Usage:
  python3 benchmarks/plot_detector_sweep.py benchmarks/results/detector_sweep_2026-06-25
"""
import csv
import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load(d):
    rows = []
    for p in sorted(glob.glob(os.path.join(d, "*_summary.json"))):
        with open(p) as f:
            rows.append(json.load(f))
    return rows


def g(block, key, sub="mean"):
    b = block.get(key) if block else None
    if isinstance(b, dict):
        return b.get(sub)
    return b


def short(label):
    return label.replace("_fast", "").replace("_no-slam", "")


def main():
    if len(sys.argv) < 2:
        print("usage: plot_detector_sweep.py <results_dir>")
        sys.exit(2)
    d = sys.argv[1]
    rows = load(d)
    if not rows:
        print(f"no *_summary.json in {d}")
        sys.exit(1)
    rows.sort(key=lambda s: s["label"])

    # ── table ────────────────────────────────────────────────────────────────
    # reach_*_sim_s are the PRIMARY mission metrics: the wall-clock ones scale with
    # whatever rate MATLAB happened to run at (measured drift 0.78-0.95x, which alone
    # stretched identical missions from 43.7 s to 54.8 s wall). Sim-time is invariant.
    cols = ["label", "detector", "model", "backend",
            "cmd_vel_hz", "cmd_vel_std", "det_hz",
            "s4_preprocess", "s5_inference", "s6_nms", "s7_shm_write",
            "detector_compute", "det_path",
            "reached_rate",
            "time_to_reached_sim_s", "time_to_reached_sim_std",
            "approach_duration_sim_s", "sim_rate",
            "time_to_reached_s_wall", "approach_duration_s_wall",
            "n_reacquire", "frozen"]
    out_csv = os.path.join(d, "detector_comparison.csv")
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for s in rows:
            dl = s.get("detector_latency_ms", {})
            m = s.get("mission", {})
            w.writerow([
                short(s["label"]), s["detector"], s["model"], s["backend"],
                s["cmd_vel_hz"]["mean"], s["cmd_vel_hz"]["std"], s["det_hz"]["mean"],
                g(dl, "s4_preprocess"), g(dl, "s5_inference"), g(dl, "s6_nms"),
                g(dl, "s7_shm_write"), g(dl, "detector_compute"), g(dl, "det_path"),
                m.get("reached_success_rate"),
                g(m, "time_to_reached_sim_s"), g(m, "time_to_reached_sim_s", "std"),
                g(m, "approach_duration_sim_s"), g(m, "sim_rate"),
                g(m, "time_to_reached_s"), g(m, "approach_duration_s"),
                g(m, "n_reacquire"),
                s.get("frozen_camera_flag"),
            ])
    print(f"→ {out_csv}")

    labels = [short(s["label"]) for s in rows]
    x = range(len(rows))

    # ── fig 1: /cmd_vel rate ± std (primary metric) ───────────────────────────
    plt.figure(figsize=(max(8, len(rows) * 0.5), 4))
    plt.bar(x, [s["cmd_vel_hz"]["mean"] for s in rows],
            yerr=[s["cmd_vel_hz"]["std"] for s in rows], capsize=3)
    plt.ylabel("/cmd_vel publish rate (Hz)")
    plt.title("Closed-loop control rate by detector")
    plt.xticks(x, labels, rotation=90)
    plt.tight_layout()
    plt.savefig(os.path.join(d, "fig_cmd_vel_hz.png"), dpi=150)
    plt.close()

    # ── fig 2: detector-only latency breakdown (stacked S4/S5/S6/S7) ──────────
    plt.figure(figsize=(max(8, len(rows) * 0.5), 4))
    stages = [("s4_preprocess", "preprocess"), ("s5_inference", "inference"),
              ("s6_nms", "NMS"), ("s7_shm_write", "SHM write")]
    bottom = [0.0] * len(rows)
    for key, name in stages:
        vals = [g(s.get("detector_latency_ms", {}), key) or 0.0 for s in rows]
        plt.bar(x, vals, bottom=bottom, label=name)
        bottom = [b + v for b, v in zip(bottom, vals)]
    plt.ylabel("detector latency (ms)")
    plt.title("Detector-only stage latency (S4–S7)")
    plt.xticks(x, labels, rotation=90)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(d, "fig_detector_latency.png"), dpi=150)
    plt.close()

    # ── fig 3: time-to-REACHED ± std ──────────────────────────────────────────
    ttr = [g(s.get("mission", {}), "time_to_reached_s") for s in rows]
    ttr_std = [g(s.get("mission", {}), "time_to_reached_s", "std") for s in rows]
    plt.figure(figsize=(max(8, len(rows) * 0.5), 4))
    plt.bar(x, [v if v is not None else 0.0 for v in ttr],
            yerr=[e if e is not None else 0.0 for e in ttr_std], capsize=3)
    plt.ylabel("time to REACHED (s)")
    plt.title("Mission time-to-target by detector (0 = never reached)")
    plt.xticks(x, labels, rotation=90)
    plt.tight_layout()
    plt.savefig(os.path.join(d, "fig_time_to_reached.png"), dpi=150)
    plt.close()
    print(f"→ 3 figures in {d}/")


if __name__ == "__main__":
    main()
