#!/usr/bin/env python3
"""
plot_ipc_cdf.py — Plot CDF of IPC latency measurements.

Reads one or more CSV files produced by ipc_latency_bench.py and
plots the empirical CDF for each mode on the same axes.

Usage:
  python3 benchmarks/plot_ipc_cdf.py benchmarks/results/ipc_latency_shm_*.csv \
                                      benchmarks/results/ipc_latency_dds_*.csv
  python3 benchmarks/plot_ipc_cdf.py benchmarks/results/ipc_latency_combined.csv
"""

import argparse
import csv
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description='Plot IPC latency CDF')
parser.add_argument('csv_files', nargs='+', help='CSV files from ipc_latency_bench.py')
parser.add_argument('--out', default='benchmarks/results/ipc_latency_cdf.png')
args = parser.parse_args()

# Load data: group by mode
data = defaultdict(list)
for path in args.csv_files:
    if not os.path.exists(path):
        print(f"WARNING: {path} not found, skipping")
        continue
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            mode = row['mode']
            lat  = float(row['latency_us'])
            data[mode].append(lat)

if not data:
    print("ERROR: no data loaded", file=sys.stderr)
    sys.exit(1)

# Colors and labels
style = {
    'shm': {'color': '#e41a1c', 'label': 'POSIX SHM seqlock'},
    'dds': {'color': '#377eb8', 'label': 'ROS2 DDS (CycloneDDS)'},
}

fig, ax = plt.subplots(figsize=(8, 5))

for mode, lats in sorted(data.items()):
    lats_sorted = np.sort(lats)
    cdf = np.arange(1, len(lats_sorted) + 1) / len(lats_sorted)
    s = style.get(mode, {'color': 'gray', 'label': mode})
    ax.plot(lats_sorted, cdf * 100, color=s['color'], linewidth=2, label=s['label'])

    p50 = np.percentile(lats_sorted, 50)
    p99 = np.percentile(lats_sorted, 99)
    ax.axvline(p50, color=s['color'], linestyle='--', alpha=0.5, linewidth=1)
    print(f"{mode}: n={len(lats)}, mean={np.mean(lats):.1f}µs, "
          f"P50={p50:.1f}µs, P95={np.percentile(lats,95):.1f}µs, P99={p99:.1f}µs")

ax.set_xlabel('Latency (µs) — log scale', fontsize=12)
ax.set_ylabel('CDF (%)', fontsize=12)
ax.set_title('IPC Delivery Latency: SHM Seqlock vs ROS2 DDS\n'
             f'Message size: 921,600 B (640×480 RGB frame) — RPi5 Cortex-A76',
             fontsize=11)
ax.legend(fontsize=11)
ax.set_xscale('log')
ax.set_ylim(0, 101)
ax.grid(True, alpha=0.3, which='both')
ax.axhline(50, color='gray', linestyle=':', alpha=0.4)
ax.axhline(95, color='gray', linestyle=':', alpha=0.4)
ax.axhline(99, color='gray', linestyle=':', alpha=0.4)

os.makedirs(os.path.dirname(args.out), exist_ok=True)
plt.tight_layout()
plt.savefig(args.out, dpi=150)
print(f"Plot saved to {args.out}")
