#!/usr/bin/env python3
"""
aggregate_slam_results.py — Aggregate per-run SLAM benchmark CSVs into slam_config_stats.csv.

Scans benchmarks/results/ for summary.csv files from all benchmark runs and
computes mean ± std across runs for each (config, sequence) pair.

Usage:
  python3 benchmarks/aggregate_slam_results.py
  # Overwrites benchmarks/results/slam_config_stats.csv and slam_baseline_summary.csv
"""

import csv
import glob
import os
import re
import statistics

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, 'results')

# Precision label for each config
PRECISION_MAP = {
    'baseline':         'float64',
    'baseline_f32':     'float32',
    'solver_tuned':     'float64',
    'solver_tuned_f32': 'float32',
    'fe_klt2':          'float64',
    'fe_maxdist65':     'float64',
    'fe_combined':      'float64',
}

def load_all_runs():
    """Return list of dicts with all valid run rows from all summary.csv files."""
    rows = []
    # Find all summary CSVs — one per benchmark run directory
    pattern = os.path.join(RESULTS_DIR, '*/summary.csv')
    for csv_path in sorted(glob.glob(pattern)):
        with open(csv_path) as f:
            for r in csv.DictReader(f):
                if r.get('ate_rmse', 'N/A') == 'N/A':
                    continue  # skip failed runs
                try:
                    float(r['ate_rmse'])
                    float(r['fe_mean_ms'])
                    float(r['ba_mean_ms'])
                    float(r['klt_mean_ms'])
                    rows.append(r)
                except (ValueError, KeyError):
                    pass
    return rows

def aggregate(rows):
    """Group by (config, sequence) and compute mean ± std."""
    groups = {}
    for r in rows:
        key = (r['config'], r['sequence'])
        if key not in groups:
            groups[key] = []
        groups[key].append(r)

    stats = []
    for (cfg, seq), runs in sorted(groups.items()):
        n = len(runs)
        ate_vals = [float(r['ate_rmse'])  for r in runs]
        fe_vals  = [float(r['fe_mean_ms']) for r in runs]
        ba_vals  = [float(r['ba_mean_ms']) for r in runs]
        klt_vals = [float(r['klt_mean_ms']) for r in runs]

        def mean(v): return sum(v) / len(v)
        def std(v):
            if len(v) < 2: return 0.0
            return statistics.stdev(v)

        stats.append({
            'config':      cfg,
            'precision':   PRECISION_MAP.get(cfg, 'float64'),
            'sequence':    seq,
            'n_runs':      n,
            'ate_mean_m':  round(mean(ate_vals), 6),
            'ate_std_m':   round(std(ate_vals), 6),
            'fe_mean_ms':  round(mean(fe_vals), 3),
            'fe_std_ms':   round(std(fe_vals), 3),
            'ba_mean_ms':  round(mean(ba_vals), 3),
            'ba_std_ms':   round(std(ba_vals), 3),
            'klt_mean_ms': round(mean(klt_vals), 3),
            'klt_std_ms':  round(std(klt_vals), 3),
        })
    return stats

def main():
    rows = load_all_runs()
    print(f"Found {len(rows)} valid runs across all summary.csv files")

    # Write slam_baseline_summary.csv (all individual runs)
    all_runs_path = os.path.join(RESULTS_DIR, 'slam_baseline_summary.csv')
    fieldnames = ['config', 'precision', 'sequence', 'run',
                  'ate_rmse_m', 'rpe_rmse',
                  'fe_mean_ms', 'ba_mean_ms', 'klt_mean_ms', 'total_frames']

    # Re-scan to get per-run data including rpe/frames
    all_run_rows = []
    pattern = os.path.join(RESULTS_DIR, '*/summary.csv')
    for csv_path in sorted(glob.glob(pattern)):
        with open(csv_path) as f:
            for r in csv.DictReader(f):
                cfg = r.get('config', '')
                entry = {
                    'config':       cfg,
                    'precision':    PRECISION_MAP.get(cfg, 'float64'),
                    'sequence':     r.get('sequence', ''),
                    'run':          r.get('run', ''),
                    'ate_rmse_m':   r.get('ate_rmse', 'N/A'),
                    'rpe_rmse':     r.get('rpe_rmse', 'N/A'),
                    'fe_mean_ms':   r.get('fe_mean_ms', 'N/A'),
                    'ba_mean_ms':   r.get('ba_mean_ms', 'N/A'),
                    'klt_mean_ms':  r.get('klt_mean_ms', 'N/A'),
                    'total_frames': r.get('total_frames', 'N/A'),
                }
                all_run_rows.append(entry)

    # Sort: config order, then sequence, then run number
    config_order = ['baseline', 'baseline_f32', 'solver_tuned', 'solver_tuned_f32',
                    'fe_klt2', 'fe_maxdist65', 'fe_combined']
    def sort_key(r):
        try: ci = config_order.index(r['config'])
        except ValueError: ci = 99
        try: ri = int(r['run'])
        except: ri = 0
        return (ci, r['sequence'], ri)
    all_run_rows.sort(key=sort_key)

    with open(all_runs_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(all_run_rows)
    print(f"Written: {all_runs_path}")

    # Write slam_config_stats.csv (aggregated)
    stats = aggregate(rows)
    stats_path = os.path.join(RESULTS_DIR, 'slam_config_stats.csv')
    stat_fields = ['config', 'precision', 'sequence', 'n_runs',
                   'ate_mean_m', 'ate_std_m',
                   'fe_mean_ms', 'fe_std_ms',
                   'ba_mean_ms', 'ba_std_ms',
                   'klt_mean_ms', 'klt_std_ms']
    with open(stats_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=stat_fields)
        w.writeheader()
        w.writerows(stats)
    print(f"Written: {stats_path}")

    print()
    print("Config Stats Summary (MH01-Easy only):")
    print(f"{'Config':<22} {'Prec':<8} {'ATE (m)':<14} {'FE (ms)':<12} {'BA (ms)':<12} {'KLT (ms)'}")
    print('-' * 85)
    for s in stats:
        if s['sequence'] != 'MH01-Easy':
            continue
        print(f"{s['config']:<22} {s['precision']:<8} "
              f"{s['ate_mean_m']:.3f}±{s['ate_std_m']:.3f}  "
              f"{s['fe_mean_ms']:.2f}±{s['fe_std_ms']:.2f}   "
              f"{s['ba_mean_ms']:.2f}±{s['ba_std_ms']:.2f}   "
              f"{s['klt_mean_ms']:.2f}±{s['klt_std_ms']:.2f}")


if __name__ == '__main__':
    main()
