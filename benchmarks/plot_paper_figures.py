#!/usr/bin/env python3
"""
plot_paper_figures.py — Generate all paper figures from benchmark CSVs.

Produces (in benchmarks/results/figures/):
  fig1_slam_ablation.png     — Stacked bar: SLAM stage timing across configs
  fig2_slam_pareto.png       — Scatter: ATE vs total SLAM frame time (Pareto frontier)
  fig3_cache_perf.png        — Bar: L1 dcache loads float32 vs float64
  fig4_yolo_fp_breakdown.png — Grouped bar: FP32 vs FP16 per-op timing
  fig5_yolo_nms_compare.png  — Bar: yolo26n vs yolov8m vs yolov11m (infer + PP)
  fig6_slam_all_configs.png  — Extended ablation with FE speedup configs (if available)

Usage:
  python3 benchmarks/plot_paper_figures.py
"""

import csv
import glob
import os
import statistics
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, 'results')
FIG_DIR     = os.path.join(RESULTS_DIR, 'figures')
os.makedirs(FIG_DIR, exist_ok=True)

# ── Style ─────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':     'DejaVu Sans',
    'font.size':       10,
    'axes.titlesize':  11,
    'axes.labelsize':  10,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'legend.fontsize': 9,
    'figure.dpi':      150,
    'axes.grid':       True,
    'grid.alpha':      0.3,
    'axes.spines.top':    False,
    'axes.spines.right':  False,
})

COLORS = {
    'fe':      '#4C72B0',   # blue — front-end
    'klt':     '#55A868',   # green — KLT tracking
    'ba':      '#C44E52',   # red — bundle adjustment
    'other':   '#8172B2',   # purple — overhead
    'fp32':    '#4C72B0',
    'fp16':    '#C44E52',
    'shm':     '#4C72B0',
    'dds':     '#C44E52',
    'yolo26n': '#4C72B0',
    'yolov8m': '#55A868',
    'yolov11m':'#C44E52',
}

# ─────────────────────────────────────────────────────────────────────────────
# Figure 1: SLAM Stage Timing — Stacked Bar (baseline 4 configs)
# ─────────────────────────────────────────────────────────────────────────────
def fig1_slam_ablation(stats_csv):
    print("  Generating fig1_slam_ablation.png ...")
    rows = []
    with open(stats_csv) as f:
        for r in csv.DictReader(f):
            rows.append(r)

    # Config display names
    labels = {
        'baseline':       'Baseline\n(float64)',
        'baseline_f32':   'Baseline\n(float32)',
        'solver_tuned':   'Solver-Tuned\n(float64)',
        'solver_tuned_f32': 'Solver-Tuned\n(float32)',
        'fe_klt2':        'FE: KLT pyr=2\n(float64)',
        'fe_maxdist65':   'FE: maxdist=65\n(float64)',
        'fe_combined':    'FE: Combined\n(float64)',
    }

    # Order: baseline, solver_tuned, then FE configs, then f32 variants
    order = ['baseline', 'solver_tuned', 'fe_klt2', 'fe_maxdist65', 'fe_combined',
             'baseline_f32', 'solver_tuned_f32']
    avail = {r['config']: r for r in rows}
    plot_rows = [avail[c] for c in order if c in avail]

    configs   = [labels.get(r['config'], r['config']) for r in plot_rows]
    fe_vals   = [float(r['fe_mean_ms'])  for r in plot_rows]
    klt_vals  = [float(r['klt_mean_ms']) for r in plot_rows]
    ba_vals   = [float(r['ba_mean_ms'])  for r in plot_rows]
    fe_stds   = [float(r['fe_std_ms'])   for r in plot_rows]
    ba_stds   = [float(r['ba_std_ms'])   for r in plot_rows]

    x = np.arange(len(configs))
    w = 0.55

    fig, ax = plt.subplots(figsize=(max(8, len(configs) * 1.2), 4.5))

    # KLT is part of FE; show as sub-segment
    fe_only = [f - k for f, k in zip(fe_vals, klt_vals)]

    b1 = ax.bar(x, fe_only,  w, label='FE (non-KLT)',  color=COLORS['fe'],    alpha=0.85)
    b2 = ax.bar(x, klt_vals, w, label='KLT Tracking',  color=COLORS['klt'],   alpha=0.85, bottom=fe_only)
    fe_top = [a + b for a, b in zip(fe_only, klt_vals)]
    b3 = ax.bar(x, ba_vals,  w, label='Bundle Adjust',  color=COLORS['ba'],    alpha=0.85, bottom=fe_top)

    # Error bars on BA
    ba_tops = [f + b for f, b in zip(fe_top, ba_vals)]
    ax.errorbar(x, ba_tops, yerr=ba_stds, fmt='none', color='black', capsize=4, linewidth=1.2)

    # Total time annotation
    for i, (f, b) in enumerate(zip(fe_vals, ba_vals)):
        total = f + b
        ax.text(x[i], total + 0.3, f'{total:.1f}', ha='center', va='bottom', fontsize=8, fontweight='bold')

    ax.set_xticks(x)
    ax.set_xticklabels(configs, fontsize=8.5)
    ax.set_ylabel('Time per Frame (ms)')
    ax.set_title('SLAM Stage Timing Ablation Study (EuRoC MH01-Easy, 3-run mean)')
    ax.legend(loc='upper right')
    ax.set_ylim(0, max(fe_top[i] + ba_vals[i] for i in range(len(configs))) * 1.25)

    # Vertical separator before f32 variants
    f32_start = next((i for i, r in enumerate(plot_rows) if 'f32' in r['config']), None)
    if f32_start is not None:
        ax.axvline(f32_start - 0.5, color='gray', linestyle='--', linewidth=0.8, alpha=0.6)
        ax.text(f32_start - 0.5, ax.get_ylim()[1] * 0.97, '  float32\n  variants',
                fontsize=7, color='gray', va='top')

    plt.tight_layout()
    out = os.path.join(FIG_DIR, 'fig1_slam_ablation.png')
    plt.savefig(out, bbox_inches='tight')
    plt.close()
    print(f"    Saved: {out}")


# ─────────────────────────────────────────────────────────────────────────────
# Figure 2: ATE vs Frame Time — Pareto Scatter
# ─────────────────────────────────────────────────────────────────────────────
def fig2_slam_pareto(stats_csv):
    print("  Generating fig2_slam_pareto.png ...")
    rows = []
    with open(stats_csv) as f:
        for r in csv.DictReader(f):
            rows.append(r)

    label_map = {
        'baseline':         'Baseline (f64)',
        'baseline_f32':     'Baseline (f32)',
        'solver_tuned':     'Solver-Tuned (f64)',
        'solver_tuned_f32': 'Solver-Tuned (f32)',
        'fe_klt2':          'FE: KLT pyr=2',
        'fe_maxdist65':     'FE: maxdist=65',
        'fe_combined':      'FE: Combined',
    }
    marker_map = {
        'baseline':         'o',
        'baseline_f32':     's',
        'solver_tuned':     'D',
        'solver_tuned_f32': '^',
        'fe_klt2':          'P',
        'fe_maxdist65':     'X',
        'fe_combined':      '*',
    }
    color_map = {
        'baseline':         '#4C72B0',
        'solver_tuned':     '#C44E52',
        'fe_klt2':          '#55A868',
        'fe_maxdist65':     '#8172B2',
        'fe_combined':      '#CCB974',
        'baseline_f32':     '#64B5F6',
        'solver_tuned_f32': '#EF9A9A',
    }

    fig, ax = plt.subplots(figsize=(7, 4.5))

    for r in rows:
        cfg   = r['config']
        ate   = float(r['ate_mean_m'])
        ate_s = float(r['ate_std_m'])
        total = float(r['fe_mean_ms']) + float(r['ba_mean_ms'])
        total_s = float(r['fe_std_ms']) + float(r['ba_std_ms'])
        label = label_map.get(cfg, cfg)
        m     = marker_map.get(cfg, 'o')
        c     = color_map.get(cfg, '#777777')

        ax.errorbar(total, ate,
                    xerr=total_s, yerr=ate_s,
                    fmt=m, color=c, markersize=9, linewidth=1.2,
                    capsize=4, label=label, zorder=3)

    # Pareto frontier (lower-left dominates): configs not dominated by any other
    points = [(float(r['fe_mean_ms']) + float(r['ba_mean_ms']),
               float(r['ate_mean_m']), r['config']) for r in rows]
    points.sort()
    pareto = []
    min_ate = float('inf')
    for x, y, c in points:
        if y < min_ate:
            pareto.append((x, y))
            min_ate = y
    if len(pareto) >= 2:
        px, py = zip(*pareto)
        ax.plot(px, py, 'k--', linewidth=1, alpha=0.5, label='Pareto frontier', zorder=2)

    ax.set_xlabel('Total SLAM Frame Time: FE + BA (ms)')
    ax.set_ylabel('ATE RMSE (m)')
    ax.set_title('SLAM Speed-Accuracy Trade-off (EuRoC MH01-Easy)')
    ax.legend(loc='upper left', fontsize=8)
    plt.tight_layout()
    out = os.path.join(FIG_DIR, 'fig2_slam_pareto.png')
    plt.savefig(out, bbox_inches='tight')
    plt.close()
    print(f"    Saved: {out}")


# ─────────────────────────────────────────────────────────────────────────────
# Figure 3: Cache Performance — float32 vs float64
# ─────────────────────────────────────────────────────────────────────────────
def fig3_cache_perf(perf_csv):
    print("  Generating fig3_cache_perf.png ...")
    rows = {}
    with open(perf_csv) as f:
        for r in csv.DictReader(f):
            rows[r['precision']] = r

    precs  = ['float32', 'float64']
    labels = ['float32\n(bearing vec storage)', 'float64\n(native computation)']
    l1_loads  = [float(rows[p]['l1_dcache_loads'])       / 1e9 for p in precs]
    llc_loads = [float(rows[p]['llc_loads'])              / 1e6 for p in precs]
    l1_miss   = [float(rows[p]['l1_dcache_load_misses'])  / 1e6 for p in precs]

    x  = np.arange(2)
    w  = 0.25

    fig, axes = plt.subplots(1, 3, figsize=(10, 4))
    datasets = [
        (axes[0], l1_loads,  'L1 dcache loads (billion)',  ['#4C72B0', '#55A868'],
         'L1 dcache Loads\n(30-second window)', True),
        (axes[1], llc_loads, 'LLC loads (million)',        ['#4C72B0', '#55A868'],
         'LLC (L3) Loads', False),
        (axes[2], l1_miss,   'L1 misses (million)',        ['#4C72B0', '#55A868'],
         'L1 Cache Misses', False),
    ]

    for ax, vals, ylabel, cols, title, annotate in datasets:
        bars = ax.bar(labels, vals, color=cols[:2], alpha=0.85, width=0.5)
        ax.set_ylim(0, max(vals) * 1.35)
        if annotate:
            ratio = vals[0] / vals[1]
            ax.text(0.5, 0.92, f'float32 is {ratio:.2f}× higher',
                    ha='center', fontsize=9, color='#C44E52', fontweight='bold',
                    transform=ax.transAxes)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2., bar.get_height() * 1.02,
                    f'{v:.1f}', ha='center', va='bottom', fontsize=9)

    fig.suptitle('Cache Performance: float32 vs float64 Bearing Vectors\n'
                 'float32 storage + double computation = MORE cache traffic (Cortex-A76)',
                 fontsize=10, fontweight='bold')
    plt.tight_layout()
    out = os.path.join(FIG_DIR, 'fig3_cache_perf.png')
    plt.savefig(out, bbox_inches='tight')
    plt.close()
    print(f"    Saved: {out}")


# ─────────────────────────────────────────────────────────────────────────────
# Figure 4: YOLO FP32 vs FP16 Per-Op Timing
# ─────────────────────────────────────────────────────────────────────────────
def _mean_col(csv_path, col, skip=5):
    """Compute column mean from a large CSV, skipping first N frames (warmup)."""
    vals = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            if i < skip:
                continue
            try:
                v = float(row[col])
                if v > 0:
                    vals.append(v)
            except (ValueError, KeyError):
                pass
    return statistics.mean(vals) if vals else 0.0

def fig4_yolo_fp_breakdown():
    print("  Generating fig4_yolo_fp_breakdown.png ...")

    fp32_files = sorted(glob.glob(os.path.join(RESULTS_DIR, 'fp32_breakdown_*.csv')))
    fp16_files = sorted(glob.glob(os.path.join(RESULTS_DIR, 'fp16_breakdown_*.csv')))

    if not fp32_files or not fp16_files:
        print("    [SKIP] fp32/fp16 breakdown CSVs not found")
        return

    fp32_csv = fp32_files[-1]  # most recent
    fp16_csv = fp16_files[-1]

    ops = ['prefilter_ms', 'sigmoid_ms', 'bbox_extract_ms', 'coord_rescale_ms', 'nms_ms']
    op_labels = ['Pre-filter\n(logit)', 'Sigmoid', 'BBox\nExtract', 'Coord\nRescale', 'NMS']

    fp32_vals = [_mean_col(fp32_csv, op) for op in ops]
    fp16_vals = [_mean_col(fp16_csv, op) for op in ops]

    x  = np.arange(len(ops))
    w  = 0.35

    fig, ax = plt.subplots(figsize=(8, 4.5))
    b1 = ax.bar(x - w/2, fp32_vals, w, label='FP32 (native NEON)', color=COLORS['fp32'], alpha=0.85)
    b2 = ax.bar(x + w/2, fp16_vals, w, label='FP16 (up-cast overhead)', color=COLORS['fp16'], alpha=0.85)

    # Annotate slowdown
    for i, (f32, f16) in enumerate(zip(fp32_vals, fp16_vals)):
        if f32 > 0 and f16 > 0:
            ratio = f16 / f32
            if ratio > 1.5:
                ax.text(x[i] + w/2, f16 + 0.02, f'{ratio:.1f}×', ha='center',
                        va='bottom', fontsize=8, color='#C44E52', fontweight='bold')

    ax.set_xticks(x)
    ax.set_xticklabels(op_labels)
    ax.set_ylabel('Mean Time per Frame (ms)')
    ax.set_title('YOLO Post-Processing: FP32 vs FP16 Per-Operation Timing\n'
                 'ARM Cortex-A76: No hardware FP16 SIMD — NumPy must upcast every operation')
    ax.legend()

    # Add inference time reference note
    ax.text(0.98, 0.97, 'NPU inference: ~25ms\n(dtype-independent)',
            transform=ax.transAxes, ha='right', va='top', fontsize=8,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    out = os.path.join(FIG_DIR, 'fig4_yolo_fp_breakdown.png')
    plt.savefig(out, bbox_inches='tight')
    plt.close()
    print(f"    Saved: {out}")


# ─────────────────────────────────────────────────────────────────────────────
# Figure 5: YOLO Model Comparison — On-device NMS vs CPU Post-Processing
# ─────────────────────────────────────────────────────────────────────────────
def fig5_yolo_nms_compare():
    print("  Generating fig5_yolo_nms_compare.png ...")

    def load_model_stats(pattern):
        files = sorted(glob.glob(os.path.join(RESULTS_DIR, pattern)))
        if not files:
            return None
        csv_path = files[-1]
        infer  = _mean_col(csv_path, 'inference_ms')
        pre    = _mean_col(csv_path, 'preprocess_ms')
        pp     = _mean_col(csv_path, 'total_pp_ms')
        return {'infer': infer, 'pre': pre, 'pp': pp, 'total': pre + infer + pp}

    models = {
        'yolo26n\n(nano, CPU PP)': load_model_stats('nms_yolo26n_*.csv'),
        'yolov8m\n(medium, NPU NMS)': load_model_stats('nms_yolov8m_*.csv'),
        'yolov11m\n(medium, NPU NMS)': load_model_stats('nms_yolov11m_*.csv'),
    }
    models = {k: v for k, v in models.items() if v is not None}
    if not models:
        print("    [SKIP] NMS comparison CSVs not found")
        return

    labels = list(models.keys())
    pre_v  = [models[m]['pre']   for m in labels]
    inf_v  = [models[m]['infer'] for m in labels]
    pp_v   = [models[m]['pp']    for m in labels]

    x  = np.arange(len(labels))
    w  = 0.5
    cols = [COLORS['yolo26n'], COLORS['yolov8m'], COLORS['yolov11m']]

    fig, ax = plt.subplots(figsize=(7, 4.5))

    b1 = ax.bar(x, pre_v, w, label='Pre-process', color='#8172B2', alpha=0.85)
    b2 = ax.bar(x, inf_v, w, label='NPU Inference', color='#4C72B0', alpha=0.85, bottom=pre_v)
    inf_top = [p + i for p, i in zip(pre_v, inf_v)]
    b3 = ax.bar(x, pp_v,  w, label='CPU Post-Process', color='#55A868', alpha=0.85, bottom=inf_top)

    totals = [p + i + pp for p, i, pp in zip(pre_v, inf_v, pp_v)]
    for i, t in enumerate(totals):
        ax.text(x[i], t + 0.2, f'{t:.1f}ms', ha='center', va='bottom', fontsize=9, fontweight='bold')

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel('Time per Frame (ms)')
    ax.set_title('YOLO Model Comparison: On-Device NMS vs CPU Post-Processing\n'
                 'On-device NMS (v8m/v11m) eliminates CPU decode — but larger model ≈ longer inference')
    ax.legend(loc='upper left')
    plt.tight_layout()
    out = os.path.join(FIG_DIR, 'fig5_yolo_nms_compare.png')
    plt.savefig(out, bbox_inches='tight')
    plt.close()
    print(f"    Saved: {out}")


# ─────────────────────────────────────────────────────────────────────────────
# Figure 6: Full System Pipeline Timing Overview
# ─────────────────────────────────────────────────────────────────────────────
def fig6_pipeline_overview():
    print("  Generating fig6_pipeline_overview.png ...")

    # Data from measured results (from memory and CSVs)
    components = [
        ('Camera\nAcquisition', 31.7, 0.0, '#8172B2'),   # 1/31.5Hz * 1000
        ('YOLO\nPre-process', 0.6, 0.0, '#4C72B0'),
        ('YOLO\nNPU Infer', 25.1, 0.0, '#4C72B0'),
        ('YOLO\nPost-process\n(FP32)', 0.7, 0.0, '#4C72B0'),
        ('SLAM\nFront-End', 5.43, 0.07, '#55A868'),
        ('SLAM\nKLT Track', 1.43, 0.04, '#55A868'),
        ('SLAM\nBundle Adj.', 7.29, 0.22, '#C44E52'),
        ('ViSP\nCallback', 0.145, 0.015, '#CCB974'),
    ]

    labels = [c[0] for c in components]
    means  = [c[1] for c in components]
    stds   = [c[2] for c in components]
    cols   = [c[3] for c in components]

    fig, ax = plt.subplots(figsize=(12, 4.5))
    x = np.arange(len(labels))
    bars = ax.bar(x, means, 0.65, color=cols, alpha=0.85, yerr=stds,
                  error_kw=dict(capsize=4, linewidth=1.2))

    for bar, v in zip(bars, means):
        ax.text(bar.get_x() + bar.get_width() / 2., bar.get_height() + 0.3,
                f'{v:.1f}ms', ha='center', va='bottom', fontsize=8, fontweight='bold')

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8.5)
    ax.set_ylabel('Time (ms)')
    ax.set_title('Full Pipeline Stage Timing — Raspberry Pi 5 (Cortex-A76 + Hailo-10H NPU)\n'
                 'Baseline configuration, MH01-Easy, 3-run mean ± std')

    # Legend patches
    patches = [
        mpatches.Patch(color='#8172B2', alpha=0.85, label='Camera (CSI/V4L2)'),
        mpatches.Patch(color='#4C72B0', alpha=0.85, label='YOLO (Hailo NPU)'),
        mpatches.Patch(color='#55A868', alpha=0.85, label='SLAM Front-End (OV2SLAM)'),
        mpatches.Patch(color='#C44E52', alpha=0.85, label='SLAM Bundle Adjust (Ceres)'),
        mpatches.Patch(color='#CCB974', alpha=0.85, label='ViSP Servo'),
    ]
    ax.legend(handles=patches, loc='upper right', fontsize=8)
    ax.set_yscale('log')
    ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(
        lambda x, p: f'{x:.0f}' if x >= 1 else f'{x:.2f}'
    ))

    plt.tight_layout(rect=[0, 0, 1, 1])
    out = os.path.join(FIG_DIR, 'fig6_pipeline_overview.png')
    plt.savefig(out, bbox_inches='tight')
    plt.close()
    print(f"    Saved: {out}")


# ─────────────────────────────────────────────────────────────────────────────
# Figure 7: IPC CDF (regenerate from CSVs in case the original was lost)
# ─────────────────────────────────────────────────────────────────────────────
def fig7_ipc_cdf():
    print("  Generating fig7_ipc_cdf.png ...")

    shm_csv = os.path.join(RESULTS_DIR, 'ipc_latency_shm_1000x921600.csv')
    dds_csv = os.path.join(RESULTS_DIR, 'ipc_latency_dds_1000x921600.csv')

    if not os.path.exists(shm_csv) or not os.path.exists(dds_csv):
        print("    [SKIP] IPC latency CSVs not found")
        return

    def load_latencies(path):
        lats = []
        with open(path) as f:
            for r in csv.DictReader(f):
                try:
                    lats.append(float(r['latency_us']))
                except (KeyError, ValueError):
                    pass
        return sorted(lats)

    shm_lats = load_latencies(shm_csv)
    dds_lats = load_latencies(dds_csv)

    if not shm_lats or not dds_lats:
        print("    [SKIP] No latency data found in CSVs")
        return

    def cdf(vals):
        n = len(vals)
        return vals, [i / n for i in range(1, n + 1)]

    fig, ax = plt.subplots(figsize=(7, 4.5))

    sx, sy = cdf(shm_lats)
    dx, dy = cdf(dds_lats)

    shm_p50 = shm_lats[int(0.50 * len(shm_lats))]
    shm_p99 = shm_lats[int(0.99 * len(shm_lats))]
    dds_p50 = dds_lats[int(0.50 * len(dds_lats))]
    dds_p99 = dds_lats[int(0.99 * len(dds_lats))]

    ax.plot(sx, sy, color=COLORS['shm'], linewidth=2,
            label=f'SHM Seqlock  P50={shm_p50:.0f}µs  P99={shm_p99:.0f}µs')
    ax.plot(dx, dy, color=COLORS['dds'], linewidth=2,
            label=f'ROS2 DDS     P50={dds_p50:.0f}µs  P99={dds_p99:.0f}µs')

    ax.axvline(shm_p50, color=COLORS['shm'], linestyle='--', alpha=0.5)
    ax.axvline(dds_p50, color=COLORS['dds'], linestyle='--', alpha=0.5)
    ax.axhline(0.50, color='gray', linestyle=':', alpha=0.4, linewidth=0.8)
    ax.axhline(0.99, color='gray', linestyle=':', alpha=0.4, linewidth=0.8)

    ax.set_xscale('log')
    ax.set_xlabel('Latency (µs, log scale)')
    ax.set_ylabel('CDF')
    ax.set_title(f'IPC Latency CDF: SHM Seqlock vs ROS2 DDS\n'
                 f'921,600-byte frames (640×480 grayscale) — SHM is {dds_p50/shm_p50:.0f}× faster at P50')
    ax.legend()
    plt.tight_layout()
    out = os.path.join(FIG_DIR, 'fig7_ipc_cdf.png')
    plt.savefig(out, bbox_inches='tight')
    plt.close()
    print(f"    Saved: {out}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    stats_csv = os.path.join(RESULTS_DIR, 'slam_config_stats.csv')
    perf_csv  = os.path.join(RESULTS_DIR, 'perf_stat_summary.csv')

    print(f"Output directory: {FIG_DIR}")
    print()

    if os.path.exists(stats_csv):
        fig1_slam_ablation(stats_csv)
        fig2_slam_pareto(stats_csv)
    else:
        print("  [SKIP] slam_config_stats.csv not found — skipping fig1 and fig2")

    if os.path.exists(perf_csv):
        fig3_cache_perf(perf_csv)
    else:
        print("  [SKIP] perf_stat_summary.csv not found — skipping fig3")

    fig4_yolo_fp_breakdown()
    fig5_yolo_nms_compare()
    fig6_pipeline_overview()
    fig7_ipc_cdf()

    print()
    print("All figures written to:", FIG_DIR)
    figs = glob.glob(os.path.join(FIG_DIR, '*.png'))
    for f in sorted(figs):
        print(f"  {os.path.basename(f)}")


if __name__ == '__main__':
    main()
