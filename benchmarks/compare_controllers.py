#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
# compare_controllers.py — aggregate N runs of each controller into
# side-by-side comparison figures + a summary CSV.
#
#   python3 benchmarks/compare_controllers.py OUT_DIR RUN_DIR [RUN_DIR ...]
#
# Each RUN_DIR is a ctrl_<controller>_N<k>_<stamp> bag dir (must already
# have been processed — we re-read the bag for the overlay traces and read
# metrics.csv for the scalar bars). Controller is inferred from meta.txt.
#
# Writes into OUT_DIR:
#   cmp_distance.png     distance-to-target vs time, all runs overlaid
#   cmp_trajectory.png   top-down xy paths, all runs overlaid
#   cmp_metrics.png      grouped bars (mean ± spread) of key metrics
#   summary.csv          per-run + per-controller-mean metrics table
# ─────────────────────────────────────────────────────────────────
import sys
import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader

FY, KNOWN_H, IMG_H, RATIO = 554.0, 1.5, 480, 0.55
STANDOFF = FY * KNOWN_H / (RATIO * IMG_H)

COLORS = {'ibvs': '#d62728', 'proportional': '#1f77b4'}
LABELS = {'ibvs': 'IBVS (TS1)', 'proportional': 'Proportional (TS2)'}


def meta_controller(rundir: Path) -> str:
    for line in (rundir / 'meta.txt').read_text().splitlines():
        if line.startswith('controller='):
            return line.split('=', 1)[1].strip()
    return 'unknown'


def find_bag(rundir: Path) -> Path:
    if (rundir / 'metadata.yaml').exists():
        return rundir
    if (rundir / 'bag' / 'metadata.yaml').exists():
        return rundir / 'bag'
    c = list(rundir.glob('**/metadata.yaml'))
    return c[0].parent if c else rundir


def read_trace(rundir: Path):
    """Return (t_rel, dist3d, xs, ys) post-engage."""
    drone, target, state = [], [], []
    topics = {'/sim/drone_pose', '/sim/target_pose', '/bench/state'}
    with AnyReader([find_bag(rundir)]) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        for conn, ts, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = ts * 1e-9
            if conn.topic == '/sim/drone_pose' and len(m.data) >= 3:
                drone.append((t, m.data[0], m.data[1], m.data[2]))
            elif conn.topic == '/sim/target_pose' and len(m.data) >= 3:
                target.append((m.data[0], m.data[1], m.data[2]))
            elif conn.topic == '/bench/state':
                state.append((t, m.data.split(',')[0]))
    if not drone or not target:
        return None
    tx, ty, tz = target[-1]
    t0 = next((t for t, s in state if s == 'APPROACHING'), drone[0][0])
    d = np.array([r for r in drone if r[0] >= t0])
    if len(d) < 3:
        return None
    tt = d[:, 0] - t0
    xs, ys, zs = d[:, 1], d[:, 2], d[:, 3]
    dist = np.sqrt((tx - xs) ** 2 + (ty - ys) ** 2 + (tz - zs) ** 2)
    return tt, dist, xs, ys, tx, ty


def read_metrics(rundir: Path) -> dict:
    out = {}
    f = rundir / 'metrics.csv'
    if not f.exists():
        return out
    with open(f) as fh:
        for row in csv.reader(fh):
            if len(row) == 2 and row[0] != 'metric':
                try:
                    out[row[0]] = float(row[1])
                except ValueError:
                    pass
    return out


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: compare_controllers.py OUT_DIR RUN_DIR [RUN_DIR ...]")
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
    runs = [Path(p) for p in sys.argv[2:]]

    grouped = {}  # controller -> list of (name, trace, metrics)
    for r in runs:
        ctrl = meta_controller(r)
        tr = read_trace(r)
        mx = read_metrics(r)
        grouped.setdefault(ctrl, []).append((r.name, tr, mx))

    # ── Fig 1: distance vs time overlay ──
    fig, ax = plt.subplots(figsize=(9, 5))
    for ctrl, items in grouped.items():
        for i, (name, tr, _) in enumerate(items):
            if tr is None:
                continue
            tt, dist = tr[0], tr[1]
            ax.plot(tt, dist, color=COLORS.get(ctrl, 'gray'), alpha=0.8, lw=1.6,
                    label=LABELS.get(ctrl, ctrl) if i == 0 else None)
    ax.axhline(STANDOFF, color='green', ls='--', lw=1, label=f'standoff {STANDOFF:.2f} m')
    ax.set_xlabel('time since engage (s)'); ax.set_ylabel('distance to target (m)')
    ax.set_title('Approach: distance to target vs time (3 runs each)')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out / 'cmp_distance.png', dpi=140); plt.close(fig)

    # ── Fig 2: top-down trajectory overlay ──
    fig, ax = plt.subplots(figsize=(7, 7))
    tgt = None
    for ctrl, items in grouped.items():
        for i, (name, tr, _) in enumerate(items):
            if tr is None:
                continue
            _, _, xs, ys, tx, ty = tr
            tgt = (tx, ty)
            ax.plot(xs, ys, color=COLORS.get(ctrl, 'gray'), alpha=0.8, lw=1.6,
                    label=LABELS.get(ctrl, ctrl) if i == 0 else None)
            ax.plot(xs[0], ys[0], 'o', color=COLORS.get(ctrl, 'gray'), ms=5)
    if tgt:
        ax.plot(*tgt, 'k*', ms=18, label='target')
        th = np.linspace(0, 2 * np.pi, 80)
        ax.plot(tgt[0] + STANDOFF * np.cos(th), tgt[1] + STANDOFF * np.sin(th),
                'g--', lw=0.9, label='standoff ring')
    ax.set_xlabel('world x (m)'); ax.set_ylabel('world y (m)')
    ax.set_title('Top-down approach trajectories'); ax.axis('equal')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(out / 'cmp_trajectory.png', dpi=140); plt.close(fig)

    # ── Fig 3: grouped metric bars ──
    metric_keys = [
        ('settling_time_s', 'Settling time (s)'),
        ('steady_state_err_m', 'Steady-state err (m)'),
        ('path_efficiency', 'Path efficiency'),
        ('rms_cmd_speed', 'RMS cmd speed (m/s)'),
        ('total_variation', 'Control jerk (total var.)'),
        ('IAE', 'IAE'),
        ('cpu_mean_pct', 'CPU (% of one core)'),
        ('mem_rss_mb', 'Memory RSS (MB)'),
    ]
    ctrls = list(grouped.keys())
    fig, axes = plt.subplots(2, 4, figsize=(16, 7))
    for ax, (key, title) in zip(axes.flat, metric_keys):
        means, errs, cols = [], [], []
        for ctrl in ctrls:
            vals = [m.get(key, np.nan) for _, _, m in grouped[ctrl]]
            vals = [v for v in vals if not np.isnan(v)]
            means.append(np.mean(vals) if vals else 0)
            errs.append(np.std(vals) if len(vals) > 1 else 0)
            cols.append(COLORS.get(ctrl, 'gray'))
        x = np.arange(len(ctrls))
        ax.bar(x, means, yerr=errs, color=cols, alpha=0.85, capsize=5)
        ax.set_xticks(x); ax.set_xticklabels([LABELS.get(c, c) for c in ctrls], fontsize=8)
        ax.set_title(title, fontsize=10); ax.grid(True, axis='y', alpha=0.3)
    fig.suptitle('Controller metrics — mean ± std over 3 runs', fontsize=13)
    fig.tight_layout(); fig.savefig(out / 'cmp_metrics.png', dpi=140); plt.close(fig)

    # ── summary.csv ──
    all_keys = [k for k, _ in metric_keys] + ['overshoot_m', 'path_length_m']
    with open(out / 'summary.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['run', 'controller'] + all_keys)
        for ctrl, items in grouped.items():
            for name, _, m in items:
                w.writerow([name, ctrl] + [m.get(k, '') for k in all_keys])
        # means
        for ctrl, items in grouped.items():
            row = [f'MEAN_{ctrl}', ctrl]
            for k in all_keys:
                vals = [m.get(k, np.nan) for _, _, m in items]
                vals = [v for v in vals if not np.isnan(v)]
                row.append(round(np.mean(vals), 4) if vals else '')
            w.writerow(row)

    print(f"wrote cmp_distance.png, cmp_trajectory.png, cmp_metrics.png, summary.csv to {out}")
    # echo the summary so the caller can see it
    print((out / 'summary.csv').read_text())


if __name__ == '__main__':
    main()