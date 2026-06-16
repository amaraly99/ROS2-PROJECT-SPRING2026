#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
# plot_controller_hil.py — offline analysis of ONE benchmark run.
#
#   python3 benchmarks/plot_controller_hil.py bags/ctrl_ibvs_N1_<stamp>
#
# Reads the rosbag (ONLY standard-typed topics — no custom message setup):
#   /bench/state     std_msgs/String          → t=0 = first APPROACHING edge
#   /sim/drone_pose  Float64MultiArray         → trajectory (ground truth)
#   /sim/target_pose Float64MultiArray         → target (ground truth)
#   /cmd_vel         geometry_msgs/Twist       → control effort / smoothness
# plus cpu.csv (sampled by the harness) → controller CPU.
#
# Computes settling time, steady-state error, overshoot, IAE/ITAE/ISE,
# path efficiency, control effort, smoothness, CPU. Writes metrics.csv +
# PNG figures into the run dir. Fly once, analyse forever.
#
# Deps:  pip install rosbags matplotlib numpy
# ─────────────────────────────────────────────────────────────────
import sys
import os
import csv
import math
from pathlib import Path

try:
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from rosbags.highlevel import AnyReader
except ImportError as e:
    sys.exit(f"missing dependency: {e}\n  pip install rosbags matplotlib numpy")

# Must match bench_fsm.yaml.
FY = 554.0
KNOWN_H = 1.5
IMG_H = 480
TARGET_BBOX_RATIO = 0.55
STANDOFF = FY * KNOWN_H / (TARGET_BBOX_RATIO * IMG_H)   # ≈ 3.15 m stop range
SETTLE_BAND = 0.30     # m, |dist - standoff| considered "settled"
SETTLE_HOLD = 2.0      # s the band must be held


def find_bag(rundir: Path) -> Path:
    if (rundir / 'metadata.yaml').exists():
        return rundir
    bag = rundir / 'bag'
    if (bag / 'metadata.yaml').exists():
        return bag
    cands = list(rundir.glob('**/metadata.yaml'))
    if cands:
        return cands[0].parent
    sys.exit(f"no rosbag (metadata.yaml) found under {rundir}")


def read_bag(bagdir: Path):
    drone, target, cmd, state = [], [], [], []
    topics = {'/sim/drone_pose', '/sim/target_pose', '/cmd_vel', '/bench/state'}
    with AnyReader([bagdir]) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        for conn, ts, raw in reader.messages(connections=conns):
            t = ts * 1e-9
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/sim/drone_pose' and len(m.data) >= 5:
                drone.append((t, m.data[0], m.data[1], m.data[2], m.data[3], m.data[4]))
            elif conn.topic == '/sim/target_pose' and len(m.data) >= 3:
                target.append((m.data[0], m.data[1], m.data[2]))
            elif conn.topic == '/cmd_vel':
                cmd.append((t, m.linear.x, m.linear.y, m.linear.z, m.angular.y, m.angular.z))
            elif conn.topic == '/bench/state':
                parts = m.data.split(',')
                state.append((t, parts[0] if parts else '?'))
    return drone, target, cmd, state


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: plot_controller_hil.py <run_dir>")
    rundir = Path(sys.argv[1])
    bagdir = find_bag(rundir)
    drone, target, cmd, state = read_bag(bagdir)

    if not drone or not target:
        sys.exit("bag missing /sim/drone_pose or /sim/target_pose — nothing to analyse")
    tx, ty, tz = target[-1]

    # t=0 = first APPROACHING edge (search phase ignored, per design).
    t0 = None
    for t, s in state:
        if s == 'APPROACHING':
            t0 = t
            break
    if t0 is None:
        print("WARN: no APPROACHING state seen — using first drone sample as t=0")
        t0 = drone[0][0]

    d = np.array([r for r in drone if r[0] >= t0])
    if len(d) < 3:
        sys.exit("too few post-engage drone samples — did the controller engage?")
    tt = d[:, 0] - t0
    xs, ys, zs = d[:, 1], d[:, 2], d[:, 3]
    dist3d = np.sqrt((tx - xs) ** 2 + (ty - ys) ** 2 + (tz - zs) ** 2)
    err = dist3d - STANDOFF        # regulation error to the stop range

    # ── scalar metrics ──
    _trapz = getattr(np, 'trapezoid', getattr(np, 'trapz', None))

    def integral(y):
        return float(_trapz(np.abs(y), tt))
    iae = integral(err)
    ise = float(_trapz(err ** 2, tt))
    itae = float(_trapz(np.abs(err) * tt, tt))

    # settling time: first t where |err|<band and stays for SETTLE_HOLD.
    settle = float('nan')
    for i in range(len(tt)):
        if abs(err[i]) <= SETTLE_BAND:
            j = i
            while j < len(tt) and tt[j] - tt[i] < SETTLE_HOLD:
                if abs(err[j]) > SETTLE_BAND:
                    break
                j += 1
            else:
                settle = tt[i]
                break
            if j < len(tt) and abs(err[j]) <= SETTLE_BAND:
                settle = tt[i]
                break

    # steady-state error: mean |err| over last 3 s.
    tail = err[tt >= (tt[-1] - 3.0)]
    ss_err = float(np.mean(np.abs(tail))) if len(tail) else float('nan')

    # overshoot: how far past the standoff the drone lunged.
    overshoot = float(max(0.0, STANDOFF - float(np.min(dist3d))))

    # path efficiency: straight-line need vs actual path.
    seg = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2 + np.diff(zs) ** 2)
    path_len = float(np.sum(seg))
    straight = float(max(0.0, dist3d[0] - STANDOFF))
    path_eff = (straight / path_len) if path_len > 1e-6 else float('nan')

    # control effort + smoothness from /cmd_vel (post-engage).
    c = np.array([r for r in cmd if r[0] >= t0]) if cmd else np.empty((0, 6))
    if len(c) > 1:
        v = np.sqrt(c[:, 1] ** 2 + c[:, 2] ** 2 + c[:, 3] ** 2)
        rms_v = float(np.sqrt(np.mean(v ** 2)))
        dv = np.sqrt(np.sum(np.diff(c[:, 1:4], axis=0) ** 2, axis=1))
        total_var = float(np.sum(dv))
    else:
        rms_v = total_var = float('nan')

    # CPU + memory from harness sampler.
    cpu_mean = float('nan')
    mem_rss_mb = float('nan')
    cpu_file = rundir / 'cpu.csv'
    if cpu_file.exists():
        cpu_vals, rss_vals = [], []
        with open(cpu_file) as f:
            for row in csv.DictReader(f):
                try:
                    cpu_vals.append(float(row['pcpu']))
                except (KeyError, ValueError):
                    pass
                try:
                    rss_vals.append(float(row['rss_kb']) / 1024.0)
                except (KeyError, ValueError):
                    pass
        if cpu_vals:
            cpu_mean = float(np.mean(cpu_vals))
        if rss_vals:
            mem_rss_mb = float(np.mean(rss_vals))

    metrics = {
        'engage_to_end_s': float(tt[-1]),
        'settling_time_s': settle,
        'steady_state_err_m': ss_err,
        'overshoot_m': overshoot,
        'IAE': iae, 'ITAE': itae, 'ISE': ise,
        'path_length_m': path_len, 'path_efficiency': path_eff,
        'rms_cmd_speed': rms_v, 'total_variation': total_var,
        'cpu_mean_pct': cpu_mean, 'mem_rss_mb': mem_rss_mb,
        'standoff_m': STANDOFF,
    }

    # ── write metrics.csv ──
    with open(rundir / 'metrics.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['metric', 'value'])
        for k, val in metrics.items():
            w.writerow([k, val])

    print(f"\n=== {rundir.name} ===")
    for k, val in metrics.items():
        print(f"  {k:22s} {val:.4f}" if isinstance(val, float) else f"  {k:22s} {val}")

    # ── figures ──
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(tt, dist3d, label='distance to target')
    ax.axhline(STANDOFF, color='g', ls='--', label=f'standoff {STANDOFF:.2f} m')
    if not math.isnan(settle):
        ax.axvline(settle, color='r', ls=':', label=f'settling {settle:.1f} s')
    ax.set_xlabel('time since search-exit (s)'); ax.set_ylabel('distance (m)')
    ax.set_title(f'{rundir.name} — distance to target'); ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(rundir / 'fig_distance.png', dpi=130); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(xs, ys, '-', label='drone path')
    ax.plot(xs[0], ys[0], 'go', label='start (engage)')
    ax.plot(tx, ty, 'r*', ms=15, label='target')
    th = np.linspace(0, 2 * np.pi, 60)
    ax.plot(tx + STANDOFF * np.cos(th), ty + STANDOFF * np.sin(th), 'g--', lw=0.8,
            label='standoff ring')
    ax.set_xlabel('world x (m)'); ax.set_ylabel('world y (m)')
    ax.set_title(f'{rundir.name} — top-down trajectory'); ax.axis('equal')
    ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(rundir / 'fig_trajectory.png', dpi=130); plt.close(fig)

    if len(c) > 1:
        ct = c[:, 0] - t0
        fig, ax = plt.subplots(figsize=(8, 4))
        for idx, lbl in [(1, 'vx'), (2, 'vy'), (3, 'vz'), (5, 'wz')]:
            ax.plot(ct, c[:, idx], label=lbl)
        ax.set_xlabel('time since search-exit (s)'); ax.set_ylabel('cmd_vel')
        ax.set_title(f'{rundir.name} — command'); ax.legend(); ax.grid(True)
        fig.tight_layout(); fig.savefig(rundir / 'fig_cmd.png', dpi=130); plt.close(fig)

    print(f"\nwrote metrics.csv + fig_*.png to {rundir}\n")


if __name__ == '__main__':
    main()