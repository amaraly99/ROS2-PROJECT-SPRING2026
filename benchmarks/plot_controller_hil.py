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

# Camera + stop distance (must match bench_fsm.yaml): one place, benchmarks/bench_geometry.py.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_geometry import FY, KNOWN_H, IMG_H, TARGET_BBOX_RATIO, STANDOFF  # noqa: E402,F401
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


def read_meta(rundir: Path) -> dict:
    """key=value lines of meta.txt (the last value wins)."""
    meta = rundir / 'meta.txt'
    out = {}
    if meta.exists():
        for line in meta.read_text().splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                out[k.strip()] = v.strip()
    return out


def read_resume_time(rundir: Path):
    """resume_wall_ns from meta.txt (written by ctrl_runner.py) as seconds, else None."""
    try:
        return int(read_meta(rundir)['resume_wall_ns']) * 1e-9
    except (KeyError, ValueError):
        return None


def heartbeat_t0(rundir: Path, bagdir: Path):
    """Stopwatch start of a runner run (approved by Amar Aly, 2026-10-17):
    the first /sim/heartbeat after the hold, i.e. the first one whose sim time is
    larger than the held value — the moment the simulator itself starts to move.
    The held value is the last heartbeat at or before the Pi sent 'resume'
    (resume_sent_ns), else the first heartbeat in the bag.

    Returns (t0_s, rule, resume_ok_minus_t0_s). Runs without resume_wall_ns in
    meta.txt return (None, None, None): the caller keeps its older rule.
    """
    meta = read_meta(rundir)
    if 'resume_wall_ns' not in meta:
        return None, None, None
    ok_t = int(meta['resume_wall_ns']) * 1e-9
    sent_t = int(meta['resume_sent_ns']) * 1e-9 if 'resume_sent_ns' in meta else None
    hb = []
    with AnyReader([bagdir]) as reader:
        conns = [c for c in reader.connections if c.topic == '/sim/heartbeat']
        for conn, ts, raw in reader.messages(connections=conns):
            hb.append((ts * 1e-9, float(reader.deserialize(raw, conn.msgtype).data)))
    if not hb:
        return ok_t, 'resume_ok (no heartbeat in bag)', 0.0
    before = [v for t, v in hb if sent_t is not None and t <= sent_t]
    held = before[-1] if before else hb[0][1]
    start = (sent_t if sent_t is not None else hb[0][0])
    moved = next((t for t, v in hb if t >= start and v > held + 1e-6), None)
    if moved is None:
        return ok_t, 'resume_ok (no heartbeat moved after the hold)', 0.0
    return moved, 'heartbeat_after_hold', ok_t - moved


def settling_time(tt, err):
    """First t where |err| <= SETTLE_BAND and stays there for SETTLE_HOLD s; NaN if never."""
    for i in range(len(tt)):
        if abs(err[i]) <= SETTLE_BAND:
            j = i
            while j < len(tt) and tt[j] - tt[i] < SETTLE_HOLD:
                if abs(err[j]) > SETTLE_BAND:
                    break
                j += 1
            else:
                return float(tt[i])
            if j < len(tt) and abs(err[j]) <= SETTLE_BAND:
                return float(tt[i])
    return float('nan')


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

    # t=0. Runner runs (benchmarks/ctrl_runner.py) write resume_wall_ns into
    # meta.txt: the shared start protocol holds the sim at its initial condition
    # (start_hold) while the controller boots, and the controller already reports
    # APPROACHING during that hold. So the stopwatch starts at 'resume', the moment
    # the drone can move. Older runs (manual Stop+Run, no hold) keep the original
    # rule: t=0 = first APPROACHING edge (search phase ignored, per design).
    first_approaching = next((t for t, s in state if s == 'APPROACHING'), None)
    hb_t0, hb_rule, ok_minus_t0 = heartbeat_t0(rundir, bagdir)
    if hb_t0 is not None:
        t0, t0_rule = hb_t0, hb_rule
    elif first_approaching is not None:
        t0, t0_rule = first_approaching, 'first_approaching'
    else:
        print("WARN: no APPROACHING state seen — using first drone sample as t=0")
        t0, t0_rule = drone[0][0], 'first_drone_sample'

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
    settle = settling_time(tt, err)
    # The same, with the original stopwatch (first APPROACHING edge in the bag),
    # kept as an extra column. In runner runs the bag starts just before 'resume',
    # after the controller already reported APPROACHING, so this edge is the first
    # recorded sample, not the true engage moment.
    settle_old = float('nan')
    if first_approaching is not None:
        d_old = np.array([r for r in drone if r[0] >= first_approaching])
        if len(d_old) >= 3:
            dist_old = np.sqrt((tx - d_old[:, 1]) ** 2 + (ty - d_old[:, 2]) ** 2 + (tz - d_old[:, 3]) ** 2)
            settle_old = settling_time(d_old[:, 0] - first_approaching, dist_old - STANDOFF)

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
        't0_rule': t0_rule,
        'resume_ok_minus_t0_s': ok_minus_t0 if ok_minus_t0 is not None else float('nan'),
        'settling_time_first_approaching_s': settle_old,
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