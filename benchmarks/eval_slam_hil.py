#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
# eval_slam_hil.py — offline ATE RMSE evaluation of one SLAM benchmark run.
#
#   python3 benchmarks/eval_slam_hil.py bags/run_orbslam2_eval_<stamp>
#
# Reads the rosbag:
#   /slam/pose      geometry_msgs/PoseStamped  → SLAM trajectory (up-to-scale)
#   /sim/drone_pose std_msgs/Float64MultiArray  → GT trajectory (world frame)
#
# Monocular SLAMs (ORB-SLAM2, OV2SLAM) are up-to-scale with an arbitrary
# reference frame.  This script applies Umeyama similarity alignment (SE3 +
# scale) before computing ATE, the same approach evo_ape uses with
# --align --correct_scale.
#
# Writes into the run dir:
#   slam_metrics.csv        scalar results (ATE RMSE, scale, n_samples, …)
#   slam_traj.tum           Umeyama-aligned SLAM trajectory (TUM format)
#   gt_traj.tum             GT trajectory resampled at SLAM timestamps (TUM format)
#   traj_xy.png             evo_traj XY overlay (evo-style, same as EuRoC benchmark)
#   fig_slam_ate.png        ATE vs time
#   fig_slam_trajectory.png GT vs aligned SLAM, full flight path, colored by ATE
#   fig_slam_error_xyz.png  per-axis translation error vs time
#
# Deps:  pip install rosbags matplotlib numpy
# evo_traj (optional, for traj_xy.png): SLAM-Former/venv/bin/evo_traj
# ─────────────────────────────────────────────────────────────────
import subprocess
import sys
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


# ── helpers ──────────────────────────────────────────────────────

def find_bag(rundir: Path):
    """Return a path list suitable for AnyReader: a directory with metadata.yaml
    or a list of .mcap files (Jazzy bags skip metadata.yaml)."""
    if (rundir / 'metadata.yaml').exists():
        return [rundir]
    bag = rundir / 'bag'
    if (bag / 'metadata.yaml').exists():
        return [bag]
    cands = list(rundir.glob('**/metadata.yaml'))
    if cands:
        return [cands[0].parent]
    # MCAP-only (no metadata.yaml) — pass .mcap files directly.
    mcaps = sorted(rundir.glob('**/*.mcap'))
    if mcaps:
        return mcaps
    sys.exit(f"no rosbag (metadata.yaml or .mcap) found under {rundir}")


def meta_val(rundir: Path, key: str) -> str:
    f = rundir / 'meta.txt'
    if not f.exists():
        return ''
    for line in f.read_text().splitlines():
        if line.startswith(f'{key}='):
            return line.split('=', 1)[1].strip()
    return ''


def read_bag(bagdir: Path):
    """Return (slam_poses, gt_poses).
    slam_poses: list of (t_s, x, y, z)   from /slam/pose PoseStamped
    gt_poses:   list of (t_s, x, y, z)   from /sim/drone_pose Float64MultiArray
    """
    slam, gt = [], []
    topics = {'/slam/pose', '/sim/drone_pose'}
    with AnyReader(bagdir) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        if not conns:
            sys.exit("bag has neither /slam/pose nor /sim/drone_pose — wrong bag?")
        for conn, ts, raw in reader.messages(connections=conns):
            t = ts * 1e-9
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/slam/pose':
                p = m.pose.position
                slam.append((t, p.x, p.y, p.z))
            elif conn.topic == '/sim/drone_pose':
                if len(m.data) >= 3:
                    gt.append((t, m.data[0], m.data[1], m.data[2]))
    return slam, gt


# ── Umeyama similarity alignment ─────────────────────────────────

def umeyama(src: np.ndarray, dst: np.ndarray):
    """
    Find s, R, t minimising sum ||s*R*src_i + t - dst_i||^2.
    src, dst: (N, 3).  Returns (s, R, t, src_aligned).
    Follows the Umeyama (1991) TPAMI formulation used by evo.
    """
    N = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    s_c = src - mu_s
    d_c = dst - mu_d
    var_s = np.mean(np.sum(s_c ** 2, axis=1))
    if var_s < 1e-12:
        sys.exit("SLAM trajectory has zero variance — SLAM never tracked?")
    H = (d_c.T @ s_c) / N
    U, S, Vt = np.linalg.svd(H)
    d = np.ones(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        d[-1] = -1
    R = U @ np.diag(d) @ Vt
    s = float(np.dot(S, d) / var_s)
    t = mu_d - s * R @ mu_s
    aligned = (s * (R @ s_c.T)).T + mu_d
    return s, R, t, aligned


# ── main ─────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        sys.exit("usage: eval_slam_hil.py <run_dir>")
    rundir = Path(sys.argv[1])
    bagpaths = find_bag(rundir)
    slam_raw, gt_raw = read_bag(bagpaths)

    if not slam_raw:
        sys.exit("no /slam/pose messages in bag — SLAM never published poses (tracking failure?)")
    if not gt_raw:
        sys.exit("no /sim/drone_pose messages in bag — GT not recorded")

    slam_arr = np.array(slam_raw)   # (N, 4): t, x, y, z
    gt_arr   = np.array(gt_raw)     # (M, 4): t, x, y, z

    slam_t = slam_arr[:, 0]
    slam_p = slam_arr[:, 1:4]

    gt_t   = gt_arr[:, 0]
    gt_p   = gt_arr[:, 1:4]

    # Interpolate GT at each SLAM timestamp (GT is typically faster ~50 Hz, SLAM ~20 Hz).
    # Only keep SLAM samples within the GT time window.
    t_min = max(slam_t[0],  gt_t[0])
    t_max = min(slam_t[-1], gt_t[-1])
    mask  = (slam_t >= t_min) & (slam_t <= t_max)
    if mask.sum() < 10:
        sys.exit(f"only {mask.sum()} overlapping samples — did SLAM and sim run simultaneously?")

    slam_t = slam_t[mask]
    slam_p = slam_p[mask]
    gt_interp = np.column_stack([
        np.interp(slam_t, gt_t, gt_p[:, i]) for i in range(3)
    ])

    # Umeyama alignment.
    s, R, t_vec, slam_aligned = umeyama(slam_p, gt_interp)

    # Per-sample translation error.
    err_xyz = slam_aligned - gt_interp          # (N, 3)
    err_mag = np.linalg.norm(err_xyz, axis=1)   # (N,)

    ate_rmse = float(np.sqrt(np.mean(err_mag ** 2)))
    ate_mean = float(np.mean(err_mag))
    ate_max  = float(np.max(err_mag))
    ate_min  = float(np.min(err_mag))
    duration = float(slam_t[-1] - slam_t[0])
    slam_hz  = len(slam_t) / max(duration, 1e-6)

    # Path length — eval window (GT interpolated at SLAM timestamps).
    path_length = float(np.sum(np.linalg.norm(np.diff(gt_interp, axis=0), axis=1)))
    ate_rmse_eval_pct = ate_rmse / max(path_length, 1e-6) * 100.0

    # Path length — full flight (all GT samples in the bag).
    gt_full = np.array(gt_raw)[:, 1:4]   # (M, 3)
    full_path_length = float(np.sum(np.linalg.norm(np.diff(gt_full, axis=0), axis=1)))
    ate_rmse_full_pct = ate_rmse / max(full_path_length, 1e-6) * 100.0

    t_rel = slam_t - slam_t[0]

    # ── metrics ──────────────────────────────────────────────────
    slam_type = meta_val(rundir, 'slam_type') or rundir.name
    metrics = {
        'slam_type':            slam_type,
        'n_samples':            len(slam_t),
        'duration_s':           round(duration, 2),
        'slam_hz':              round(slam_hz, 2),
        'full_path_length_m':   round(full_path_length, 3),
        'eval_path_length_m':   round(path_length, 3),
        'umeyama_scale':        round(s, 6),
        'ATE_RMSE_m':           round(ate_rmse, 6),
        'ATE_RMSE_full_pct':    round(ate_rmse_full_pct, 4),
        'ATE_RMSE_eval_pct':    round(ate_rmse_eval_pct, 4),
        'ATE_mean_m':           round(ate_mean, 6),
        'ATE_max_m':            round(ate_max,  6),
        'ATE_min_m':            round(ate_min,  6),
    }

    with open(rundir / 'slam_metrics.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['metric', 'value'])
        for k, v in metrics.items():
            w.writerow([k, v])

    print(f"\n=== SLAM ATE — {rundir.name} ===")
    for k, v in metrics.items():
        print(f"  {k:22s}  {v}")

    # ── fig 1: ATE vs time ───────────────────────────────────────
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t_rel, err_mag * 100, color='#d62728', lw=1.5)
    ax.axhline(ate_rmse * 100, color='k', ls='--', lw=1,
               label=f'ATE RMSE {ate_rmse * 100:.1f} cm')
    ax.set_xlabel('time (s)'); ax.set_ylabel('translation error (cm)')
    ax.set_title(f'{slam_type} — ATE vs time (Umeyama-aligned)')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(rundir / 'fig_slam_ate.png', dpi=130)
    plt.close(fig)

    # ── fig 2: top-down trajectory ───────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    # Full flight GT in light gray so you can see how much of the flight SLAM covered.
    gt_full_arr = np.array(gt_raw)
    ax.plot(gt_full_arr[:, 1], gt_full_arr[:, 2], color='#cccccc', lw=1.5,
            label=f'GT full flight ({full_path_length:.1f} m)', zorder=1)
    # Eval-window GT in blue.
    ax.plot(gt_interp[:, 0], gt_interp[:, 1], 'b-', lw=2.0,
            label=f'GT eval window ({path_length:.1f} m)', zorder=2)
    # SLAM estimated path, scatter-colored by ATE error for instant hot-spot reading.
    sc = ax.scatter(slam_aligned[:, 0], slam_aligned[:, 1],
                    c=err_mag * 100, cmap='plasma', s=10, zorder=3,
                    label=f'SLAM aligned (s={s:.3f})')
    cbar = fig.colorbar(sc, ax=ax, fraction=0.035, pad=0.04)
    cbar.set_label('ATE error (cm)')
    # Start / end markers.
    ax.plot(gt_interp[0, 0],  gt_interp[0, 1],  'go', ms=10, zorder=4, label='SLAM start')
    ax.plot(gt_interp[-1, 0], gt_interp[-1, 1], 'rs', ms=10, zorder=4, label='SLAM end')
    ax.set_xlabel('world x (m)'); ax.set_ylabel('world y (m)')
    ax.set_title(f'{slam_type} — trajectory\n'
                 f'ATE RMSE {ate_rmse * 100:.1f} cm  '
                 f'({ate_rmse_full_pct:.2f}% of full path, {ate_rmse_eval_pct:.2f}% of eval path)')
    ax.axis('equal'); ax.legend(loc='best', fontsize=8); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(rundir / 'fig_slam_trajectory.png', dpi=130)
    plt.close(fig)

    # ── fig 3: per-axis error ────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, (ax, lbl, col) in enumerate(zip(
            axes, ['x error (m)', 'y error (m)', 'z error (m)'],
            ['#1f77b4', '#2ca02c', '#ff7f0e'])):
        ax.plot(t_rel, err_xyz[:, i], color=col, lw=1.2)
        ax.axhline(0, color='k', lw=0.8)
        ax.set_ylabel(lbl); ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('time (s)')
    axes[0].set_title(f'{slam_type} — per-axis translation error (Umeyama-aligned)')
    fig.tight_layout()
    fig.savefig(rundir / 'fig_slam_error_xyz.png', dpi=130)
    plt.close(fig)

    # ── TUM files + evo_traj XY plot ─────────────────────────────
    # Write Umeyama-aligned SLAM and GT as TUM so evo_traj can read them.
    # TUM format: timestamp tx ty tz qx qy qz qw  (identity quaternion for GT/aligned)
    slam_tum = rundir / 'slam_traj.tum'
    gt_tum   = rundir / 'gt_traj.tum'
    with open(slam_tum, 'w') as f:
        for i, t in enumerate(slam_t):
            x, y, z = slam_aligned[i]
            f.write(f"{t:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1\n")
    with open(gt_tum, 'w') as f:
        for i, t in enumerate(slam_t):
            x, y, z = gt_interp[i]
            f.write(f"{t:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1\n")

    # Try to run evo_traj from the SLAM-Former venv (same venv used by benchmark_euroc.py).
    script_dir = Path(__file__).resolve().parent
    evo_bin = script_dir.parent / 'SLAM-Former' / 'venv' / 'bin' / 'evo_traj'
    traj_xy = rundir / 'traj_xy.png'
    if evo_bin.exists():
        cmd = (
            f"{evo_bin} tum {slam_tum} --ref {gt_tum} "
            f"--plot_mode xy --save_plot {traj_xy} --no_warnings -s"
        )
        result = subprocess.run(['bash', '-lc', cmd], capture_output=True, text=True)
        # evo sometimes appends suffixes; find what was actually written.
        if not traj_xy.exists():
            for candidate in sorted(rundir.glob('traj_xy*.png')):
                traj_xy = candidate
                break
        if traj_xy.exists():
            print(f"  evo_traj → {traj_xy.name}")
        else:
            print(f"  evo_traj ran but no PNG found ({result.stderr.strip()[:120]})")
    else:
        print(f"  evo_traj not found at {evo_bin} — skipping traj_xy.png")

    print(f"\nwrote slam_metrics.csv + fig_slam_*.png + traj_xy.png to {rundir}\n")


if __name__ == '__main__':
    main()
