#!/usr/bin/env python3
"""Turn the recorded bags of one matrix run into results.csv + report.txt.

WHAT THIS DOES, IN ORDER, FOR EACH TRIAL
    1. Read the ground-truth drone path and the SLAM path out of the bag.
    2. Decide the scoring window (which slice of the flight is scored).
    3. Hand that windowed pair to evo_ape -- the standard trajectory tool -- to
       get the accuracy number. This script does NOT implement its own
       alignment or error metric; evo does, exactly as it did for the offline
       EuRoC evaluation, so the accuracy figure is comparable and not bespoke.
    4. Compute the systems metrics evo does not do: initialisation distance,
       front-end latency, throughput, CPU. These stay in this file because they
       are HIL-specific and have no evo equivalent.

ACCURACY IS evo_ape, NOT CUSTOM CODE
    evo_ape tum <gt> <est> -a -s
      -a  align with Umeyama (rotation + translation)
      -s  also solve for scale -- MANDATORY for monocular, whose map scale is
          arbitrary. Applied identically to every arm; not a tuning knob.
    We read evo's stats.json (rmse/mean/median/max/std) and recover the scale
    factor from its saved Sim(3) alignment matrix. The only thing this file
    decides about accuracy is WHICH POSES go in -- the window below -- and that
    choice is made auditable by reporting the error at three window paddings.

THE SCORING WINDOW (the single most load-bearing choice here)
    start = max(first real SLAM pose, mission start).
            Before the first pose the backend publishes zeros: that is SLAM not
            existing yet, not SLAM being wrong, so it cannot be scored. The
            mission-start term excludes the sideways initialisation leg, which
            every arm flies; without it, a backend that initialises early would
            be scored over more of that leg than one that initialises late, and
            the ranking would reflect the protocol instead of the algorithm.
    end   = the Nth SLAM sample after the drone stops (ground-truth speed below
            ARRIVAL_SPEED_MPS on a short displacement window), N = --end-pad.
            Including the parked tail pulls the Sim(3) fit toward the parking
            spot and lowers RMSE ~10% for free, so the error is reported at
            padding 0 / 3 / 10 and the metres travelled during the pad, making
            the choice checkable per run rather than trusted.
"""
import argparse
import csv
import io
import json
import math
import os
import subprocess
import tempfile
import zipfile
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

# The physical target the drone approaches, in world metres. d_target_m below is
# the distance from the final scored ground-truth position to this point.
SIGN_POSITION_M = np.array([35.5, 23.7, 3.2])

ARRIVAL_SPEED_MPS = 0.10   # below this ground-truth speed, the drone has arrived
ARRIVAL_SPEED_WINDOW_S = 0.5   # displacement window the speed is measured over
GT_SLAM_MATCH_TOL_S = 0.05     # max time gap for a ground-truth<->SLAM pose pair
WINDOW_PADDINGS = (0, 3, 10)   # end-pad values the RMSE is reported at

# The ground-truth topic carries [x y z ...]; a pose of exact zeros means the
# publisher had no fix yet, and a SLAM pose of exact zeros means "not
# initialised". Both are dropped as non-poses rather than scored as the origin.
GT_TOPIC = '/sim/drone_pose'
SLAM_TOPIC = '/slam/pose'


# ════════════════════════════════════════════════════════════════════════════
#  Reading the bag and the sidecar CSVs
# ════════════════════════════════════════════════════════════════════════════

def read_trajectories_from_bag(bag_dir):
    """Return (ground_truth, slam) as two Nx4 arrays of [time_s, x, y, z].

    SLAM rows of all-zero position are dropped: they are frames published before
    the map initialised, not a pose at the world origin.
    """
    ground_truth, slam = [], []
    with AnyReader([Path(bag_dir)]) as reader:
        wanted = [c for c in reader.connections if c.topic in (GT_TOPIC, SLAM_TOPIC)]
        for conn, stamp_ns, raw in reader.messages(connections=wanted):
            msg = reader.deserialize(raw, conn.msgtype)
            time_s = stamp_ns * 1e-9
            if conn.topic == GT_TOPIC:
                d = msg.data
                if len(d) >= 5 and not (d[0] == d[1] == d[2] == 0):
                    ground_truth.append((time_s, d[0], d[1], d[2]))
            else:
                p = msg.pose.position
                slam.append((time_s, p.x, p.y, p.z))
    ground_truth = np.array(ground_truth)
    slam = np.array(slam)
    if len(slam):
        slam = slam[np.linalg.norm(slam[:, 1:4], axis=1) > 1e-9]
    return ground_truth, slam


def read_frontend_timing(run_dir):
    """Return (timestamps, frontend_ms, other_layer_means) for the front end.

    Every backend emits a 'frontend/full_tracking' event per frame, and that is
    the ONLY layer comparable across backends: an earlier version read
    'tracking/track_total' for ORB-SLAM2 against 'frontend/full_tracking' for
    OV2SLAM -- different layers -- which understated ORB-SLAM2 by ~3x. The other
    layers are returned as means so the inner-vs-outer cost split stays visible.
    """
    for filename in ('ov2slam_timing_events.csv', 'timing_events.csv'):
        path = run_dir / filename
        if not path.exists():
            continue
        events_by_category = {}
        for row in csv.DictReader(open(path)):
            events_by_category.setdefault(row.get('category'), []).append(
                (float(row['wall_ts']), float(row['duration_ms'])))
        if 'frontend/full_tracking' not in events_by_category:
            continue
        frontend = events_by_category['frontend/full_tracking']
        timestamps = np.array([t for t, _ in frontend])
        frontend_ms = np.array([ms for _, ms in frontend])
        other_layer_means = {}
        for layer in ('wrapper/callback_total', 'tracking/track_total',
                      'tracking/track_local_map', 'backend/local_ba'):
            if layer in events_by_category:
                other_layer_means[layer] = float(
                    np.mean([ms for _, ms in events_by_category[layer]]))
        return timestamps, frontend_ms, other_layer_means
    return None, None, {}


def read_cpu_samples(run_dir):
    """Return (cpu_percent, mem_mb, thread_count) sample arrays, or None."""
    path = run_dir / 'slam_process_cpu.csv'
    if not path.exists():
        return None
    rows = list(csv.DictReader(open(path)))
    cpu_percent = np.array([float(r['cpu_percent']) for r in rows])
    mem_mb = np.array([float(r['mem_mb']) for r in rows])
    thread_count = np.array([float(r['threads']) for r in rows])
    return cpu_percent, mem_mb, thread_count


def read_thread_cpu(run_dir):
    """Return {thread_name: mean_cpu_percent} for the per-thread breakdown."""
    path = run_dir / 'slam_thread_cpu.csv'
    if not path.exists():
        return {}
    samples_by_thread = {}
    for row in csv.DictReader(open(path)):
        samples_by_thread.setdefault(row['thread_name'], []).append(
            float(row['cpu_percent']))
    return {name: float(np.mean(v)) for name, v in samples_by_thread.items()}


# ════════════════════════════════════════════════════════════════════════════
#  Accuracy: delegated to evo_ape
# ════════════════════════════════════════════════════════════════════════════

def find_evo(repo_root, tool='evo_ape'):
    """Locate an evo executable (`evo_ape` or `evo_rpe`).

    Prefer the project's own evo virtualenv (the same one the offline EuRoC
    evaluation used); then a TOOL-name env override (EVO_APE / EVO_RPE); then the
    user-local install; then whatever is on PATH. Checked in that order so every
    caller -- the matrix report and PAPER_IMAGES_HIL/generate.py alike --
    resolves the same binary regardless of its working directory.
    """
    override = os.environ.get(tool.upper())   # EVO_APE or EVO_RPE
    candidates = [
        repo_root / 'benchmarks' / '.evo_venv' / 'bin' / tool,
        Path(override) if override else None,
        Path.home() / '.local' / 'bin' / tool,
    ]
    for c in candidates:
        if c and c.exists():
            return str(c)
    return tool


def find_evo_ape(repo_root):
    """Back-compat shim: PAPER_IMAGES_HIL/generate.py may call this by name."""
    return find_evo(repo_root, 'evo_ape')


def _write_tum(path, poses):
    """Write an Nx4 [t,x,y,z] array as a TUM trajectory file.

    Orientation is not recorded in the bag, so identity quaternions are written.
    evo_ape's translation-part APE and its Umeyama alignment use positions only,
    so the orientation column does not affect the result (verified against a
    synthetic scaled trajectory: Sim(3) recovered RMSE ~0).
    """
    with open(path, 'w') as fh:
        for t, x, y, z in poses:
            fh.write('%.9f %.6f %.6f %.6f 0 0 0 1\n' % (t, x, y, z))


def run_evo(evo_bin, reference, estimate, workdir, tag, extra_args=()):
    """Align `estimate` to `reference` with an evo tool (Sim(3)) and return stats.

    Used for both evo_ape (global error) and evo_rpe (local drift) -- they share
    an identical CLI and result-zip format, so `extra_args` carries the only
    difference (e.g. RPE's --delta 1 --delta_unit m).

    Returns {rmse, mean, median, max, std, scale, n_matched}. `scale` is the
    Sim(3) scale factor evo applied to the estimate to match the reference,
    recovered as the cube root of the determinant of evo's 3x3 alignment block
    (det(sR) = s^3 for a proper rotation R). Raises on evo failure.
    """
    ref_path = workdir / ('%s_reference.tum' % tag)
    est_path = workdir / ('%s_estimate.tum' % tag)
    zip_path = workdir / ('%s_result.zip' % tag)
    _write_tum(ref_path, reference)
    _write_tum(est_path, estimate)

    completed = subprocess.run(
        [evo_bin, 'tum', str(ref_path), str(est_path),
         '-a', '-s',
         '--t_max_diff', str(GT_SLAM_MATCH_TOL_S),
         *extra_args,
         '--save_results', str(zip_path)],
        capture_output=True, text=True)
    if completed.returncode != 0 or not zip_path.exists():
        raise RuntimeError('%s failed: %s'
                           % (Path(evo_bin).name,
                              (completed.stderr or completed.stdout)[-300:]))

    with zipfile.ZipFile(zip_path) as z:
        stats = json.loads(z.read('stats.json'))
        sim3 = np.load(io.BytesIO(z.read('alignment_transformation_sim3.npy')))
        n_matched = len(np.load(io.BytesIO(z.read('timestamps.npy'))))
    scale = float(abs(np.linalg.det(sim3[:3, :3])) ** (1.0 / 3.0))
    return dict(rmse=stats['rmse'], mean=stats['mean'], median=stats['median'],
                max=stats['max'], std=stats['std'], scale=scale,
                n_matched=n_matched)


# ════════════════════════════════════════════════════════════════════════════
#  The scoring window
# ════════════════════════════════════════════════════════════════════════════

def ground_truth_speed(ground_truth):
    """Return (time, speed) for the ground-truth path on a uniform 0.05 s grid.

    Speed is measured as displacement over a short window, not sample-to-sample:
    ground-truth timestamps jitter down to sub-millisecond gaps, so naive
    differencing turns millimetre moves into 100+ m/s spikes. Resampling onto a
    uniform grid first removes that entirely.
    """
    time, xyz = ground_truth[:, 0], ground_truth[:, 1:4]
    grid = np.arange(time[0], time[-1], 0.05)
    resampled = np.column_stack([np.interp(grid, time, xyz[:, i]) for i in range(3)])
    step = max(1, int(ARRIVAL_SPEED_WINDOW_S / 0.05))
    displacement = np.linalg.norm(resampled[step:] - resampled[:-step], axis=1)
    speed = np.r_[displacement / ARRIVAL_SPEED_WINDOW_S, [0] * step]
    return grid, speed


def mission_start_time(ground_truth, fallback_time):
    """Time the drone first translates toward the sign (forward > 0.15 m).

    This is where scoring begins for every arm, so the pre-mission
    initialisation leg is excluded identically regardless of when each backend
    happened to initialise.
    """
    forward = ground_truth[:, 1] - ground_truth[0, 1]
    moved = np.where(forward > 0.15)[0]
    return ground_truth[moved[0], 0] if len(moved) else fallback_time


def arrival_time(ground_truth):
    """Last time the ground-truth speed was above the arrival threshold, or None."""
    grid, speed = ground_truth_speed(ground_truth)
    moving = grid[speed > ARRIVAL_SPEED_MPS]
    return moving[-1] if len(moving) else None


def scoring_window_end(slam, arrival, padding):
    """The SLAM timestamp `padding` samples after arrival (or at arrival if 0)."""
    after = np.where(slam[:, 0] > arrival)[0]
    if padding <= 0 or not len(after):
        return arrival
    return slam[min(after[0] + padding - 1, len(slam) - 1), 0]


def slice_by_time(track, start, end):
    return track[(track[:, 0] >= start) & (track[:, 0] <= end)]


def path_length_m(track):
    """Arc length of an Nx4 [t,x,y,z] track."""
    if len(track) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(track[:, 1:4], axis=0), axis=1)))


# ════════════════════════════════════════════════════════════════════════════
#  Per-trial evaluation
# ════════════════════════════════════════════════════════════════════════════

def evaluate(run_dir, end_pad, evo_ape_bin=None):
    """Evaluate one trial's bag. Public entry point; also used by generate.py.

    evo_ape_bin is resolved from the repo root when not supplied, so callers
    that only have a bag path (generate.py) do not need to know where evo lives.
    """
    run_dir = Path(run_dir)
    if evo_ape_bin is None:
        evo_ape_bin = find_evo(run_dir.parent.parent, 'evo_ape')
    ground_truth, slam = read_trajectories_from_bag(run_dir / 'bag')
    if len(slam) < 10:
        return {'error': 'SLAM never initialised (%d non-zero poses)' % len(slam)}

    first_pose_time = slam[0, 0]
    arrival = arrival_time(ground_truth)
    if arrival is None:
        return {'error': 'no GT motion above %.2f m/s' % ARRIVAL_SPEED_MPS}

    # Window start: exclude the init leg (see module docstring).
    score_start = max(first_pose_time, mission_start_time(ground_truth, first_pose_time))

    out = {}
    workdir = Path(tempfile.mkdtemp(prefix='evo_'))
    try:
        # Accuracy at each window padding, so the reader can see how much the
        # parked tail flatters the number. The reported padding (--end-pad)
        # keeps the full stats; the others contribute only their RMSE.
        reported = None
        reported_estimate = None
        reported_end = None
        for padding in sorted(set(WINDOW_PADDINGS) | {end_pad}):
            window_end = scoring_window_end(slam, arrival, padding)
            estimate = slice_by_time(slam, score_start, window_end)
            try:
                stats = run_evo(evo_ape_bin, ground_truth, estimate,
                                    workdir, 'pad%d' % padding)
            except Exception as exc:
                stats = None
                if padding == end_pad:
                    return {'error': 'evo_ape: %s' % exc}
            if padding in WINDOW_PADDINGS:
                out['ape_rmse_pad%d' % padding] = stats['rmse'] if stats else float('nan')
            if padding == end_pad:
                reported, reported_estimate, reported_end = stats, estimate, window_end

        if reported is None:
            return {'error': 'evo_ape produced no result for the reported window'}

        out.update({
            'n_poses': reported['n_matched'],
            'ape_rmse': reported['rmse'],
            'ape_mean': reported['mean'],
            'ape_median': reported['median'],
            'ape_max': reported['max'],
            'ape_std': reported['std'],
            'umeyama_scale': reported['scale'],
            'window_s': float(reported_end - score_start),
        })

        # RPE deliberately NOT computed here yet. It is a relative-POSE metric,
        # and the bag gives this script positions only (identity quaternions in
        # the TUM files). With no real orientation evo cannot form relative
        # poses, so RPE collapses to differencing two short translation vectors
        # whose direction is dominated by per-pose position noise (~APE-sized),
        # producing an uninterpretable ~1.6 m per 1 m. Real orientation IS
        # available (SLAM publishes a full quaternion; GT carries pitch+yaw with
        # roll=0), so proper RPE + rotational error need read_trajectories_from_bag
        # to keep orientation and _write_tum to emit real quaternions. Until then
        # RPE is omitted rather than reported wrong.

        # Scale drift: solve scale on each half of the window. A map being
        # progressively re-scaled shows up here even when whole-window RMSE is
        # fine, and a large value also means the geometry is not pinning scale.
        half = len(reported_estimate) // 2
        if half > 10:
            try:
                first = run_evo(evo_ape_bin, ground_truth,
                                    reported_estimate[:half], workdir, 'half1')
                second = run_evo(evo_ape_bin, ground_truth,
                                     reported_estimate[half:], workdir, 'half2')
                out['scale_first_half'] = first['scale']
                out['scale_second_half'] = second['scale']
                out['scale_drift_pct'] = (
                    100 * abs(second['scale'] - first['scale']) / first['scale']
                    if first['scale'] else float('nan'))
            except Exception:
                pass
    finally:
        for f in workdir.glob('*'):
            f.unlink()
        workdir.rmdir()

    _add_initialisation_cost(out, ground_truth, slam, run_dir,
                             score_start, first_pose_time)
    _add_coverage_and_outcome(out, ground_truth, slam, reported_estimate,
                              arrival, reported_end, score_start,
                              reported['scale'])
    _add_cost_metrics(out, run_dir, reported_estimate, score_start, reported_end)
    return out


def _add_initialisation_cost(out, ground_truth, slam, run_dir,
                             score_start, first_pose_time):
    """Initialisation distance/time, split from harness startup delay.

    harness_delay_*  mission start -> the backend's first frame. This is
                     startup_delay plus container/vocabulary load; it is a
                     harness property, not an algorithm one.
    init_*           first frame -> first pose. This is the algorithmic cost:
                     how much motion the initialiser needs once it can see.
    Both are measured to the FIRST POSE, never to the scoring-window start: any
    backend that initialised during the init leg has window-start == mission
    start, which would make init_dist_m read as the length of the init leg
    (~3.4 m for everyone) instead of the distance the initialiser actually used.
    """
    xyz = ground_truth[:, 1:4]
    moved_index = int(np.argmax(np.linalg.norm(xyz - xyz[0], axis=1) > 0.05))
    first_pose_index = int(np.argmin(np.abs(ground_truth[:, 0] - first_pose_time)))
    cumulative = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]

    frame_times, _, _ = read_frontend_timing(run_dir)
    first_frame_time = (float(frame_times.min())
                        if frame_times is not None and len(frame_times)
                        else float('nan'))

    mission_start_index = int(np.argmin(np.abs(ground_truth[:, 0] - score_start)))
    if not math.isnan(first_frame_time):
        frame_index = int(np.argmin(np.abs(ground_truth[:, 0] - first_frame_time)))
        out['harness_delay_s'] = float(max(0.0, first_frame_time - ground_truth[moved_index, 0]))
        out['harness_delay_m'] = float(max(0.0, cumulative[frame_index] - cumulative[moved_index]))
        out['ready_margin_s'] = float(max(0.0, ground_truth[moved_index, 0] - first_frame_time))
        out['init_time_s'] = float(max(0.0, first_pose_time - first_frame_time))
        out['init_dist_m'] = float(max(0.0, cumulative[first_pose_index] - cumulative[frame_index]))
    else:
        out['harness_delay_s'] = float('nan')
        out['harness_delay_m'] = float('nan')
        out['ready_margin_s'] = float('nan')
        out['init_time_s'] = float(max(0.0, first_pose_time - ground_truth[moved_index, 0]))
        out['init_dist_m'] = float(max(0.0, cumulative[first_pose_index] - cumulative[moved_index]))
    out['first_pose_dist_from_start_m'] = float(
        max(0.0, cumulative[mission_start_index] - cumulative[moved_index]))


def _add_coverage_and_outcome(out, ground_truth, slam, estimate,
                              arrival, window_end, score_start, scale):
    """Tracking gaps, mission outcome, and path-length ratio."""
    # Coverage: a temporal gap catches a mid-run tracking loss.
    gaps_dt = np.diff(estimate[:, 0])
    median_dt = float(np.median(gaps_dt)) if len(gaps_dt) else float('nan')
    gaps = gaps_dt[gaps_dt > 3 * median_dt] if len(gaps_dt) else np.array([])
    out['pose_rate_hz'] = float(len(estimate) / (window_end - score_start))
    out['n_gaps'] = int(len(gaps))
    out['max_gap_s'] = float(gaps.max()) if len(gaps) else 0.0

    # Mission outcome: how close the drone actually parked to the sign, and how
    # far it travelled during the end pad (a large pad-travel means the pad may
    # be truncating a slow arrival).
    final_gt = ground_truth[int(np.argmin(np.abs(ground_truth[:, 0] - window_end))), 1:4]
    out['d_target_m'] = float(np.linalg.norm(final_gt - SIGN_POSITION_M))
    cumulative = np.r_[0, np.cumsum(np.linalg.norm(np.diff(ground_truth[:, 1:4], axis=0), axis=1))]
    at_arrival = int(np.argmin(np.abs(ground_truth[:, 0] - arrival)))
    at_end = int(np.argmin(np.abs(ground_truth[:, 0] - window_end)))
    out['pad_travel_m'] = float(abs(cumulative[at_end] - cumulative[at_arrival]))
    if out['pad_travel_m'] > 0.05:
        out['warn'] = ('pad travel %.3f m > 5 cm: end pad may be truncating a '
                       'slow arrival' % out['pad_travel_m'])

    # Path-length ratio: SLAM over-reports distance because per-frame noise
    # integrates into fictitious arc length. Scale the SLAM path to metres first.
    gt_window = slice_by_time(ground_truth, score_start, window_end)
    gt_path = path_length_m(gt_window)
    slam_path = scale * path_length_m(estimate)
    out['gt_path_m'] = gt_path
    out['path_ratio_pct'] = 100 * slam_path / gt_path if gt_path else float('nan')
    # Error per metre travelled: the only fair way to compare arms scored over
    # slightly different distances.
    out['err_per_m_pct'] = 100 * out['ape_rmse'] / gt_path if gt_path else float('nan')


def _add_cost_metrics(out, run_dir, estimate, score_start, window_end):
    """Front-end latency, throughput, CPU, per-thread breakdown."""
    frame_times, frontend_ms, other_layers = read_frontend_timing(run_dir)
    for layer, mean_ms in other_layers.items():
        out['ms_' + layer.replace('/', '_')] = mean_ms
    if frame_times is not None and len(frame_times) > 2:
        span = frame_times[-1] - frame_times[0]
        out['frames'] = len(frame_times)
        out['input_rate_hz'] = float(len(frame_times) / span) if span else float('nan')
        out['fe_ms_mean'] = float(frontend_ms.mean())
        out['fe_ms_std'] = float(frontend_ms.std())
        out['fe_ms_p50'] = float(np.percentile(frontend_ms, 50))
        out['fe_ms_p95'] = float(np.percentile(frontend_ms, 95))
        out['fe_ms_p99'] = float(np.percentile(frontend_ms, 99))
        # Capacity is 1/latency: what the backend COULD sustain, versus what the
        # pipe actually delivered (input_rate_hz).
        out['capacity_hz'] = float(1000.0 / frontend_ms.mean()) if frontend_ms.mean() else float('nan')
        frames_in_window = ((frame_times >= score_start) & (frame_times <= window_end)).sum()
        out['poses_per_frame_pct'] = (float(100 * len(estimate) / frames_in_window)
                                      if frames_in_window else float('nan'))

    cpu = read_cpu_samples(run_dir)
    if cpu is not None:
        cpu_percent, mem_mb, thread_count = cpu
        out['cpu_mean_pct'] = float(cpu_percent.mean())
        out['cpu_max_pct'] = float(cpu_percent.max())
        out['mem_max_mb'] = float(mem_mb.max())
        out['threads_max'] = float(thread_count.max())

    threads = read_thread_cpu(run_dir)
    if threads:
        top = sorted(threads.items(), key=lambda kv: -kv[1])[:6]
        out['threads_top'] = '; '.join('%s=%.1f' % (name, v) for name, v in top)


# ════════════════════════════════════════════════════════════════════════════
#  Reporting
# ════════════════════════════════════════════════════════════════════════════

def mean_and_std(values):
    clean = [v for v in values
             if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not clean:
        return float('nan'), float('nan')
    return float(np.mean(clean)), float(np.std(clean))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir')
    parser.add_argument('--end-pad', type=int, default=3)
    args = parser.parse_args()

    base = Path(args.run_dir)
    # utf-8-sig: PowerShell's Out-File -Encoding utf8 writes a BOM.
    manifest = json.loads((base / 'manifest.json').read_text(encoding='utf-8-sig'))
    trials = manifest['runs']
    repo_root = base.parent.parent
    evo_ape_bin = find_evo_ape(repo_root)

    rows = []
    for trial in trials:
        if trial.get('status') != 'ok' or not trial.get('bag'):
            rows.append(dict(trial, error=trial.get('status', 'no bag')))
            continue
        try:
            metrics = evaluate(repo_root / 'bags' / trial['bag'],
                               args.end_pad, evo_ape_bin)
        except Exception as exc:
            metrics = {'error': '%s: %s' % (type(exc).__name__, exc)}
        rows.append(dict(trial, **metrics))

    # results.csv: every column any row produced, in first-seen order.
    columns, seen = [], set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                columns.append(key)
    with open(base / 'results.csv', 'w', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    lines = []
    lines.append('=' * 108)
    lines.append('SLAM BENCHMARK MATRIX  %s   vx=%s m/s   end_pad=%d samples'
                 % (manifest['stamp'], manifest['vx'], args.end_pad))
    lines.append('window: max(first non-zero /slam/pose, mission start)  ->  '
                 '%d samples past GT speed < %.2f m/s'
                 % (args.end_pad, ARRIVAL_SPEED_MPS))
    lines.append('APE: evo_ape tum -a -s (Sim(3) Umeyama, scale solved), '
                 'match tol %.3f s' % GT_SLAM_MATCH_TOL_S)
    lines.append('=' * 108)

    # gt_path_m and err_per_m sit under RMSE on purpose: a backend scored over a
    # shorter path can show a lower RMSE while being worse per metre.
    display = [('ape_rmse', 'APE RMSE m'), ('gt_path_m', 'scored path m'),
               ('err_per_m_pct', 'err %/m'), ('ape_median', 'APE med m'),
               ('ape_max', 'APE max m'), ('umeyama_scale', 'scale'),
               ('scale_drift_pct', 'scale drift %'),
               ('harness_delay_s', 'harness lag s'), ('harness_delay_m', 'harness lag m'),
               ('ready_margin_s', 'ready margin s'),
               ('init_time_s', 'init s'), ('init_dist_m', 'init m'),
               ('fe_ms_mean', 'track ms'),
               ('fe_ms_p95', 'track p95'), ('input_rate_hz', 'input Hz'),
               ('capacity_hz', 'capacity Hz'), ('pose_rate_hz', 'pose Hz'),
               ('poses_per_frame_pct', 'pose/frame %'), ('path_ratio_pct', 'path %'),
               ('d_target_m', 'd_target m'), ('cpu_mean_pct', 'CPU mean %'),
               ('cpu_max_pct', 'CPU max %'), ('mem_max_mb', 'mem MB')]

    for arm in dict.fromkeys(row['arm'] for row in rows):
        arm_rows = [row for row in rows if row['arm'] == arm]
        valid = [row for row in arm_rows if 'error' not in row]
        lines.append('')
        lines.append('%-14s  scene=%-18s  VALID %d/%d'
                     % (arm, arm_rows[0].get('scene', '?'), len(valid), len(arm_rows)))
        lines.append('-' * 108)
        if not valid:
            for row in arm_rows:
                lines.append('    FAILED: %s' % row.get('error'))
            continue
        for key, label in display:
            mean, std = mean_and_std([row.get(key) for row in valid])
            if not math.isnan(mean):
                lines.append('    %-16s %10.3f  +/- %.3f' % (label, mean, std))
        lines.append('    %-16s %s' % ('pad sensitivity', '  '.join(
            'pad%d=%.3f' % (p, mean_and_std([row.get('ape_rmse_pad%d' % p) for row in valid])[0])
            for p in WINDOW_PADDINGS)))
        if valid[0].get('threads_top'):
            lines.append('    %-16s %s' % ('top threads', valid[0]['threads_top']))
        for row in arm_rows:
            if 'error' in row:
                lines.append('    FAILED t%s: %s' % (row.get('trial'), row.get('error')))
            elif row.get('warn'):
                lines.append('    WARN   t%s: %s' % (row.get('trial'), row.get('warn')))

    lines.append('')
    lines.append('=' * 108)
    lines.append('per-run rows -> %s' % (base / 'results.csv'))
    report = '\n'.join(lines)
    (base / 'report.txt').write_text(report)
    print(report)


if __name__ == '__main__':
    main()
