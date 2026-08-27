#!/usr/bin/env python3
"""Evaluate every bag in a matrix run and emit results.csv + report.txt.

EVALUATION WINDOW (frozen)
    start = max(first non-zero /slam/pose, mission start).
            Before the first non-zero pose, OV2SLAM/ORB-SLAM2 publish exact
            zeros -- that is not a SLAM error, it is SLAM not existing yet, so
            it cannot be scored. The mission-start term is what makes arms
            comparable: it excludes the sideways init leg, which every arm flies
            but which a backend that initialised early would otherwise be scored
            over and a late one would not.
    end   = the Nth SLAM sample after GT speed drops below SPEED_THR on a
            SPEED_WIN displacement window (N = --end-pad, default 3).
            The drone decelerates into the standoff, so the last fraction below
            threshold is still real arrival. Measured: pad=3 moves d_sign from
            3.19 m to 3.16 m (the true resting distance) and changes RMSE by
            <=0.002 m; pad=10 starts leaking parked poses in (+2%).

The window boundary is the single most load-bearing choice in this benchmark --
including the parked tail pulls the Sim(3) fit toward the parking spot and
lowers RMSE by ~10% for free. So every run also reports RMSE at pad 0/3/10, and
the metres travelled during the pad, so the choice is auditable per run instead
of taken on trust.

Scale correction (-s / Sim(3)) is MANDATORY for monocular: map scale is
arbitrary. It is not a tuning knob and is applied identically to every arm.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

SIGN = np.array([35.5, 23.7, 3.2])
SPEED_THR = 0.10      # m/s
SPEED_WIN = 0.5       # s
T_MAX_DIFF = 0.05     # s, GT<->SLAM association tolerance
PAD_VARIANTS = (0, 3, 10)


# ── loading ───────────────────────────────────────────────────────────────
def load_bag(bag):
    gt, sl = [], []
    with AnyReader([Path(bag)]) as r:
        conns = [c for c in r.connections
                 if c.topic in ('/sim/drone_pose', '/slam/pose')]
        for c, ts, raw in r.messages(connections=conns):
            m, t = r.deserialize(raw, c.msgtype), ts * 1e-9
            if c.topic == '/sim/drone_pose':
                d = m.data
                if len(d) >= 5 and not (d[0] == d[1] == d[2] == 0):
                    gt.append((t, d[0], d[1], d[2]))
            else:
                p = m.pose.position
                sl.append((t, p.x, p.y, p.z))
    gt = np.array(gt)
    sl = np.array(sl)
    return gt, sl[np.linalg.norm(sl[:, 1:4], axis=1) > 1e-9]


def load_timing(run):
    """Front-end per-frame timing, on the SAME LAYER for both backends.

    Both emit 'frontend/full_tracking', and that is the only comparable one.
    An earlier version pulled 'tracking/track_total' for ORB-SLAM2 against
    'frontend/full_tracking' for OV2SLAM -- different layers, which understated
    ORB-SLAM2 by ~3x (9.44 ms vs its true 29.34 ms) and produced a bogus
    'ORB2 11.4 ms vs OV2fast 4.2 ms' comparison.

    Returns (timestamps, frontend_ms, extras) where extras carries the other
    layers so the inner-vs-outer cost split stays visible: for ORB-SLAM2,
    tracking/track_total is only ~28% of wrapper/callback_total, and which one
    you quote changes the claim entirely.
    """
    for fn in ('ov2slam_timing_events.csv', 'timing_events.csv'):
        f = run / fn
        if not f.exists():
            continue
        cats = {}
        for row in csv.DictReader(open(f)):
            cats.setdefault(row.get('category'), []).append(
                (float(row['wall_ts']), float(row['duration_ms'])))
        if 'frontend/full_tracking' not in cats:
            continue
        prim = cats['frontend/full_tracking']
        ts = np.array([a for a, _ in prim])
        dur = np.array([b for _, b in prim])
        extras = {}
        for k in ('wrapper/callback_total', 'tracking/track_total',
                  'tracking/track_local_map', 'backend/local_ba'):
            if k in cats:
                extras[k] = float(np.mean([b for _, b in cats[k]]))
        return ts, dur, extras
    return None, None, {}


def load_cpu(run):
    f = run / 'slam_process_cpu.csv'
    if not f.exists():
        return None
    rows = list(csv.DictReader(open(f)))
    cpu = np.array([float(r['cpu_percent']) for r in rows])
    mem = np.array([float(r['mem_mb']) for r in rows])
    thr = np.array([float(r['threads']) for r in rows])
    return cpu, mem, thr


def load_threads(run):
    f = run / 'slam_thread_cpu.csv'
    if not f.exists():
        return {}
    agg = {}
    for r in csv.DictReader(open(f)):
        agg.setdefault(r['thread_name'], []).append(float(r['cpu_percent']))
    return {k: float(np.mean(v)) for k, v in agg.items()}


# ── geometry ──────────────────────────────────────────────────────────────
def umeyama(G, S):
    mG, mS = G.mean(0), S.mean(0)
    Gc, Sc = G - mG, S - mS
    U, D, Vt = np.linalg.svd((Gc.T @ Sc) / len(S))
    W = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        W[2, 2] = -1
    R = U @ W @ Vt
    var = np.sum(Sc ** 2) / len(S)
    s = np.trace(np.diag(D) @ W) / var if var > 1e-12 else float('nan')
    err = np.linalg.norm((s * (R @ S.T)).T + (mG - s * R @ mS) - G, axis=1)
    return s, err


def gt_speed(gt):
    """Speed on a SPEED_WIN displacement window over an interpolated uniform
    grid. Differentiating the raw stream sample-to-sample is useless here: its
    timestamps jitter down to sub-millisecond gaps, which produce 100+ m/s
    spikes out of millimetre moves."""
    t, X = gt[:, 0], gt[:, 1:4]
    tg = np.arange(t[0], t[-1], 0.05)
    Xg = np.column_stack([np.interp(tg, t, X[:, i]) for i in range(3)])
    k = max(1, int(SPEED_WIN / 0.05))
    spd = np.r_[np.linalg.norm(Xg[k:] - Xg[:-k], axis=1) / SPEED_WIN, [0] * k]
    return tg, spd


def associate(gt, sl_win):
    P = [(gt[j, 1:4], sl_win[i, 1:4])
         for i in range(len(sl_win))
         for j in [int(np.argmin(np.abs(gt[:, 0] - sl_win[i, 0])))]
         if abs(gt[j, 0] - sl_win[i, 0]) <= T_MAX_DIFF]
    if not P:
        return None, None
    return np.array([p[0] for p in P]), np.array([p[1] for p in P])


def window_end(sl, t_thr, pad):
    idx = np.where(sl[:, 0] > t_thr)[0]
    if pad <= 0 or not len(idx):
        return t_thr
    return sl[min(idx[0] + pad - 1, len(sl) - 1), 0]


# ── per-run metrics ───────────────────────────────────────────────────────
def evaluate(run_dir, end_pad):
    run = Path(run_dir)
    gt, sl = load_bag(run / 'bag')
    if len(sl) < 10:
        return {'error': 'SLAM never initialised (%d non-zero poses)' % len(sl)}

    t_slam_first = sl[0, 0]
    tg, spd = gt_speed(gt)
    moving = tg[spd > SPEED_THR]
    if not len(moving):
        return {'error': 'no GT motion above %.2f m/s' % SPEED_THR}
    t_thr = moving[-1]

    # IDENTICAL WINDOW FOR EVERY ARM: score the MISSION, from the moment the drone
    # starts translating toward the sign, not from each backend's own first pose.
    #
    # Starting at first-pose made the scored path a function of WHERE a backend
    # happened to initialise: ORB-SLAM3 (init 0.29 m) was scored over 36.8 m while
    # OV2SLAM-fast (init 10.75 m) was scored over 25.7 m of the SAME flight. The
    # arm that initialised worst got the shortest ruler, which flatters it.
    #
    # The init cycle is a pre-mission calibration manoeuvre and is excluded for
    # everyone. Initialisation quality is reported separately as init_dist_m, and
    # the orchestrator now aborts any trial whose backend has not initialised by
    # the end of that cycle -- so every scored run enters the mission with a map.
    X_all = gt[:, 1:4]
    x0 = X_all[0, 0]
    fwd = np.where(X_all[:, 0] - x0 > 0.15)[0]
    t_mission = gt[fwd[0], 0] if len(fwd) else t_slam_first
    t_init = max(t_slam_first, t_mission)

    out = {}

    # RMSE at each pad variant -- the built-in sensitivity check.
    for pad in PAD_VARIANTS:
        t_end = window_end(sl, t_thr, pad)
        G, S = associate(gt, sl[(sl[:, 0] >= t_init) & (sl[:, 0] <= t_end)])
        if G is None:
            out['ape_rmse_pad%d' % pad] = float('nan')
            continue
        _, e = umeyama(G, S)
        out['ape_rmse_pad%d' % pad] = float(np.sqrt(np.mean(e ** 2)))

    # The reported window.
    t_end = window_end(sl, t_thr, end_pad)
    sw = sl[(sl[:, 0] >= t_init) & (sl[:, 0] <= t_end)]
    G, S = associate(gt, sw)
    if G is None:
        return {'error': 'no GT/SLAM pose pairs within %.3f s' % T_MAX_DIFF}

    s, e = umeyama(G, S)
    out.update({
        'n_poses': len(G),
        'ape_rmse': float(np.sqrt(np.mean(e ** 2))),
        'ape_mean': float(e.mean()),
        'ape_median': float(np.median(e)),
        'ape_max': float(e.max()),
        'ape_std': float(e.std()),
        'umeyama_scale': float(s),
        'window_s': float(t_end - t_init),
    })

    # Scale drift: fit scale on each half. A backend whose map is being
    # progressively re-scaled shows up here even when whole-window RMSE looks
    # fine, and a large value also means the geometry is not pinning scale down.
    h = len(G) // 2
    if h > 10:
        s1, _ = umeyama(G[:h], S[:h])
        s2, _ = umeyama(G[h:], S[h:])
        out['scale_first_half'] = float(s1)
        out['scale_second_half'] = float(s2)
        out['scale_drift_pct'] = float(100 * abs(s2 - s1) / abs(s1)) if s1 else float('nan')

    # Init cost, in seconds AND in metres flown -- metres is the one that
    # matters, since it is mission distance travelled with no map.
    X = gt[:, 1:4]
    d_from_start = np.linalg.norm(X - X[0], axis=1)
    i_move = int(np.argmax(d_from_start > 0.05))
    j_init = int(np.argmin(np.abs(gt[:, 0] - t_init)))
    cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(X, axis=0), axis=1))]
    # SPLIT, because these are different things and mixing them publishes a
    # harness bug as an algorithm property.
    #
    #   harness_delay_*  mission start -> backend's FIRST FRAME. Ours: it is
    #                    startup_delay_sec plus container/vocabulary load. Says
    #                    nothing about the algorithm. Measured at 8 s for
    #                    ORB-SLAM2 vs 1 s for OV2SLAM, which alone made
    #                    ORB-SLAM2 look like it needed 19.7 m to initialise.
    #   init_*           backend's first frame -> its first pose. THIS is the
    #                    algorithmic property: how much motion the initialiser
    #                    actually needs once it can see. Only this is reportable.
    #
    # BOTH are measured to t_slam_first (the backend's FIRST POSE), never to
    # t_init. t_init is the SCORING window start, max(t_slam_first, t_mission),
    # and using it here silently measured the wrong thing: any backend that
    # initialised during the init leg -- which the Phase 0 protocol now requires
    # of every arm -- got t_init == t_mission, so init_dist_m came out as the
    # LENGTH OF THE INIT LEG rather than the distance its initialiser needed.
    # That is why three different backends reported 3.40 / 3.30 / 3.40 m: the
    # column was measuring the protocol, not the algorithm. Only a backend that
    # initialised LATE (OV2SLAM-fast, 10.75 m) ever produced a real value.
    ts_fe, _, _ = load_timing(run)
    t_first_frame = float(ts_fe.min()) if ts_fe is not None and len(ts_fe) else float('nan')
    j_first_pose = int(np.argmin(np.abs(gt[:, 0] - t_slam_first)))
    if not math.isnan(t_first_frame):
        j_ff = int(np.argmin(np.abs(gt[:, 0] - t_first_frame)))
        # Clamped at zero, matching harness_delay_m, which always was. Leaving
        # the seconds signed printed "-12.455 s" beside "0.000 m" for the same
        # event. A NEGATIVE value is not a delay at all: it means the backend
        # was already consuming frames before the drone moved, which is exactly
        # what the orchestrator's readiness gate now guarantees. That healthy
        # margin is reported separately rather than as a negative lag.
        out['harness_delay_s'] = float(max(0.0, t_first_frame - gt[i_move, 0]))
        out['harness_delay_m'] = float(max(0.0, cum[j_ff] - cum[i_move]))
        out['ready_margin_s'] = float(max(0.0, gt[i_move, 0] - t_first_frame))
        out['init_time_s'] = float(max(0.0, t_slam_first - t_first_frame))
        out['init_dist_m'] = float(max(0.0, cum[j_first_pose] - cum[j_ff]))
    else:
        out['harness_delay_s'] = float('nan')
        out['harness_delay_m'] = float('nan')
        out['ready_margin_s'] = float('nan')
        out['init_time_s'] = float(max(0.0, t_slam_first - gt[i_move, 0]))
        out['init_dist_m'] = float(max(0.0, cum[j_first_pose] - cum[i_move]))
    # Total, for provenance only -- never quote this as an algorithm number.
    out['first_pose_dist_from_start_m'] = float(max(0.0, cum[j_init] - cum[i_move]))

    # Coverage. Temporal gaps catch a mid-run tracking loss; poses/frames
    # catches the backend silently skipping frames it did receive.
    dt = np.diff(sw[:, 0])
    med = float(np.median(dt)) if len(dt) else float('nan')
    gaps = dt[dt > 3 * med] if len(dt) else np.array([])
    out['pose_rate_hz'] = float(len(sw) / (t_end - t_init))
    out['n_gaps'] = int(len(gaps))
    out['max_gap_s'] = float(gaps.max()) if len(gaps) else 0.0

    # Mission outcome + how much the pad actually bought.
    out['d_target_m'] = float(np.linalg.norm(G[-1] - SIGN))
    j_thr = int(np.argmin(np.abs(gt[:, 0] - t_thr)))
    j_end = int(np.argmin(np.abs(gt[:, 0] - t_end)))
    out['pad_travel_m'] = float(abs(cum[j_end] - cum[j_thr]))
    if out['pad_travel_m'] > 0.05:
        out['warn'] = 'pad travel %.3f m > 5 cm: end pad may be truncating a slow arrival' % out['pad_travel_m']

    # Path length ratio: SLAM consistently over-reports distance flown, because
    # per-frame noise integrates into fictitious arc length.
    Lg = float(np.sum(np.linalg.norm(np.diff(G, axis=0), axis=1)))
    Ls = float(s * np.sum(np.linalg.norm(np.diff(S, axis=0), axis=1)))
    out['gt_path_m'] = Lg
    out['path_ratio_pct'] = 100 * Ls / Lg if Lg else float('nan')
    # The normalised figure. Raw RMSE is only comparable between arms that were
    # scored over the same distance, which late initialisation silently breaks.
    out['err_per_m_pct'] = 100 * out['ape_rmse'] / Lg if Lg else float('nan')

    # Front-end timing + delivery rate. Capacity is 1/latency: what the backend
    # COULD sustain, versus what the pipe actually delivers.
    ts, dur, extras = load_timing(run)
    for k, v in extras.items():
        out['ms_' + k.replace('/', '_')] = v
    if ts is not None and len(ts) > 2:
        span = ts[-1] - ts[0]
        out['frames'] = len(ts)
        out['input_rate_hz'] = float(len(ts) / span) if span else float('nan')
        out['fe_ms_mean'] = float(dur.mean())
        out['fe_ms_std'] = float(dur.std())
        out['fe_ms_p50'] = float(np.percentile(dur, 50))
        out['fe_ms_p95'] = float(np.percentile(dur, 95))
        out['fe_ms_p99'] = float(np.percentile(dur, 99))
        out['capacity_hz'] = float(1000.0 / dur.mean()) if dur.mean() else float('nan')
        in_win = ((ts >= t_init) & (ts <= t_end)).sum()
        out['poses_per_frame_pct'] = float(100 * len(sw) / in_win) if in_win else float('nan')

    cpu = load_cpu(run)
    if cpu is not None:
        c, mem, thr = cpu
        out['cpu_mean_pct'] = float(c.mean())
        out['cpu_max_pct'] = float(c.max())
        out['mem_max_mb'] = float(mem.max())
        out['threads_max'] = float(thr.max())

    th = load_threads(run)
    if th:
        top = sorted(th.items(), key=lambda kv: -kv[1])[:6]
        out['threads_top'] = '; '.join('%s=%.1f' % (k, v) for k, v in top)

    return out


# ── reporting ─────────────────────────────────────────────────────────────
def mstd(vals):
    v = [x for x in vals if x is not None and not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return float('nan'), float('nan')
    return float(np.mean(v)), float(np.std(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--end-pad', type=int, default=3)
    args = ap.parse_args()

    base = Path(args.run_dir)
    # utf-8-sig: PowerShell's Out-File -Encoding utf8 writes a BOM.
    manifest = json.loads((base / 'manifest.json').read_text(encoding='utf-8-sig'))
    runs = manifest['runs']
    repo = base.parent.parent

    rows = []
    for r in runs:
        if r.get('status') != 'ok' or not r.get('bag'):
            rows.append(dict(r, error=r.get('status', 'no bag')))
            continue
        try:
            m = evaluate(repo / 'bags' / r['bag'], args.end_pad)
        except Exception as exc:
            m = {'error': '%s: %s' % (type(exc).__name__, exc)}
        rows.append(dict(r, **m))

    keys, seen = [], set()
    for r in rows:
        for k in r:
            if k not in seen:
                seen.add(k)
                keys.append(k)
    with open(base / 'results.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    L = []
    L.append('=' * 108)
    L.append('SLAM BENCHMARK MATRIX  %s   vx=%s m/s   end_pad=%d samples'
             % (manifest['stamp'], manifest['vx'], args.end_pad))
    L.append('window: max(first non-zero /slam/pose, mission start)  ->  '
             '%d samples past GT speed < %.2f m/s'
             % (args.end_pad, SPEED_THR))
    L.append('APE: Sim(3) Umeyama (scale correction mandatory for mono), assoc tol %.3f s'
             % T_MAX_DIFF)
    L.append('=' * 108)

    # gt_path_m and err_per_m sit directly under RMSE on purpose. A backend that
    # initialises late is scored over a SHORTER path, so a lower RMSE can mean
    # "measured less", not "tracked better" -- measured: ORB-SLAM2 initialised
    # 19.7 m into the mission vs OV2SLAM-accurate at 1.0 m, which made ORB-SLAM2
    # look 2x better on raw RMSE while being worse per metre flown.
    spec = [('ape_rmse', 'APE RMSE m'), ('gt_path_m', 'scored path m'),
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

    for arm in dict.fromkeys(r['arm'] for r in rows):
        sub = [r for r in rows if r['arm'] == arm]
        good = [r for r in sub if 'error' not in r]
        L.append('')
        L.append('%-14s  scene=%-18s  VALID %d/%d'
                 % (arm, sub[0].get('scene', '?'), len(good), len(sub)))
        L.append('-' * 108)
        if not good:
            for r in sub:
                L.append('    FAILED: %s' % r.get('error'))
            continue
        for key, lab in spec:
            m, s = mstd([r.get(key) for r in good])
            if not math.isnan(m):
                L.append('    %-16s %10.3f  +/- %.3f' % (lab, m, s))
        L.append('    %-16s %s' % ('pad sensitivity',
                 '  '.join('pad%d=%.3f' % (p, mstd([r.get('ape_rmse_pad%d' % p) for r in good])[0])
                           for p in PAD_VARIANTS)))
        if good[0].get('threads_top'):
            L.append('    %-16s %s' % ('top threads', good[0]['threads_top']))
        for r in sub:
            if 'error' in r:
                L.append('    FAILED t%s: %s' % (r.get('trial'), r.get('error')))
            elif r.get('warn'):
                L.append('    WARN   t%s: %s' % (r.get('trial'), r.get('warn')))

    L.append('')
    L.append('=' * 108)
    L.append('per-run rows -> %s' % (base / 'results.csv'))
    txt = '\n'.join(L)
    (base / 'report.txt').write_text(txt)
    print(txt)


if __name__ == '__main__':
    main()
