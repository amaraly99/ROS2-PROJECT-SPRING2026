#!/usr/bin/env python3
"""
mission_from_visp_csv.py — derive mission-state metrics from the visp telemetry CSV.

WHY: mission_monitor.py subscribes to the LATCHED /visp/state topic, but only after
the blocking stack boot (~17 s), by which time visp_servo is already flying and has
transitioned past SEARCHING/APPROACHING. Its first latched sample is the current
APPROACHING (recorded as initial_state, NOT an event) with t0 = attach-time, so
approach_duration_s collapses to null and time_to_reached_s is measured from the wrong
zero. recompute_mission_metrics.py cannot rescue that batch either — its transitions
list is just [[t, "REACHED"]] and recovers to NO_REACH.

The genuine mission survives in <run>_visp.csv, which visp_servo_node writes itself
from its own boot (monotonic t_mono_ns, one row per control tick), BEFORE it ever
commands motion. That is the boot-aligned ground truth. This module recomputes the
same metric schema as mission_monitor._metrics() straight from that CSV, immune to
when any external monitor attaches.

CLI:
  single:  python3 mission_from_visp_csv.py --visp-csv <f.csv> --output <mission.json>
  batch:   python3 mission_from_visp_csv.py --sweep <dir> [--min-approach 2.0]
           For each <dir>/*_visp.csv: back up <prefix>_mission.json to
           <prefix>_mission.json.latched.bak (once), overwrite it from the CSV,
           re-aggregate each config's <config>_summary.json "mission" block, and
           write <dir>/mission_audit.csv (raw vs recovered per run).
"""
import argparse
import bisect
import csv
import glob
import json
import math
import os
import re
import sys

STATES = ("SEARCHING", "APPROACHING", "REACQUIRE", "REACHED")
DEBOUNCE_S = 1.0


def _segments(rel_rows):
    """rel_rows: list of (t_rel_s, state). Return [[state, t_start_s, t_end_s], ...].

    A segment is a maximal run of equal states. Its end is the timestamp of the first
    sample of the next segment (the transition instant); the last segment ends at the
    final sample. Boot flicker shows up as very short segments and is handled by the
    caller's debounce, not here."""
    segs = []
    for t, s in rel_rows:
        if not segs or segs[-1][0] != s:
            if segs:
                segs[-1][2] = t          # previous segment ended when this state began
            segs.append([s, t, t])
        else:
            segs[-1][2] = t              # extend current segment's end to latest sample
    return segs


def metrics_from_visp_csv(path, debounce_s=DEBOUNCE_S):
    """Return the mission-metric dict (mission_monitor schema) or None if unusable."""
    try:
        with open(path) as f:
            r = csv.reader(f)
            try:
                hdr = next(r)
            except StopIteration:
                return None
            if "state" not in hdr:
                return None
            ti = hdr.index("t_mono_ns") if "t_mono_ns" in hdr else 0
            si = hdr.index("state")
            rows = []
            for row in r:
                try:
                    rows.append((int(row[ti]), row[si].strip()))
                except (ValueError, IndexError):
                    continue
    except OSError:
        return None
    if not rows:
        return None

    t0 = rows[0][0]
    rel = [((t - t0) / 1e9, s) for t, s in rows]
    segs = _segments(rel)

    initial_state = segs[0][0]                      # first row's state (== mission start)
    transitions = [[round(ts, 3), st] for st, ts, _ in segs[1:]]

    def first_time(state):
        for st, ts, _ in segs:
            if st == state:
                return ts
        return None

    t_reach = first_time("REACHED")
    t_first_app = first_time("APPROACHING")

    # approach start = last APPROACHING segment strictly before the first REACHED
    # (naturally selects the sustained approach, skipping boot flicker) — matches
    # recompute_mission_metrics.py "last approach before reach" semantics.
    approach = None
    if t_reach is not None:
        t_app_sustained = None
        for st, ts, _ in segs:
            if ts >= t_reach:
                break
            if st == "APPROACHING":
                t_app_sustained = ts
        if t_app_sustained is not None:
            approach = round(t_reach - t_app_sustained, 3)

    # instability counts: debounce out sub-`debounce_s` segments (boot flicker) so
    # they do not inflate n_lost / n_reacquire.
    n_reacq = sum(1 for st, ts, te in segs
                  if st == "REACQUIRE" and (te - ts) >= debounce_s)
    n_lost = 0
    seen_app = False
    for st, ts, te in segs:
        if st == "APPROACHING":
            seen_app = True
        elif st == "SEARCHING" and seen_app and (te - ts) >= debounce_s:
            n_lost += 1

    dwell = {s: 0.0 for s in STATES}
    for st, ts, te in segs:
        if st in dwell:
            dwell[st] += te - ts

    return {
        "reached": t_reach is not None,
        "time_to_reached_s": round(t_reach, 3) if t_reach is not None else None,
        "approach_duration_s": approach,
        "time_to_first_approach_s": round(t_first_app, 3) if t_first_app is not None else None,
        "n_reacquire": n_reacq,
        "n_lost": n_lost,
        "time_in_state_s": {s: round(v, 3) for s, v in dwell.items()},
        "initial_state": initial_state,
        "final_state": segs[-1][0],
        "n_transitions": len(transitions),
        "transitions": transitions,
        "source": "visp_csv",
    }


def sim_clock(cam_stamps_path):
    """Return (mono_ns[], sim_ns[]) for the CURRENT mission only, or None.

    <run>_cam_stamps.csv pairs the MATLAB sim clock (t_sim_stamp_ns) with the Pi
    monotonic clock (t_recv_mono_ns) — the same clock the visp CSV stamps with. The
    sim clock RESETS to 0 when the model restarts at the start of each run, and the
    frames before that reset are the previous run's frozen final frame. We therefore
    keep only the samples after the LAST reset: sim_t = 0 is the true mission start.
    """
    try:
        with open(cam_stamps_path) as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return None
    mono, sim = [], []
    for r in rows:
        try:
            mono.append(int(r["t_recv_mono_ns"]))
            sim.append(int(r["t_sim_stamp_ns"]))
        except (KeyError, ValueError):
            continue
    if len(mono) < 10:
        return None
    reset = 0
    for i in range(1, len(sim)):
        if sim[i] < sim[i - 1] - 5e8:      # >0.5 s backwards = model restart
            reset = i
    mono, sim = mono[reset:], sim[reset:]
    return (mono, sim) if len(mono) >= 10 else None


def _mono_to_sim_s(t_mono_ns, mono, sim):
    """Linearly interpolate a Pi-monotonic stamp onto the sim clock (seconds)."""
    if t_mono_ns < mono[0] or t_mono_ns > mono[-1]:
        return None
    i = min(max(bisect.bisect_left(mono, t_mono_ns), 1), len(mono) - 1)
    m0, m1, s0, s1 = mono[i - 1], mono[i], sim[i - 1], sim[i]
    s = s0 if m1 == m0 else s0 + (s1 - s0) * (t_mono_ns - m0) / (m1 - m0)
    return s / 1e9


def sim_metrics(visp_csv, cam_stamps_path):
    """Sim-time mission metrics — immune to MATLAB sim-rate drift.

    WHY THIS MATTERS: wall-clock reach conflates the detector with how fast MATLAB
    happened to be running. Measured sim_rate drifts 0.78-0.95x between runs, which
    alone stretched wall reach from 43.7 s to 54.8 s for identical missions. In sim
    time every reaching config lands at ~41 s (actuation-limited, not detection-limited).
    Report the sim-time numbers; treat wall-clock reach as simulator-dependent.
    """
    clk = sim_clock(cam_stamps_path)
    if clk is None:
        return {}
    mono, sim = clk
    try:
        with open(visp_csv) as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return {}
    if not rows:
        return {}

    # Build state segments so we can mirror the wall-clock metric semantics exactly:
    # approach starts at the LAST APPROACHING entry before the first REACHED (i.e. the
    # sustained approach), NOT the first one — otherwise an early stale-scene lock or a
    # mid-mission re-acquire inflates approach_duration.
    segs = []
    for r in rows:
        try:
            t = int(r["t_mono_ns"])
        except (KeyError, ValueError):
            continue
        st = r.get("state")
        if not segs or segs[-1][0] != st:
            segs.append([st, t])

    t_reach = next((t for st, t in segs if st == "REACHED"), None)
    t_app = None
    for st, t in segs:
        if t_reach is not None and t >= t_reach:
            break
        if st == "APPROACHING":
            t_app = t

    reach_sim = _mono_to_sim_s(t_reach, mono, sim) if t_reach else None
    app_sim = _mono_to_sim_s(t_app, mono, sim) if t_app else None
    rate = ((sim[-1] - sim[0]) / (mono[-1] - mono[0])) if mono[-1] != mono[0] else None

    out = {"sim_rate": round(rate, 4) if rate else None}
    # sim clock starts at 0 on the model restart, so the sim stamp IS the mission time.
    if reach_sim is not None:
        out["time_to_reached_sim_s"] = round(reach_sim, 3)
        if app_sim is not None:
            out["approach_duration_sim_s"] = round(reach_sim - app_sim, 3)
    return out


def _write_json(obj, path):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2)
    os.replace(tmp, path)


def _mean_std(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return {"mean": None, "std": None, "n": 0}
    m = sum(vals) / len(vals)
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals)) if len(vals) > 1 else 0.0
    return {"mean": round(m, 3), "std": round(sd, 3), "n": len(vals)}


def _aggregate_mission(missions):
    """Mirror the mission block built in run_hil_experiments.sh's summary heredoc."""
    if not missions:
        return {"runs": 0, "reached_success_rate": None}
    n_reached = sum(1 for m in missions if m.get("reached"))
    return {
        "runs": len(missions),
        "reached_success_rate": round(n_reached / len(missions), 3),
        # PRIMARY mission metric — sim-time, immune to MATLAB sim-rate drift.
        "time_to_reached_sim_s":   _mean_std([m.get("time_to_reached_sim_s") for m in missions]),
        "approach_duration_sim_s": _mean_std([m.get("approach_duration_sim_s") for m in missions]),
        "sim_rate":                _mean_std([m.get("sim_rate") for m in missions]),
        # Wall-clock — retained for reference ONLY. Do not compare across configs:
        # it scales with whatever sim_rate MATLAB happened to run at (0.78-0.95x).
        "time_to_reached_s":        _mean_std([m.get("time_to_reached_s") for m in missions]),
        "approach_duration_s":      _mean_std([m.get("approach_duration_s") for m in missions]),
        "time_to_first_approach_s": _mean_std([m.get("time_to_first_approach_s") for m in missions]),
        "n_reacquire":              _mean_std([m.get("n_reacquire") for m in missions]),
        "n_lost":                   _mean_std([m.get("n_lost") for m in missions]),
    }


def _run_batch(sweep, debounce_s):
    csvs = sorted(glob.glob(os.path.join(sweep, "*_visp.csv")))
    if not csvs:
        sys.exit(f"ERROR: no *_visp.csv found in {sweep}")

    audit_rows = []
    by_config = {}   # config prefix -> list of recovered mission dicts
    n_ok = n_skip = 0

    for csv_path in csvs:
        prefix = os.path.basename(csv_path)[:-len("_visp.csv")]   # e.g. label_run1
        mission_path = os.path.join(sweep, f"{prefix}_mission.json")

        recovered = metrics_from_visp_csv(csv_path, debounce_s)
        if recovered is None:
            print(f"[skip] {prefix}: visp CSV empty/unreadable — left mission.json as-is",
                  file=sys.stderr)
            n_skip += 1
            continue
        recovered.update(sim_metrics(csv_path,
                                     os.path.join(sweep, f"{prefix}_cam_stamps.csv")))

        raw = {}
        if os.path.exists(mission_path):
            try:
                with open(mission_path) as f:
                    raw = json.load(f)
            except (ValueError, OSError):
                raw = {}
            bak = mission_path + ".latched.bak"
            if not os.path.exists(bak):
                _write_json(raw, bak)          # preserve the original once

        _write_json(recovered, mission_path)
        n_ok += 1

        m = re.match(r"(.+)_run\d+$", prefix)
        config = m.group(1) if m else prefix
        by_config.setdefault(config, []).append(recovered)

        raw_trans = raw.get("transitions") or [[None, ""]]
        audit_rows.append({
            "config": config,
            "run": prefix,
            "raw_initial_state": raw.get("initial_state"),
            "raw_time_to_reached_s": raw.get("time_to_reached_s"),
            "raw_approach_duration_s": raw.get("approach_duration_s"),
            "recovered_initial_state": recovered["initial_state"],
            "recovered_time_to_reached_s": recovered["time_to_reached_s"],
            "recovered_approach_duration_s": recovered["approach_duration_s"],
            "recovered_final_state": recovered["final_state"],
        })

    # ── re-aggregate each config's summary.json mission block (in place) ──────────
    for config, missions in by_config.items():
        summary_path = os.path.join(sweep, f"{config}_summary.json")
        if not os.path.exists(summary_path):
            continue
        try:
            with open(summary_path) as f:
                summary = json.load(f)
        except (ValueError, OSError):
            continue
        summary["mission"] = _aggregate_mission(missions)
        _write_json(summary, summary_path)

    # ── audit CSV (non-destructive) ──────────────────────────────────────────────
    if audit_rows:
        audit_path = os.path.join(sweep, "mission_audit.csv")
        with open(audit_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(audit_rows[0].keys()))
            w.writeheader()
            w.writerows(audit_rows)
        print(f"audit -> {audit_path}")

    print(f"\nrewrote {n_ok} mission.json from visp CSV ({n_skip} skipped), "
          f"{len(by_config)} config summaries updated.\n")
    print(f"{'run':<28} {'init':<11} {'reach_s':>8} {'appr_s':>8} {'lost':>4}")
    for row in audit_rows:
        reach = "—" if row["recovered_time_to_reached_s"] is None else f"{row['recovered_time_to_reached_s']:.1f}"
        appr = "—" if row["recovered_approach_duration_s"] is None else f"{row['recovered_approach_duration_s']:.1f}"
        print(f"{row['run']:<28} {row['recovered_initial_state']:<11} {reach:>8} {appr:>8}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--visp-csv", help="single-run visp telemetry CSV")
    ap.add_argument("--cam-stamps", help="single-run cam_stamps CSV (adds sim-time metrics)")
    ap.add_argument("--output", help="single-run mission JSON output path")
    ap.add_argument("--sweep", help="batch: sweep dir with *_visp.csv")
    ap.add_argument("--debounce", type=float, default=DEBOUNCE_S,
                    help="ignore state segments shorter than this (s) for instability "
                         "counts (default 1.0)")
    args = ap.parse_args()

    if args.sweep:
        _run_batch(args.sweep, args.debounce)
        return

    if not (args.visp_csv and args.output):
        ap.error("single mode requires --visp-csv and --output (or use --sweep)")

    metrics = metrics_from_visp_csv(args.visp_csv, args.debounce)
    if metrics is None:
        sys.exit(f"ERROR: could not derive metrics from {args.visp_csv}")
    cam = args.cam_stamps
    if not cam and args.visp_csv.endswith("_visp.csv"):
        cam = args.visp_csv[:-len("_visp.csv")] + "_cam_stamps.csv"
    if cam and os.path.exists(cam):
        metrics.update(sim_metrics(args.visp_csv, cam))
    _write_json(metrics, args.output)
    print(f"[mission_from_visp_csv] {args.output}: initial={metrics['initial_state']} "
          f"reach_sim={metrics.get('time_to_reached_sim_s')} "
          f"(wall={metrics['time_to_reached_s']}, sim_rate={metrics.get('sim_rate')})",
          file=sys.stderr)


if __name__ == "__main__":
    main()
