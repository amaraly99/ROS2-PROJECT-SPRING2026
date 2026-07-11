#!/usr/bin/env python3
"""
recompute_mission_metrics.py — recover REAL mission metrics from mission JSONs.

WHY: /visp/state is latched (transient_local) and run_hil_experiments.sh used to
start the stack BEFORE the per-run sim reset. visp_servo briefly saw the previous
run's end-scene (drone parked at the sign), latched REACHED, and mission_monitor
received that stale sample at t~0.013s. Result: time_to_reached_s collapsed to ~0
and approach_duration_s went negative for otherwise-successful runs, and per-config
means in detector_comparison.csv averaged real reaches with artifacts.

The genuine mission survives in each JSON's `transitions` list. Recovery rule:
  * transitions[0] is the INITIAL LATCHED STATE, not an event (a fresh clean run's
    first sample is SEARCHING at t~0; a contaminated run's is REACHED/APPROACHING).
  * time_to_reached  = first REACHED among transitions[1:]
  * approach start   = last APPROACHING before that REACHED (None if the approach
    itself was the latched baseline -> approach_duration null)
  * SUSPECT          = approach_duration < MIN_APPROACH_S (drone cannot physically
    fly start->sign that fast; catches sim-reset races). Excluded from aggregates.

Outputs (never overwrites originals):
  <sweep>/mission_audit.csv               per-run raw vs recovered + verdict
  <sweep>/detector_comparison_fixed.csv   original CSV with reached_rate /
                                          time_to_reached_s / approach_duration_s
                                          replaced by recovered aggregates

Usage:
  python3 benchmarks/recompute_mission_metrics.py \
      [--sweep benchmarks/results/detector_sweep_2026-06-25] [--min-approach 2.0]
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys

MIN_APPROACH_S = 2.0


def recover(mission: dict, min_approach: float):
    """Return (verdict, t_reach, approach_dur) from one mission JSON."""
    trans = mission.get("transitions") or []
    if not trans:
        return "NO_DATA", None, None
    events = trans[1:]  # transitions[0] = initial latched state, not an event
    t_reach = next((t for t, s in events if s == "REACHED"), None)
    if t_reach is None:
        return "NO_REACH", None, None
    t_app = None
    for t, s in events:
        if t >= t_reach:
            break
        if s == "APPROACHING":
            t_app = t
    approach = round(t_reach - t_app, 3) if t_app is not None else None
    if approach is not None and approach < min_approach:
        return "SUSPECT", t_reach, approach
    if trans[0][1] in ("REACHED", "APPROACHING"):
        return "RECOVERED", t_reach, approach  # was contaminated, now fixed
    return "REAL", t_reach, approach


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep",
                    default="benchmarks/results/detector_sweep_2026-06-25")
    ap.add_argument("--min-approach", type=float, default=MIN_APPROACH_S)
    args = ap.parse_args()

    sweep = args.sweep
    comparison = os.path.join(sweep, "detector_comparison.csv")
    if not os.path.exists(comparison):
        sys.exit(f"ERROR: {comparison} not found")

    # ── per-run recovery ─────────────────────────────────────────────────────
    audit_rows = []
    per_label = {}   # csv label -> list of (verdict, t_reach, approach)
    for path in sorted(glob.glob(os.path.join(sweep, "*_mission.json"))):
        base = os.path.basename(path)
        m = re.match(r"(.+)_run(\d+)_mission\.json$", base)
        if not m:
            continue
        prefix, run = m.group(1), int(m.group(2))
        label = re.sub(r"_(fast|accurate)$", "", prefix)
        with open(path) as f:
            mission = json.load(f)
        verdict, t_reach, approach = recover(mission, args.min_approach)
        audit_rows.append({
            "label": label, "run": run,
            "raw_time_to_reached_s": mission.get("time_to_reached_s"),
            "raw_approach_duration_s": mission.get("approach_duration_s"),
            "initial_latched_state": (mission.get("transitions") or [[None, ""]])[0][1],
            "recovered_time_to_reached_s": t_reach,
            "recovered_approach_duration_s": approach,
            "verdict": verdict,
        })
        per_label.setdefault(label, []).append((verdict, t_reach, approach))

    audit_path = os.path.join(sweep, "mission_audit.csv")
    with open(audit_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(audit_rows[0].keys()))
        w.writeheader()
        w.writerows(audit_rows)

    # ── aggregate per config (SUSPECT runs excluded) ─────────────────────────
    def agg(vals):
        if not vals:
            return None, None
        mean = sum(vals) / len(vals)
        std = (math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))
               if len(vals) > 1 else 0.0)
        return round(mean, 3), round(std, 3)

    summary = {}
    for label, runs in per_label.items():
        valid = [r for r in runs if r[0] != "SUSPECT"]
        reaches = [r[1] for r in valid if r[1] is not None]
        approaches = [r[2] for r in valid if r[2] is not None]
        rate = round(len(reaches) / len(valid), 3) if valid else 0.0
        t_mean, t_std = agg(reaches)
        a_mean, _ = agg(approaches)
        summary[label] = {
            "reached_rate": rate,
            "time_to_reached_s": t_mean,
            "time_to_reached_std": t_std,
            "approach_duration_s": a_mean,
            "n_valid": len(valid),
            "n_suspect": len(runs) - len(valid),
        }

    # ── rewrite comparison CSV with recovered mission columns ────────────────
    with open(comparison) as f:
        rows = list(csv.DictReader(f))
        fields = rows and list(rows[0].keys())
    for row in rows:
        s = summary.get(row["label"])
        if s is None:
            continue
        row["reached_rate"] = s["reached_rate"]
        row["time_to_reached_s"] = ("" if s["time_to_reached_s"] is None
                                    else s["time_to_reached_s"])
        row["approach_duration_s"] = ("" if s["approach_duration_s"] is None
                                      else s["approach_duration_s"])
    fixed_path = os.path.join(sweep, "detector_comparison_fixed.csv")
    with open(fixed_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    # ── console report ───────────────────────────────────────────────────────
    print(f"audit  -> {audit_path}")
    print(f"fixed  -> {fixed_path}\n")
    print(f"{'config':<16} {'rate':>5} {'reach_s':>8} {'±std':>6} {'n':>2} "
          f"{'suspect':>7}")
    for label in sorted(summary):
        s = summary[label]
        reach = "—" if s["time_to_reached_s"] is None else f"{s['time_to_reached_s']:.1f}"
        std = "" if s["time_to_reached_std"] is None else f"{s['time_to_reached_std']:.1f}"
        print(f"{label:<16} {s['reached_rate']:>5} {reach:>8} {std:>6} "
              f"{s['n_valid']:>2} {s['n_suspect']:>7}")


if __name__ == "__main__":
    main()
