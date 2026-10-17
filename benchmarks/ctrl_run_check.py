#!/usr/bin/env python3
"""ctrl_run_check.py — validity + message timing of ONE controller run.

    python3 benchmarks/ctrl_run_check.py bags/ctrl_proportional_N1_<stamp>

Valid run (rule approved by Amar Aly, 2026-10-17): from t=0 to the end of the
bag, the FSM states on /bench/state, with repeats joined, are exactly
    SEARCHING -> APPROACHING -> REACHED    or    APPROACHING -> REACHED.
Anything else (REACQUIRE in between, REACHED lost again, no REACHED) is not valid.

t=0 is the first /sim/heartbeat after the hold (plot_controller_hil.heartbeat_t0); for
old runs without resume_wall_ns in meta.txt, t=0 is the first sample in the bag.

Message timing, from the bag receive times after t=0, for /sim/heartbeat,
/sim/drone_pose and /cmd_vel: count, mean rate, gap median / 95th percentile /
max, and the number of gaps longer than 0.2 s (the FSM's heartbeat_gap_sec).

Wi-Fi load: mean and max receive rate from wifi.csv (written by ctrl_runner.py).

Writes run_check.json into the run dir and prints a short summary.
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_controller_hil import find_bag, heartbeat_t0  # noqa: E402

VALID_SEQUENCES = (["SEARCHING", "APPROACHING", "REACHED"], ["APPROACHING", "REACHED"])
GAP_LIMIT_S = 0.2
TIMED_TOPICS = ("/sim/heartbeat", "/sim/drone_pose", "/cmd_vel")


def read_times_and_states(bagdir):
    times = {t: [] for t in TIMED_TOPICS}
    states = []
    with AnyReader([bagdir]) as reader:
        conns = [c for c in reader.connections if c.topic in TIMED_TOPICS or c.topic == "/bench/state"]
        for conn, ts, raw in reader.messages(connections=conns):
            t = ts * 1e-9
            if conn.topic == "/bench/state":
                m = reader.deserialize(raw, conn.msgtype)
                states.append((t, m.data.split(",")[0] if m.data else "?"))
            else:
                times[conn.topic].append(t)
    return times, states


def join_repeats(seq):
    out = []
    for s in seq:
        if not out or out[-1] != s:
            out.append(s)
    return out


def timing(ts, t0, t_end):
    ts = np.array([t for t in ts if t >= t0])
    if len(ts) < 2:
        return {"count": int(len(ts)), "rate_hz": float("nan"), "gap_p50_ms": float("nan"),
                "gap_p95_ms": float("nan"), "gap_max_ms": float("nan"), "gaps_over_0.2s": 0}
    gaps = np.diff(ts)
    span = max(t_end - t0, 1e-9)
    return {
        "count": int(len(ts)),
        "rate_hz": float(len(ts) / span),
        "gap_p50_ms": float(np.percentile(gaps, 50) * 1e3),
        "gap_p95_ms": float(np.percentile(gaps, 95) * 1e3),
        "gap_max_ms": float(gaps.max() * 1e3),
        "gaps_over_0.2s": int((gaps > GAP_LIMIT_S).sum()),
    }


def wifi_load(rundir):
    f = rundir / "wifi.csv"
    if not f.exists():
        return {"rx_mean_MBps": float("nan"), "rx_max_MBps": float("nan")}
    rates = []
    with open(f) as fh:
        for row in csv.DictReader(fh):
            try:
                rates.append(float(row["rx_MBps"]))
            except (KeyError, ValueError):
                pass
    if not rates:
        return {"rx_mean_MBps": float("nan"), "rx_max_MBps": float("nan")}
    return {"rx_mean_MBps": float(np.mean(rates)), "rx_max_MBps": float(np.max(rates))}


def check(rundir):
    rundir = Path(rundir)
    bagdir = find_bag(rundir)
    times, states = read_times_and_states(bagdir)
    all_t = [t for v in times.values() for t in v] + [t for t, _ in states]
    if not all_t:
        return {"valid": False, "reason": "empty bag", "sequence": []}
    resume_t, rule, _ = heartbeat_t0(rundir, bagdir)
    t0 = resume_t if resume_t is not None else min(all_t)
    t_end = max(all_t)
    seq = join_repeats([s for t, s in states if t >= t0])
    valid = seq in VALID_SEQUENCES
    if valid:
        reason = "ok"
    elif not seq:
        reason = "no /bench/state after t=0"
    elif "REACHED" not in seq:
        reason = "never REACHED"
    elif seq[-1] != "REACHED":
        reason = "REACHED lost again"
    else:
        reason = "unexpected state order"
    return {
        "valid": valid,
        "reason": reason,
        "sequence": seq,
        "t0_rule": rule if resume_t is not None else "first_sample",
        "window_s": float(t_end - t0),
        "timing": {topic: timing(ts, t0, t_end) for topic, ts in times.items()},
        "wifi": wifi_load(rundir),
    }


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: ctrl_run_check.py <run_dir>")
    rundir = Path(sys.argv[1])
    res = check(rundir)
    (rundir / "run_check.json").write_text(json.dumps(res, indent=2))
    print(f"valid: {res['valid']}  ({res['reason']})  sequence: {' -> '.join(res['sequence']) or '-'}")
    for topic, t in res.get("timing", {}).items():
        print(f"  {topic:16s} n={t['count']:5d} rate={t['rate_hz']:6.2f} Hz  gap p50={t['gap_p50_ms']:6.1f} "
              f"p95={t['gap_p95_ms']:6.1f} max={t['gap_max_ms']:7.1f} ms  >0.2s: {t['gaps_over_0.2s']}")
    w = res.get("wifi", {})
    print(f"  wifi rx mean {w.get('rx_mean_MBps', float('nan')):.2f} MB/s  max {w.get('rx_max_MBps', float('nan')):.2f} MB/s")
    return 0 if res["valid"] else 2


if __name__ == "__main__":
    sys.exit(main())
