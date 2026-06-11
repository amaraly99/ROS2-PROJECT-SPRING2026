#!/usr/bin/env python3
"""
latency_stage_report.py — join per-run latency sources into one stage table.

Inputs (any subset; stages without data are reported as n/a):
  --telemetry  producer 7-stamp CSV (S3-S7)            [/tmp/yolo_telemetry.csv]
  --probe      e2e_latency_probe CSV (S8, S9, E2E)
  --bridge-log sim_camera_bridge stamp CSV (S1, S2)    [/tmp/sim_cam_stamps.csv]

Stage definitions: see benchmarks/LATENCY_STAGES.md. The monotonic camera
stamp t_ns (= telemetry ts_capture = probe t_det_stamp_ns = bridge-log
t_shm_write_mono_ns) is the join key.

Output: human table on stdout; --out writes the same as CSV
(stage,name,n,mean_ms,p50_ms,p95_ms); --json writes a JSON dict.
No ROS2 required — runs on host or in Docker.
"""

import argparse
import csv
import json
import os
import sys


def _dist(vals):
    vals = sorted(v for v in vals if v is not None)
    if not vals:
        return None
    n = len(vals)
    return {
        "n": n,
        "mean_ms": round(sum(vals) / n, 3),
        "p50_ms": round(vals[n // 2], 3),
        "p95_ms": round(vals[min(n - 1, int(n * 0.95))], 3),
    }


def _read_csv(path):
    if not path or not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser(description="Per-stage latency report")
    ap.add_argument("--telemetry", default=None)
    ap.add_argument("--probe", default=None)
    ap.add_argument("--bridge-log", dest="bridge_log", default=None)
    ap.add_argument("--out", default=None, help="write stage table CSV here")
    ap.add_argument("--json", dest="json_out", default=None)
    args = ap.parse_args()

    telem = _read_csv(args.telemetry)
    probe = _read_csv(args.probe)
    bridge = _read_csv(args.bridge_log)

    stages = {}  # key -> (description, dist)

    # ── S1/S2 from the camera-bridge stamp log ───────────────────────────────
    if bridge:
        s2 = [(int(r["t_shm_write_mono_ns"]) - int(r["t_recv_mono_ns"])) / 1e6
              for r in bridge]
        stages["S2"] = ("cam recv → SHM write (rgb→NV12)", _dist(s2))
        # S1 is cross-clock: report inter-frame delivery jitter (offset-free)
        jit = []
        for a, b in zip(bridge, bridge[1:]):
            d_recv = int(b["t_recv_mono_ns"]) - int(a["t_recv_mono_ns"])
            d_sim = int(b["t_sim_stamp_ns"]) - int(a["t_sim_stamp_ns"])
            jit.append(abs(d_recv - d_sim) / 1e6)
        stages["S1*"] = ("MATLAB→Pi delivery jitter (|Δrecv−Δstamp|)", _dist(jit))

    # ── S3-S7 from producer telemetry ────────────────────────────────────────
    def col(rows, a, b):
        out = []
        for r in rows:
            try:
                va, vb = int(r[a]), int(r[b])
            except (KeyError, ValueError):
                continue
            if va and vb:
                out.append((va - vb) / 1e6)
        return out

    telem_by_stamp = {}
    if telem:
        stages["S3"] = ("SHM write → detector pickup (frame age)",
                        _dist(col(telem, "ts_yolo_shm_read", "ts_capture")))
        stages["S4"] = ("preprocess",
                        _dist(col(telem, "ts_yolo_preprocess_done", "ts_yolo_shm_read")))
        stages["S5"] = ("inference",
                        _dist(col(telem, "ts_inference_done", "ts_inference_start")))
        stages["S6"] = ("postprocess / NMS",
                        _dist(col(telem, "ts_nms_done", "ts_inference_done")))
        stages["S7"] = ("detection SHM write",
                        _dist(col(telem, "ts_yolo_shm_write", "ts_nms_done")))
        for r in telem:
            try:
                telem_by_stamp[int(r["ts_capture"])] = r
            except (KeyError, ValueError):
                pass

    # ── S8/S9/E2E from the probe (joined to telemetry on the stamp) ─────────
    if probe:
        s9 = [float(r["approx_e2e_ms"]) for r in probe if r.get("approx_e2e_ms")]
        stages["S9"] = ("det recv → /cmd_vel recv (controller)", _dist(s9))

        if "cam_to_det_ms" in (probe[0] if probe else {}):
            c2d = [float(r["cam_to_det_ms"]) for r in probe
                   if r.get("cam_to_det_ms")]
            stages["S3-S8"] = ("camera SHM write → det recv (pipeline)", _dist(c2d))
            e2e = [float(r["cam_to_det_ms"]) + float(r["approx_e2e_ms"])
                   for r in probe
                   if r.get("cam_to_det_ms") and r.get("approx_e2e_ms")]
            stages["E2E"] = ("camera SHM write → /cmd_vel recv", _dist(e2e))

            if telem_by_stamp:
                s8 = []
                for r in probe:
                    t = telem_by_stamp.get(int(r.get("t_det_stamp_ns", 0) or 0))
                    if t is None:
                        continue
                    try:
                        det_path = (int(t["ts_yolo_shm_write"])
                                    - int(t["ts_capture"])) / 1e6
                        s8.append(float(r["cam_to_det_ms"]) - det_path)
                    except (KeyError, ValueError):
                        continue
                stages["S8"] = ("det SHM write → det recv (bridge+DDS)", _dist(s8))

    # ── render ───────────────────────────────────────────────────────────────
    order = ["S1*", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9", "S3-S8", "E2E"]
    rows_out = []
    print(f"{'stage':7} {'n':>6} {'mean ms':>9} {'p50 ms':>9} {'p95 ms':>9}  description")
    print("─" * 78)
    for k in order:
        if k not in stages:
            continue
        desc, d = stages[k]
        if d is None:
            print(f"{k:7} {'-':>6} {'n/a':>9} {'n/a':>9} {'n/a':>9}  {desc}")
            continue
        print(f"{k:7} {d['n']:>6} {d['mean_ms']:>9.3f} {d['p50_ms']:>9.3f} "
              f"{d['p95_ms']:>9.3f}  {desc}")
        rows_out.append({"stage": k, "name": desc, **d})

    if not rows_out:
        print("No data found — check input paths.", file=sys.stderr)
        sys.exit(1)

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
            w.writeheader()
            w.writerows(rows_out)
        print(f"→ {args.out}")

    if args.json_out:
        with open(args.json_out, "w") as f:
            json.dump({r["stage"]: r for r in rows_out}, f, indent=2)
        print(f"→ {args.json_out}")


if __name__ == "__main__":
    main()
