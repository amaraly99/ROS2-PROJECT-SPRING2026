#!/usr/bin/env python3
"""
inloop_accuracy.py — detection quality against a geometric reference, inside the loop.

The COCO numbers say how good a detector is on a photograph given unlimited time.
They say nothing about the quality of the detections the *controller* receives. In
the closed loop the detector can skip frames because it is too slow, or deliver a
box computed from a frame that is already stale. The model's arithmetic does not
change under load; the usefulness of its output does.

This scores a recorded HIL run against the target's known world position:

    centroid error (px)  distance between the detected box centre and the
                         projected true target centre
    miss rate            fraction of frames where the target was in view and
                         nothing was detected
    false-positive rate  fraction of detections that are not the target class

Because the model is deterministic, any in-loop change must come from timing.
So the error is reported twice, and the pair is the experiment's own bug check:

    at-capture   against the drone pose when the frame was captured. This is
                 detection quality proper and should be FLAT across detector
                 configurations. If it moves, suspect a bug before a finding.
    at-delivery  against the drone pose when the detection reached the consumer.
                 This is what the controller actually acts on, and it should grow
                 with detector latency.

Geometry, established against recorded runs rather than assumed (see --triangulate):
camera is pinhole, 640x480, fx=fy=554, cx=320, cy=240 (HFOV 60 deg), per
matlab/hil_ros_init_LT.m. World bearing of an image column is

    bearing = yaw - atan2(u - cx, fx)

with /sim/drone_pose carrying [x, y, z, pitch, yaw]. Fitting that convention to
671 stop-sign observations in bags/hil_full_20260606_002324 triangulates a single
static sign with a median residual of 0.32 m; the opposite sign convention gives
2.27 m, so the choice is measured, not guessed.

    # check what the data says the target position is, before trusting any
    python3 benchmarks/inloop_accuracy.py --bag BAG.mcap --triangulate

    # score a run against a known target position
    python3 benchmarks/inloop_accuracy.py --bag BAG.mcap --target 29.42,6.01,4.5 \
        [--telemetry run1_telemetry.csv] [--label yolo26n_npu]

Known blocker, measured rather than assumed. /sim/target_pose is published by
matlab/hil_ros_init_LT.m as a hardcoded constant (35.5, 23.7, 3.2) and
republished at ~1 Hz; it is not a live feed of the scene. Fitting the observed
sign position in the two archived HIL bags gives:

    bag                      x            y             z
    hil_full_20260606   35.50/34.75   23.70/ 3.14   3.20/3.15
    hil_full_20260523   35.50/33.77   23.70/ 3.43   3.20/2.87
                        (published/observed)

x and z agree to within 1.7 m, y is off by 20.56 m and 20.27 m. Two runs two
weeks apart give the same offset, so this is a fixed frame or origin mismatch on
the y axis, not a wrong target. It must be resolved on the simulator side before
any in-loop accuracy number is scored against /sim/target_pose.

Requires: mcap, mcap-ros2-support (pure python, no ROS install needed).
"""

import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys

FX = FY = 554.0
CX, CY = 320.0, 240.0
WIDTH, HEIGHT = 640, 480
TARGET_CLASS = "stop sign"

POSE_TOPIC = "/sim/drone_pose"
TARGET_TOPIC = "/sim/target_pose"
DET_TOPIC = "/yolo/detections"


def read_bag(path, cache_dir=None):
    """Extract the three streams we need. Cached: these bags are gigabytes."""
    cache = None
    if cache_dir:
        os.makedirs(cache_dir, exist_ok=True)
        st = os.stat(path)
        key = f"{os.path.basename(path)}.{st.st_size}.{int(st.st_mtime)}.json"
        cache = os.path.join(cache_dir, key)
        if os.path.exists(cache):
            with open(cache) as f:
                blob = json.load(f)
            return ([tuple(p) for p in blob["poses"]], blob["dets"], blob["target"])
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError:
        sys.exit("needs mcap-ros2-support: pip install mcap mcap-ros2-support")
    poses, dets = [], []
    target_pose = None
    for m in read_ros2_messages(path, topics=[POSE_TOPIC, TARGET_TOPIC, DET_TOPIC]):
        topic = m.channel.topic
        if topic == POSE_TOPIC:
            d = list(m.ros_msg.data)
            if len(d) >= 5:
                poses.append((m.log_time_ns, d[0], d[1], d[2], d[4]))
        elif topic == TARGET_TOPIC:
            target_pose = list(m.ros_msg.data)
        elif topic == DET_TOPIC:
            h = m.ros_msg.header.stamp
            dets.append({
                "log_ns": m.log_time_ns,
                "stamp_ns": h.sec * 10**9 + h.nanosec,
                "dets": [(d.class_name, d.confidence, d.center_x, d.center_y,
                          d.size_width, d.size_height) for d in m.ros_msg.detections],
            })
    poses.sort()
    dets.sort(key=lambda r: r["log_ns"])
    if cache:
        with open(cache, "w") as f:
            json.dump({"poses": poses, "dets": dets, "target": target_pose}, f)
    return poses, dets, target_pose


class PoseTrack:
    """Drone pose, looked up by bag (epoch) timestamp."""

    def __init__(self, poses):
        self.t = [p[0] for p in poses]
        self.p = poses

    def at(self, ts):
        i = bisect.bisect_left(self.t, ts)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.p)]
        if not cands:
            return None
        j = min(cands, key=lambda j: abs(self.t[j] - ts))
        return self.p[j]


def project(pose, target):
    """World target -> (u, v, ground_range). None if behind the camera."""
    _, x, y, z, yaw = pose
    dx, dy, dz = target[0] - x, target[1] - y, target[2] - z
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
    right = -dx * math.sin(yaw) + dy * math.cos(yaw)
    if fwd <= 0.1:
        return None
    # Image x is mirrored relative to the textbook right-handed convention on
    # this simulator's camera: the minus sign here is what makes project() agree
    # with the bearing convention the triangulation fits (bearing = yaw -
    # atan2(u - cx, fx)). Measured, not assumed -- against 671 stop-sign
    # observations the minus gives a 9.4 px median column error and the plus
    # gives 202 px.
    u = CX - FX * (right / fwd)
    v = CY - FY * (dz / fwd)
    return u, v, math.hypot(dx, dy)


def in_view(uv):
    return uv is not None and 0 <= uv[0] < WIDTH and 0 <= uv[1] < HEIGHT


def triangulate(poses, dets, min_conf=0.5):
    """Estimate the target position from the run, and say whether to believe it.

    Two stages. A linear bearing-only intersection gives a starting point, then a
    direct search minimises actual pixel reprojection error over (x, y, z).

    The reason for the second stage is that bearing-only intersection is
    degenerate when the drone flies a near-constant heading: the rays are almost
    parallel, depth is unconstrained, and the perpendicular residual stays small
    for a whole family of wrong answers along the viewing direction. That is not
    hypothetical -- it is what bags/hil_full_20260606_002324 does, where the
    bearing fit reports a 0.32 m residual yet the reprojection error sweeps from
    -400 px at close range to +270 px at long range.

    So the number to trust is `pixel_rms`, not the bearing residual, and
    `bearing_spread_deg` says whether the geometry constrained the answer at all.

    This uses the detector's own bearings, so it is a consistency check on the
    target position and the camera convention. It is not an independent ground
    truth, and a good fit does not prove the point is where the simulator put the
    object.
    """
    track = PoseTrack(poses)
    rows = []
    for rec in dets:
        # Only frames holding exactly ONE instance of the target class. With two
        # signs in view, "highest confidence" jumps between objects and the fit
        # degrades badly: on bags/hil_full_20260606_002324 that contamination
        # alone moves the reprojection RMS from 38 px to 46 px, and it was what
        # first made the geometry look irreconcilable.
        hits = [d for d in rec["dets"] if d[0] == TARGET_CLASS]
        if len(hits) != 1 or hits[0][1] < min_conf:
            continue
        pose = track.at(rec["log_ns"])
        if pose:
            rows.append((pose, hits[0]))
    if len(rows) < 10:
        return None

    # --- stage 1: linear bearing-only intersection -----------------------
    a11 = a12 = a22 = b1 = b2 = 0.0
    thetas = []
    for pose, d in rows:
        _, x, y, _, yaw = pose
        th = yaw - math.atan2(d[2] - CX, FX)
        thetas.append(th)
        s_, c_ = math.sin(th), math.cos(th)
        rhs = x * s_ - y * c_
        a11 += s_ * s_; a12 += -s_ * c_; a22 += c_ * c_
        b1 += s_ * rhs; b2 += -c_ * rhs
    det = a11 * a22 - a12 * a12
    if abs(det) < 1e-9:
        return None
    tx = (b1 * a22 - a12 * b2) / det
    ty = (a11 * b2 - b1 * a12) / det

    # How much angular baseline was there? Near zero means depth is guesswork.
    tmin, tmax = min(thetas), max(thetas)
    spread = math.degrees(abs(math.atan2(math.sin(tmax - tmin), math.cos(tmax - tmin))))

    zs = []
    for pose, d in rows:
        _, x, y, z, _ = pose
        rng = math.hypot(tx - x, ty - y)
        if rng > 3:
            zs.append(z + rng * ((CY - d[3]) / FY))
    tz = statistics.median(zs) if zs else 0.0

    # --- stage 2: minimise pixel reprojection error over (x, y, z) --------
    def cost(t):
        tot = n = 0
        for pose, d in rows:
            uv = project(pose, t)
            if uv is None:
                continue
            tot += (uv[0] - d[2]) ** 2 + (uv[1] - d[3]) ** 2
            n += 1
        return (tot / n) if n else float("inf")

    # Coarse global grid first. A local search seeded from the linear solution
    # walks away down the flat valley that bearing-only geometry creates, and
    # reports a confident-looking position tens of metres from anything real.
    # The grid is bounded by where the drone actually flew, padded generously.
    xs = [p[1] for p in poses]; ys = [p[2] for p in poses]; zs_p = [p[3] for p in poses]
    lo = [min(xs) - 40, min(ys) - 40, min(zs_p) - 10]
    hi = [max(xs) + 40, max(ys) + 40, max(zs_p) + 10]
    best_t, best_c = [tx, ty, tz], cost([tx, ty, tz])
    steps = 12
    for i in range(steps + 1):
        gx = lo[0] + (hi[0] - lo[0]) * i / steps
        for j in range(steps + 1):
            gy = lo[1] + (hi[1] - lo[1]) * j / steps
            for k in range(7):
                gz = lo[2] + (hi[2] - lo[2]) * k / 6
                c = cost([gx, gy, gz])
                if c < best_c:
                    best_t, best_c = [gx, gy, gz], c

    # Then refine locally, clamped to the searched box. Without the clamp the
    # search walks out to hundreds of metres whenever no point fits well, and
    # reports that runaway position as though it were an answer.
    step = max((hi[0] - lo[0]) / steps, 1.0)
    for _ in range(300):
        start_c = best_c
        for axis in range(3):
            for delta in (step, -step):
                cand = list(best_t)
                cand[axis] = min(hi[axis], max(lo[axis], cand[axis] + delta))
                c = cost(cand)
                if c < best_c:
                    best_t, best_c = cand, c
        if best_c > start_c * (1.0 - 1e-6):
            step /= 2.0
            if step < 0.01:
                break
    at_bound = any(abs(best_t[a] - lo[a]) < 1e-6 or abs(best_t[a] - hi[a]) < 1e-6
                   for a in range(3))

    resid = []
    for (pose, d), th in zip(rows, thetas):
        _, x, y, _, _ = pose
        resid.append(abs((tx - x) * math.sin(th) - (ty - y) * math.cos(th)))

    return {
        "x": best_t[0], "y": best_t[1], "z": best_t[2],
        "pixel_rms": math.sqrt(best_c),
        "linear_xy": (tx, ty),
        "bearing_residual_m": statistics.median(resid),
        "bearing_spread_deg": spread,
        "at_search_bound": at_bound,
        "n": len(rows),
    }


def _best(dets):
    hits = [d for d in dets if d[0] == TARGET_CLASS]
    return max(hits, key=lambda d: d[1]) if hits else None


def load_capture_times(path):
    """frame telemetry -> sorted (publish_mono_ns, capture_mono_ns) pairs."""
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                out.append((int(row["ts_yolo_shm_write"]), int(row["ts_capture"])))
            except (KeyError, ValueError):
                continue
    out.sort()
    return out


def score(poses, dets, target, capture_pairs=None):
    track = PoseTrack(poses)

    # Detection headers are stamped on the Pi's monotonic clock; the bag logs
    # epoch. The offset is constant, so estimate it robustly and reuse it to put
    # capture timestamps on the bag's clock.
    offs = [r["log_ns"] - r["stamp_ns"] for r in dets]
    offset = statistics.median(offs) if offs else 0
    offset_spread_ms = (max(offs) - min(offs)) / 1e6 if offs else 0.0

    cap_t = [p[0] for p in capture_pairs] if capture_pairs else []

    err_capture, err_delivery = [], []
    misses = present = 0
    fp = total_dets = 0

    for rec in dets:
        pose_del = track.at(rec["log_ns"])
        if pose_del is None:
            continue
        uv_del = project(pose_del, target)

        # Capture-time pose: telemetry maps publish -> capture on the monotonic
        # clock; without it fall back to the header stamp.
        cap_ns = rec["stamp_ns"]
        if cap_t:
            i = bisect.bisect_left(cap_t, rec["stamp_ns"])
            cands = [j for j in (i - 1, i) if 0 <= j < len(capture_pairs)]
            if cands:
                j = min(cands, key=lambda j: abs(cap_t[j] - rec["stamp_ns"]))
                cap_ns = capture_pairs[j][1]
        pose_cap = track.at(cap_ns + offset)
        uv_cap = project(pose_cap, target) if pose_cap else None

        total_dets += len(rec["dets"])
        fp += sum(1 for d in rec["dets"] if d[0] != TARGET_CLASS)

        best = _best(rec["dets"])
        if in_view(uv_cap):
            present += 1
            if best is None:
                misses += 1
        if best is not None:
            if in_view(uv_cap):
                err_capture.append(math.hypot(uv_cap[0] - best[2], uv_cap[1] - best[3]))
            if in_view(uv_del):
                err_delivery.append(math.hypot(uv_del[0] - best[2], uv_del[1] - best[3]))

    def stats(v):
        if not v:
            return None
        v = sorted(v)
        return {"n": len(v), "median_px": statistics.median(v),
                "mean_px": statistics.fmean(v),
                "p90_px": v[min(len(v) - 1, int(0.9 * len(v)))]}

    return {
        "frames_with_detections": len(dets),
        "clock_offset_spread_ms": offset_spread_ms,
        "target_in_view_frames": present,
        "miss_rate": (misses / present) if present else None,
        "false_positive_rate": (fp / total_dets) if total_dets else None,
        "centroid_err_at_capture": stats(err_capture),
        "centroid_err_at_delivery": stats(err_delivery),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="path to the .mcap file")
    ap.add_argument("--target", help="true target as x,y,z in world coordinates")
    ap.add_argument("--telemetry", help="<run>_telemetry.csv, for capture timestamps")
    ap.add_argument("--triangulate", action="store_true",
                    help="estimate the target position from the run and exit")
    ap.add_argument("--force", action="store_true",
                    help="score even if the consistency check fails")
    ap.add_argument("--label", default=None)
    ap.add_argument("--json-out", default=None)
    ap.add_argument("--cache-dir", default=None,
                    help="cache extracted streams here; these bags are GB-sized")
    args = ap.parse_args()

    poses, dets, bag_target = read_bag(args.bag, args.cache_dir)
    print(f"poses={len(poses)}  detection messages={len(dets)}")
    if bag_target:
        print(f"/sim/target_pose in bag: {bag_target[:3]}")

    tri = triangulate(poses, dets)
    if tri:
        print(f"estimated from {tri['n']} detections: "
              f"x={tri['x']:.2f} y={tri['y']:.2f} z={tri['z']:.2f}")
        print(f"  reprojection RMS      : {tri['pixel_rms']:.1f} px  "
              f"(from {tri['n']} single-instance detections)")
        print(f"  bearing spread        : {tri['bearing_spread_deg']:.1f} deg")
        if tri["bearing_spread_deg"] < 20.0:
            print("  WARNING: little angular baseline, so depth is poorly "
                  "constrained and this position is not trustworthy.")
        if tri["at_search_bound"]:
            print("  WARNING: the best fit sits on the edge of the search box, "
                  "which means no interior point fitted well.")
        if tri["pixel_rms"] > 40.0:
            print()
            print("  FAILED CONSISTENCY CHECK: no single static point reprojects "
                  "onto these detections")
            print(f"  (best achievable RMS {tri['pixel_rms']:.0f} px on a "
                  f"{WIDTH}x{HEIGHT} image).")
            print("  The projection model does not describe this run. Check, in "
                  "order: whether the")
            print("  camera is fixed to the body yaw axis, whether /sim/drone_pose "
                  "is time-aligned")
            print("  with the imagery, and whether more than one instance of the "
                  "target class is")
            print("  in the scene. Do not score until this check passes.")
        if bag_target:
            d = [bag_target[a] - tri[k] for a, k in enumerate("xyz")]
            gap = math.sqrt(sum(v * v for v in d))
            print()
            print("  against /sim/target_pose "
                  f"({bag_target[0]:.2f}, {bag_target[1]:.2f}, {bag_target[2]:.2f}):")
            for a, k in enumerate("xyz"):
                flag = "  <-- disagrees" if abs(d[a]) > 2.0 else ""
                print(f"    {k}: published {bag_target[a]:7.2f}  "
                      f"observed {tri[k]:7.2f}  diff {d[a]:+7.2f} m{flag}")
            if gap > 2.0:
                axes = [k for a, k in enumerate("xyz") if abs(d[a]) > 2.0]
                print(f"  WARNING: /sim/target_pose disagrees on {', '.join(axes)}. "
                      "A single-axis gap")
                print("  points at a frame or origin mismatch rather than a wrong "
                      "target. Resolve it")
                print("  on the simulator side before scoring against this topic.")
    else:
        print("estimation failed: too few target-class detections")

    if args.triangulate:
        return 0
    if not args.target:
        sys.exit("--target x,y,z is required to score (run --triangulate first)")

    target = [float(v) for v in args.target.split(",")]
    if len(target) != 3:
        sys.exit("--target needs three comma-separated numbers")
    if tri and tri["pixel_rms"] > 40.0 and not args.force:
        sys.exit("\nrefusing to score: the consistency check above failed, so any "
                 "metric this\nproduced would be meaningless. Pass --force only if "
                 "you understand why.")

    caps = load_capture_times(args.telemetry) if args.telemetry else None
    if args.telemetry and not caps:
        print(f"warning: no usable rows in {args.telemetry}; "
              "falling back to header stamps for capture time")

    res = score(poses, dets, target, caps)
    res["label"] = args.label or os.path.basename(args.bag)
    res["target"] = target
    res["capture_times_from_telemetry"] = bool(caps)

    print()
    print(json.dumps(res, indent=2))
    if args.json_out:
        os.makedirs(os.path.dirname(args.json_out) or ".", exist_ok=True)
        with open(args.json_out, "w") as f:
            json.dump(res, f, indent=2)
        print(f"\nwrote {args.json_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
