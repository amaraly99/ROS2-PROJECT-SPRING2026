# In-loop detection accuracy — tooling, geometry, and the blocker

Status 2026-09-16: tooling written and validated, experiment blocked on the
simulator side. No results yet.

## What the experiment measures

COCO accuracy says how good a detector is on a photograph given unlimited time.
It says nothing about the quality of the detections the controller receives. In
the closed loop the detector can skip frames, or deliver a box computed from a
frame that is already stale.

The model is deterministic, so its output for a given image cannot change under
load. Any in-loop accuracy change must therefore come from timing. That gives the
experiment its own bug check, and `benchmarks/inloop_accuracy.py` reports the
error twice:

- **at-capture**, against the drone pose when the frame was taken. This is
  detection quality proper and should be flat across detector configurations.
  If it moves, suspect a bug before believing a finding.
- **at-delivery**, against the pose when the detection reached the consumer.
  This is what the controller acts on, and should grow with detector latency.

Metrics are centroid error in pixels, miss rate and false-positive rate. IoU is
not reported: `/sim/target_pose` carries a 3D point, not an extent, so box
overlap against the reference is undefined.

## Geometry, established from data rather than assumed

Camera is pinhole, 640x480, `fx=fy=554`, `cx=320`, `cy=240` (HFOV 60 deg), from
`matlab/hil_ros_init_LT.m:34`. `/sim/drone_pose` carries `[x, y, z, pitch, yaw]`
and `/sim/target_pose` carries `[x, y, z, yaw]`.

The world bearing of an image column is

    bearing = yaw - atan2(u - cx, fx)

The sign of that term was chosen by measurement, not convention: fitting it to
the archived detections triangulates a single static sign with a 0.32 m bearing
residual, while the opposite sign gives 2.27 m.

## The blocker

**`/sim/target_pose` is a hardcoded constant, not a live feed.**
`matlab/hil_ros_init_LT.m:81` sets it to `[35.5, 23.7, 3.2, pi]` and republishes
that value at ~1 Hz. Both archived bags carry the identical value two weeks
apart.

Fitting the position that actually reprojects onto the detections:

| bag | x pub/obs | y pub/obs | z pub/obs |
|---|---|---|---|
| `hil_full_20260606_002324` | 35.50 / 34.75 | 23.70 / 3.14 | 3.20 / 3.15 |
| `hil_full_20260523_153453` | 35.50 / 33.77 | 23.70 / 3.43 | 3.20 / 2.87 |

x and z agree to within 1.7 m. y is wrong by 20.56 m and 20.27 m. Two
independent runs give the same offset, so this is a fixed frame or origin
mismatch on one axis, not a wrong target.

**Ahmed to resolve on the simulator side**: either publish the sign's live scene
position on `/sim/target_pose`, or confirm the fixed world coordinates and the
frame they are expressed in. Until then no in-loop accuracy number can be scored
against that topic, and the tool refuses to try.

## Two traps found while validating, both now guarded

1. **A second stop sign in the scene.** 53 of 680 frames contain two, and
   "highest confidence" jumps between them. That contamination alone moved the
   reprojection RMS from 38 px to 46 px and initially made the geometry look
   irreconcilable. The estimator now uses only frames holding exactly one
   instance of the target class, above a confidence floor.
2. **A runaway local search.** Bearing-only geometry creates a long flat valley,
   and an unclamped search walks hundreds of metres down it and reports the
   result as an answer. The search is now a bounded grid plus a clamped refine,
   and it says when the best fit sits on the box edge.

The tool refuses to emit metrics when the reprojection RMS exceeds 40 px, since
nothing downstream of a failed geometry check is meaningful.

## Usage

```
pip install mcap mcap-ros2-support

# always run this first: does the geometry check out, and does the
# published target agree with what the detector sees?
python3 benchmarks/inloop_accuracy.py --bag RUN.mcap --triangulate

# score, once a trustworthy target position is known
python3 benchmarks/inloop_accuracy.py --bag RUN.mcap --target X,Y,Z \
    --telemetry <run>_telemetry.csv --label yolo26n_cpu --json-out out.json
```

`--telemetry` supplies true capture timestamps from `ts_capture`, which is what
separates the at-capture and at-delivery errors. Without it both collapse onto
the publish stamp and the bug check cannot run.

## Recording requirement

`/sim/drone_pose` and `/sim/target_pose` are in `FULL_EXTRA_TOPICS` in
`scripts/record_bag.sh` but not in the lightweight set. Detector-sweep runs must
either record in full mode or have those two topics added to the lightweight
list. They are small (~20 Hz and ~1 Hz) so adding them is the cheaper option.
