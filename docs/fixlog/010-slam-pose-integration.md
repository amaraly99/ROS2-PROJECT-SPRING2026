---
id: FIX-010
title: SLAM pose → IBVS controller integration (staleness + confidence gates)
date: 2026-06-29
status: done
component: src/visp_servo/, src/orbslam2/, run_stack_hil.sh
critic_verdict: pass
kiss_verdict: pass
open_todos: [TODO-N-closed, TODO-P]
---

## Symptom / Motivation

Phase B.5: use SLAM pose inside the IBVS control law, not just for depth projection.
Two immediate sub-problems also surfaced:

1. **TODO-N**: `SlamDepthSource` latched `has_pose_=true` forever — tracking loss went
   undetected and stale SLAM data was used indefinitely for depth estimation.
2. **Bag corruption**: all SLAM benchmark bags (`run_orbslam2_eval_*`) had
   "File end magic is invalid" because `run_stack_hil.sh stop` used `docker kill`
   (SIGKILL) which killed `ros2 bag record` mid-write before the MCAP footer was written.

## Root causes

- `SlamDepthSource::depth()` checked `has_pose_` (a latch) but never checked time
  since last pose — so a dropped SLAM stream silently kept using the last known pose.
- `docker kill` = SIGKILL, no cleanup. `ros2 bag record` needs SIGINT to flush the
  MCAP footer.
- No mechanism existed to route SLAM tracking confidence into the controller.

## Diffs

### TODO-N — staleness gate in SlamDepthSource

`slam_depth_source.hpp`: added `last_pose_time_` + `kStalenessSec = 2.0`.  
`slam_depth_source.cpp`: record `node_->now()` in `on_pose()`; check age in `depth()`
before using SLAM data. Logs a 5-second-throttled warning on fallback.

### Bag stop fix

`run_stack_hil.sh` stop block and stale-cleanup block:
```diff
- sudo docker kill ros2_perception_stack
+ sudo docker exec ... bash -c "pkill -INT -f 'ros2 bag record' || true"
+ sleep 2
+ sudo docker stop --time 5 ros2_perception_stack
```

### Option 1 — staleness-gated vx (new `SlamPoseSource`)

New class `visp_servo/slam_pose_source.{hpp,cpp}`:
- Subscribes `/slam/pose` (canonical, hardcoded); records `last_pose_time_`.
- Subscribes `/slam/tracking_state` (Int32, ORB-SLAM2 only); default `-1` = no data.
- `is_usable()`:
  1. Must have received ≥1 pose.
  2. Pose age < `kStalenessSec` (2.0 s).
  3. If tracking state was received, must be `OK = 2` (ORB_SLAM2 eTrackingState).
  For OV2SLAM: tracking_state never fires → only staleness check applies.

`IBVSController`: new params `use_slam_pose` (bool, default false) + `standoff_m`
(double, default 3.0 m). When `use_slam_pose=true` and gate is open:
```cpp
vel.vx = k_fwd_ * std::max(0.0, Z_cur - standoff_m_);   // SLAM range approach
```
else:
```cpp
vel.vx = k_fwd_ * std::max(0.0, target_bbox_ratio_ - in.bbox_ratio);  // bbox fallback
```

### Option 2 — tracking-state confidence (ORB-SLAM2 patch)

`monocular-slam-node.{hpp,cpp}`: added `tracking_state_pub_`
(`std_msgs/Int32` on `slam_tracking_state` → remapped to `/slam/tracking_state`).
Published every frame after `UpdateSLAMState()`. Values from `Tracking.h`:
`OK=2, LOST=3`.

`orbslam2_eval.yaml` remap: added `"slam_tracking_state:=/slam/tracking_state"`.
`parse_stack.py`: added `CONTROLLER_USE_SLAM_POSE`.
`run_stack_hil.sh`: passes `use_slam_pose:=...` to ros2 launch.
`hil_simulation.launch.py`: declares `use_slam_pose` arg + passes to controller node.
`bench_ibvs.yaml`: documents `standoff_m`.

## Critic verdict

Pass. `is_usable()` degrades gracefully: OV2SLAM (no tracking_state) → staleness-only;
ORB-SLAM2 without the patch → same as OV2SLAM until the new image is deployed.
`use_slam_pose` defaults false → zero runtime impact on existing configs.
Warn logged when `use_slam_pose=true` but `use_slam_depth=false` (Z_cur is bbox, 
not SLAM range — vx would come from bbox depth, defeating the purpose).

## KISS verdict

Pass. SlamPoseSource is ~50 lines. It re-uses the existing `/slam/pose` canonical
topic (no new wiring for Option 1). Option 2 adds 5 lines to ORB-SLAM2 node.
Both options share one `is_usable()` call at the controller level.

## How to enable

In any stack config that uses `controller: ibvs` and has SLAM running:
```yaml
controller:
  use_slam_depth: true    # needed so Z_cur is metric SLAM range
  use_slam_pose:  true    # enables the vx gate
```
Then tune `standoff_m` in `bench_ibvs.yaml` (default 3.0 m).
Suggested first config to create: `orbslam2_ibvs_slampose.yaml`.

## Open TODOs

- **TODO-P** (Option 3 — weighted fusion): deferred. Monocular scale drift
  (see TODO-L) makes raw translation blending unreliable. Only yaw/bearing is
  scale-invariant and a candidate for blending. Revisit after GT-anchored scale
  correction is in place.
- Re-run ORB-SLAM2 + OV2SLAM benchmark bags (existing bags were corrupted by the
  `docker kill` issue; stop fix prevents recurrence).
