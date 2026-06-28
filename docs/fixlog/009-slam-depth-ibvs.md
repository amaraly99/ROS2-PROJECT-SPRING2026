---
id: FIX-009
title: Wire any SLAM's pose+map-points into ViSP IBVS depth (dead path revived + configurable)
date: 2026-06-28
status: partial
component: visp_servo / config/hil / parse_stack.py / run_stack_hil.sh / launch
critic_verdict: partial
kiss_verdict: simplify-recommended
open_todos: [TODO-L, TODO-M, TODO-N, TODO-O]
---

## Symptom

The "add SLAM depth to IBVS" path (`use_slam_depth`) existed but was DEAD: setting it true
changed nothing. IBVS always used bbox depth even with OV2SLAM running.

## Root cause

`SlamDepthSource` subscribed to HARDCODED `/vo_pose` + `/point_cloud`, but the stack configs
remap OV2SLAM's pose onto `/slam/pose` and never remapped the cloud at all
(`/ov2slam_node/point_cloud`). Topic mismatch on BOTH inputs -> `has_pose_`/`has_cloud_`
never latched -> silent permanent bbox fallback. The toggle was a no-op.

Secondary: `use_slam_depth` lived in the shared gains file `bench_ibvs.yaml`, so an A/B
(bbox vs SLAM depth) benchmark on the SAME controller was impossible without two gains files.

## Diff (final, after KISS simplification)

Canonical-topic contract: `/slam/pose` (PoseStamped) + `/slam/cloud` (PointCloud2),
HARDCODED in SlamDepthSource. Every SLAM remaps its native topics onto them.

- `slam_depth_source.{hpp,cpp}`: ctor `(node, depth_scale)`; subscribes the two canonical
  constants `kPoseTopic`/`kCloudTopic`; `depth()` returns `depth_scale_ * median` for the
  SLAM value (bbox fallback unscaled). Documented single-threaded-executor assumption.
- `ibvs_controller.{hpp,cpp}`: declares `use_slam_depth` (bool) + `slam_depth_scale` (double);
  passes scale to SlamDepthSource.
- `config/hil/bench_ibvs.yaml`: `slam_depth_scale: 1.0` (use_slam_depth removed -> per stack-config).
- `parse_stack.py`: emits `CONTROLLER_USE_SLAM_DEPTH` from `controller.use_slam_depth`.
- `run_stack_hil.sh`: `LAUNCH_ARGS+=" use_slam_depth:=${CONTROLLER_USE_SLAM_DEPTH:-false}"`.
- `hil_simulation.launch.py`: declares `use_slam_depth` arg; appends it to the controller
  node's trailing param dict (overrides yaml; harmless override for non-ibvs controllers).
- A/B pair: `ov2slam_ibvs.yaml` (false, bbox baseline) + `ov2slam_ibvs_slamdepth.yaml` (true),
  identical except the toggle. Both add remap `point_cloud:=/slam/cloud`.

Build: visp_servo compiles clean. Parse verified: baseline false / variant true, cloud remap present.

## Critic verdict & concerns

PARTIAL. Plumbing is mechanically sound; QoS (best_effort cloud sub vs reliable pub; QoS(10)
pose), param precedence, and thread-safety (single-threaded spin) are all benign TODAY. But the
PREMISE is unvalidated: feeding raw, up-to-scale, drifting, possibly-background-biased SLAM Z
into a controller whose L is ~1/Z and that already "breakdanced" (FIX-006) may make IBVS WORSE.
Main concern = scale + background bias + mid-flight bbox->SLAM Z step into the centering channels.

## KISS verdict

SIMPLIFY-RECOMMENDED -> APPLIED. Dropped the `slam_pose_topic`/`slam_cloud_topic` ROS params
and hardcoded the canonical topics as constants: a tunable topic param is a second source of
truth that can silently disagree with the SLAM-side remap and resurrect the exact dead-wiring
bug being fixed. Kept the A/B config pair (benchmarks want named, version-controlled run defs),
the per-stack-config toggle, and `slam_depth_scale` (explicit user decision).

## Open TODOs

- TODO-L: Monocular SCALE. depth_scale=1.0 cannot model a drifting mono scale. Resolve scale
  (align SLAM scale to known target height / sim GT) before trusting SLAM-depth IBVS numbers.
- TODO-M: BACKGROUND bias. median of map points in the bbox can measure background, not the
  target. Gate to nearest cluster / depth-percentile instead of full-bbox median.
- TODO-N: WARMUP discontinuity + tracking-lost. Z steps bbox->SLAM at acquisition (~20-40s);
  has_pose_/has_cloud_ latch forever so a LOST mono track keeps feeding stale Z with no fallback.
  Ramp Z on transition + add staleness (timestamp-age) fallback to bbox.
- TODO-O: A/B fairness. Baseline (false) doesn't subscribe to /slam/cloud, so OV2SLAM (guarded
  on subscription_count) doesn't publish it -> different SLAM CPU/bw load than the variant.
  For a clean depth-only comparison, force both arms to subscribe.
