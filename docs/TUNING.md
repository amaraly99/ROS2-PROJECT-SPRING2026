# Tuning reference

Parameters that matter, and what they trade off. See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) when
something is outright broken rather than merely slow.

## OV²SLAM

Set in the camera YAML (e.g. `camera_calib/ov5647_ov2slam.yaml`).

### Initialization

| Parameter | Live default | Effect |
|---|---|---|
| `finit_parallax` | 10.0 | Minimum pixel parallax before map initialization. Lower initializes more easily but triangulates less accurately. Was 25, which is too hard to hit handheld. |
| `fransac_err` | 1.5 | RANSAC inlier threshold (px). Tighter gives fewer but better inliers. |
| `fmax_reproj_err` | 2.0 | Maximum reprojection error for accepting a 3D point. |

SLAM needs roughly 10 px of **lateral translation** between frames to triangulate the first map
points. Pan the camera side to side — rotation alone will never initialize it.

### Performance on the Pi

| Parameter | Default | Effect |
|---|---|---|
| `force_realtime` | 1 | Drop frames that arrive while processing is busy. Keep at 1 on the Pi; otherwise the queue builds and latency grows without bound. |
| `nmaxdist` | 35 | Minimum pixel distance between keypoints. Higher = fewer keypoints = faster. |
| `nklt_pyr_lvl` | 3 | KLT pyramid levels. Dropping to 2 saves CPU at the cost of tracking range. |
| `bdo_track_localmap` | 1 | Track against the local map: more accurate, heavier. 0 for speed. |

### Bundle adjustment

| Parameter | Default | Effect |
|---|---|---|
| `robust_mono_th` | 5.9915 | Robust cost threshold (χ² at 5%). |
| `apply_l2_after_robust` | 1 | Refine with L2 after the robust iteration. |
| `nmin_covscore` | 25 | Minimum co-observations for a keyframe to enter BA. |
| `ba_max_iterations` | — | Cap solver iterations. Tightening this buys ~21% BA speedup at the cost of ATE (0.071 → 0.160 m). |
| `ba_max_solver_time` | — | Wall-clock budget per BA call. |

### Loop closure

| Parameter | Default | Effect |
|---|---|---|
| `buse_loop_closer` | 0 | Off by default — ibow-lcd's memory cost is high on the Pi. |

## Detector

Set in [`models/model_registry.json`](../models/model_registry.json), overridable per run via
`run_stack_hil.sh --model/--backend/--conf`.

| Parameter | Default | Effect |
|---|---|---|
| `conf` | 0.20 | Detection confidence floor. **Do not raise this above 0.20 when comparing backends:** the YOLOv8/v11 HEFs bake a 0.20 NMS score threshold into the on-device op and cannot be raised at runtime, so a higher CPU-side threshold would discard detections the NPU path is still given. |
| `iou` | 0.45 | NMS IoU threshold. |
| `input_size` | 640 | Inference resolution. NPU inference is essentially resolution-invariant (~25 ms at every size tested), so lowering this buys little on the NPU and a lot on the CPU. |

CPU inference is **core-bound**: ONNX Runtime uses two intra-op threads, so pinning the detector to
a single core roughly halves throughput. Give CPU detectors two cores (`--detector-core 0,1`).

## Visual servoing

Set in [`config/hil/hil_servo_params.yaml`](../config/hil/hil_servo_params.yaml).

| Parameter | Default | Effect |
|---|---|---|
| `target_class` | `stop sign` | COCO class to servo toward. |
| `min_confidence` | 0.30 | Detection confidence floor for the controller. |
| `lambda_xy` | 0.5 | IBVS gain for centering. |
| `target_bbox_ratio` | 0.55 | Desired bbox-height / image-height — this is what sets the standoff distance. |
| `hold_bbox_ratio` | 0.50 | Closeness floor for declaring REACHED. |
| `reach_consec_ticks` | 4 | Consecutive close detections before REACHED latches. |
| `reach_require_far_ratio` | 0.30 | The sign must have been seen *smaller* than this before an arrival can count. Guards against latching REACHED on the previous run's final frame, which the simulator keeps streaming between runs. |
| `detection_lost_sec` | 5.0 | Time with no detection before the target counts as lost. **Must exceed the slowest detector's frame gap** (CPU-medium is ~3.4 s) or a normal gap between slow frames is misread as a loss and resets the lock-on and reach counters every tick. |
| `lockon_consec` | 1 | Consecutive centred detections to lock on. At 0.5 Hz, two consecutive centred detections essentially never occur before the sign leaves frame. |

The last three exist because a controller tuned at 16 Hz silently breaks at 1 Hz. If you add a
slower detector, re-check them first.
