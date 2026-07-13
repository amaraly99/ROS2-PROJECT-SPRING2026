# visp_servo

Image-based visual servoing (IBVS). Consumes detections, tracks a target class, and emits 4-DOF
velocity commands. Built on ViSP; runs **in Docker**.

Four-corner IBVS over the target's bounding box, driven by a five-state machine:

```
SEARCHING → APPROACHING → REACHED
     ↑            │
     └─ REACQUIRE ┘        (+ LOST → hover)
```

## Build

```bash
# inside the container
cd /workspace
colcon build --packages-select yolo_msgs visp_servo --symlink-install
source install/setup.bash
```

## Run

```bash
taskset -c 0,1 ros2 run visp_servo visp_servo_node \
  --ros-args --params-file /workspace/config/hil/hil_servo_params.yaml
```

## Topics

| Direction | Topic | Type |
|---|---|---|
| in | `/yolo/detections` | `yolo_msgs/DetectionArray` — highest-confidence match for `target_class` |
| in | `/vo_pose` | `geometry_msgs/PoseStamped` — camera pose from OV²SLAM |
| in | `/point_cloud` | `sensor_msgs/PointCloud2` — SLAM map, used for depth |
| out | `/cmd_vel` | `geometry_msgs/Twist` — vx, vy, vz, wz |
| out | `/visp/state` | `std_msgs/String` — mission state, for benchmarking |
| out | `/visp/debug_image` | `sensor_msgs/Image` — only published when subscribed |

**Depth** comes from the SLAM point cloud when OV²SLAM is running (median Z of map points near the
target pixel), falling back to a bbox-height heuristic otherwise. The log line tells you which:
`Z=X.XXm(SLAM)` or `Z=X.XXm(bbox)`.

## Parameters

Full list in [`config/hil/hil_servo_params.yaml`](../../config/hil/hil_servo_params.yaml); the tuning
rationale is in [`docs/TUNING.md`](../../docs/TUNING.md).

## The controller must survive slow detectors

This node runs at whatever rate the detector delivers — from 16 Hz on the NPU down to 0.5 Hz on a CPU
medium model. Three parameters exist purely because a controller tuned at 16 Hz silently breaks at
1 Hz, and they are easy to "clean up" back into bugs:

- **`detection_lost_sec` is not the watchdog period.** The watchdog timer stays fast to keep search
  responsive, but the target only counts as *lost* after `detection_lost_sec` (5 s). When these were
  the same value, a normal gap between two slow frames read as a loss and reset the lock-on and reach
  counters every tick — CPU missions could never complete.
- **REACHED latches on sustained closeness, not closeness *and* centering.** At 0.5–2 Hz the lateral
  loop limit-cycles, so tight centering never holds even after the drone has physically arrived.
- **`reach_require_far_ratio` guards against a stale scene.** Between runs the simulator keeps
  streaming the previous mission's final frame (drone parked at the target, bbox ≈ 0.7), which
  latched a bogus REACHED in ~0.2 s. The target must have been seen small before an arrival counts.

Per-frame centroid error is logged to the `telemetry_csv` parameter path when set — that is the
source of the paper's Centroid-RMS metric.
