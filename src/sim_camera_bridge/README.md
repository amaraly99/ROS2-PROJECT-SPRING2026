# sim_camera_bridge

Hardware-in-the-loop counterpart to `ovcam_producer`. Instead of a physical camera, frames arrive
over ROS 2 from a MATLAB/Simulink simulator; this node writes them into the same `/ovcam_frames`
shared-memory segment the rest of the stack already reads.

That is the whole point: everything downstream — `yolo_producer`, `ovcam_bridge`, OV²SLAM,
`visp_servo` — is byte-identical between the live camera and the simulator, because both feed the
same segment in the same format.

## Run

Via the launch file, which brings up the whole HIL stack:

```bash
ros2 launch sim_camera_bridge hil_simulation.launch.py
```

Normally you don't call this directly — `./run_stack_hil.sh` wraps it.

## Launch arguments

| Argument | Default | Effect |
|---|---|---|
| `node_cores` | `0-3` | `taskset` CPU list for the ROS-side nodes. Default is all cores (a no-op pin). Placement configs C and D pass `0` to co-locate everything on one core. |
| `visp_telemetry_csv` | `''` | Path for per-frame servo centroid telemetry; empty disables it. |

## The loop

```
Simulink ──/sim/camera──► sim_camera_bridge ──► /ovcam_frames (shm)
    ▲                                                  │
    │                                          detector + SLAM + servo
    └──────────────/cmd_vel──────────────────────────┘
```

The sim host is authoritative for camera frames and drone state. See
[`matlab/`](../../matlab/) for the Simulink side and
[`benchmarks/matlab/README.md`](../../benchmarks/matlab/README.md) for the run protocol.

**Mission timing must be read in simulation time.** The sim rate drifts 0.78–0.95× of real time
between runs, so wall-clock durations measured on the Pi are not comparable across configurations.
`--stamp-log` makes this node record per-frame sim/host stamp pairs (`_cam_stamps.csv`), which is
what allows wall-clock to be converted to sim time afterwards.
