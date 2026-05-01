# sim-hil Branch — Hardware-In-the-Loop Simulation

## What this branch does

Replaces the physical OV5647 camera with a synthetic Unreal Engine NYC scene
streamed from a MATLAB/Simulink Windows host. The full
**YOLO → OV2SLAM → ViSP → /cmd_vel** pipeline runs unmodified on real RPi5
hardware. Only two things change vs. main:

1. The image source is a ROS topic from MATLAB instead of `ovcam_producer`
   reading the OV5647 over libcamera.
2. ViSP's `target_class` parameter is overridden from `"person"` to `"car"`
   (the NYC scene has no people).

The ViSP, OV2SLAM, yolo_producer, yolo_bridge, and ovcam_bridge **binaries
are byte-for-byte identical** between `main` and `sim-hil`. Behaviour
changes come from launch-file parameter overrides, not source edits.

## Architecture

```
MATLAB/Simulink (Windows)                    
  Unreal NYC scene → ROS2 Image publisher → /sim/camera/image_raw
                                            (rgb8, 640x480, ~20 Hz,
                                             RELIABLE+VOLATILE)
                                                ▼
┌──── RPi5  Docker (--ipc=host) ────────────────────────────────────┐
│                                                                   │
│  sim_camera_bridge   subs /sim/camera/image_raw                   │
│  (NEW C++ node)      → RGB → NV12                                 │
│                      → writes /ovcam_frames POSIX SHM             │
│                        (matches ovcam_producer wire format        │
│                         exactly — same magic, slot, seqlock)      │
│                            ▼                                      │
│        ┌───────────────────┴────────────────────┐                 │
│        ▼                                        ▼                 │
│  ovcam_bridge                            yolo_producer            │
│  /ovcam/image_raw mono8                  (host, native, Core 1)   │
│        ▼                                        ▼                 │
│  ov2slam (Cores 2,3)                     yolo_bridge              │
│  /vo_pose, /point_cloud                  /yolo/detections         │
│        └───────────────┬────────────────────────┘                 │
│                        ▼                                          │
│                visp_servo (Core 0)                                │
│                target_class = "car"  (LAUNCH PARAM ONLY)          │
│                        ▼                                          │
│                     /cmd_vel                                      │
└───────────────────────────────────────────────────────────────────┘
                        ▼
       MATLAB Simulink drone dynamics → Unreal scene update
```

## MATLAB-side requirements

| Field | Value |
|---|---|
| Topic | `/sim/camera/image_raw` |
| Type | `sensor_msgs/Image` |
| Encoding | `rgb8` |
| Resolution | **640 × 480** (must match — `sim_camera_bridge` rejects mismatches) |
| Rate | ~20 Hz (any rate ≤ 30 Hz is fine; pipeline runs at MATLAB publish rate) |
| QoS | RELIABLE + VOLATILE + KEEP_LAST(10) |
| `ROS_DOMAIN_ID` | `0` |
| DDS | matches the RPi5 side — choose one (see below) |

The Hailo HEF model is compiled for **640 × 640** input; YOLO letterboxes
the 640 × 480 frame internally with 80 px gray padding top/bottom.
Resolution other than 640 × 480 will be rejected by `sim_camera_bridge`.

## DDS middleware — which side switches?

You need both sides on the same DDS implementation. The repo runs
CycloneDDS by default for the on-device stack. Two ways to align:

### Option A — RPi5 stays CycloneDDS, MATLAB switches to CycloneDDS

MATLAB ROS Toolbox supports CycloneDDS from R2024a onward. No change on
the RPi5 side; on the Windows side select CycloneDDS as the middleware.
Use `config/hil/cyclonedds_hil.xml` on the Pi (default).

### Option B — RPi5 switches to FastDDS for HIL

If your MATLAB is older or you prefer FastDDS, run:

```bash
DDS=fastrtps MATLAB_HOST_IP=192.168.1.42 ./run_stack_hil.sh
```

This loads `config/hil/fastrtps_hil.xml` and exports
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` only inside the HIL stack — `main`
branch stays CycloneDDS.

Both XML profiles substitute `${MATLAB_HOST_IP}` at launch time so no IP
is hardcoded in the repo.

## Running on the Pi

```bash
# Required env: MATLAB host IP for unicast DDS discovery
export MATLAB_HOST_IP=192.168.1.42

# Default — CycloneDDS
./run_stack_hil.sh

# Or — FastDDS
DDS=fastrtps ./run_stack_hil.sh

# Stop
./run_stack_hil.sh stop
```

`run_stack_hil.sh` does (in order):

1. Render the DDS XML profile with `MATLAB_HOST_IP` substituted.
2. Start the Docker container (same image as main: `ros2_perception_stack`).
3. `ros2 launch sim_camera_bridge hil_simulation.launch.py` → starts the
   SHM filler, the two ROS bridges, OV2SLAM, and `visp_servo` with
   `target_class:=car`.
4. Wait for `/dev/shm/ovcam_frames` to appear.
5. `taskset -c 1 yolo_producer.py` on the host (same as main pipeline).

Logs: `/tmp/hil_launch.log` and `/tmp/yolo_producer.log`.

## Verifying it works

After `run_stack_hil.sh` completes successfully, on a separate Pi shell:

```bash
ros2 topic hz /sim/camera/image_raw       # should match MATLAB rate (~20 Hz)
ros2 topic hz /ovcam/image_raw            # ≈ same rate
ros2 topic hz /yolo/detections            # ≈ same rate
ros2 topic echo /yolo/detections --field detections | head -40
ros2 topic echo /cmd_vel                  # nonzero when a car is in frame
```

Expected log lines:
- `sim_camera_bridge: ready — subscribed to /sim/camera/image_raw, writing /ovcam_frames SHM (640x480 NV12, 4 slots)`
- `[hailo] input: 640x640x3` (yolo_producer)
- `visp_servo_node started — tracking 'car' (conf >= 0.30)`

## Known limitations

- **OV2SLAM on Unreal textures**: synthetic textures sometimes lack the
  KLT-trackable features that real-world footage has. If SLAM fails to
  initialize, ViSP falls back to its bbox-based depth estimate (uses
  `known_target_height` — already set to 1.5 m for sedans in
  `config/hil/hil_servo_params.yaml`). The control demo still works.
- **Frame rate ceiling = MATLAB publish rate.** Real OV5647 publishes at
  60 Hz on main; HIL is bottlenecked by Simulink's timer (~20 Hz typical).
- **No multicast across subnets**: the DDS XML profiles assume unicast
  discovery to `MATLAB_HOST_IP`. Both Pi and Windows must be on the same
  L2 segment, or you must add explicit firewall holes for the chosen DDS
  implementation's discovery + data ports.

## Files added on this branch

| Path | Purpose |
|---|---|
| `src/sim_camera_bridge/` | New C++ ROS2 package — SHM filler that replaces `ovcam_producer` in HIL mode. Includes `ovcam_shm.hpp` from the producer package to guarantee binary compatibility. |
| `src/sim_camera_bridge/launch/hil_simulation.launch.py` | ROS2 launch file for the in-Docker node group. |
| `config/hil/hil_servo_params.yaml` | ViSP params overriding `target_class: "car"` and `known_target_height: 1.5`. |
| `config/hil/cyclonedds_hil.xml` | CycloneDDS profile (default), with `${MATLAB_HOST_IP}` placeholder. |
| `config/hil/fastrtps_hil.xml` | FastDDS profile (alternate). |
| `run_stack_hil.sh` | Bash wrapper — Docker + DDS env + ros2 launch + native yolo_producer. |
| `SIM_HIL.md` | This file. |

## Files NOT modified

`git diff main..sim-hil --stat` should show only additions. ViSP, YOLO
producer, yolo_bridge, ovcam_bridge, ov2slam, and `run_stack.sh` are all
untouched.
