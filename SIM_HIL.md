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
                                            (bgr8, 640x480, ~20 Hz,
                                             BEST_EFFORT+VOLATILE)
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
| Encoding | `bgr8` (Unreal RGB channels swapped before publish — YOLO/OpenCV expects BGR) |
| Resolution | **640 × 480** (must match — `sim_camera_bridge` rejects mismatches) |
| Rate | ~20 Hz (any rate ≤ 30 Hz is fine; pipeline runs at MATLAB publish rate) |
| QoS | BEST_EFFORT + VOLATILE + Depth 5 |
| `ROS_DOMAIN_ID` | `0` |
| DDS | matches the RPi5 side — choose one (see below) |

The Hailo HEF model is compiled for **640 × 640** input; YOLO letterboxes
the 640 × 480 frame internally with 80 px gray padding top/bottom.
Resolution other than 640 × 480 will be rejected by `sim_camera_bridge`.

## MATLAB startup sequence

Run these steps **in order every session**. Do not skip or reorder.

### Step 1 — Clear and set environment (before anything else)

In the MATLAB Command Window immediately after opening MATLAB:

```matlab
clear all
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
setenv('ROS_DOMAIN_ID','0')
setenv('FASTRTPS_DEFAULT_PROFILES_FILE','C:\path\to\matlab\HIL\fastrtps_matlab.xml')
```

> **Path note:** replace the `FASTRTPS_DEFAULT_PROFILES_FILE` path with the actual location of `fastrtps_matlab.xml` on your machine. The file ships with this repo at `matlab/` — copy it locally if needed.  
> **`clear all` is only safe here.** Never run it again after step 2 — it destroys the ROS context and requires a full MATLAB restart.

### Step 2 — Initialise ROS

```matlab
run hil_ros_init_LT
```

Creates the `/matlab_bridge` node, waits 3 s for FastRTPS peer discovery, and sets up all publishers and subscribers (`/sim/camera/image_raw`, `/cmd_vel`, `/vo_pose`, state topics). Watch for `=== LT Init complete ===` in the console.

### Step 3 — Open the Simulink model

Open `hil_closed_loop.slx`. Do **not** press Run yet.

### Step 4 — Pre-register the MATLAB Function block

In the Command Window:

```matlab
read_cmdvel_live_interp
```

This will print an error (wrong number of arguments) — **that is expected and safe to ignore.** Running it once forces MATLAB to compile and register the function. Without this step, pressing Run on the Simulink model produces a block-level error. After running it here, that error goes away.

### Step 5 — Run the Simulink model

Press **Run** on `hil_closed_loop.slx`. Set simulation pacing to 1 s wall clock (real time). Unreal Engine launches and `latest_frame` starts populating in the workspace. Confirm with:

```matlab
whos latest_frame   % expect 480x640x3 uint8
```

At this point, if the RPi5 stack is already running (`./run_stack_hil.sh`), the drone will begin moving according to the control law as soon as the Pi receives frames. If the Pi stack is not running yet, the drone sits idle — that is normal.

### Step 6 — Start the camera publisher

Once `latest_frame` is confirmed in the workspace:

```matlab
run sim_camera_publisher_timer_LT
```

Set `PUBLISH_HZ` at the top of the script before running (default 20). This starts sending bgr8 frames to the RPi5 at 20 Hz. The loop is now closed — the Pi receives frames, YOLO detects, ViSP computes `/cmd_vel`, and Simulink moves the drone.

### Step 7 — Live view *(optional)*

```matlab
run live_camera_view_LT
```

Single-panel 5 Hz viewer showing exactly what the Pi is receiving. No network traffic (reads local workspace only). Close the figure or call `stop(live_view_timer_LT)` to stop.

---

**Summary**

| # | Command | Gate |
|---|---|---|
| 1 | `clear all` + 3× `setenv(...)` | Fresh MATLAB session only |
| 2 | `run hil_ros_init_LT` | Must see `=== LT Init complete ===` |
| 3 | Open `hil_closed_loop.slx` | — |
| 4 | `read_cmdvel_live_interp` *(expect error — ignore)* | Fixes Simulink block error |
| 5 | Run `hil_closed_loop.slx` | `latest_frame` must appear in workspace |
| 6 | `run sim_camera_publisher_timer_LT` | Loop closes; Pi receives frames |
| 7 | `run live_camera_view_LT` *(optional)* | — |

## MATLAB topics and workspace variables

### Published by MATLAB → RPi5

| Topic | Type | QoS | Rate | Purpose |
|---|---|---|---|---|
| `/sim/camera/image_raw` | `sensor_msgs/Image` | BEST_EFFORT + VOLATILE + Depth 5 | 20 Hz | bgr8 frames from Unreal, deduped on checksum |
| `/sim/pitch_angle` | `std_msgs/Float64` | RELIABLE + VOLATILE | 20 Hz | Drone pitch (rad) for ViSP state feedback |
| `/sim/drone_pose` | `std_msgs/Float64MultiArray` | RELIABLE + VOLATILE | 20 Hz | `[x, y, z, pitch, yaw]` from Simulink integrators |
| `/sim/heartbeat` | `std_msgs/Float64` | RELIABLE + VOLATILE | 20 Hz | Simulink simulation time (used as image timestamp) |
| `/sim/target_pose` | `std_msgs/Float64MultiArray` | RELIABLE + TRANSIENT_LOCAL | 1 Hz | Fixed target position `[35.5, 23.7, 3.2, π]` |

### Subscribed by MATLAB ← RPi5

| Topic | Type | Written to workspace | Purpose |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `sim_vx`, `sim_vy`, `sim_vz`, `sim_wy`, `sim_wz`; batch: `sim_cmdvel`, `sim_cmdvel_ver` | ViSP velocity commands → Simulink drone dynamics |
| `/vo_pose` | `geometry_msgs/PoseStamped` | `sim_vo_pose` `[x,y,z,qx,qy,qz,qw]`, `sim_vo_pose_count` | OV2SLAM output — logged for SLAM experiment |

### Camera publish rate guide

Test in order: **14 → 20 → 30 Hz** (set `PUBLISH_HZ` in `sim_camera_publisher_timer_LT.m`).

| Rate | Bandwidth | Notes |
|---|---|---|
| 14 Hz | ~12.9 MB/s | Safe starting point |
| 20 Hz | ~18.4 MB/s | Target; test after Pi backward traffic confirmed off |
| 30 Hz | ~27.6 MB/s | Only try if 20 Hz shows zero sim lag. Simulink fixed-step is 50 ms so frames above 20 Hz are deduped and not re-sent. |

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
| `matlab/hil_ros_init_LT.m` | MATLAB session init — sets DDS env vars, creates ROS2 node, all publishers/subscribers. Run once per MATLAB session before Simulink. |
| `matlab/sim_camera_publisher_timer_LT.m` | 20 Hz timer publisher — reads `latest_frame` from Simulink workspace, RGB→BGR, deduplication checksum, sends `sensor_msgs/Image` to `/sim/camera/image_raw`. |
| `matlab/live_camera_view_LT.m` | Optional 5 Hz live viewer — shows `latest_frame` in a single figure panel. No network traffic (reads local workspace only). |
| `matlab/read_cmdvel_live_interp.m` | MATLAB Function block embedded in `hil_closed_loop.slx` — reads `sim_cmdvel` from workspace (version-cached, 1–2 evalin calls per step), applies safety clamps `[±2, ±2, ±1, ±1, ±1]` m/s, writes `sim_pitch_angle` and `sim_pose` back for the state publisher. |
| `matlab/hil_closed_loop.slx` | Simulink model — Unreal Engine NYC drone dynamics, `Simulation 3D Camera` block (640×480 RGB), `To Workspace` block (`latest_frame`), `read_cmdvel_live_interp` MATLAB Function block for `/cmd_vel` feedback. |

## Files NOT modified

`git diff main..sim-hil --stat` should show only additions. ViSP, YOLO
producer, yolo_bridge, ovcam_bridge, ov2slam, and `run_stack.sh` are all
untouched.
