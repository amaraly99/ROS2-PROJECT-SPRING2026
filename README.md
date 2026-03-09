# ROS 2 Perception Stack

> Raspberry Pi 5 · OV5647 camera · Hailo-10H NPU · OV²SLAM · ViSP visual servoing  
> **ROS 2 Jazzy** · Ubuntu 24.04 (Docker) · CycloneDDS

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Hardware & Prerequisites](#2-hardware--prerequisites)
3. [Repository Layout](#3-repository-layout)
4. [Docker Image](#4-docker-image)
5. [Packages](#5-packages)
6. [Running the Stack](#6-running-the-stack)
7. [EuRoC Dataset Replay](#7-euroc-dataset-replay)
8. [Camera Calibration](#8-camera-calibration)
9. [Tuning Reference](#9-tuning-reference)
10. [Troubleshooting](#10-troubleshooting)
11. [Applied Bug Fixes (2026-03-01)](#11-applied-bug-fixes-2026-03-01)
12. [Contributors](#12-contributors)

---

## 1. Architecture Overview

```
┌──────────────────────── HOST (Raspberry Pi 5) ──────────────────────────────┐
│                                                                              │
│  Core 0 — ovcam_producer  (libcamera)                                       │
│       │  NV12 frames → /ovcam_frames  (POSIX shm ring)                      │
│       │                                                                      │
│  Core 1 — yolo_producer   (Hailo-10H NPU)                                   │
│       │  reads /ovcam_frames → letterbox 640×640 → YOLO → /yolo_shm         │
│                                                                              │
│  ┌───────────────────── DOCKER CONTAINER ───────────────────────────────┐   │
│  │                                                                       │   │
│  │  Core 0,1 — ovcam_bridge   /ovcam_frames → /ovcam/image_raw (mono8) │   │
│  │  Core 0,1 — yolo_bridge    /yolo_shm     → /yolo/detections          │   │
│  │                                           → /yolo/annotated           │   │
│  │                                                                       │   │
│  │  Core 2,3 — OV2SLAM        /ovcam/image_raw → /vo_pose               │   │
│  │                                              → /point_cloud           │   │
│  │                                                                       │   │
│  │  Core 0,1 — visp_servo     /yolo/detections ──┐                      │   │
│  │                             /vo_pose          ├─► /cmd_vel (Twist)   │   │
│  │                             /point_cloud      ┘                      │   │
│  └───────────────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Key design rules

- **Hardware on the host, ROS 2 in Docker.** Anything touching hardware (libcamera, Hailo NPU) runs natively.
- **Detached Docker (`-d`).** Never use `docker run -it` for long-running containers — the interactive tty causes a spin-loop burning ~170% CPU.
- **Core pinning via `taskset`.** Every process is pinned to specific cores (see [Running the Stack](#6-running-the-stack)).

### Data flow

1. **ovcam_producer** captures NV12 frames → `/ovcam_frames` shm.
2. **yolo_producer** reads frames → Hailo NPU → DFL decode + NMS → `/yolo_shm`.
3. **ovcam_bridge** publishes `/ovcam/image_raw` (mono8, ~30 Hz).
4. **yolo_bridge** publishes `/yolo/detections` (DetectionArray) + `/yolo/annotated`.
5. **OV²SLAM** runs visual odometry → publishes `/vo_pose` and `/point_cloud`.
6. **visp_servo** tracks target from YOLO detections using IBVS, uses SLAM point-cloud depth when available (falls back to bbox heuristic) → publishes `/cmd_vel` (Twist).

---

## 2. Hardware & Prerequisites

| Item | Details |
|---|---|
| **Board** | Raspberry Pi 5 (Cortex-A76 × 4, **16 GB RAM**) |
| **Camera** | OV5647 (CSI), running at 640 × 480 ~30 fps |
| **Accelerator** | Hailo-10H AI HAT+ 2 (passed through as `/dev/hailo0`) |
| **Host OS** | **Raspberry Pi OS Trixie (64-bit) — REQUIRED** |
| **Docker** | Docker Engine ≥ 24 |
| **libcamera** | Installed on the **host** (used by `ovcam_producer` outside Docker) |

> **⚠ Raspberry Pi OS Trixie is required.** The Hailo-10H AI HAT+ 2 drivers and `libcamera` integration depend on the Trixie release. Earlier releases (Bookworm, etc.) are **not supported**.

### Host packages (outside Docker)

```bash
sudo apt install -y libcamera-dev cmake g++ pkg-config
```

---

## 3. Repository Layout

```
ROS2-PROJECT-SPRING2026/
├── Dockerfile                          # Image definition (Ubuntu 24.04 + ROS 2 Jazzy)
├── README.md                           # ← this file
├── .gitignore
│
├── camera_calib/
│   └── ov5647_ov2slam.yaml             # Live camera params (640×480, calibrated)
│
├── custom_params/
│   └── euroc_mono.yaml                 # EuRoC dataset params (1280×720, placeholder calib)
│
├── models/
│   └── yolo26n_10h.hef                 # YOLO26n compiled for Hailo-10H NPU
│
├── src/
│   ├── ov2slam_ros/                    # C++ — OV²SLAM visual SLAM (main package)
│   │   ├── src/ov2slam_node.cpp        # ROS 2 node entry point
│   │   ├── include/                    # Headers (slam_params, visual_front_end, etc.)
│   │   ├── Thirdparty/                 # Ceres, Sophus, obindex2, ibow-lcd
│   │   └── CMakeLists.txt
│   │
│   ├── ovcam_producer/                 # C++ — native camera capture (runs on HOST)
│   │   ├── src/producer.cpp            # libcamera → /ovcam_frames shm
│   │   └── include/ovcam_shm.hpp       # Shared memory protocol
│   │
│   ├── ovcam_bridge/                   # C++ — ROS 2 node (runs in DOCKER)
│   │   └── src/ovcam_bridge_node.cpp   # /ovcam_frames shm → /ovcam/image_raw
│   │
│   ├── yolo_producer/                  # Python — NPU inference (runs on HOST)
│   │   ├── yolo_producer.py            # Reads camera shm → Hailo NPU → /yolo_shm
│   │   └── include/yolo_shm.hpp        # YOLO shm protocol (shared with yolo_bridge)
│   │
│   ├── yolo_bridge/                    # C++ — ROS 2 node (runs in DOCKER)
│   │   └── src/yolo_bridge_node.cpp    # /yolo_shm → /yolo/detections + /yolo/annotated
│   │
│   ├── yolo_msgs/                      # ROS 2 message definitions
│   │   └── msg/
│   │       ├── Detection.msg           # class_name, confidence, center_x/y, size_w/h
│   │       └── DetectionArray.msg      # header + Detection[]
│   │
│   └── yolo_ros/                       # Python — ROS 2 nodes
│       └── yolo_ros/controller_node.py # Subscribes DetectionArray → control logic
│
├── opencv/                             # OpenCV 4.10 source (built from source in Docker)
├── opencv_contrib/                     # OpenCV contrib modules
│
├── build/                              # colcon build output  (git-ignored)
├── install/                            # colcon install output (git-ignored)
└── log/                                # colcon log output    (git-ignored)
```

---

## 4. Docker Image

### Build

```bash
cd ~/ROS2-PROJECT-SPRING2026
docker build -t ros2_perception_stack .
```

The `Dockerfile` installs:
- Ubuntu 24.04 base with locale setup
- ROS 2 Jazzy (`ros-jazzy-ros-base`, `ros-jazzy-foxglove-bridge`, CycloneDDS, etc.)
- C++ toolchain, CMake, Eigen3, Ceres Solver *without SuiteSparse*
- Python 3 + colcon build tools
- An entrypoint that auto-sources `/opt/ros/jazzy/setup.bash` and the workspace overlay

> **Note:** OpenCV 4.10 with contrib is built from source inside the container on first use. The source lives at `/workspace/opencv/` and `/workspace/opencv_contrib/`. Both are git-ignored.

### Run

```bash
./start_container.sh
```

Or manually:

```bash
sudo docker run -it --rm \
  --name ros2_perception_stack \
  --net=host \
  --ipc=host \
  --privileged \
  --device=/dev/hailo0:/dev/hailo0 \
  -v $(pwd):/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/bus/usb:/dev/bus/usb \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  ros2_perception_stack /bin/bash
```

| Flag | Purpose |
|---|---|
| `--net=host` | Share host network for DDS discovery |
| `--ipc=host` | Access POSIX shared memory from ovcam_producer |
| `--privileged` | Device access (camera, Hailo) |
| `--device=/dev/hailo0` | Hailo-10H NPU pass-through |
| `-v $(pwd):/workspace` | Mount project directory |

### Entering additional terminals

```bash
sudo docker exec -it ros2_perception_stack /bin/bash
# then inside:
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```

---

## 5. Packages

### 5.1 ovcam_producer (Host — Native)

**Purpose:** Captures frames from the OV5647 via `libcamera` and writes them into a POSIX shared-memory ring buffer.

**Location:** `src/ovcam_producer/`

**Key files:**
- `src/producer.cpp` — main capture loop
- `include/ovcam_shm.hpp` — shared memory ring buffer protocol (`/ovcam_shm`)

**Build (on the host, NOT in Docker):**

```bash
cd src/ovcam_producer
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```

**Run:**

```bash
./ovcam_producer --width 640 --height 480 --fps 30
```

> The producer must be running **before** starting `ovcam_bridge` inside Docker.

---

### 5.2 ovcam_bridge (Docker — ROS 2 C++)

**Purpose:** Reads frames from the shared memory ring buffer and publishes them as `sensor_msgs/msg/Image` on `/ovcam/image_raw`.

**Location:** `src/ovcam_bridge/`

**Build:**

```bash
# Inside Docker
cd /workspace
colcon build --packages-select ovcam_bridge --symlink-install
source install/setup.bash
```

**Run:**

```bash
ros2 run ovcam_bridge ovcam_bridge_node
```

---

### 5.3 ov2slam (Docker — ROS 2 C++)

**Purpose:** Monocular visual SLAM. Subscribes to camera images, performs feature tracking (KLT + ORB/BRIEF), estimates camera motion, runs local Bundle Adjustment via Ceres, and maintains a sparse 3D map.

**Location:** `src/ov2slam_ros/`

> **Note:** The directory is named `ov2slam_ros` but the ROS 2 **package name** is `ov2slam`. Always use `ov2slam` in `ros2 run` and `colcon build` commands.

**Key source files:**
| File | Role |
|---|---|
| `src/ov2slam_node.cpp` | ROS 2 node: subscribes to image topic, feeds frames to SLAM |
| `src/ov2slam.cpp` | Main SLAM orchestrator |
| `src/visual_front_end.cpp` | Feature extraction, KLT tracking, motion estimation |
| `include/visual_front_end.hpp` | MotionModel class (inline), tracking parameters |
| `src/frame.cpp` | Frame class — manages pose (`Twc_`, `Tcw_`), keypoints |
| `src/mapper.cpp` | Map management, keyframe selection, BA invocation |
| `src/map_manager.cpp` | Map point lifecycle, co-visibility graph |
| `include/slam_params.hpp` | YAML parameter loader (reads all `Camera.*` and SLAM params) |

**Thirdparty dependencies (vendored):**
- **Ceres Solver** — nonlinear least squares (BA)
- **Sophus** — SE3/SO3 Lie group library
- **obindex2 / ibow-lcd** — bag-of-words for loop closure (disabled by default)

**Build:**

```bash
cd /workspace

# 1. Build Thirdparty libs (first time only)
cd src/ov2slam_ros/Thirdparty
./build_thirdparty.sh
cd /workspace

# 2. Build OV²SLAM ROS 2 package
colcon build --packages-select ov2slam --symlink-install
source install/setup.bash
```

**Run (live camera):**

```bash
export LD_LIBRARY_PATH=/workspace/opencv/build/lib:$LD_LIBRARY_PATH
ros2 run ov2slam ov2slam_node \
  /workspace/camera_calib/ov5647_ov2slam.yaml
```

**Run (EuRoC dataset):**

```bash
export LD_LIBRARY_PATH=/workspace/opencv/build/lib:$LD_LIBRARY_PATH
ros2 run ov2slam ov2slam_node \
  /workspace/custom_params/euroc_mono.yaml
```

**Published topics:**
- `/vo_pose` — `geometry_msgs/msg/PoseStamped` (visual odometry pose)
- `/point_cloud` — `sensor_msgs/msg/PointCloud2` (sparse 3D map)
- `/image_track` — feature tracking visualization
- `/tf` — camera transform frames

---

### 5.4 yolo_producer (Host — Python)

**Purpose:** Reads NV12 camera frames from `/ovcam_frames` shm, runs YOLO inference on the Hailo-10H NPU, post-processes (DFL bbox decode + NMS), and writes detections + annotated BGR to `/yolo_shm`.

**Location:** `src/yolo_producer/`

**Run (on the host):**
Download the HailoRT Python wheel for your Python version from the 
[Hailo Developer Zone](https://hailo.ai/developer-zone/request-access/) (account required)

```bash
pip3 install hailort-*.whl --break-system-packages
cd ~/ROS2-PROJECT-SPRING2026
python3 src/yolo_producer/yolo_producer.py --hef models/yolo26n_10h.hef
```

| Flag | Default | Effect |
|---|---|---|
| `--hef` | `models/yolo26n_10h.hef` | Path to compiled YOLO HEF model |
| `--conf` | `0.25` | Minimum detection confidence |
| `--iou` | `0.45` | NMS IoU threshold |

---

### 5.5 yolo_bridge (Docker — ROS 2 C++)

**Purpose:** Reads `/yolo_shm` and publishes ROS 2 topics.

**Published topics:**
| Topic | Type | QoS |
|---|---|---|
| `/yolo/detections` | `yolo_msgs/DetectionArray` | best_effort depth 1 |
| `/yolo/annotated` | `sensor_msgs/Image` (bgr8) | best_effort depth 1 |

---

### 5.6 yolo_msgs (Docker — ROS 2 Messages)

| Message | Fields |
|---|---|
| `Detection.msg` | `class_name`, `confidence`, `center_x`, `center_y`, `size_width`, `size_height` (all pixel coords) |
| `DetectionArray.msg` | `std_msgs/Header header` + `Detection[] detections` |

---

### 5.7 visp_servo (Docker — ROS 2 C++)

**Purpose:** Image-Based Visual Servoing (IBVS). Tracks a target class from YOLO detections and outputs velocity commands for a drone.

**Location:** `src/visp_servo/`

**Subscriptions:**
| Topic | Type | Notes |
|---|---|---|
| `/yolo/detections` | `DetectionArray` | Picks highest-confidence match for `target_class` |
| `/vo_pose` | `PoseStamped` | Current camera pose from OV²SLAM |
| `/point_cloud` | `PointCloud2` | SLAM sparse map for depth estimation |

**Published topics:**
| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 4-DOF velocity command (vx, vy, vz, wz) |
| `/visp/debug_image` | `sensor_msgs/Image` | Annotated debug overlay (only when subscribed) |

**Depth estimation:** Uses SLAM point cloud when OV²SLAM is running (searches for map points near the target pixel, takes median Z). Falls back to a bbox-height heuristic when SLAM data is unavailable. The log line shows `Z=X.XXm(SLAM)` or `Z=X.XXm(bbox)`.

**Key tunable parameters** (`config/servo_params.yaml`):
| Param | Default | Effect |
|---|---|---|
| `target_class` | `person` | YOLO class to track |
| `min_confidence` | `0.3` | Minimum detection confidence |
| `lambda_xy` | `0.5` | IBVS gain for xy centering |
| `target_bbox_ratio` | `0.35` | Desired bbox-height/image-height ratio (controls approach distance) |
| `lost_threshold` | `15` | Frames without detection before entering LOST state (hover) |

---

## 6. Running the Stack

### First-time build (inside Docker)

Only needed after a fresh clone or code changes:

```bash
./start_container.sh

sudo docker exec ros2_perception_stack bash -lc "
  source /opt/ros/jazzy/setup.bash
  cd /workspace
  colcon build --packages-select yolo_msgs ovcam_bridge yolo_bridge visp_servo --symlink-install
"

# OV2SLAM — only needed once or after source changes:
sudo docker exec ros2_perception_stack bash -lc "
  cd /workspace/src/ov2slam_ros/Thirdparty && ./build_thirdparty.sh
  cd /workspace
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select ov2slam --symlink-install
"
```

---

### Live startup — 6 terminals

Open each terminal in order. Wait for each one to print its startup message before opening the next.

#### Terminal 0 — Docker container

```bash
cd ~/ROS2-PROJECT-SPRING2026
./start_container.sh
```
Expected: `Container started (detached).`

#### Terminal 1 — Camera producer · Core 0

```bash
cd ~/ROS2-PROJECT-SPRING2026
taskset -c 0 src/ovcam_producer/build/ovcam_producer --width 640 --height 480 --fps 30
```
Expected:
```
[shm] created '/ovcam_frames': 4 slots x ...
[cam] actual: 640x480 stride=640
```

#### Terminal 2 — YOLO producer · Core 1

```bash
cd ~/ROS2-PROJECT-SPRING2026
taskset -c 1 python3 src/yolo_producer/yolo_producer.py --hef models/yolo26n_10h.hef
```
Expected:
```
[hailo] loading HEF: models/yolo26n_10h.hef
[yolo_shm] created: 640x480 slots=4
[yolo_producer] running — conf=0.25 iou=0.45
```

#### Terminal 3 — ROS bridges · Cores 0,1

```bash
sudo docker exec ros2_perception_stack bash -lc "
  source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  taskset -c 0,1 ros2 run ovcam_bridge ovcam_bridge_node &
  taskset -c 0,1 ros2 run yolo_bridge yolo_bridge_node
"
```
Expected:
```
[INFO] [ovcam_bridge]: shm mapped 640x480
[INFO] [yolo_bridge]: yolo_shm mapped: 640x480, 4 slots
```

#### Terminal 4 — OV2SLAM · Cores 2,3

```bash
sudo docker exec ros2_perception_stack bash -lc "
  source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\$LD_LIBRARY_PATH
  taskset -c 2,3 ros2 run ov2slam ov2slam_node /workspace/camera_calib/ov5647_ov2slam.yaml
"
```
Expected (after slow lateral camera movement):
```
 - [Visual-Front-End]: SLAM is ready to start !
        >>> All threads are ready!
```
> **Initialization tip:** SLAM needs ~10px of lateral (side-to-side) translation between frames to triangulate the first map points. Slowly pan the camera left/right — not rotation. Threshold is set to `finit_parallax: 10` px.

#### Terminal 5 — ViSP servo · Cores 0,1

```bash
sudo docker exec ros2_perception_stack bash -lc "
  source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  taskset -c 0,1 ros2 run visp_servo visp_servo_node
"
```
Expected:
```
[INFO] [visp_servo_node]: visp_servo_node started — tracking 'person' (conf >= 0.30)
[INFO] [visp_servo_node]: Camera: fx=543.1 fy=539.7 cx=310.7 cy=222.9
```
Once a person is in frame: `[TRACKING] person conf=0.XX ... Z=X.XXm(SLAM)`

---

### Monitor CPU per core

```bash
watch -n1 'ps -eo pid,psr,pcpu,comm --sort=-pcpu | grep -E "PSR|ovcam|yolo|ov2slam|visp" | head -12'
```

```
PSR = processor core (0–3)
```

Expected steady-state:
```
  PID PSR %CPU COMMAND
  ...  0   ~5  ovcam_producer
  ...  1  ~15  python3          (yolo_producer)
  ...  0   ~3  ovcam_bridge_nod
  ...  0   ~3  yolo_bridge_node
  ...  2  ~40  ov2slam_node
  ...  3  ~40  ov2slam_node     (threads)
  ...  0   ~3  visp_servo_node
```

---

### Stop everything

```bash
./run_stack.sh stop
```

Or manually:
```bash
# Ctrl+C in each terminal (reverse order: 5 → 4 → 3 → 2 → 1)
# Then:
sudo docker kill ros2_perception_stack && sudo docker rm ros2_perception_stack
sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready
```

---

## 7. EuRoC Dataset Replay

### 1. Download a bag2 sequence

```bash
mkdir -p /workspace/datasets && cd /workspace/datasets
# Download the ROS bag version directly — example: MH_01_easy
# (use the bag download links from the EuRoC dataset page)
```

### 2. Convert bag1 → bag2 (if needed)

If only a ROS 1 bag (`.bag`) is available, convert it to ROS 2 format:

```bash
ros2 bag convert -i MH_01_easy.bag -o MH_01_easy_bag2 -s rosbag_v2
```

### 3. Run OV²SLAM

```bash
export LD_LIBRARY_PATH=/workspace/opencv/build/lib:$LD_LIBRARY_PATH
ros2 run ov2slam ov2slam_node /workspace/custom_params/euroc_mono.yaml
```

### 4. In a separate terminal, play the bag

```bash
ros2 bag play /workspace/datasets/MH_01_easy_bag --clock
```

### 5. Evaluate trajectory

OV²SLAM writes trajectory files to the working directory:
- `ov2slam_traj.txt` — full trajectory (TUM format)
- `ov2slam_kfs_traj.txt` — keyframe trajectory
- `ov2slam_traj_kitti.txt` — KITTI format

Use [evo](https://github.com/MichaelGrupp/evo) to evaluate:

```bash
pip install evo
evo_ape tum /workspace/datasets/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
    ov2slam_traj.txt -va --plot
```

---

## 8. Camera Calibration

The OV5647 was calibrated at 640 × 480 using a checkerboard pattern.

| Parameter | Value |
|---|---|
| Resolution | 640 × 480 |
| fx | 543.098 |
| fy | 539.702 |
| cx | 310.687 |
| cy | 222.928 |
| k1 | −0.00984 |
| k2 | +0.07654 |
| p1 | −0.00247 |
| p2 | +0.00278 |
| RMS reprojection error | 0.9742 px |

The calibration file lives at `camera_calib/ov5647_ov2slam.yaml`.

### Recalibrating

If the resolution or lens changes, recalibrate using the provided scripts:

```bash
# 1. Capture ~30 checkerboard images at different angles
python3 capture.py

# 2. Run calibration
python3 calibrate.py
```

Update the values in `ov5647_ov2slam.yaml` with the output.

---

## 9. Tuning Reference

Key parameters in the YAML config files:

### Initialization

| Parameter | Default (live) | Effect |
|---|---|---|
| `finit_parallax` | **10.0** | Minimum pixel parallax to trigger map initialization. Lower = easier init but less accurate triangulation. (Was 25 — too hard to init handheld.) |
| `fransac_err` | 1.5 | RANSAC inlier threshold (px). Tighter = fewer but better inliers. |
| `fmax_reproj_err` | 2.0 | Max reprojection error for 3D point acceptance. |

### Performance (Raspberry Pi)

| Parameter | Default | Effect |
|---|---|---|
| `force_realtime` | 1 | When `1`, drops frames that arrive while processing is busy. **Set to 1 on Pi** to avoid queue buildup. |
| `nmaxdist` | 35 | Min pixel distance between keypoints. Higher = fewer keypoints = faster. |
| `nklt_pyr_lvl` | 3 | KLT pyramid levels. Reducing to 2 saves CPU at the cost of tracking range. |
| `bdo_track_localmap` | 1 | Track against local map (more accurate but heavier). Set to 0 for speed. |

### Bundle Adjustment

| Parameter | Default | Effect |
|---|---|---|
| `robust_mono_th` | 5.9915 | Robust cost threshold (chi² at 5%). |
| `apply_l2_after_robust` | 1 | Refine with L2 after robust iteration. |
| `nmin_covscore` | 25 | Minimum co-observations to include a keyframe in BA. |

### Loop Closure

| Parameter | Default | Effect |
|---|---|---|
| `buse_loop_closer` | 0 | Disabled by default (ibow-lcd has high RAM cost on Pi). |

---

## 10. Troubleshooting

### yolo_producer shows 0 dets

- **Not a bug** if nothing detectable is in frame. Point the camera at a person, cup, phone, or any COCO object.
- If objects are present but no detections: try lowering `--conf` to `0.1`.

### yolo_bridge can't connect to shm

- Start `yolo_producer` **before** `yolo_bridge`. The producer creates `/yolo_shm`; the bridge only reads it.
- If you restart `yolo_producer` while the bridge is running, you must also restart the bridge (it holds a stale mmap of the old shm).

### "Lost communication with the server" from Hailo

- This happens when the Python process crashes mid-inference. Not harmful — just restart `yolo_producer.py`.
- If persistent: check `ls /dev/hailo0` exists and `hailortcli fw-control identify` works.

### OV²SLAM fails to initialize

- **Symptom:** Repeated "Parallax X.X < threshold" messages.
- **Fix:** Move the camera with a strong lateral motion (translation, not pure rotation). Reduce `finit_parallax` if needed (e.g., 15–20).

### Foxglove shows low frame rate (~10 Hz)

- **Not a bug.** The Pi processes each frame through the full SLAM pipeline before accepting the next. At 640×480 with KLT + BA, ~10 Hz is expected.
- Set `force_realtime: 1` in the YAML to avoid queuing stale frames.

### Container can't see shared memory

- Ensure Docker is started with `--ipc=host`.
- Ensure `ovcam_producer` is running **before** `ovcam_bridge`.

### Image topic not received by OV²SLAM

- Check that `Camera.topic_left` in the YAML matches the actual topic:
  ```bash
  ros2 topic list | grep image
  ```
- For live camera: topic should be `/ovcam/image_raw`.

### Core dumps filling disk

- Core dumps (`core.*`) are generated on SLAM crashes. They are git-ignored.
- Clean them periodically: `rm -f ~/ROS2-PROJECT-SPRING2026/core.*`
- To disable: `ulimit -c 0` before running the node.

### Build error: "Sophus/se3.hpp not found"

- Build and install all thirdparty deps:
  ```bash
  cd src/ov2slam_ros/Thirdparty && ./build_thirdparty.sh
  ```

### Build error: "SuiteSparse not found"

- The Dockerfile intentionally omits SuiteSparse. Ceres is built without it. This is expected and does not affect functionality.

---

## 11. Applied Bug Fixes (2026-03-01)

Four patches were applied to fix NaN/Inf crashes and SLAM initialization failures. See `ov2slam_fixes.md` for the full write-up with root-cause analysis and proof.

### Summary

| Fix | File | Issue |
|---|---|---|
| **A — Timestamp** | `src/ov2slam_node.cpp` | Mono path used only `stamp.sec`, discarding nanoseconds. 97.4% of frame pairs got dt=0 → division by zero. |
| **B — Motion Model** | `include/visual_front_end.hpp` | `MotionModel` divided by dt without guarding dt≈0. Added dt threshold + `allFinite()` checks. |
| **C — Pose Validation** | `src/frame.cpp` | `setTwc()` accepted NaN/Inf poses unchecked. Added `allFinite()` and `det(R)≈1` guards. |
| **D — Topic Subscription** | `src/ov2slam_node.cpp` | Left topic was hardcoded; right topic subscription crashed in mono mode when topic string was empty. |

**Backups:** Original source files saved in `src/ov2slam_ros/src_backup_20260301_212752/`.

---

## 12. Contributors

<!-- Add your name and contribution below -->
| Name | Role / Contribution |
|---|---|
| Amar Aly | OV²SLAM setup, integration, ROS2 Environment setup and Camera Calibration |
| Ahmad Dhaoudi | YOLOv26n conversion to .hef for Hailo10H, Yolo detection and setup, ViSP integration |

---

*Last updated: 2026-03-07*
