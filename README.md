# ROS 2 Perception Stack — Comprehensive Guide

> Raspberry Pi + OV5647 mono camera → OV²SLAM visual odometry  
> **ROS 2 Jazzy** · Ubuntu 24.04 · Docker · CycloneDDS

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Hardware & Prerequisites](#2-hardware--prerequisites)
3. [Repository Layout](#3-repository-layout)
4. [Docker Image](#4-docker-image)
5. [Packages](#5-packages)
6. [Terminal Workflow (Live Camera)](#6-terminal-workflow-live-camera)
7. [EuRoC Dataset Replay](#7-euroc-dataset-replay)
8. [Camera Calibration](#8-camera-calibration)
9. [Tuning Reference](#9-tuning-reference)
10. [Troubleshooting](#10-troubleshooting)
11. [Applied Bug Fixes (2026-03-01)](#11-applied-bug-fixes-2026-03-01)
12. [Contributors](#12-contributors)

---

## 1. Architecture Overview

```
┌──────────────────────── HOST (Raspberry Pi) ────────────────────────┐
│                                                                      │
│  Terminal 1 — ovcam_producer (native, libcamera + POSIX shm)        │
│        ↓  shared memory ring buffer (/ovcam_shm)                    │
│  ┌────────────────────── DOCKER CONTAINER ──────────────────────┐   │
│  │                                                                │   │
│  │  Terminal 2 — ovcam_bridge                                     │   │
│  │        reads shm → publishes /ovcam/image_raw (sensor_msgs)   │   │
│  │                          ↓                                     │   │
│  │  Terminal 3 — OV²SLAM                                          │   │
│  │        subscribes /ovcam/image_raw → visual SLAM               │   │
│  │        publishes /ov2slam/odom, /ov2slam/pointcloud, etc.      │   │
│  │                          ↓                                     │   │
│  │  Terminal 4 — Foxglove Bridge (port 8765)                      │   │
│  │        exposes all topics to Foxglove Studio desktop app       │   │
│  │                                                                │   │
│  │  Terminal 5 — Debug shell (ros2 topic hz, echo, rqt, etc.)     │   │
│  └────────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────┘
```

**Data flow summary:**

1. **ovcam_producer** (native) captures frames via `libcamera` and writes them into a POSIX shared-memory ring buffer.
2. **ovcam_bridge** (Docker) reads the ring buffer and publishes `sensor_msgs/msg/Image` on `/ovcam/image_raw`.
3. **OV²SLAM** subscribes, runs feature extraction → KLT tracking → motion estimation → local BA (Ceres) → map management.
4. **Foxglove Bridge** exposes every ROS 2 topic over WebSocket for remote visualization.

---

## 2. Hardware & Prerequisites

| Item | Details |
|---|---|
| **Board** | Raspberry Pi 5 (Cortex-A76 × 4) |
| **Camera** | OV5647 (CSI), running at 640 × 480 ~30 fps |
| **Accelerator** | Hailo-8L M.2 (passed through as `/dev/hailo0`) |
| **Host OS** | Raspberry Pi OS or Ubuntu 24.04 (64-bit) |
| **Docker** | Docker Engine ≥ 24 |
| **libcamera** | Installed on the **host** (used by `ovcam_producer` outside Docker) |

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
├── src/
│   ├── ov2slam_ros/                    # C++ — OV²SLAM visual SLAM (main package)
│   │   ├── src/ov2slam_node.cpp        # ROS 2 node entry point
│   │   ├── include/                    # Headers (slam_params, visual_front_end, etc.)
│   │   ├── Thirdparty/                 # Ceres, Sophus, obindex2, ibow-lcd
│   │   └── CMakeLists.txt
│   │
│   ├── ovcam_producer/                 # C++ — native camera capture (runs on host)
│   │   ├── src/producer.cpp            # libcamera → shared memory ring buffer
│   │   └── include/ovcam_shm.hpp       # Shared memory header (ring buffer protocol)
│   │
│   ├── ovcam_bridge/                   # C++ — ROS 2 node (runs in Docker)
│   │   └── src/bridge_node.cpp         # Reads shm → publishes Image on /ovcam/image_raw
│   │
│   └── ovcam_ros/                      # (WIP — planned integration)
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
| `--device=/dev/hailo0` | Hailo-8L accelerator pass-through |
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
colcon build --packages-select ovcam_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**Run:**

```bash
ros2 run ovcam_bridge ovcam_bridge_node
```

---

### 5.3 ov2slam_ros (Docker — ROS 2 C++)

**Purpose:** Monocular visual SLAM. Subscribes to camera images, performs feature tracking (KLT + ORB/BRIEF), estimates camera motion, runs local Bundle Adjustment via Ceres, and maintains a sparse 3D map.

**Location:** `src/ov2slam_ros/`

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
colcon build --packages-select ov2slam_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**Run (live camera):**

```bash
ros2 run ov2slam_ros ov2slam_node \
  /workspace/camera_calib/ov5647_ov2slam.yaml
```

**Run (EuRoC dataset):**

```bash
ros2 run ov2slam_ros ov2slam_node \
  /workspace/custom_params/euroc_mono.yaml
```

**Published topics:**
- `/ov2slam/odom` — `nav_msgs/msg/Odometry`
- `/ov2slam/pointcloud` — `sensor_msgs/msg/PointCloud2`
- Frame-to-frame tracking visualization (image with keypoints)

---

### 5.4 Foxglove Bridge

**Purpose:** Exposes all ROS 2 topics over WebSocket (port 8765) for visualization in [Foxglove Studio](https://foxglove.dev/).

**Run:**

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then connect Foxglove Studio (desktop or web) to `ws://<PI_IP>:8765`.

---

## 6. Terminal Workflow (Live Camera)

Open four terminal sessions. Terminal 1 runs on the **host**; terminals 2–5 run **inside the Docker container**.

### Terminal 1 — Camera Producer (Host)

```bash
cd ~/ROS2-PROJECT-SPRING2026/src/ovcam_producer/build
./ovcam_producer --width 640 --height 480 --fps 30
```

### Terminal 2 — Image Bridge (Docker)

```bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
ros2 run ovcam_bridge ovcam_bridge_node
```

### Terminal 3 — OV²SLAM (Docker)

```bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
ros2 run ov2slam_ros ov2slam_node /workspace/camera_calib/ov5647_ov2slam.yaml
```

### Terminal 4 — Foxglove Bridge (Docker)

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### Terminal 5 — Debug Shell (Docker)

```bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash

# Useful commands:
ros2 topic list
ros2 topic hz /ovcam/image_raw
ros2 topic echo /ov2slam/odom --once
```

---

## 7. EuRoC Dataset Replay

See also `README.md` for downloadable links.

> **Important:** Download the **ROS bag** version of each sequence (not the ASL / zip format).  
> The bag files are available on the [EuRoC MAV Dataset page](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) under the "ROS bag" column.

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
ros2 run ov2slam_ros ov2slam_node /workspace/custom_params/euroc_mono.yaml
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
| `finit_parallax` | 25.0 | Minimum pixel parallax to trigger map initialization. Lower = easier init but less accurate triangulation. |
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

*Last updated: 2026-03-01*
