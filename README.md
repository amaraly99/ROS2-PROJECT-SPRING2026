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
6. [YOLO NPU Pipeline — Quick-Start & Verification](#6-yolo-npu-pipeline--quick-start--verification)
7. [Foxglove Live Visualization](#7-foxglove-live-visualization)
8. [Terminal Workflow — Full Stack (Camera + YOLO + SLAM)](#8-terminal-workflow--full-stack-camera--yolo--slam)
9. [EuRoC Dataset Replay](#9-euroc-dataset-replay)
10. [Camera Calibration](#10-camera-calibration)
11. [Tuning Reference](#11-tuning-reference)
12. [Troubleshooting](#12-troubleshooting)
13. [Applied Bug Fixes (2026-03-01)](#13-applied-bug-fixes-2026-03-01)
14. [Contributors](#14-contributors)

---

## 1. Architecture Overview

```
┌───────────────────────── HOST (Raspberry Pi 5) ─────────────────────────┐
│                                                                          │
│  ovcam_producer  (C++, libcamera)                                       │
│       │  writes NV12 frames                                              │
│       ▼                                                                  │
│  /ovcam_frames   ◄── POSIX shared memory ring buffer ──►  (both read)   │
│       │                                                       │          │
│       │                                                       │          │
│       │                                    yolo_producer  (Python)       │
│       │                                    reads NV12 → BGR → letterbox   │
│       │                                    Hailo-10H NPU inference       │
│       │                                    post-process (DFL+NMS)        │
│       │                                    draws annotations             │
│       │                                         │  writes detections     │
│       │                                         ▼  + annotated BGR       │
│       │                                    /yolo_shm  (POSIX shm)       │
│       │                                         │                        │
│  ┌────┼─────────────── DOCKER CONTAINER ────────┼──────────────────┐     │
│  │    │                                         │                  │     │
│  │    ▼                                         ▼                  │     │
│  │  ovcam_bridge                           yolo_bridge             │     │
│  │  shm → /ovcam/image_raw (mono8)         shm → /yolo/detections │     │
│  │         │                                     /yolo/annotated   │     │
│  │         ▼                                         │             │     │
│  │  OV²SLAM (visual SLAM)                           ▼             │     │
│  │  → /ov2slam/odom                          controller_node      │     │
│  │  → /ov2slam/pointcloud                   (drone control logic) │     │
│  │         │                                                       │     │
│  │         ▼                                                       │     │
│  │  Foxglove Bridge (:8765) ── all topics → Foxglove Studio       │     │
│  └─────────────────────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────────────────────┘
```

### Key design rule

> **Hardware on the host, ROS 2 in Docker.**  
> Anything that touches hardware (camera, NPU) runs natively on the host.  
> Docker only does ROS 2 bridging and compute that doesn't need device access.

### Data flow summary

1. **ovcam_producer** (host) captures NV12 frames via `libcamera` → writes to `/ovcam_frames` shared memory.
2. **yolo_producer** (host) reads the same NV12 frames → converts to BGR → letterbox-resizes to 640×640 (preserving aspect ratio, gray padding) → runs YOLO inference on Hailo-10H NPU → post-processes (DFL bbox decode + NMS) → writes detections + annotated BGR frame to `/yolo_shm`.
3. **ovcam_bridge** (Docker) reads `/ovcam_frames` → publishes mono8 on `/ovcam/image_raw` (for SLAM).
4. **yolo_bridge** (Docker) reads `/yolo_shm` → publishes `DetectionArray` on `/yolo/detections` + BGR `Image` on `/yolo/annotated`.
5. **controller_node** (Docker) subscribes to `/yolo/detections` → picks highest-confidence target → outputs control direction.
6. **OV²SLAM** (Docker) subscribes to `/ovcam/image_raw` → visual odometry.
7. **Foxglove Bridge** (Docker) exposes every ROS 2 topic over WebSocket for visualization.

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

### 5.4 Foxglove Bridge

**Purpose:** Exposes all ROS 2 topics over WebSocket (port 8765) for visualization in [Foxglove Studio](https://foxglove.dev/).

**Run:**

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then connect Foxglove Studio (desktop or web) to `ws://<PI_IP>:8765`.

---

### 5.5 yolo_producer (Host — Python)

**Purpose:** Reads NV12 camera frames from `/ovcam_frames` shared memory, runs YOLO inference on the Hailo-10H NPU, post-processes detections (DFL bounding box decode + NMS), draws annotated bounding boxes, and writes results to `/yolo_shm`.

**Location:** `src/yolo_producer/`

**Key files:**
- `yolo_producer.py` — main inference loop
- `include/yolo_shm.hpp` — shared memory protocol (shared with `yolo_bridge`)

**Run (on the host, NOT in Docker):**

```bash
cd ~/ROS2-PROJECT-SPRING2026
python3 src/yolo_producer/yolo_producer.py --hef models/yolo26n_10h.hef
```

> Requires `ovcam_producer` to be running first.

---

### 5.6 yolo_bridge (Docker — ROS 2 C++)

**Purpose:** Reads detections + annotated BGR frame from `/yolo_shm` and publishes them as ROS 2 topics.

**Location:** `src/yolo_bridge/`

**Published topics:**
| Topic | Type | QoS | Content |
|---|---|---|---|
| `/yolo/detections` | `yolo_msgs/DetectionArray` | best_effort | Array of detections (class, confidence, bbox) |
| `/yolo/annotated` | `sensor_msgs/Image` (bgr8) | best_effort | Camera frame with drawn bounding boxes |

**Build & Run:**

```bash
# Inside Docker
colcon build --packages-select yolo_bridge --symlink-install
source install/setup.bash
ros2 run yolo_bridge yolo_bridge_node
```

---

### 5.7 yolo_msgs (Docker — ROS 2 Messages)

**Purpose:** Defines the custom message types for YOLO detections.

**Location:** `src/yolo_msgs/msg/`

| Message | Fields |
|---|---|
| `Detection.msg` | `class_name`, `confidence`, `center_x`, `center_y`, `size_width`, `size_height` |
| `DetectionArray.msg` | `std_msgs/Header header` + `Detection[] detections` |

**Build:**

```bash
colcon build --packages-select yolo_msgs
```

---

### 5.8 yolo_ros (Docker — Python)

**Purpose:** High-level ROS 2 nodes that consume YOLO detections.

**Location:** `src/yolo_ros/`

**Nodes:**
- `controller` — subscribes to `/yolo/detections`, picks highest-confidence detection, outputs directional control (left/center/right based on target position).

**Run:**

```bash
ros2 run yolo_ros controller
```

---

## 6. YOLO NPU Pipeline — Quick-Start & Verification

This section walks you through running the full YOLO NPU pipeline from scratch and verifying everything works. **Read the whole section first**, then follow step by step.

### What you're about to run

```
Camera (OV5647, 30fps)
  └─► ovcam_producer (HOST) ─── /ovcam_frames shm ───┐
                                                       ├─► ovcam_bridge (DOCKER) ─► /ovcam/image_raw (mono8, 30Hz)
                                                       │
                                                       └─► yolo_producer (HOST)
                                                             │  NV12 → BGR → letterbox 640×640
                                                             │  Hailo-10H NPU inference (~30fps)
                                                             │  DFL decode + NMS
                                                             ▼
                                                        /yolo_shm ─► yolo_bridge (DOCKER)
                                                                       ├─► /yolo/detections  (DetectionArray, ~25Hz)
                                                                       └─► /yolo/annotated   (Image bgr8, ~25Hz)
```

### Prerequisites

- `ovcam_producer` is already built (see [section 5.1](#51-ovcam_producer-host--native))
- Docker container `ros2_perception_stack` is running
- All Docker packages are built:
  ```bash
  # Inside Docker:
  source /opt/ros/jazzy/setup.bash
  cd /workspace
  colcon build --packages-select yolo_msgs ovcam_bridge yolo_bridge yolo_ros --symlink-install
  source install/setup.bash
  ```

### Step-by-step (4 terminals)

#### Terminal 1 — Camera producer (HOST)

Open a terminal **on the Pi** (not in Docker):

```bash
cd ~/ROS2-PROJECT-SPRING2026
src/ovcam_producer/build/ovcam_producer --width 640 --height 480 --fps 30
```

You should see:
```
[shm] created '/ovcam_frames': 4 slots x ...
[cam] actual: 640x480 stride=640
```

> Leave this running. It streams camera frames into shared memory.

#### Terminal 2 — YOLO producer (HOST)

Open another terminal **on the Pi** (not in Docker):

```bash
cd ~/ROS2-PROJECT-SPRING2026
python3 src/yolo_producer/yolo_producer.py --hef models/yolo26n_10h.hef
```

You should see:
```
[hailo] loading HEF: models/yolo26n_10h.hef
[hailo] input: 640×640×3
[hailo] outputs: ['yolo26n_v2/conv61', ...]
[ovcam] opened: 640×480 stride=640 slots=4
[yolo_shm] created: 640×480 slots=4 slot_size=922752
[yolo_producer] running — conf=0.25 iou=0.45
[yolo_producer] 100 frames, 25.0 fps, 1 dets
```

> The `N dets` count shows how many objects YOLO detected in the latest frame.
> Point the camera at a person or a common object (cup, phone, etc.) to see detections.

#### Terminal 3 — ROS 2 bridges (DOCKER)

Enter the Docker container:

```bash
sudo docker exec -it ros2_perception_stack /bin/bash
```

Then inside:

```bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
ros2 run ovcam_bridge ovcam_bridge_node &
ros2 run yolo_bridge yolo_bridge_node
```

You should see:
```
[INFO] [yolo_bridge]: yolo_shm mapped: 640x480, 4 slots
```

#### Terminal 4 — Verify everything (DOCKER)

Enter the Docker container in a new terminal:

```bash
sudo docker exec -it ros2_perception_stack /bin/bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
```

Now run these checks:

**Check 1 — Topics exist:**
```bash
ros2 topic list
```
Expected output should include:
```
/ovcam/image_raw
/yolo/annotated
/yolo/detections
```

**Check 2 — Detections flowing at ~25 Hz:**
```bash
ros2 topic hz /yolo/detections
```
Expected:
```
average rate: 25.0
```

**Check 3 — See actual detections (point camera at a person):**
```bash
ros2 topic echo /yolo/detections --once
```
Expected:
```yaml
header:
  stamp:
    sec: 6249
    nanosec: 661330294
  frame_id: camera
detections:
- class_name: person
  confidence: 0.7796
  center_x: 354.5
  center_y: 279.0
  size_width: 417.0
  size_height: 396.0
```

**Check 4 — Annotated images flowing:**
```bash
ros2 topic hz /yolo/annotated
```
Expected: `average rate: ~25`

**Check 5 — Camera bridge (SLAM feed) flowing:**
```bash
ros2 topic hz /ovcam/image_raw
```
Expected: `average rate: ~30`

### Stopping everything

**Important:** Always stop processes when you're done, or CPU stays pegged.

```bash
# In Docker — stop bridges:
pkill -f yolo_bridge_node
pkill -f ovcam_bridge_node

# On the host — stop producers:
pkill -f yolo_producer.py
pkill -f ovcam_producer
```

Or press `Ctrl+C` in each terminal window (in reverse order: bridges first, then producers).

### Configuration options

| Flag | Default | What it does |
|---|---|---|
| `--hef <path>` | `models/yolo26n_10h.hef` | Path to compiled YOLO model |
| `--conf <float>` | `0.25` | Minimum confidence to report a detection |
| `--iou <float>` | `0.45` | NMS IoU threshold (lower = fewer overlapping boxes) |

Example with tighter threshold:
```bash
python3 src/yolo_producer/yolo_producer.py --hef models/yolo26n_10h.hef --conf 0.5
```

### What gets detected

The model recognizes the **80 COCO classes**: person, bicycle, car, motorcycle, airplane, bus, train, truck, boat, traffic light, fire hydrant, stop sign, bench, bird, cat, dog, horse, cow, elephant, bear, zebra, giraffe, backpack, umbrella, handbag, suitcase, bottle, cup, fork, knife, spoon, bowl, banana, apple, sandwich, orange, pizza, donut, cake, chair, couch, bed, dining table, toilet, tv, laptop, mouse, remote, keyboard, cell phone, microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, teddy bear, toothbrush, and more.

---

## 7. Foxglove Live Visualization

See YOLO detections and annotated camera feed live in your browser or desktop app.

### Step 1 — Start the Foxglove Bridge (DOCKER)

After starting all 4 terminals from [section 6](#6-yolo-npu-pipeline--quick-start--verification), open one more Docker terminal:

```bash
sudo docker exec -it ros2_perception_stack /bin/bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

You should see:
```
[foxglove_bridge]: Starting Foxglove WebSocket bridge on port 8765
```

### Step 2 — Connect Foxglove Studio

1. Open [Foxglove Studio](https://app.foxglove.dev/) in a browser on any computer on the same network, OR download the desktop app from https://foxglove.dev/download
2. Click **"Open connection"**
3. Select **"Foxglove WebSocket"**
4. Enter: `ws://<PI_IP>:8765` (e.g., `ws://192.168.0.120:8765`)
5. Click **Connect**

### Step 3 — Add panels

- **Image panel** → set topic to `/yolo/annotated` → you'll see the live camera feed with bounding boxes drawn on detected objects
- **Image panel** → set topic to `/ovcam/image_raw` → raw mono camera feed (what SLAM sees)
- **Log panel** or **Raw Messages panel** → set topic to `/yolo/detections` → see detection messages in real time

> **Tip:** The Pi's IP can be found by running `hostname -I` on the Pi.

---

## 8. Terminal Workflow — Full Stack (Camera + YOLO + SLAM)

To run everything together (YOLO detection + OV²SLAM + visualization), open 6 terminals:

### Terminal 1 — Camera Producer (HOST)

```bash
cd ~/ROS2-PROJECT-SPRING2026
src/ovcam_producer/build/ovcam_producer --width 640 --height 480 --fps 30
```

### Terminal 2 — YOLO Producer (HOST)

```bash
cd ~/ROS2-PROJECT-SPRING2026
python3 src/yolo_producer/yolo_producer.py --hef models/yolo26n_10h.hef
```

### Terminal 3 — Bridges (DOCKER)

```bash
sudo docker exec -it ros2_perception_stack /bin/bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
ros2 run ovcam_bridge ovcam_bridge_node &
ros2 run yolo_bridge yolo_bridge_node
```

### Terminal 4 — OV²SLAM (DOCKER)

```bash
sudo docker exec -it ros2_perception_stack /bin/bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
export LD_LIBRARY_PATH=/workspace/opencv/build/lib:$LD_LIBRARY_PATH
ros2 run ov2slam ov2slam_node /workspace/camera_calib/ov5647_ov2slam.yaml
```

### Terminal 5 — Foxglove Bridge (DOCKER)

```bash
sudo docker exec -it ros2_perception_stack /bin/bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### Terminal 6 — Debug Shell (DOCKER)

```bash
sudo docker exec -it ros2_perception_stack /bin/bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash

# Useful commands:
ros2 topic list
ros2 topic hz /yolo/detections
ros2 topic echo /yolo/detections --once
ros2 topic hz /ovcam/image_raw
ros2 topic echo /ov2slam/odom --once
```

---

## 9. EuRoC Dataset Replay

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

## 10. Camera Calibration

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

## 11. Tuning Reference

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

## 12. Troubleshooting

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

## 13. Applied Bug Fixes (2026-03-01)

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

## 14. Contributors

<!-- Add your name and contribution below -->
| Name | Role / Contribution |
|---|---|
| Amar Aly | OV²SLAM setup, integration, ROS2 Environment setup and Camera Calibration |
| Ahmad Dhaoudi | YOLOv26n conversion to .hef for Hailo10H, Yolo detection and setup, ViSP integration |

---

*Last updated: 2026-03-05*
