# HIL Stack — System Status Report
**Date:** 2026-05-15  
**Branch:** `sim-hil`  **Baseline Commit:** `5f9f6c5`  
**Status: FULLY OPERATIONAL**

---

## 1. Hardware Platform

| Component | Specification |
|---|---|
| Board | Raspberry Pi 5 Model B Rev 1.1 |
| CPU | ARM Cortex-A76 × 4 cores @ 2.4 GHz (aarch64) |
| RAM | 16 GB LPDDR4X (15.8 GB usable; ~1.3 GB in use at idle) |
| NPU | Hailo-10H HAT (`/dev/hailo0`) |
| Camera (physical) | OV5647 CSI, 640×480 @ 30 fps (unused in HIL mode) |
| Storage | microSD (OS) + NVMe (workspace) |
| Kernel | Linux 6.12.62+rpt-rpi-2712, Debian 13 (Trixie), aarch64 |

**Network interfaces:**

| Interface | IP | Role |
|---|---|---|
| eth0 | 192.168.56.2/24 | VirtualBox host-only — DDS link to MATLAB |
| wlan0 | 192.168.0.120/24 | Lab WiFi (NOT used for DDS) |
| docker0 | 172.17.0.1/16 | Docker bridge (NOT used for DDS) |

---

## 2. Software Stack

### Host (Raspberry Pi OS / Debian 13)
| Component | Version / Detail |
|---|---|
| OS | Debian GNU/Linux 13 (Trixie) |
| Python | 3.x (host, for yolo_producer) |
| HailoRT | Python SDK + /dev/hailo0 device node |
| Docker | Running `ros2_perception_stack` container with `--net=host --ipc=host --privileged` |

### Docker Container (Ubuntu 24.04 LTS)
| Component | Version |
|---|---|
| Base OS | Ubuntu 24.04.4 LTS (Noble) |
| ROS2 | Jazzy Jalopy |
| RMW | `rmw_fastrtps_cpp` v8.4.3 (Fast DDS) |
| Fast DDS | 8.4.3 |
| OpenCV | Custom build with contrib @ `/workspace/opencv/build` |
| cv_bridge | ros-jazzy-cv-bridge 4.1.0 |

### MATLAB Side (Windows, 192.168.56.1)
| Component | Detail |
|---|---|
| Platform | Windows 11, VirtualBox host-only adapter |
| ROS Toolbox | MATLAB ROS2 Toolbox |
| RMW | rmw_fastrtps_cpp (set via `setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')`) |
| ROS Domain | 0 |

---

## 3. HIL Architecture

```
MATLAB/Simulink (Windows 192.168.56.1)
│
│   /sim/camera/image_raw  (sensor_msgs/Image, rgb8, 640×480, up to 50 Hz)
│   /sim/drone_pose        (Float64MultiArray [x,y,z,pitch,yaw], 20 Hz)
│   /sim/pitch_angle       (Float64, 20 Hz)
│   /sim/heartbeat         (Float64 = sim_time, 20 Hz)
│   /sim/target_pose       (Float64MultiArray [x,y,z,yaw], 1 Hz latch)
│                  ↓ (Fast DDS, eth0 unicast)
┌──────────────────────────────────────────────────────────────────┐
│ Docker container: ros2_perception_stack  (--net=host, --ipc=host)│
│                                                                  │
│  sim_camera_bridge_node (Core 0)                                 │
│    Subscribes /sim/camera/image_raw (BEST_EFFORT)                │
│    Converts rgb8 → NV12                                          │
│    Writes /ovcam_frames POSIX SHM (seqlock, 4 slots)            │
│    Posts sem /ovcam_ready                                        │
│                  ↓ SHM                                           │
│  ovcam_bridge_node (Core 0)           yolo_producer (HOST Core1) │
│    Reads /ovcam_frames SHM             Reads /ovcam_frames SHM   │
│    Publishes /ovcam/image_raw          Runs YOLOv8-nano on Hailo │
│    (mono8, RELIABLE)                   Writes /yolo_shm          │
│           ↓                            Posts sem /yolo_ready     │
│  ov2slam_node (Cores 2-3)                     ↓ SHM             │
│    Subscribes /ovcam/image_raw          yolo_bridge_node (Core 0)│
│    Runs visual SLAM (KLT+BA)            Reads /yolo_shm          │
│    Publishes:                           Publishes:               │
│      /vo_pose (PoseStamped)               /yolo/detections       │
│      /image_track (Image)                 /yolo/annotated        │
│      /point_cloud (PointCloud2)                   ↓             │
│      /tf, /cam_pose_visual             visp_servo_node (Core 0)  │
│      /kfs_traj, /vo_traj               Subscribes:              │
│                                          /yolo/detections        │
│                                          /sim/drone_pose         │
│                                          /sim/pitch_angle        │
│                                          /sim/heartbeat          │
│                                          /sim/target_pose        │
│                                        Publishes:                │
│                                          /cmd_vel (Twist)        │
│                                          /visp/debug_image       │
└──────────────────────────────────────────────────────────────────┘
        │ /cmd_vel (geometry_msgs/Twist, 20 Hz) → MATLAB
        │ /visp/debug_image → MATLAB (optional display)
```

### Core Layout (Pi 5, 4 cores)
| Core | Process | Affinity |
|---|---|---|
| Core 0 | sim_camera_bridge + ovcam_bridge + yolo_bridge + visp_servo | `taskset` inside Docker |
| Core 1 | yolo_producer (host, native) | `taskset -c 1` |
| Core 2–3 | ov2slam | launch affinity |

---

## 4. IPC: Shared Memory Protocol

The `ovcam_frames` SHM is the backbone of the frame pipeline. Key properties:

| Parameter | Value |
|---|---|
| SHM name | `/ovcam_frames` |
| Format | NV12 (Y-plane full res + interleaved UV half-res) |
| Frame size | 640 × 480 × 1.5 = 460,800 bytes |
| Ring buffer slots | 4 |
| Total SHM size | ~1.84 MB |
| Synchronization | POSIX named semaphore `/ovcam_ready` + seqlock per slot |
| Magic | `0x4F564342` ("OVCB") |
| Slot header | 64 bytes (alignas(64) with seqlock, timestamp, dims, fourcc) |
| Global header | 128 bytes (alignas(64), write_seq on own cache line) |
| Atomics | GCC `__atomic_*` builtins (NOT std::atomic — unsafe in mmap'd SHM) |

**Semaphore permissions:** `chmod 666` applied at launch so host `yolo_producer` (non-root) can post/wait on the root-created semaphore.

---

## 5. ROS2 Topic Map

| Topic | Type | Publishers | Subscribers | Direction | Hz (MATLAB 20 Hz) |
|---|---|---|---|---|---|
| `/sim/camera/image_raw` | `sensor_msgs/Image` | 1 (MATLAB) | 2 (sim_camera_bridge + MATLAB sub) | MATLAB→Pi | = MATLAB rate |
| `/sim/drone_pose` | `std_msgs/Float64MultiArray` [x,y,z,pitch,yaw] | 1 (MATLAB) | 1 (visp_servo) | MATLAB→Pi | ~17 Hz |
| `/sim/pitch_angle` | `std_msgs/Float64` | 1 (MATLAB) | 1 (visp_servo) | MATLAB→Pi | ~17 Hz |
| `/sim/heartbeat` | `std_msgs/Float64` (sim_time) | 1 (MATLAB) | 1 (visp_servo) | MATLAB→Pi | ~17 Hz |
| `/sim/target_pose` | `std_msgs/Float64MultiArray` [x,y,z,yaw] | 1 (MATLAB) | 1 (visp_servo) | MATLAB→Pi | ~1 Hz (latch) |
| `/ovcam/image_raw` | `sensor_msgs/Image` (mono8) | 1 (ovcam_bridge) | 2 (ov2slam + visp_servo diag) | Pi-internal | = camera rate |
| `/yolo/detections` | `yolo_msgs/DetectionArray` | 1 (yolo_bridge) | 1 (visp_servo) | Pi-internal | ~16 Hz (sim running) |
| `/yolo/annotated` | `sensor_msgs/Image` | 1 (yolo_bridge) | 0 | Pi-internal | ~16 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` [vx,vy,vz,_,wy,wz] | 1 (visp_servo) | 1 (MATLAB) | Pi→MATLAB | ~16–20 Hz |
| `/visp/debug_image` | `sensor_msgs/Image` | 1 (visp_servo) | 1 (MATLAB) | Pi→MATLAB | = camera rate |
| `/image_track` | `sensor_msgs/Image` | 1 (ov2slam) | 0 | Pi-internal | = camera rate |
| `/vo_pose` | `geometry_msgs/PoseStamped` | 1 (ov2slam) | 0 | Pi-internal | = camera rate |
| `/point_cloud` | `sensor_msgs/PointCloud2` | 1 (ov2slam) | 0 | Pi-internal | low |
| `/cam_pose_visual` | `visualization_msgs/MarkerArray` | 1 (ov2slam) | 0 | Pi-internal | low |
| `/tf` | `tf2_msgs/TFMessage` | 1 (ov2slam) | 0 | Pi-internal | low |

### Confirmed Topic Rates (baseline session, MATLAB @ 20 Hz, Simulink running)

| Topic | Measured Rate | Notes |
|---|---|---|
| `/sim/camera/image_raw` | **~17–20 Hz** | Scales 1:1 with MATLAB timer period |
| `/sim/drone_pose` | **17.1 Hz** | Tiny msg, 85% delivery pre-fix was 6.5 Hz |
| `/sim/heartbeat` | **17.2 Hz** | Same MATLAB timer |
| `/sim/pitch_angle` | **17.1 Hz** | Same MATLAB timer |
| `/sim/target_pose` | **0.9 Hz** | 1 Hz republish, transient_local durability |
| `/ovcam/image_raw` | **9.2 Hz** | Matches camera input rate (MATLAB @ 10 Hz test) |
| `/yolo/detections` | **~16 Hz** | Sim running, Hailo NPU throughput limited |
| `/cmd_vel` | **~16–20 Hz** | When visp TRACKING or APPROACHING |
| `/visp/debug_image` | **9.4 Hz** | Matches camera rate |
| `/image_track` | **8.3 Hz** | OV2SLAM output |
| `/vo_pose` | **8.2 Hz** | OV2SLAM pose |

**Rate scaling law:** `/sim/camera/image_raw` delivery rate = MATLAB timer period reciprocal, verified at 10 Hz (period=0.1s) and confirmed scalable to 50 Hz (period=0.02s).

---

## 6. DDS Configuration (Fixed — 3 Bugs)

### Fast DDS Profile: `config/hil/fastrtps_hil.xml`

Two UDPv4 transport descriptors:

**udp_eth0** (cross-machine MATLAB↔Pi):
- `sendBufferSize` / `receiveBufferSize`: **16,777,216 bytes (16 MB)**
- `interfaceWhiteList`: `${PI_LOCAL_IP}` (= 192.168.56.2, eth0 only)
- Purpose: Large buffers prevent fragment loss for 921 KB rgb8 frames (~660 UDP datagrams at MTU 1500); whitelist prevents wlan0/docker0 from polluting SIMPLE discovery

**udp_lo** (intra-Pi node communication):
- Default buffer sizes
- `interfaceWhiteList`: `127.0.0.1` (loopback only)
- Purpose: Without this, `useBuiltinTransports=false` kills loopback DDS and Pi nodes cannot discover each other

Participant settings:
- `useBuiltinTransports`: `false`
- `discoveryProtocol`: `SIMPLE`
- `leaseDuration`: `2,147,483,647` sec (effectively infinite — prevents re-discovery churn)
- `initialPeersList`: MATLAB IP only (`192.168.56.1`)
- No multicast locators (absent entirely — NOT self-closing `<tag/>`)

**Kernel sysctl tuning** (applied idempotently at stack start):
```
net.core.rmem_max          = 16,777,216  (16 MB)
net.core.rmem_default      = 16,777,216
net.core.wmem_max          = 16,777,216
net.core.wmem_default      = 16,777,216
net.ipv4.ipfrag_high_thresh = 33,554,432  (32 MB)
net.ipv4.ipfrag_low_thresh  = 25,165,824  (24 MB)
net.ipv4.ipfrag_time        = 3 sec
```

### The 3 Stacked DDS Bugs (Root Cause Analysis)

**Bug 1 — Interface pollution (original config, symptom: 32% delivery)**  
No `interfaceWhiteList` → Fast DDS ran SIMPLE discovery on all three interfaces (eth0, wlan0, docker0). MATLAB only exists on the 192.168.56.x subnet. Discovery state became unstable. Even 200-byte `/sim/drone_pose` messages arrived at only 6.5/20 Hz = 32%. Fix: `interfaceWhiteList` → `192.168.56.2`.

**Bug 2 — Self-closing empty XML tags (symptom: profile silently ignored)**  
This version of Fast DDS (Jazzy, v8.4.3) rejects `<tagname/>` self-closing empty tags. The parser logs "Node `<tagname>` without content" and aborts parsing the entire `<participant>` block, silently falling back to defaults. Effect: the entire profile had zero effect. Fix: remove all empty tags (absent entirely, not self-closing).

**Bug 3 — Missing loopback transport (symptom: `/ovcam/image_raw` vanished)**  
`interfaceWhiteList=eth0` + `useBuiltinTransports=false` removed the loopback UDP transport. DDS uses loopback for intra-machine node discovery. Result: Pi nodes inside the same container could not discover each other — `/ovcam/image_raw` disappeared from `ros2 topic list`, ovcam_bridge ran silently outputting nothing. Fix: added second `udp_lo` transport with `interfaceWhiteList=127.0.0.1`.

---

## 7. Camera & Calibration

**Camera:** OV5647 (physical, used on `main` branch; replaced by SHM bridge in HIL)

| Parameter | Value |
|---|---|
| Resolution | 640 × 480 |
| fx | 543.097664 px |
| fy | 539.701638 px |
| cx | 310.686570 px |
| cy | 222.928406 px |
| k1 | −0.00984 |
| k2 | +0.07654 |
| p1 | −0.00247 |
| p2 | +0.00278 |
| Calibration RMS | 0.9742 px |
| ROS topic | `/ovcam/image_raw` (mono8, NV12 Y-plane extracted) |

---

## 8. YOLO Object Detection

**Model:** YOLOv8-nano compiled to Hailo HEF (INT8), file `models/yolo26n_10h.hef` (4.8 MB)  
**Runtime:** HailoRT Python SDK on Hailo-10H NPU via `/dev/hailo0`  
**Process:** `yolo_producer.py` on host Core 1, reads SHM, writes `/yolo_shm`

### NPU Inference Timing (measured, steady-state)

| Stage | Time |
|---|---|
| Preprocess (host, frame copy) | ~0.48 ms |
| NPU inference (HEF INT8) | **25.1 ms** (25.3 ms P50) |
| Post-process decode (FP32) | **0.75–0.85 ms** |
| NMS (`cv2.dnn.NMSBoxes`) | **0.075–0.083 ms** |
| **Total end-to-end** | **~26 ms** → **~32 Hz throughput** |

### NPU Resolution Invariance (Finding 5)
NPU inference time is **constant regardless of input resolution** (INT8 compute bound, not memory bound):

| Resolution | Mean Inference | Std |
|---|---|---|
| 320×240 | 25.1 ms | ±0.4 ms |
| 416×312 | 25.0 ms | ±0.3 ms |
| 640×480 | 24.9 ms | ±0.4 ms |

### FP16 Post-Processing on ARM (Finding 1 — Negative Result)
FP16 decode is **5× SLOWER** than FP32 on ARM Cortex-A76. No hardware FP16 SIMD; NumPy upcasts every operation to FP32 internally.

| Precision | Decode time | Factor |
|---|---|---|
| FP32 | **0.75 ms** | 1.0× |
| FP16 | **3.35 ms** | **4.5× slower** |

### CPU vs NPU Comparison (Finding 7, n≥30 frames each)

| Model | Backend | FPS | Mean Inference | vs NPU |
|---|---|---|---|---|
| yolo26n_10h | **Hailo-NPU INT8** | **32.1** | **25.1 ms** | 1.0× (baseline) |
| yolo26n | ONNX-Runtime-CPU | 5.4 | 179.8 ms | 7.2× slower |
| yolo26n | PyTorch-CPU-FP32 | 2.2 | 448.5 ms | 17.9× slower |
| yolo26s | ONNX-Runtime-CPU | 2.1 | 472.6 ms | 18.8× slower |
| yolo26s | PyTorch-CPU-FP32 | 0.8 | 1,249.6 ms | 49.8× slower |
| yolo26m | PyTorch-CPU-FP32 | 0.3 | 3,463.4 ms | 138× slower |
| yolo26l | PyTorch-CPU-FP32 | 0.2 | 4,345.5 ms | 173× slower |

**Key insight:** ONNX Runtime uses ARM-optimized kernels → ~2.5× faster than PyTorch on same ARM hardware. Still 7× slower than NPU at best. PyTorch 2.10 builds lack SVE intrinsics (generic ARM).

---

## 9. OV2SLAM Visual SLAM

**Node:** `ov2slam_node`, C++, Cores 2–3  
**Subscribes:** `/ovcam/image_raw` (mono8)  
**Algorithm:** Monocular visual odometry, KLT optical flow + Bundle Adjustment (Ceres)  
**Dataset used for benchmarking:** EuRoC MH01-Easy (3,656–3,677 frames)

### Config Comparison (3 runs each, EuRoC MH01-Easy)

| Config | Precision | ATE mean±std | FE time | BA time | KLT time |
|---|---|---|---|---|---|
| **baseline** | float64 | **0.071 ± 0.014 m** | 5.4 ms | **7.3 ms** | 1.4 ms |
| baseline_f32 | float32 | 0.150 ± 0.052 m | 5.4 ms | 7.6 ms | 1.4 ms |
| fe_klt2 (pyr 3→2) | float64 | 0.093 ± 0.023 m | 5.2 ms | 7.2 ms | 1.3 ms |
| solver_tuned | float64 | 0.160 ± 0.060 m | 5.7 ms | **5.7 ms** (−22%) | 1.4 ms |
| solver_tuned_f32 | float32 | 0.129 ± 0.032 m | 6.6 ms | 7.1 ms | 1.8 ms |
| fe_maxdist65 | float64 | 0.240 ± 0.092 m | 4.7 ms | 3.8 ms | 1.0 ms |
| fe_combined | float64 | 0.400 ± 0.203 m | 4.7 ms | 3.8 ms | 1.0 ms |

### Key SLAM Findings
- **Finding 2 (Negative):** Float32 bearing vectors generate **1.71× MORE L1 cache traffic** than float64 (float32: 11.6B loads; float64: 6.8B loads). Cast overhead at `opengv` boundaries dominates.
- **Finding 3:** Solver tuning (Dogleg, 3 iterations, 0.1s budget) gives **21% BA speedup** (7.3 ms → 5.7 ms) but ATE degrades from 0.071 m → 0.160 m.
- **Finding 6:** Core pinning gives no measurable benefit: unpinned 19.86 FPS ± 1.0 vs pinned 19.85 FPS ± 1.3 — variance slightly higher with pinning.

---

## 10. IPC Latency: SHM vs DDS (Finding 4)

Benchmark: 1,000 transfers of a 921,600-byte frame (640×480 rgb8)

| Transport | P50 | P95 | P99 | Mean |
|---|---|---|---|---|
| **POSIX SHM seqlock** | **169.2 µs** | 371.7 µs | 536.8 µs | 201.8 µs |
| CycloneDDS | 39,882.6 µs | 43,254.2 µs | 45,759.5 µs | 40,352.7 µs |
| **Ratio (DDS/SHM)** | **236× slower** | 116× slower | 85× slower | 200× slower |

SHM seqlock is **236× faster at median** than DDS for 921 KB frames. DDS adds ~40 ms of latency per frame — would cap the pipeline at ~25 Hz max just from transport overhead.

---

## 11. ViSP Servo Node

**Node:** `visp_servo_node`, C++, Core 0  
**Algorithm:** Image-Based Visual Servoing (IBVS), 4-corner bounding-box formulation  

### State Machine (4 states)
```
SEARCHING ──[N≥2 consecutive detections AND |ex|<lockon_ex_tol]──▶ APPROACHING
APPROACHING ◀──[REACQUIRE timeout expires]──────────── REACQUIRE
APPROACHING ──[detection lost]──────────────────────▶ REACQUIRE
APPROACHING ──[bbox_ratio≥0.50 AND |ex|<0.12 for 8 ticks]──▶ REACHED (sticky)
REACHED ──[sim reset / heartbeat dead]──────────────▶ SEARCHING (reset)
```

### SEARCHING Sub-sequence (deterministic)
1. Yaw to anchor − 60° (right)
2. Yaw to anchor + 60° (left)
3. Yaw to anchor (center)
4. Strafe right 0.6 m/s for 3.0 s (~1.8 m)
5. Repeat with new anchor

### Control Law (APPROACHING)
```
vx = k_fwd × (target_bbox_ratio − current_bbox_ratio)   [≥ 0, forward only]
vy = −k_lat × ex_norm                                    [lateral centering]
vz = −k_vz  × ey_norm  (if |ey_norm| > vert_deadband)   [clamped ≥ 0 during approach]
wy = −k_pitch_return × current_pitch                     [camera leveling]
wz = 0                                                   [yaw never used in APPROACHING]
```

### Key Parameters (HIL tuning, `config/hil/hil_servo_params.yaml`)
| Parameter | Value | Note |
|---|---|---|
| `target_class` | `"stop sign"` | COCO label |
| `min_confidence` | 0.20 | Low threshold — early lock-on |
| `k_fwd` | 3.0 | Aggressive in sim (no physical crash risk) |
| `k_lat` | 1.5 | Lateral centering gain |
| `k_vz` | 1.5 | Vertical gain |
| `max_linear` | 3.0 m/s | Sim speed cap |
| `max_angular` | 0.8 rad/s | Moderate to avoid motion blur |
| `target_bbox_ratio` | 0.55 | Asymptotic stop at 55% frame fill |
| `hold_bbox_ratio` | 0.50 | REACHED threshold |
| `lockon_consec` | 2 | Detections needed to leave SEARCHING |
| `search_yaw_target_deg` | 60° | ±60° sweep from anchor |
| `search_strafe_speed` | 0.6 m/s | Lateral strafe during SEARCHING |

### Heartbeat Watchdog (Sim Health)
- `sim=PAUSED` when heartbeat advances at < 1% real-time → cmd_vel forced to 0
- `sim=DEAD` when no heartbeat for 1.5 s → cmd_vel forced to 0
- Recovery requires 0.5 s of sustained healthy heartbeat

### Observed Callback Latency
- ViSP callback latency: **0.13–0.16 ms** (measured with `perf_counter` in prior session)

---

## 12. Simulink / MATLAB Integration

**Startup sequence:**
1. `hil_ros_init.m` — creates `/matlab_bridge` ROS2 node, publishers, subscribers
2. Open and run Simulink model
3. `sim_camera_publisher_timer.m` — starts MATLAB timer publishing frames

**MATLAB publishes (→ Pi):**
| Publisher | QoS | Note |
|---|---|---|
| `/sim/camera/image_raw` | BEST_EFFORT, volatile, depth=5 | rgb8, bgr8 encoding |
| `/sim/drone_pose` | RELIABLE, volatile | [x, y, z, pitch, yaw] |
| `/sim/pitch_angle` | RELIABLE, volatile | Simulink pitch integrator output |
| `/sim/heartbeat` | RELIABLE, volatile | Simulink sim_time (NOT wall clock) |
| `/sim/target_pose` | RELIABLE, transient_local | Stop-sign world coords, 1 Hz republish |

**MATLAB subscribes (← Pi):**
| Subscriber | QoS |
|---|---|
| `/cmd_vel` | RELIABLE, volatile |
| `/visp/debug_image` | BEST_EFFORT, volatile |

**Stop-sign world position (Unreal NYC scene, Simulink ICs):**
```
Target: [35.5, 23.7, 3.2, π]   (x, y, z, yaw)
Drone ICs: x=0, y=15, z=10, yaw=0, pitch=0
Initial distance to target: ~56 m, bearing error: ~−48.8°
```

**cmd_vel coordinate convention (Simulink-confirmed):**
```
linear.x  = forward velocity   (+ = forward)
linear.y  = lateral velocity   (+ = LEFT, − = RIGHT)
linear.z  = vertical velocity  (+ = UP)
angular.y = pitch rate         (+ = NOSE DOWN, integrated by Simulink)
angular.z = yaw rate           (+ = YAW LEFT)
```

---

## 13. run_stack_hil.sh — Startup Script

```bash
./run_stack_hil.sh              # start full stack (SLAM + all nodes)
./run_stack_hil.sh --no-slam    # start without OV2SLAM (faster startup)
./run_stack_hil.sh stop         # kill everything, clean SHM
./run_stack_hil.sh hz [topic]   # check rate (default: /sim/camera/image_raw)
```

**What it does on start:**
1. Detects `PI_LOCAL_IP` automatically via `ip route get ${MATLAB_HOST_IP}`
2. Tunes kernel sysctl (16 MB UDP buffers, 32 MB ipfrag threshold) — idempotent
3. Generates `/tmp/fastrtps_hil.resolved.xml` from template via `envsubst`
4. Starts Docker container (`--net=host --ipc=host --privileged`)
5. Launches `hil_simulation.launch.py` inside container
6. Waits for `/dev/shm/ovcam_frames` to appear (up to 10 s)
7. `chmod 666` on SHM and semaphore for host yolo_producer
8. Starts `yolo_producer.py` on host Core 1
9. Waits for `/dev/shm/yolo_shm` to appear

**Environment variables:**
| Variable | Default | Note |
|---|---|---|
| `MATLAB_HOST_IP` | `192.168.56.1` | Windows host IP |
| `DDS` | `fastrtps` | `fastrtps` or `cyclonedds` |
| `ROS_DOMAIN_ID` | `0` | |

---

## 14. Key Files & Paths

| File | Description |
|---|---|
| `run_stack_hil.sh` | Master HIL start/stop/hz script |
| `config/hil/fastrtps_hil.xml` | Pi-side Fast DDS profile (dual-transport) |
| `config/hil/fastrtps_matlab.xml` | MATLAB-side DDS reference config |
| `config/hil/hil_servo_params.yaml` | ViSP HIL tuning parameters |
| `src/sim_camera_bridge/src/sim_camera_bridge_node.cpp` | MATLAB→SHM bridge |
| `src/sim_camera_bridge/launch/hil_simulation.launch.py` | Launch file for all Pi nodes |
| `src/ovcam_bridge/src/ovcam_bridge_node.cpp` | SHM→ROS2 bridge (publishes /ovcam/image_raw) |
| `src/yolo_producer/yolo_producer.py` | Host-side Hailo NPU inference |
| `src/visp_servo/src/visp_servo_node.cpp` | IBVS servo controller |
| `src/visp_servo/config/servo_params.yaml` | Physical camera params |
| `camera_calib/ov5647_ov2slam.yaml` | OV5647 intrinsics + OV2SLAM config |
| `models/yolo26n_10h.hef` | YOLOv8-nano Hailo INT8 model (4.8 MB) |
| `/tmp/fastrtps_hil.resolved.xml` | Resolved DDS profile (at runtime) |
| `/tmp/hil_launch.log` | ROS2 launch stdout (all node logs) |
| `/tmp/yolo_producer.log` | yolo_producer stdout |
| `/dev/shm/ovcam_frames` | Camera frame ring buffer (~1.84 MB) |
| `/dev/shm/yolo_shm` | YOLO detection ring buffer (~4.7 KB) |

---

## 15. Research Findings Summary

All findings measured on RPi5 + Hailo-10H, Ubuntu 24.04 Docker, ROS2 Jazzy.

| # | Finding | Measured Value |
|---|---|---|
| 1 | FP16 on ARM Cortex-A76 is **SLOWER** than FP32 | 5× slower (3.35 ms vs 0.75 ms decode) |
| 2 | Float32 bearing vectors → MORE cache traffic than float64 | 1.71× more L1 loads (11.6B vs 6.8B) |
| 3 | Solver tuning → speedup at accuracy cost | 22% BA speedup; ATE 0.071→0.160 m |
| 4 | SHM seqlock vs DDS for 921 KB frames | **236× faster** at P50 (169 µs vs 39,883 µs) |
| 5 | NPU inference is resolution-invariant | 25.0–25.1 ms across 320×240 to 640×480 |
| 6 | CPU core pinning on RPi5 | No measurable benefit (single pipeline) |
| 7 | NPU vs ARM CPU for YOLO | **7–173× faster** (vs ONNX-RT nano to PyTorch large) |
| 8 | DDS delivery rate with multi-interface Pi | **32% delivery** without interface lock; **85%** with fix |

---

## 16. Git / Version Control

```
Branch:  sim-hil
Commit:  5f9f6c5  "sim-hil: full HIL baseline — DDS fixed, all topics live"
Date:    2026-05-15

Commit history (sim-hil):
  5f9f6c5  sim-hil: full HIL baseline — DDS fixed, all topics live
  94b659c  sim-hil: SHM-filler bridge + HIL launch wrapper for Unreal NYC scene
  1542daf  Fix YOLO CPU benchmark: semaphore naming, synthetic mode, ONNX support
  ca1b970  Research pivot: config comparison, E2E latency, YOLO CPU baseline
  f8af6a7  Research session: profiling, ViSP redesign, SLAM tuning, YOLO benchmarks
```

**Push status:** Commit saved locally. Remote push (`git push origin sim-hil`) requires GitHub token auth — run once credentials are available.

**To restore this exact baseline:**
```bash
git checkout sim-hil          # already on this branch
git log --oneline | head -1   # should show 5f9f6c5
./run_stack_hil.sh
./run_stack_hil.sh hz /sim/drone_pose   # expect ~17 Hz when MATLAB running at 20 Hz
```

---

## 17. Known Limitations / Next Steps

- `/sim/camera/image_raw` delivery efficiency: ~85% (17.1/20 Hz) — DDS BEST_EFFORT over a single UDP hop, some fragments dropped. Acceptable for vision pipeline.
- YOLO at 4–5 Hz when Simulink is PAUSED (sim_heartbeat frozen → visp stops querying); full 16 Hz when simulation is actively running.
- `ov2slam` on synthetic frames: SLAM will work but ATE is untested on Unreal scene (no ground truth transform defined for virtual camera→world frame yet).
- GitHub push blocked (no stored credentials on Pi); commit is safe locally.
- `yolo_producer.log` is empty (Python stdout buffered when redirected to file) — use `tail -f` with `stdbuf -oL` if live monitoring needed.
