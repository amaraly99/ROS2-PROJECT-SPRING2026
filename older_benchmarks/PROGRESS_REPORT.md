# Edge AI Perception-to-Control Pipeline — Progress Report

**Date**: March 19, 2026
**Team**: Ahmed (YOLO + ViSP), Teammate (OV2SLAM)
**Hardware**: Raspberry Pi 5 (Cortex-A76, 4-core, 16GB) + Hailo-10H NPU
**Deadline**: April 15, 2026

---

## System Architecture

```
OV5647 Camera (640x480@30fps)
    │
    ├──► ovcam_producer (Core 0) ──► POSIX SHM ──► ovcam_bridge ──► /ovcam/image_raw
    │
    └──► yolo_producer (Core 1, Hailo NPU) ──► POSIX SHM ──► yolo_bridge ──► /yolo/detections
                                                                                    │
    OV2SLAM (Cores 2-3) ◄── /ovcam/image_raw                                       │
        │                                                                           │
        ├──► /vo_pose                                                               │
        └──► /point_cloud                                                           │
                    │                                                               │
                    └──────────────► visp_servo ◄───────────────────────────────────┘
                                        │
                                        └──► /cmd_vel (Twist)
```

**IPC**: POSIX shared memory with seqlock + semaphores (zero-copy)
**Core pinning**: Camera/YOLO on cores 0-1, SLAM on cores 2-3
**ROS2**: Jazzy, CycloneDDS, Ubuntu 24.04 in Docker

---

## Completed Work

### 1. Full Pipeline Integration Test (PASSED)

**Test Date**: March 19, 2026
**Result**: All 4 ROS2 nodes running, data flowing end-to-end

| Topic | Rate | Description |
|-------|------|-------------|
| `/ovcam/image_raw` | 31.5 Hz | Camera images |
| `/yolo/detections` | 16 Hz | YOLO object detections (Hailo NPU) |
| `/vo_pose` | 26 Hz | Visual odometry poses |
| `/cmd_vel` | 16 Hz | Velocity commands from ViSP |

**ViSP tracking log** (live person detection):
```
[TRACKING] person conf=0.78 bbox=[18,2 535x477] Z=1.92m(bbox) err=1.04 cmd: vx=-0.50 vy=0.10 vz=-0.10 wz=0.01 cb=0.15ms
```
- State machine transitions observed: TRACKING → LOST → SEARCHING (correct behavior)
- ViSP callback latency: **0.13-0.16ms**

---

### 2. ViSP Visual Servoing Redesign (Complete)

**Previous implementation**: Single bbox-center feature (2 visual features, 2 DOF control)
**New implementation**: 4 bbox-corner features (8 visual features, 4 DOF control)

#### Changes Made:
- **4-corner IBVS**: `vpFeaturePoint` for TL, TR, BR, BL corners of detection bbox
- **5-state machine**: SEARCHING → APPROACHING → TRACKING → HOVERING → LOST
- **Bbox-based depth**: `Z = fy * known_height / bbox_height_px` (no SLAM dependency)
- **SLAM depth fusion**: Optional, with 4x subsampling and spatial bbox filter
- **Velocity ramping**: Smooth transitions, `vel_ramp_rate` parameter
- **Edge safety margins**: Re-centering when target near image edge
- **State-dependent gain**: APPROACHING 1.2x, TRACKING 1.0x, HOVERING 0.3x

#### Key Parameters (servo_params.yaml):
| Parameter | Value | Purpose |
|-----------|-------|---------|
| `lambda` | 0.5 | IBVS gain |
| `target_bbox_ratio` | 0.35 | Desired bbox height / image height |
| `known_target_height` | 1.7m | Person height for depth estimation |
| `lost_threshold` | 15 frames | Frames before LOST state |
| `search_yaw_rate` | 0.3 rad/s | Yaw during SEARCHING |
| `edge_margin_px` | 50 | Pixels from edge for safety |
| `vel_ramp_rate` | 0.1 | Max velocity change per step |

#### Files Modified:
- `src/visp_servo/src/visp_servo_node.cpp` — Full rewrite (~600 lines)
- `src/visp_servo/config/servo_params.yaml` — All new parameters

---

### 3. SLAM Solver Tuning (Complete)

Created configurable solver parameters and an optimized config variant.

#### Code Changes:
- `src/ov2slam_ros/include/slam_params.hpp` — Added `ba_max_iterations_`, `ba_max_solver_time_`
- `src/ov2slam_ros/src/slam_params.cpp` — YAML loading with defaults (5 iters, 0.2s)
- `src/ov2slam_ros/src/optimizer.cpp` — Use configurable values instead of hardcoded

#### Baseline Config (`euroc_mono.yaml`):
| Parameter | Value |
|-----------|-------|
| `apply_l2_after_robust` | 1 (two Ceres::Solve calls) |
| `use_dogleg` | 0 (Levenberg-Marquardt) |
| `ba_max_iterations` | 5 (default) |
| `ba_max_solver_time` | 0.2s (default) |
| `nmin_covscore` | 25 |

#### Solver-Tuned Config (`euroc_mono_solver_tuned.yaml`):
| Parameter | Value | Expected Impact |
|-----------|-------|-----------------|
| `apply_l2_after_robust` | **0** | Eliminates 2nd Ceres::Solve — ~40% BA time reduction |
| `use_dogleg` | **1** | Fewer iterations to converge |
| `ba_max_iterations` | **3** | Reduced from 5 |
| `ba_max_solver_time` | **0.1s** | Halved time budget |
| `nmin_covscore` | **40** | Smaller BA problem, fewer weak constraints |

**Build status**: Both configs build and run successfully.

---

### 4. YOLO Post-Processing Optimizations (Complete)

#### A. NMS Replacement
- **Before**: Custom Python NMS loop (`_nms()`)
- **After**: `cv2.dnn.NMSBoxes()` — C++ internally, single call
- **Impact**: NMS time consistently 0.0-0.1ms

#### B. Per-Stage Timing Instrumentation
Added timing for decode, NMS, and total post-processing. Output every 100 frames:
```
100 frames, 32.1 fps | pre=0.6ms infer=25.1ms decode=0.7ms nms=0.1ms pp_total=0.7ms
```

#### C. Float16 Post-Processing Experiment
- **`--fp16` flag**: Casts Hailo outputs to float16 before DFL decode + sigmoid
- **Result**: **NEGATIVE** — fp16 is slower (3.5ms decode vs 0.7ms fp32)
- **Reason**: ARM Cortex-A76 has native float32 SIMD (NEON) but no float16 arithmetic. NumPy must up-cast float16→float32 for every operation, adding overhead.
- **Paper value**: This is a publishable finding — contrasts with GPU behavior where float16 is 2x faster. On 64-bit ARM, float32 is the sweet spot for CPU operations. Float16 only saves memory/bandwidth, not compute.

#### Timing Comparison:
| Metric | FP32 | FP16 |
|--------|------|------|
| FPS | 32.1 | 29.1 |
| Decode | 0.7ms | 3.5ms |
| NMS | 0.1ms | 0.1ms |
| Total PP | 0.7ms | 3.5ms |

**Conclusion**: Keep FP32 for post-processing. The Hailo NPU already does INT8 inference — the CPU post-processing bottleneck is in DFL softmax + sigmoid, which is fastest in native float32 on ARM.

---

### 5. Benchmark Infrastructure (Complete)

#### Created Files:
- `benchmarks/run_benchmark.sh` — Automated EuRoC benchmark runner
  - Launches OV2SLAM + EuRoC image publisher
  - Collects timing CSV, trajectory, pidstat
  - Runs evo_ape/evo_rpe evaluation
  - Generates summary CSV
- `benchmarks/euroc_publisher.py` — ROS2 EuRoC image publisher
  - Reads ASL format (mav0/cam0/data/)
  - Publishes at original timestamps with configurable rate multiplier
- `benchmarks/run_benchmark.sh` accepts config name and num runs

#### Profiler CSV Export:
- Added `exportCSV()` to `Profiler` class in `profiler.hpp`
- Auto-exports `ov2slam_timings.csv` on SLAM shutdown
- Format: `timer,mean_ms,std_ms,min_ms,max_ms`

#### Blocked:
- EuRoC dataset download (robotics.ethz.ch server returning 503)
- Alternative: ETH Research Collection bulk download available but large (12GB per location group)

---

## Pending Work

### Priority 1: EuRoC Baseline Benchmarks
- Download MH_01_easy, V1_01_easy, V2_01_easy datasets (retry when ETH server recovers)
- Run baseline + solver-tuned configs (3 runs each)
- Collect ATE, RPE, FE/BA timing, FPS

### Priority 2: SLAM Float32 Precision Switch (Novel Contribution)
- Create `precision_config.hpp` with `using Scalar = float;`
- Convert front-end math (bearing vectors, epipolar geometry) to float
- Keep double at Ceres boundary (Ceres requires it)
- Measure cache miss reduction with `perf stat`

### Priority 3: Front-End Speedups
- Reduce KLT pyramid levels (3→2)
- Increase feature grid spacing (nmaxdist 50→60-70)
- Disable CLAHE
- Skip epipolar filtering conditionally

### Priority 4: Paper Writing
- System architecture diagram
- Timing breakdown (stacked bar: baseline vs optimized)
- Pareto frontier (ATE vs frame time)
- Cache performance (double vs float)
- Ablation study table

---

## Key Research Findings So Far

### 1. Float16 on ARM is Counterproductive for Compute
ARM Cortex-A76 has native float32 SIMD (NEON) but no hardware float16 arithmetic. Float16 post-processing is **5x slower** than float32 due to mandatory up-casting. This contrasts with GPUs where float16 halves compute. On 64-bit ARM, float16 only saves memory bandwidth, not arithmetic throughput.

### 2. Float32 vs Float64 (Planned — Novel)
The converse may hold for float32 vs float64: while the FPU handles both at similar throughput, float32 halves cache footprint. On cache-constrained RPi5 (512KB L2), this could significantly reduce cache misses in SLAM's tight inner loops (KLT tracking, bearing vector computation). This is a distinct mechanism from traditional 8/16-bit embedded quantization.

### 3. Solver Tuning is Low-Hanging Fruit
Disabling L2 refinement eliminates an entire `ceres::Solve()` call. Combined with Dogleg trust region (fewer iterations) and raised covisibility threshold (smaller problem), we expect ~40% BA time reduction with minimal accuracy impact.

---

## File Change Summary

| File | Change | Status |
|------|--------|--------|
| `src/visp_servo/src/visp_servo_node.cpp` | Full IBVS redesign | Built & tested |
| `src/visp_servo/config/servo_params.yaml` | New parameters | Active |
| `src/yolo_producer/yolo_producer.py` | NMS replacement, fp16 flag, timing | Tested |
| `src/ov2slam_ros/include/profiler.hpp` | CSV export | Built |
| `src/ov2slam_ros/src/ov2slam.cpp` | Auto-export timings | Built |
| `src/ov2slam_ros/CMakeLists.txt` | OpenCV path fix | Built |
| `src/ov2slam_ros/include/slam_params.hpp` | Configurable solver params | Built |
| `src/ov2slam_ros/src/slam_params.cpp` | YAML loading for solver params | Built |
| `src/ov2slam_ros/src/optimizer.cpp` | Use configurable iterations/time | Built |
| `parameters_files/fast/euroc/euroc_mono.yaml` | Enabled log_timings | Active |
| `parameters_files/fast/euroc/euroc_mono_solver_tuned.yaml` | New solver-tuned config | Active |
| `benchmarks/run_benchmark.sh` | Full benchmark automation | Ready |
| `benchmarks/euroc_publisher.py` | EuRoC image publisher | Ready |

---

## How to Run

### Full Live Pipeline
```bash
./run_stack.sh              # Start everything
./run_stack.sh --no-slam    # Without SLAM
./run_stack.sh stop         # Stop
```

### EuRoC Benchmarks (inside Docker)
```bash
# Download datasets first to datasets/euroc/
bash /workspace/benchmarks/run_benchmark.sh baseline 3
bash /workspace/benchmarks/run_benchmark.sh solver_tuned 3
```

### YOLO Precision Experiment
```bash
# FP32 (default)
python3 yolo_producer.py --hef models/yolo26n_10h.hef

# FP16 experiment
python3 yolo_producer.py --hef models/yolo26n_10h.hef --fp16
```

### Build Commands (inside Docker)
```bash
# ViSP
colcon build --packages-select visp_servo --symlink-install

# OV2SLAM
colcon build --packages-select ov2slam --symlink-install

# All packages
colcon build --packages-select yolo_msgs ovcam_bridge yolo_bridge visp_servo --symlink-install
```
