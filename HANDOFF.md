# Project Handoff — ROS2 HIL Visual Servoing (2026-06-29)

**Branch:** `controller-benchmark` | **Pi SSH:** `ssh amaraly@192.168.1.60`
**Pi repo:** `/home/amaraly/ROS2-PROJECT-SPRING2026/` | **Container:** `ros2_perception_stack`

> This document supersedes the 2026-06-24 handoff. See `docs/fixlog/` for individual fix reasoning.

---

## 1. What this project is

MATLAB/Simulink/Unreal (Windows) streams a synthetic camera feed + drone pose over CycloneDDS to a
Raspberry Pi 5. The Pi runs a perception + visual-servoing stack inside Docker and returns `/cmd_vel`.

Three swappable modules:
- **detector** — `yolo` (Hailo NPU) | `oracle` (synthetic bbox from sim pose)
- **controller** — `proportional` (TS2) | `ibvs` (TS1) | `h_vs` (TS3) | `pbvs` (TS4, paused)
- **slam** — `ov2slam` | `orbslam2` | (orbslam3 planned) — runs as Docker sidecar

Entry point: `./run_stack_hil.sh --config <name>` reads `config/hil/stack/<name>.yaml`.

---

## 2. Current state (what's DONE vs pending)

| Area | Status | Notes |
|------|--------|-------|
| Phase A — modular stack | **DONE, Pi-tested** | `run_stack_hil.sh` + `parse_stack.py` + all stack configs |
| ORB-SLAM2 sidecar | **DONE, Pi-tested** | `src/orbslam2/` + `orbslam2_fixed` Docker image |
| SLAM depth → IBVS | **DONE, compiled** | `SlamDepthSource` wired to canonical `/slam/pose` + `/slam/cloud` |
| Phase A.5 — ATE eval | **DONE** | `benchmarks/eval_slam_hil.py` (Umeyama + ATE RMSE) |
| N=10 controller benchmark | **DONE** | Results in `reports/N10_all_controllers/REPORT.md` |
| TS4 PBVS | **PAUSED** | Diverges in APPROACHING — see `docs/fixlog/005-vx-unbounded.md`, TODO-H |
| ORB-SLAM3 live integration | **PLANNED** | Needs adapter + C++ pose-topic patch (see §8) |
| Windows ↔ Pi sync | **PENDING** | All Pi-side changes NOT in the Windows repo — see §7 |

---

## 3. How to run everything

### 3a. Stack launch

```bash
# Scout mode (engage on boot, no recording)
./run_stack_hil.sh --config ov2slam_ibvs          # YOLO + VISP IBVS + OV2SLAM
./run_stack_hil.sh --config full_ov2slam           # YOLO + Proportional + OV2SLAM
./run_stack_hil.sh --config default                # YOLO + IBVS, no SLAM
./run_stack_hil.sh --config oracle                 # Oracle + Proportional, no SLAM

# Benchmark mode (FSM waits for clean sim reset → records bag to bags/run_<cfg>_<stamp>/)
./run_stack_hil.sh --config orbslam2_eval          # Oracle + Proportional + ORB-SLAM2 → SLAM ATE bag
./run_stack_hil.sh --config ov2slam_oracle         # Oracle + Proportional + OV2SLAM → SLAM ATE bag

# SLAM depth A/B pair
./run_stack_hil.sh --config ov2slam_ibvs           # baseline: bbox depth
./run_stack_hil.sh --config ov2slam_ibvs_slamdepth # SLAM depth via /slam/cloud

# Stop everything (kills stack + SLAM sidecar)
./run_stack_hil.sh stop

# Build
./run_stack_hil.sh --build --config ov2slam_ibvs   # build then launch
./run_stack_hil.sh build visp_servo                 # build one package
```

### 3b. Evaluate a SLAM benchmark bag

```bash
# After running any benchmark-mode config:
python3 benchmarks/eval_slam_hil.py bags/run_orbslam2_eval_<stamp>

# Output (written into the run dir):
#   slam_metrics.csv           — ATE RMSE + scale + Hz + sample count
#   fig_slam_ate.png           — ATE vs time
#   fig_slam_trajectory.png    — GT vs aligned SLAM (top-down XY)
#   fig_slam_error_xyz.png     — per-axis translation error
```

Key metric: `ATE_RMSE_m` — Absolute Trajectory Error after Umeyama SE(3)+scale alignment.
The scale factor (`umeyama_scale`) tells you how far off raw monocular scale is from metric.

### 3c. Evaluate a controller benchmark bag

```bash
python3 benchmarks/plot_controller_hil.py bags/run_<ctrl>_<stamp>

# Compare across multiple runs (N=10 example):
python3 benchmarks/compare_controllers.py reports/N10_all_controllers/ \
    bags/ctrl_ibvs_N{1..10}_* bags/ctrl_proportional_N{1..10}_* bags/ctrl_h_vs_N{1..10}_*
```

---

## 4. Stack configs (all in `config/hil/stack/`)

| Config | Detector | Controller | SLAM | Mode | Purpose |
|--------|----------|------------|------|------|---------|
| `default.yaml` | yolo | ibvs | off | scout | base no-SLAM test |
| `oracle.yaml` | oracle | proportional | off | scout | no Hailo needed |
| `ibvs.yaml` | yolo | ibvs | off | scout | IBVS standalone |
| `h_vs.yaml` | yolo | h_vs | off | scout | homography VS |
| `full_ov2slam.yaml` | yolo | proportional | ov2slam | scout | OV2SLAM live flight |
| `ov2slam_ibvs.yaml` | yolo | ibvs | ov2slam | scout | SLAM depth A/B baseline |
| `ov2slam_ibvs_slamdepth.yaml` | yolo | ibvs | ov2slam | scout | SLAM depth active |
| `ov2slam_oracle.yaml` | oracle | proportional | ov2slam | **benchmark** | OV2SLAM ATE capture |
| `orbslam2_eval.yaml` | oracle | proportional | orbslam2 | **benchmark** | ORB-SLAM2 ATE capture |

**Key config fields:**
```yaml
mode: scout | benchmark
controller:
  use_slam_depth: true | false   # toggle SLAM depth for IBVS (per-stack)
slam:
  enabled: true
  image: orbslam2_fixed          # Docker image
  cpu: "2,3"                     # taskset inside sidecar
  startup_delay_sec: 15
  remap:
    - "camera:=/ovcam/image_raw"
    - "slam_pose:=/slam/pose"
```

---

## 5. SLAM integration details

### Canonical topics (contract between all SLAMs and the rest of the stack)
- `/slam/pose` — `geometry_msgs/PoseStamped` — SLAM camera pose in map frame
- `/slam/cloud` — `sensor_msgs/PointCloud2` — map points (used by SlamDepthSource)

Every SLAM remaps its native topics onto these two. Adding a new SLAM = add two remaps.

### ORB-SLAM2 sidecar (`orbslam2_fixed` image)
- Code: `src/orbslam2/ros2-ORB_SLAM2/` (copied from `/home/amaraly/ORB_SLAM2/` Pi-side)
- Build: `cd /home/amaraly/ROS2-PROJECT-SPRING2026/src/orbslam2 && ./start_container`
- Binary: `/workspace/src/orbslam2/install_stereo/orbslam/lib/orbslam/mono`
- Vocab: `/workspace/src/orbslam2/Vocabulary/ORBvoc.txt`
- Calib: `src/orbslam2/ros2-ORB_SLAM2/src/monocular/hil_sim.yaml` (fx=fy=554, cx=320, cy=240, Viewer.UseViewer:0)
- Publishes: `slam_pose` → remapped to `/slam/pose`
- Datasets: `/home/amaraly/ROS2-PROJECT-SPRING2026/datasets/euroc/` (do NOT re-copy)

### Critical ORB-SLAM2 gotchas
1. **Direct binary — not `ros2 run`**: `ros2 run` fails (`librcl_action.so` missing in `orbslam2_fixed` for rclpy). Use full binary path directly.
2. **`LD_LIBRARY_PATH`**: image lacks ldconfig for ROS2 libs. Must set via `slam.ld_prefix` in config: `/workspace/src/orbslam2/lib:/usr/local/lib:/opt/ros/jazzy/lib`
3. **`rclcpp::remove_ros_arguments`**: called AFTER `rclcpp::init` in `mono.cpp` so positional args survive `--ros-args` remaps.
4. **`--entrypoint ""`**: `ros2_perception_stack` has a custom entrypoint — all sidecar `docker run` calls must use `--entrypoint ""` or the entrypoint loops.
5. **Tcw size guard**: `monocular-slam-node.cpp` checks `!Tcw.empty() && Tcw.rows==4 && Tcw.cols==4` before publishing — ORB-SLAM2 returns partial matrices during tracking loss.
6. **Headless**: `Viewer.UseViewer: 0` required in hil_sim.yaml — Pangolin crashes without display.

### OV2SLAM sidecar (`ros2_perception_stack` image)
- Publishes `vo_pose` → remapped to `/slam/pose`; `point_cloud` → remapped to `/slam/cloud`
- Image does NOT need LD_LIBRARY_PATH fixes (already configured)
- `--entrypoint ""` required (custom entrypoint in image)

### SLAM depth for IBVS (`SlamDepthSource`)
- File: `src/visp_servo/src/depth/slam_depth_source.cpp`
- Subscribes canonical `/slam/pose` + `/slam/cloud` (hardcoded constants — no params)
- Projects map points into camera frame, returns median depth of points inside bbox
- Falls back to bbox-ratio Z when no SLAM data
- Toggle: `controller.use_slam_depth: true` in stack config
- Scale: `slam_depth_scale` in `config/hil/bench_ibvs.yaml` (default 1.0)
- **Thread safety note**: runs on single-threaded executor (visp node's `rclcpp::spin`) — no mutex. If moved to MultiThreadedExecutor, add one.

---

## 6. Open TODOs (active — resume here)

| ID | Fix | Description | Priority |
|----|-----|-------------|----------|
| **TODO-H** | FIX-005 | PBVS paused: re-enable `wz` to track bearing, or diagnose why TS2 survives same geometry without yaw. Resume via `docs/fixlog/005-vx-unbounded.md`. | Medium |
| **TODO-K** | FIX-006 | IBVS residual oscillation after decoupled fix — confirm lambda tuning or close-range dropout. | Medium |
| **TODO-L** | FIX-009 | Monocular scale drift: `depth_scale_` constant calibration doesn't hold across scale drift. Fix: align SLAM scale to known target height or sim GT before trusting SLAM-depth IBVS numbers. | High before SLAM-depth IBVS flights |
| **TODO-M** | FIX-009 | Background bias: bbox-median depth includes background points. Fix: gate to nearest depth cluster or lower percentile. | Low |
| **TODO-N** | FIX-009 | Warmup Z-step + tracking-lost staleness: `has_pose_/has_cloud_` latch forever, no fallback after SLAM track loss. Add timestamp-age staleness fallback. | Medium |
| **TODO-O** | FIX-009 | A/B fairness: baseline arm (`ov2slam_ibvs`) doesn't subscribe `/slam/cloud`, so OV2SLAM skips publishing it (subscription count guard) → different SLAM CPU load between arms. | Low (note for analysis) |

---

## 7. Windows ↔ Pi sync (CRITICAL — do before running on Windows)

All code changes from this session are **Pi-side only**. The Windows repo (`controller-benchmark` branch) does NOT have:

```
run_stack_hil.sh                              (sidecar block, mode, SLAM_LD_PREFIX, use_slam_depth)
scripts/parse_stack.py                        (SLAM_SETUP_OVERLAY, SLAM_LD_PREFIX, CONTROLLER_USE_SLAM_DEPTH)
src/sim_camera_bridge/launch/hil_simulation.launch.py  (use_slam_depth arg)
config/hil/bench_ibvs.yaml                    (slam_depth_scale, removed use_slam_depth)
config/hil/stack/ov2slam_ibvs.yaml            (use_slam_depth:false + /slam/cloud remap)
config/hil/stack/ov2slam_ibvs_slamdepth.yaml  (NEW — A/B variant)
config/hil/stack/orbslam2_eval.yaml           (NEW)
src/visp_servo/include/visp_servo/depth/      (NEW — slam_depth_source.hpp)
src/visp_servo/src/depth/                     (NEW — slam_depth_source.cpp)
src/visp_servo/include/visp_servo/ibvs_controller.hpp  (use_slam_depth_, slam_depth_scale_)
src/visp_servo/src/ibvs_controller.cpp        (SlamDepthSource construction)
src/orbslam2/                                  (ENTIRE directory — NEW)
benchmarks/eval_slam_hil.py                   (NEW — Phase A.5)
docs/fixlog/009-slam-depth-ibvs.md            (NEW)
docs/fixlog/README.md                         (index updated)
```

**To sync Pi→Windows:**
```bash
# On Pi — commit and push
cd /home/amaraly/ROS2-PROJECT-SPRING2026
git add .
git commit -m "Phase A+A.5: ORB-SLAM2 sidecar, IBVS depth wiring, eval_slam_hil.py"
git push origin controller-benchmark

# On Windows — pull
git fetch origin
git reset --hard origin/controller-benchmark
```

> **Warning:** `git reset --hard` will overwrite any local Windows-only changes (MATLAB plots etc.). If you have uncommitted Windows-side work, stash it first.

---

## 8. Phase B — ORB-SLAM3 live integration (PLANNED, not started)

ORB-SLAM3 as-found on the Pi is an offline EuRoC benchmark harness, NOT a live SLAM node:
1. Waits for `/mono_py_driver/experiment_settings` handshake (String naming a config YAML)
2. Reads images from disk (`cv2.imread` EuRoC files), not from a ROS topic
3. Publishes trajectory to a `.txt` file, NOT to a pose topic

**Needed before it works as a sidecar:**
- `slam_live_adapter.py` (~50 lines): sends handshake + republishes `/ovcam/image_raw` → `/mono_py_driver/img_msg` + `/timestep_msg`
- C++ patch to ORB-SLAM3 node: publish `PoseStamped` on a pose topic (same pattern as ORB-SLAM2 fix)
- Config schema needs no changes — `slam.command` can launch `adapter & orbslam3_node`

---

## 9. N=10 Controller benchmark results (for reference)

**Date:** 2026-06-23 | **N=10 per controller** | All healthy, zero overshoot

| Metric | IBVS (TS1) | Proportional (TS2) | h_vs (TS3) | Winner |
|--------|----------:|------------------:|----------:|--------|
| Settling time (s) | **19.17 ± 0.97** | 34.14 ± 1.31 | 35.22 ± 1.58 | IBVS |
| Steady-state err (m) | **0.180 ± 0.014** | 0.214 ± 0.014 | 0.183 ± 0.023 | IBVS ≈ h_vs |
| Path efficiency | 0.887 ± 0.013 | **0.998 ± 0.005** | 0.986 ± 0.002 | Proportional |
| RMS cmd speed (m/s) | 2.380 ± 0.343 | 0.957 ± 0.131 | **0.840 ± 0.200** | h_vs |
| Control jerk (TV) | 36.53 ± 5.03 | 4.831 ± 0.274 | **1.692 ± 0.066** | h_vs (22× smoother) |
| IAE | **329.0 ± 20.5** | 515.7 ± 33.8 | 514.6 ± 27.9 | IBVS |

Full report: `reports/N10_all_controllers/REPORT.md`

---

## 10. Key file map

```
run_stack_hil.sh                              — entry point (modular, SLAM sidecar, benchmark recording)
scripts/parse_stack.py                        — nested YAML → flat env vars
config/hil/stack/                             — stack configs (one per experiment)
config/hil/bench_ibvs.yaml                   — IBVS gains (lambda, k_fwd, slam_depth_scale)
config/hil/bench_proportional.yaml           — proportional gains
config/hil/bench_bag_qos.yaml                — QoS overrides for bag recording

src/sim_camera_bridge/launch/hil_simulation.launch.py  — ROS2 launch (controller/detector selection, CPU pinning)
src/visp_servo/                               — IBVS controller (TS1)
  src/ibvs_controller.cpp                     — use_slam_depth, SlamDepthSource construction
  src/depth/slam_depth_source.cpp             — subscribes /slam/pose + /slam/cloud, median depth
  include/visp_servo/depth/slam_depth_source.hpp
src/hil_servo/                                — Proportional controller (TS2)
src/h_vs_servo/                               — Homography VS controller (TS3)
src/visp_pbvs_servo/                          — PBVS controller (TS4, PAUSED)
src/oracle_detector/                          — oracle detector node
src/orbslam2/                                 — ORB-SLAM2 source + HIL wrapper
  ros2-ORB_SLAM2/src/monocular/               — modified node (pose publisher, hil_sim.yaml)
  Vocabulary/ORBvoc.txt                       — vocab file
  start_container                             — build script

benchmarks/eval_slam_hil.py                  — Phase A.5: Umeyama ATE eval on benchmark bag
benchmarks/plot_controller_hil.py            — per-run controller metrics + figures
benchmarks/compare_controllers.py            — N-run comparison figures

docs/fixlog/                                 — ADR-style fix archive (one file per fix)
  001…009-*.md                               — individual fix entries
  README.md                                  — index + template

camera_calib/hil_sim_ov2slam.yaml            — OV2SLAM camera calibration (fx=fy=554)
datasets/euroc/                              — EuRoC bags (for ORB-SLAM2 offline testing)
```

---

## 11. Known gotchas / Pi environment

- **Interface:** `wlan0` has no IPv4 on the Pi. All configs use `interface: wlan0` but DDS goes over eth0. The actual network binding is via `MATLAB_HOST_IP` unicast. If Pi can't reach MATLAB: check eth0 IP.
- **Bare `colcon build` crashes** on vendored OpenCV in `/workspace/opencv/`. Always use `--base-paths src --packages-up-to <pkgs>`.
- **`yolo_producer`** hardcoded to `/usr/bin/python3` — pyenv shim at `~/.pyenv/shims/python3` does NOT have hailo_platform.
- **Bags created root inside Docker**: `sudo chmod -R 777 bags/run_*` if PermissionError on host.
- **`taskset` in launch**: must be a string prefix (`'taskset -c 0'`), NOT a list — list form concatenates without spaces → `taskset-c0` (not a real command). See FIX-007.
- **OV2SLAM `--entrypoint ""`**: required on `ros2_perception_stack`. See FIX-007.
- **ORB-SLAM2 `SLAM_SETUP_OVERLAY` / `SLAM_LD_PREFIX`** bash lines in `run_stack_hil.sh` use NO inner double-quotes: `[[ -n ${SLAM_SETUP_OVERLAY:-} ]]` — double-quoting with ssh causes local shell to expand before writing.
- **Sidecar container naming**: all SLAM containers named `slam_<type>`. `stop` uses `docker rm -f $(docker ps -aq --filter 'name=^slam_')` — this is why containers must use the `slam_` prefix convention.
