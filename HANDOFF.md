# Project Handoff — ROS2 HIL Visual Servoing (2026-07-02)

**Branch:** `controller-benchmark` | **Pi SSH:** `ssh amaraly@192.168.1.60` (passwordless, key deployed)
**Pi repo:** `/home/amaraly/ROS2-PROJECT-SPRING2026/` | **Container:** `ros2_perception_stack`

> This document supersedes the 2026-06-29 handoff. See `docs/fixlog/` for individual applied
> fixes and `docs/ideas_to_check/` for open hypotheses that were investigated but not yet
> resolved/implemented. Read `docs/ideas_to_check/001-orbslam2-monocular-init-reliability.md`
> before touching `orbslam2_eval.yaml` CPU pinning again — it's been tried three ways already.

---

## 0. TL;DR for a fresh session

1. **Nothing from today (2026-07-02) is committed anywhere.** Windows has uncommitted changes,
   Pi has *different* uncommitted changes, and they conflict with each other on one file. See
   §7 before running `git` anything.
2. **Don't re-litigate CPU pinning on `orbslam2_eval.yaml`.** It was tried three configurations
   (unpinned, pinned "2,3", fully-separated across 3 cores) across 5 runs and made no
   statistically meaningful difference to SLAM init reliability. Root cause is believed to be
   monocular-initializer sensitivity to frame-pair luck under forward-dominant motion, not CPU
   scheduling. Full writeup: `docs/ideas_to_check/001-orbslam2-monocular-init-reliability.md`.
3. If you're picking up the SLAM-init-reliability thread, the next concrete step is **TODO-Q**
   (§6) — decide whether to implement the opt-in FSM search-gate fix, or accept variance.

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
| Phase A.5 — ATE eval | **DONE** | `benchmarks/eval_slam_hil.py` (Umeyama + ATE RMSE, path length, evo_traj plot) |
| Phase A.6 — SLAM timing/CPU/thread instrumentation | **DONE** | `timing_events.csv`, `slam_process_cpu.csv`, `slam_thread_cpu.csv` (per-thread, matches EuRoC report's `ORBFrontEnd`/`ORBLocalMap`/`ORBGBA` role names) |
| N=10 controller benchmark | **DONE** | Results in `reports/N10_all_controllers/REPORT.md` |
| `--run-tag` flag | **DONE** | Label benchmark run dirs: `bags/run_<cfg>_<tag>_<stamp>/` |
| **ORB-SLAM2 monocular init reliability** | **OPEN, unresolved** | Highly erratic across runs (2–32% eval-window coverage, scale factor 2.2–30.9×). CPU pinning hypothesis tested and falsified. See `docs/ideas_to_check/001-*.md` and §9 below. |
| TS4 PBVS | **PAUSED** | Diverges in APPROACHING — see `docs/fixlog/005-vx-unbounded.md`, TODO-H |
| ORB-SLAM3 live integration | **PLANNED** | Needs adapter + C++ pose-topic patch (see §10) |
| Windows ↔ Pi sync | **PENDING, CONFLICTING** | Both sides have uncommitted, *different* local edits. See §7 — resolve before syncing. |

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

# Control the run dir label: bags/run_<cfg>_<tag>_<stamp>/
./run_stack_hil.sh --config orbslam2_eval --run-tag run3

# SLAM depth A/B pair
./run_stack_hil.sh --config ov2slam_ibvs           # baseline: bbox depth
./run_stack_hil.sh --config ov2slam_ibvs_slamdepth # SLAM depth via /slam/cloud

# Stop everything (kills stack + SLAM sidecar + CPU/thread samplers)
./run_stack_hil.sh stop

# Build
./run_stack_hil.sh --build --config ov2slam_ibvs   # build then launch
./run_stack_hil.sh build visp_servo                 # build one package

# Rate check with correct DDS env
./run_stack_hil.sh hz [topic]                       # defaults to /sim/camera/image_raw
```

### 3b. Evaluate a SLAM benchmark bag

```bash
# After running any benchmark-mode config:
python3 benchmarks/eval_slam_hil.py bags/run_orbslam2_eval_<tag>_<stamp>

# Output (written into the run dir):
#   slam_metrics.csv           — ATE RMSE + scale + Hz + sample count + FE latency + CPU + threads
#   fig_slam_ate.png           — ATE vs time
#   fig_slam_trajectory.png    — GT vs aligned SLAM (top-down XY, full flight in gray, eval window in blue)
#   fig_slam_error_xyz.png     — per-axis translation error
#   fig_slam_thread_cpu.png    — per-thread CPU bar chart (ORBFrontEnd/ORBLocalMap/ORBGBA/etc., matches EuRoC report style)
#   traj_xy_trajectories.png   — evo_traj-style XY overlay (matches EuRoC benchmark plots)
```

Key metrics: `ATE_RMSE_m` (Absolute Trajectory Error after Umeyama SE(3)+scale alignment),
`umeyama_scale` (how far raw monocular scale is from metric — should be near 1–2 for a healthy
run; values >5 indicate a poorly-conditioned initial map), `eval_path_length_m` vs
`full_path_length_m` (what fraction of the flight actually got evaluated — this has been the
main problem this session, see §9).

### 3c. Evaluate a controller benchmark bag

```bash
python3 benchmarks/plot_controller_hil.py bags/run_<ctrl>_<stamp>

# Compare across multiple runs (N=10 example):
python3 benchmarks/compare_controllers.py reports/N10_all_controllers/ \
    bags/ctrl_ibvs_N{1..10}_* bags/ctrl_proportional_N{1..10}_* bags/ctrl_h_vs_N{1..10}_*
```

**Note:** the N=10 controller benchmark uses a **completely separate launch harness**
(`benchmarks/controller_bench.launch.py` via `benchmarks/controller_hil_bench.sh`) from the SLAM
HIL stack (`hil_simulation.launch.py` via `run_stack_hil.sh`). Same `oracle_detector_node` code
and same `bench_oracle.yaml` params in both — but the N=10 harness runs **unpinned, in total
isolation** (no camera bridge, no SLAM, no YOLO — nothing else running), whereas
`orbslam2_eval.yaml` runs oracle+controller in whatever CPU configuration is currently set
(see §7/§9) alongside `sim_camera_bridge`, `ovcam_bridge`, and the SLAM sidecar. **These two are
not directly comparable environments** — don't assume N=10 controller numbers transfer 1:1 to
SLAM-run conditions.

---

## 4. Stack configs (all in `config/hil/stack/`)

| Config | Detector | Controller | SLAM | Mode | Purpose |
|--------|----------|------------|------|------|---------|
| `default.yaml` | yolo | ibvs | off | scout | base no-SLAM test |
| `oracle.yaml` | oracle | ibvs | off | scout | no Hailo needed |
| `ibvs.yaml` | yolo | ibvs | off | scout | IBVS standalone |
| `h_vs.yaml` | yolo | h_vs | off | scout | homography VS |
| `full_ov2slam.yaml` | yolo | proportional | ov2slam | scout | OV2SLAM live flight |
| `ov2slam_ibvs.yaml` | yolo | ibvs | ov2slam | scout | SLAM depth A/B baseline (bbox depth) |
| `ov2slam_ibvs_slamdepth.yaml` | yolo | ibvs | ov2slam | scout | SLAM depth A/B active (`/slam/cloud`) |
| `ov2slam_oracle.yaml` | oracle | proportional | ov2slam | **benchmark** | OV2SLAM ATE capture — `use_fast: 0` (accurate mono mode) |
| `orbslam2_eval.yaml` | oracle | proportional | orbslam2 | **benchmark** | ORB-SLAM2 ATE capture — **CPU fields currently uncommitted/diverged, see §7** |

**Key config fields:**
```yaml
mode: scout | benchmark
controller:
  use_slam_depth: true | false   # toggle SLAM depth for IBVS (per-stack)
  use_slam_pose:  true | false   # gate vx on SLAM pose freshness (IBVS only)
slam:
  enabled: true
  image: orbslam2_fixed          # Docker image
  cpu: "2,3"                     # taskset inside sidecar — SEE §7, this is currently divergent
  startup_delay_sec: 15          # DO NOT reduce — prevents ORB-SLAM2 near-zero-parallax auto-exit bug
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
- **Thread names** (set via `pthread_setname_np` in the source): `ORBFrontEnd`, `ORBLocalMap`,
  `ORBGBA`, `ORBLoopClose`, `ORBExtractL`, `ORBExtractR`, `ORBInitH`, `ORBInitF` — these are
  read directly by `benchmarks/slam_thread_sampler.py` via `/proc/1/task/*/comm` and match the
  exact names used in the original EuRoC benchmark report's thread plots.
- **`tracking_state` values** (ORB-SLAM2's own `eTrackingState` enum, recorded per-event in
  `timing_events.csv`): `-1=SYSTEM_NOT_READY, 0=NO_IMAGES_YET, 1=NOT_INITIALIZED, 2=OK, 3=LOST`.
  Very useful for diagnosing init failures — see §9.

### Critical ORB-SLAM2 gotchas
1. **Direct binary — not `ros2 run`**: `ros2 run` fails (`librcl_action.so` missing in `orbslam2_fixed` for rclpy). Use full binary path directly.
2. **`LD_LIBRARY_PATH`**: image lacks ldconfig for ROS2 libs. Must set via `slam.ld_prefix` in config: `/workspace/src/orbslam2/lib:/usr/local/lib:/opt/ros/jazzy/lib`
3. **`rclcpp::remove_ros_arguments`**: called AFTER `rclcpp::init` in `mono.cpp` so positional args survive `--ros-args` remaps.
4. **`--entrypoint ""`**: `ros2_perception_stack` has a custom entrypoint — all sidecar `docker run` calls must use `--entrypoint ""` or the entrypoint loops.
5. **Tcw size guard**: `monocular-slam-node.cpp` checks `!Tcw.empty() && Tcw.rows==4 && Tcw.cols==4` before publishing — ORB-SLAM2 returns partial matrices during tracking loss.
6. **Headless**: `Viewer.UseViewer: 0` required in hil_sim.yaml — Pangolin crashes without display.
7. **`startup_delay_sec: 15` is load-bearing** — it exists so the drone is already moving
   (accumulating parallax) *before* ORB-SLAM2's node comes online, avoiding a near-zero-parallax
   auto-exit bug in the wrapper. Do not shrink this without testing for that regression.

### OV2SLAM sidecar (`ros2_perception_stack` image)
- Publishes `vo_pose` → remapped to `/slam/pose`; `point_cloud` → remapped to `/slam/cloud`
- Image does NOT need LD_LIBRARY_PATH fixes (already configured)
- `--entrypoint ""` required (custom entrypoint in image)
- `camera_calib/hil_sim_ov2slam.yaml`: `use_fast: 0` → **accurate mode** (not fast). Front-end
  ~10.7ms mean in accurate-mono mode (vs. ~5ms fast-stereo in the EuRoC report — different
  operating point, don't compare directly without noting mono vs stereo).
- `eval_slam_hil.py` does **not** currently parse OV2SLAM's native `ov2slam_timings.csv` format
  (different columns from ORB-SLAM2's `timing_events.csv`) — ATE/CPU work fine, FE latency
  columns will be blank for OV2SLAM runs unless a `read_ov2slam_timing()` parser is added.

### SLAM depth for IBVS (`SlamDepthSource`)
- File: `src/visp_servo/src/depth/slam_depth_source.cpp`
- Subscribes canonical `/slam/pose` + `/slam/cloud` (hardcoded constants — no params)
- Projects map points into camera frame, returns median depth of points inside bbox
- Falls back to bbox-ratio Z when no SLAM data
- Toggle: `controller.use_slam_depth: true` in stack config
- Scale: `slam_depth_scale` in `config/hil/bench_ibvs.yaml` (default 1.0)
- **Thread safety note**: runs on single-threaded executor (visp node's `rclcpp::spin`) — no mutex. If moved to MultiThreadedExecutor, add one.
- `orbslam2_eval.yaml` explicitly does **not** set `use_slam_depth`/`use_slam_pose` (defaults
  `false` in `parse_stack.py`). Confirmed by code inspection: SLAM's pose is recorded for
  evaluation only in this config, it never touches `/cmd_vel`. The real GT drone trajectory was
  independently verified smooth/consistent across runs regardless of SLAM behavior (see §9).

---

## 6. Open TODOs (active — resume here)

| ID | Fix/Idea | Description | Priority |
|----|-----|-------------|----------|
| **TODO-Q** | `docs/ideas_to_check/001` | Decide: implement opt-in FSM search-gate fix (force `SEARCHING` through `STRAFE_RIGHT` before `APPROACHING` lock-on, gated by a new default-`false` param) vs. accept run-to-run SLAM-init variance vs. report the variance itself as a paper finding. | **High — next concrete step** |
| **TODO-R** | `docs/ideas_to_check/001` | If pursuing TODO-Q: confirm the new gate parameter defaults `false` and verify `controller_bench.launch.py`/N=10 benchmark behavior is provably unchanged before merging (shared FSM code risk). | High, blocks TODO-Q |
| **TODO-H** | FIX-005 | PBVS paused: re-enable `wz` to track bearing, or diagnose why TS2 survives same geometry without yaw. Resume via `docs/fixlog/005-vx-unbounded.md`. | Medium |
| **TODO-K** | FIX-006 | IBVS residual oscillation after decoupled fix — confirm lambda tuning or close-range dropout. | Medium |
| **TODO-L** | FIX-009 | Monocular scale drift: `depth_scale_` constant calibration doesn't hold across scale drift. Fix: align SLAM scale to known target height or sim GT before trusting SLAM-depth IBVS numbers. | High before SLAM-depth IBVS flights |
| **TODO-M** | FIX-009 | Background bias: bbox-median depth includes background points. Fix: gate to nearest depth cluster or lower percentile. | Low |
| **TODO-N** | FIX-009 | Warmup Z-step + tracking-lost staleness: `has_pose_/has_cloud_` latch forever, no fallback after SLAM track loss. Add timestamp-age staleness fallback. | Medium |
| **TODO-O** | FIX-009 | A/B fairness: baseline arm (`ov2slam_ibvs`) doesn't subscribe `/slam/cloud`, so OV2SLAM skips publishing it (subscription count guard) → different SLAM CPU load between arms. | Low (note for analysis) |

---

## 7. Windows ↔ Pi sync (CRITICAL — read before running any git command)

**Direction reversed from last handoff.** Most of today's session originated on **Windows**
(per-thread sampler, `--run-tag` flag, `iproute2` in `start_container.sh`, `docs/ideas_to_check/`).
But the CPU-pinning investigation happened live on the **Pi**, with the user hand-editing
`orbslam2_eval.yaml` directly over SSH between test runs — those edits were never synced back
to Windows and are **not the same** as what Windows currently has uncommitted.

### Current uncommitted state (as of 2026-07-02, verified via `git status`/`git diff`)

**Windows**, uncommitted:
```
M  config/hil/stack/orbslam2_eval.yaml     — detector.cpu:"0"→"", controller.cpu:"0"→"", slam.cpu:""→"2,3"
M  matlab/slprj/.../*.mat                  — Simulink autosave noise, unrelated, ignore/exclude
?? docs/ideas_to_check/                    — NEW (this session)
```
Committed but not yet pushed to remote (last push was the `add7c90` commit from 2026-07-01):
already on `origin/controller-benchmark`? **Check `git log origin/controller-benchmark -1`
before assuming** — the previous session pushed `add7c90`, this session's edits on top of that
are still local.

**Pi**, uncommitted (git_sha at HEAD is still `add7c90`, same as Windows' last push):
```
M  benchmarks/slam_cpu_sampler.sh          — mode-only diff (644→755 chmod), zero content change, safe to discard
M  config/hil/stack/orbslam2_eval.yaml     — detector.cpu:"0"→"0" (unchanged), controller.cpu:"0"→"3", slam.cpu:""→"1,2"
```

**These two `orbslam2_eval.yaml` edits conflict** — Windows has `("", "", "2,3")` for
(detector, controller, slam) CPU fields; Pi has `("0", "3", "1,2")`. **Neither combination fixed
the SLAM init variance** (see §9/`docs/ideas_to_check/001`), so there's no evidence-based reason
to prefer one over the other. Recommended resolution: pick one (or a fresh third option) as the
canonical value, `git checkout --` the losing side, and note in the commit message that both
were empirically tested with no measurable difference — don't waste a future session
re-discovering this.

### To sync (once the conflict above is resolved)

```bash
# Decide the winning orbslam2_eval.yaml CPU config first (see above), then:

# On Windows — commit + push
cd "c:/Users/homie/Desktop/ROS2-PROJECT-SPRING2026"
git add benchmarks/slam_thread_sampler.py run_stack_hil.sh start_container.sh \
        config/hil/stack/orbslam2_eval.yaml docs/ideas_to_check/ HANDOFF.md
git commit -m "<describe: thread sampler, run-tag, CPU-pin conclusion, ideas_to_check>"
git push origin controller-benchmark

# On Pi — discard the stale/losing local edits, then pull
ssh amaraly@192.168.1.60
cd ~/ROS2-PROJECT-SPRING2026
git checkout -- benchmarks/slam_cpu_sampler.sh   # discards mode-only diff, no content lost
git checkout -- config/hil/stack/orbslam2_eval.yaml   # discards Pi's local CPU-pin experiment
git fetch origin
git reset --hard origin/controller-benchmark
chmod +x benchmarks/slam_cpu_sampler.sh benchmarks/slam_thread_sampler.py
```

> **Warning:** `git reset --hard` on the Pi will overwrite the Pi's uncommitted CPU-pin
> experiment. That's intentional per the resolution above — but if a future session wants to
> preserve both experimental values for reference, copy them into
> `docs/ideas_to_check/001-orbslam2-monocular-init-reliability.md`'s run-history table first
> (already partially done — verify it's current).

---

## 8. SLAM benchmark run history (2026-07-01 → 2026-07-02, `orbslam2_eval` config)

All runs below are on the **same underlying scenario** (`full_path_length_m` ≈ 93–95m every
time — same flight replayed) but with the ORB-SLAM2 sidecar `cpu` field changed between
attempts while chasing the init-reliability problem. Full analysis in
`docs/ideas_to_check/001-orbslam2-monocular-init-reliability.md`.

| Run tag | CPU (det/ctrl/slam) | duration_s | eval_path_m | umeyama_scale | ATE_RMSE_m | Notes |
|---|---|---|---|---|---|---|
| (historical, pre-session) | pinned "0"/"0"/"2,3" | 63.9 | 23.5 | ~1–2 | 0.017 | Original good run in old paper.tex table |
| 33 | "0"/"0"/unpinned | 9.18 | 2.12 | 2.23 | 0.085 | First unpin attempt — short window |
| 35 | "0"/"0"/unpinned | 46.66 | 17.17 | **9.91** | 0.493 (max 8.01) | Two internal resets before init succeeded (rushed 3rd attempt) |
| 37 | "0"/"3"/"1,2" | 11.51 | 2.59 | 2.56 | 0.178 | Fully-separated cores — still short |
| 38 | "0"/"3"/"1,2" | 31.30 | 30.48 | **30.93** | 0.203 | Best coverage, worst scale |
| 39 | "0"/"3"/"1,2" | 27.48 | 13.75 | **17.08** | 0.063 | Middling on both |

**Conclusion: CPU pinning does not explain the variance.** Root cause under active
investigation is monocular-initializer sensitivity to which frame pair happens to pass RANSAC
acceptance, compounded by the fact that the `SEARCHING` FSM state's one lateral-motion step
(`STRAFE_RIGHT`) never executes because the oracle detector locks onto the target almost
instantly, before the search sequence reaches it. Full detail in `docs/ideas_to_check/001-*.md`.

Bag directories for these runs live at `~/ROS2-PROJECT-SPRING2026/bags/run_orbslam2_eval_<tag>_<stamp>/`
on the Pi — not synced to Windows (bags are gitignored/too large to sync routinely).

---

## 9. Phase B — ORB-SLAM3 live integration (PLANNED, not started)

ORB-SLAM3 as-found on the Pi is an offline EuRoC benchmark harness, NOT a live SLAM node:
1. Waits for `/mono_py_driver/experiment_settings` handshake (String naming a config YAML)
2. Reads images from disk (`cv2.imread` EuRoC files), not from a ROS topic
3. Publishes trajectory to a `.txt` file, NOT to a pose topic

**Needed before it works as a sidecar:**
- `slam_live_adapter.py` (~50 lines): sends handshake + republishes `/ovcam/image_raw` → `/mono_py_driver/img_msg` + `/timestep_msg`
- C++ patch to ORB-SLAM3 node: publish `PoseStamped` on a pose topic (same pattern as ORB-SLAM2 fix)
- Config schema needs no changes — `slam.command` can launch `adapter & orbslam3_node`

---

## 10. N=10 Controller benchmark results (for reference)

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

**Reminder (see §3c):** these numbers come from `controller_bench.launch.py`, unpinned, fully
isolated (no camera bridge/SLAM/YOLO running alongside). Not directly comparable to any
`orbslam2_eval`/`ov2slam_oracle` run's controller behavior without accounting for the different
resource environment.

---

## 11. Key file map

```
run_stack_hil.sh                              — entry point (modular, SLAM sidecar, benchmark recording, --run-tag)
scripts/parse_stack.py                        — nested YAML → flat env vars
config/hil/stack/                             — stack configs (one per experiment)
config/hil/bench_ibvs.yaml                   — IBVS gains (lambda, k_fwd, slam_depth_scale)
config/hil/bench_proportional.yaml           — proportional gains
config/hil/bench_bag_qos.yaml                — QoS overrides for bag recording

src/sim_camera_bridge/launch/hil_simulation.launch.py  — ROS2 launch (controller/detector selection, CPU pinning)
src/servo_core/src/servo_fsm_node.cpp         — shared FSM (SEARCHING/APPROACHING/REACQUIRE/REACHED),
                                                  build_search_command() has the STRAFE_RIGHT step (§9/TODO-Q)
src/visp_servo/                               — IBVS controller (TS1)
  src/ibvs_controller.cpp                     — use_slam_depth, SlamDepthSource construction
  src/depth/slam_depth_source.cpp             — subscribes /slam/pose + /slam/cloud, median depth
  include/visp_servo/depth/slam_depth_source.hpp
src/hil_servo/                                — Proportional controller (TS2)
src/h_vs_servo/                               — Homography VS controller (TS3)
src/visp_pbvs_servo/                          — PBVS controller (TS4, PAUSED)
src/oracle_detector/                          — oracle detector node (identical code/params in both harnesses, §3c)
src/orbslam2/                                 — ORB-SLAM2 source + HIL wrapper
  ros2-ORB_SLAM2/src/monocular/               — modified node (pose publisher, hil_sim.yaml)
  Vocabulary/ORBvoc.txt                       — vocab file
  start_container                             — build script
  src/BenchmarkUtils.cc                       — ScopedBenchmarkTimer, SetBenchmarkThreadName (thread naming)

benchmarks/eval_slam_hil.py                  — ATE eval + path length + all fig_slam_*.png + thread CPU plot
benchmarks/slam_cpu_sampler.sh               — whole-process CPU/mem/thread-count sampler (docker stats)
benchmarks/slam_thread_sampler.py            — NEW (2026-07-01): per-thread CPU sampler (/proc/1/task/*/comm)
benchmarks/controller_bench.launch.py        — ISOLATED oracle+controller harness (N=10 benchmark, no SLAM)
benchmarks/controller_hil_bench.sh           — driver script for the above
benchmarks/plot_controller_hil.py            — per-run controller metrics + figures
benchmarks/compare_controllers.py            — N-run comparison figures

docs/fixlog/                                 — ADR-style archive of APPLIED fixes (one file per fix)
  001…010-*.md                               — individual fix entries
  README.md                                  — index + template
docs/ideas_to_check/                         — NEW (2026-07-02): open hypotheses NOT yet implemented
  001-orbslam2-monocular-init-reliability.md — today's full SLAM-init investigation writeup
  README.md                                  — index + template

start_container.sh                           — NEW: installs gettext-base/figlet/iproute2, builds packages
camera_calib/hil_sim_ov2slam.yaml            — OV2SLAM camera calibration (fx=fy=554, use_fast:0=accurate)
datasets/euroc/                              — EuRoC bags (for ORB-SLAM2 offline testing)
matlab/hil_closed_loop.slx                   — Simulink model; drone SPAWN POSITION lives here (block IC,
                                                 not text-searchable — open in Simulink to change)
matlab/hil_ros_init_LT.m                     — only publishes /sim/drone_pose, does NOT set initial position
```

---

## 12. Known gotchas / Pi environment

- **Interface:** `wlan0` is the real DDS interface (confirmed via SSH 2026-07-01: `inet 192.168.1.60/24` on wlan0). An earlier note claiming "wlan0 has no IPv4, DDS goes over eth0" was stale/wrong — corrected.
- **`run_stack_hil.sh` wlan0 startup race**: the interface-IP check (`ip -4 addr show wlan0 | grep -oP 'inet \K...'`) runs once with no retry. If the script launches right after a Pi reboot/WiFi reconnect, wlan0 may still be mid-DHCP and the script dies with `Interface 'wlan0' has no IPv4 address` even though the IP appears a few seconds later. Fix: just re-run — no code retry loop added (declined, logged here instead of TODO).
- **`ip` (iproute2) missing** was the root cause of a related failure — installing it fixed the issue. `iproute2` is now installed automatically in `start_container.sh` step 2/3 alongside `gettext-base`/`figlet`.
- **CPU pinning on `orbslam2_eval.yaml` is inconclusive, don't re-litigate without new evidence** — see §7/§9/`docs/ideas_to_check/001-*.md`. Three configurations tested, no measurable improvement in SLAM init reliability.
- **`docker exec`-based samplers inflate `docker stats` PID counts**: `slam_thread_sampler.py` runs `docker exec <container> bash -c "..."` every 2s to read `/proc/1/task/`. While that exec is alive, it transiently adds a process to the container's PID namespace, which `slam_cpu_sampler.sh`'s `docker stats --format {{.PIDs}}` also reads — inflates the reported `threads` metric in `slam_metrics.csv` by a few counts (observed 15→17-20). Not a real thread-count change; a measurement artifact from having both samplers running simultaneously.
- **Bare `colcon build` crashes** on vendored OpenCV in `/workspace/opencv/`. Always use `--base-paths src --packages-up-to <pkgs>`.
- **`yolo_producer`** hardcoded to `/usr/bin/python3` — pyenv shim at `~/.pyenv/shims/python3` does NOT have hailo_platform.
- **Bags created root inside Docker**: `sudo chmod -R 777 bags/run_*` if PermissionError on host.
- **`taskset` in launch**: must be a string prefix (`'taskset -c 0'`), NOT a list — list form concatenates without spaces → `taskset-c0` (not a real command). See FIX-007.
- **OV2SLAM `--entrypoint ""`**: required on `ros2_perception_stack`. See FIX-007.
- **ORB-SLAM2 `SLAM_SETUP_OVERLAY` / `SLAM_LD_PREFIX`** bash lines in `run_stack_hil.sh` use NO inner double-quotes: `[[ -n ${SLAM_SETUP_OVERLAY:-} ]]` — double-quoting with ssh causes local shell to expand before writing.
- **Sidecar container naming**: all SLAM containers named `slam_<type>`. `stop` uses `docker rm -f $(docker ps -aq --filter 'name=^slam_')` — this is why containers must use the `slam_` prefix convention.
- **CLAUDE.md is `.gitignore`'d** (repo root, line 64) — any working-agreement edits made there (e.g. "LAUNCH INTERVIEW MODE" section) are local to whichever machine made them and do NOT sync via git. If both Windows and Pi need the same CLAUDE.md content, it must be copied manually.
