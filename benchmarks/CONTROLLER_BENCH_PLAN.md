# Controller Benchmark — IBVS (TS1) vs Proportional (TS2)

**Branch:** `controller-benchmark` · **DDS:** CycloneDDS only · **Mode:** real-time HIL, closed-loop, N-run.

This benchmarks two *servoing laws* on the identical plant, FSM, safety
filters and perception, so any measured difference is the control law alone.

---

## 1. What is being compared

| Test subject | Package / node | Servoing law |
|---|---|---|
| **TS1** | `visp_servo` → `visp_servo_node` | **IBVS** — ViSP `vpServo`, 4 corner features, interaction matrix + pseudo-inverse: `v = −λ·L⁺·(s − s*)` |
| **TS2** | `hil_servo` → `hil_servo_node` | **Proportional** — decoupled P-law: `vx=k_fwd·(ratio*−ratio)`, `vy=−k_lat·ex`, `vz=−k_vz·ey` |

Both are **standalone ROS2 nodes**. Both wrap the **same** `servo_core::ServoFsmNode`
(an `rclcpp::Node`): same FSM (SEARCHING→APPROACHING↔REACQUIRE→REACHED), same
safety filters (sim-health, clamp, floor, velocity ramp), same `/bench/state`
instrumentation. The ONLY difference is the injected `IServoController`.

```
            /yolo/detections (oracle bbox)
                       │
        ┌──────────────▼───────────────┐
        │  servo_core::ServoFsmNode     │  shared — identical for TS1 & TS2
        │  FSM + filters + /bench/state │
        └──────────────┬───────────────┘
            APPROACHING │ delegates body velocity
        ┌──────────────▼───────────────┐
        │ IServoController  (SWAP HERE) │  TS1 = IBVS   TS2 = Proportional
        └──────────────┬───────────────┘
                       ▼  /cmd_vel
```

**Fairness invariants** (held identical, see `config/hil/bench_fsm.yaml`):
camera intrinsics, image size, **bbox-based depth** `Z = fy·H/bh` (no SLAM —
so depth is not a confound), velocity clamps, slew-rate limiter, FSM
thresholds, target, and start pose.

## 2. Package layout (new on this branch)

```
src/servo_core/      shared FSM base (library) + IServoController interface
src/visp_servo/      TS1 — IBVS node   (depends servo_core + ViSP)
src/hil_servo/       TS2 — proportional node (depends servo_core)
src/oracle_detector/ YOLO-agnostic perfect-bbox node (Python)
config/hil/bench_fsm.yaml          shared FSM/geometry params  (/** wildcard)
config/hil/bench_ibvs.yaml         TS1 gains (lambda)
config/hil/bench_proportional.yaml TS2 gains (k_fwd/k_lat/k_vz)
config/hil/bench_oracle.yaml       oracle params
config/hil/bench_bag_qos.yaml      rosbag QoS overrides
benchmarks/controller_bench.launch.py  oracle + one controller
benchmarks/controller_hil_bench.sh     one run (N=1): record bag + CPU
benchmarks/plot_controller_hil.py      offline analysis → metrics + figures
"older files, ignore/"   pre-refactor controllers (gitignored, kept for ref)
```

The existing full HIL stack (`run_stack_hil.sh` / `hil_simulation.launch.py`)
now launches `hil_servo` (TS2 proportional) — same behaviour as before the
refactor. The benchmark uses its own minimal launch.

### `visp_servo` internal layout (modular)

```
visp_servo_main.cpp     thin — just init + create node + set controller + spin
ibvs_controller.cpp     thin orchestration (depth module + law + vpServo task)
ibvs_law.{hpp,cpp}      pure IBVS math, one job per function (ROS-free)
depth/depth_source.hpp  IDepthSource interface
depth/bbox_depth_source bbox depth (default — fair vs TS2)
depth/slam_depth_source SLAM depth via /vo_pose + /point_cloud (fallback bbox)
```

**Adding SLAM is one flag.** Set `use_slam_depth: true` in `bench_ibvs.yaml`
(or `ros2 run ... -p use_slam_depth:=true`). The controller swaps the depth
module and the `/vo_pose` + `/point_cloud` subscriptions appear — the IBVS law
is untouched. Default `false` keeps depth identical to TS2 for the fair
comparison.

## 3. The oracle detector (YOLO-agnostic)

`oracle_detector` projects the **known** target world point through the sim
camera using **ground-truth** `/sim/drone_pose`, and publishes a perfect box on
`/yolo/detections` — same topic/type as real YOLO, so the controller can't tell.
The box **shrinks when far, grows when near** (`size_height = fy·H/Zc`), exactly
like a real detector, but it is deterministic and identical every run. This
removes the detector as a variable. (Phase-2: dial in `pixel_noise_std` /
`dropout_prob` to study detector sensitivity — for free.)

> ⚠ **Validate the projection once** before trusting runs: with the live
> camera view up, confirm the oracle box lands on the actual target. If it's
> vertically inverted, set `pitch_sign: -1.0` in `bench_oracle.yaml`. This is
> the one piece that depends on Simulink axis conventions.

## 4. Methodology — why real-time HIL, closed-loop, N runs

- **Closed-loop, not open-loop.** A controller's job is feedback (converge,
  stay stable). Open-loop ("given this input, what command?") cannot measure
  settling, overshoot or stability — it deletes the thing under test. So every
  run is live against the MATLAB plant.
- **Variance is controlled, not feared.** Everything controllable is pinned
  (fixed IC, deterministic oracle, fixed rates, one DDS, pinned core). The only
  residual is real-time timing jitter — which we *report* (cross-run spread) as
  a robustness result, not hide.
- **Measure from search-exit.** The FSM/search phase is identical and out of
  scope. The stopwatch starts at the first SEARCHING→APPROACHING edge (marked
  on `/bench/state`); everything before is ignored.
- **N escalation (cheap variance-sensitive stopping).** Start N=1 to validate
  the pipeline end-to-end. Then N=3. If TS1 and TS2 separate cleanly on the
  primary metric (`|μ_A−μ_B| > 2·max(σ_A,σ_B)` with within-arm CV < ~0.25),
  stop. If ambiguous → N=5 → N=10. Re-fly only invalid runs (never averaged in;
  logged as success rate). Spend runs only where the result is close.

## 5. Metrics (all computed offline from the bag)

From `/sim/drone_pose` + `/sim/target_pose` (ground truth), `/cmd_vel`, and the
CPU trace. `e(t) = ‖drone − target‖₃D − standoff`, standoff ≈ 3.15 m.

settling time · steady-state error · overshoot · **IAE / ITAE / ISE** ·
path efficiency · RMS command speed · total variation (smoothness) ·
controller CPU % · (loop latency available from node logs).

Figures: distance-to-target vs time, top-down XY trajectory + standoff ring,
command vs time. Aggregated across runs as mean ± std on the sim-time axis.

> Honest note: both laws are sub-ms per tick, but IBVS does an 8×6 pseudo-inverse
> every frame vs the P-law's handful of multiplies, so CPU *is* a real axis here.
> The headline differentiators will be trajectory shape / settling / smoothness.

---

## 6. N=1 RUNBOOK — do this, don't be an idiot

**Goal of N=1:** prove the whole pipeline works end-to-end and the oracle box is
correct — *before* spending runs on statistics. CycloneDDS on BOTH sides.

### A. One-time build (inside Docker, once)

```bash
# host
./start_container.sh        # or your usual container start
sudo docker exec ros2_perception_stack bash -lc "
  source /opt/ros/jazzy/setup.bash
  cd /workspace
  colcon build --packages-select yolo_msgs servo_core hil_servo visp_servo oracle_detector --symlink-install
"
```
`visp_servo` needs ViSP installed in the image (it already is — the old node
linked it). If the IBVS build fails on ViSP, build the other three first and
benchmark TS2 alone for N=1.

Install `rosbags` for analysis (host or container, wherever you run the plot):
```bash
pip install rosbags matplotlib numpy
```

### B. MATLAB side — EXACTLY your normal sequence, with ONE change

Follow `SIM_HIL.md` steps 1–5 **as usual**. The only change is forcing
CycloneDDS in step 1 (mandated):

```matlab
clear all
setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')   % <-- Cyclone, not fastrtps
setenv('ROS_DOMAIN_ID','0')
run hil_ros_init_LT
% open hil_closed_loop.slx ; run read_cmdvel_live_interp once (expect error) ; Run the model
```

You do **NOT** need step 6 (the camera publisher) for this benchmark — the
oracle uses pose, not the image. Leaving the camera off is fine and lighter.
**Do not run `clear all` again after init.** Nothing else about MATLAB changes,
so there's no new command that can brick it.

### C. Pi side — start ONE controller + recorder

Do **not** run `run_stack_hil.sh` (that's the full YOLO/SLAM stack). The bench
is self-contained:

```bash
# host, repo root
export MATLAB_HOST_IP=<your Windows IP>      # same one you use for HIL
# TS2 first (no ViSP dependency — safest first run):
./benchmarks/controller_hil_bench.sh proportional
```

The script renders the CycloneDDS profile, starts the oracle + controller, and
begins recording. When you see **`recording — restart the Simulink sim NOW`**:

### D. Trigger a clean t=0

In MATLAB: **Stop** the Simulink model, then **Run** it again. The heartbeat
jumps back to 0 → the controller auto-resets to SEARCHING (clean t=0). Watch the
Pi log: SEARCHING → APPROACHING → (drone approaches) → REACHED.

Let it hold REACHED for a few seconds, then **Ctrl-C** the bench script.

### E. Sanity-check, then analyse

```bash
python3 benchmarks/plot_controller_hil.py bags/ctrl_proportional_N1_<stamp>
```
Open `fig_distance.png` (should fall toward ~3.15 m and settle) and
`fig_trajectory.png` (path should head to the target). `metrics.csv` has the
numbers. If the distance curve never falls, the controller never engaged —
check the oracle box (step F).

### F. Validate the oracle box (do this on the first run)

While running, in another Pi shell inside the container:
```bash
ros2 topic echo /yolo/detections --once     # center_x/y, size_w/h sane?
```
`center_x` near 320 when the drone faces the target; `size_height` should grow
as it approaches. If behaviour is inverted vertically, set `pitch_sign: -1.0` in
`config/hil/bench_oracle.yaml` and re-run.

### G. Then the other subject

```bash
./benchmarks/controller_hil_bench.sh ibvs
# ... same Stop/Run in MATLAB ... Ctrl-C ... then:
python3 benchmarks/plot_controller_hil.py bags/ctrl_ibvs_N1_<stamp>
```

Compare the two `metrics.csv`. If the pipeline is clean and the curves look
sane, you're ready to scale to N=3.

### Common gotchas
- **Nothing moves:** MATLAB not on CycloneDDS, or wrong `MATLAB_HOST_IP`, or
  `/sim/*` not flowing — check `ros2 topic hz /sim/heartbeat` in the container.
- **Two controllers at once:** never — both publish `/cmd_vel`. The launch runs
  exactly one.
- **Bag empty for /yolo/detections:** QoS — the bench already passes
  `bench_bag_qos.yaml`; don't record without it.
- **IBVS build fails:** ViSP not found in image; benchmark TS2 for N=1, fix ViSP
  for TS1 separately.