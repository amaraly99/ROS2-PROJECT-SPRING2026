# Hardware-in-the-Loop Benchmarks: SLAM and Visual Servoing

**Author:** Amar Aly
**Date:** 2026-08-23

> Monocular visual SLAM and visual-servoing control laws evaluated in closed loop
> on a Raspberry Pi 5, driven by a MATLAB/Simulink + Unreal Engine flight
> simulator.
> ROS 2 Jazzy - Ubuntu 24.04 (Docker) - CycloneDDS

This repository holds a hardware-in-the-loop (HIL) rig and the perception and
visual-servoing stack it closes the loop through. Two benchmarks run on it:

- **The SLAM benchmark** (sections 1 to 9) swaps the pose estimator and holds the
  controller fixed, comparing monocular SLAM backends under a realistic embedded
  compute budget.
- **The controller benchmark** (sections 10 to 11) swaps the servoing law and
  holds everything else fixed, including perception, so any measured difference
  is the control law alone.

They share the rig, the plant, the FSM and the safety filters, which is what makes
each one a clean single-variable study. The subject of this document is those two
benchmarks: what the rig is, what was measured, how, what the numbers are, and
which caveats they carry. The stack is documented only insofar as it is the
vehicle for the measurement.

---

## Table of Contents

1. [What is being measured](#1-what-is-being-measured)
2. [The HIL rig](#2-the-hil-rig)
3. [Candidates](#3-candidates)
4. [Method](#4-method)
5. [Measurement caveats](#5-measurement-caveats-read-before-the-tables)
6. [Results](#6-results)
7. [RTAB-Map: stereo reference point, excluded from the tables](#7-rtab-map-stereo-reference-point-excluded-from-the-tables)
8. [Open issues](#8-open-issues)
9. [Data provenance](#9-data-provenance)
10. [The controller benchmark](#10-the-controller-benchmark)
11. [Controller results](#11-controller-results)
12. [Hardware and prerequisites](#12-hardware-and-prerequisites)
13. [Running the stack](#13-running-the-stack)
14. [Repository layout](#14-repository-layout)
15. [Contributors](#15-contributors)

---

## 1. What is being measured

The question is a systems question, not a SLAM-accuracy question: on a four-core
ARM board that must simultaneously run object detection and a visual servo
controller, which monocular SLAM backend gives the best accuracy per unit of
compute, and does its pose output actually hold a control loop together?

Two experiment families answer it:

- **Offline EuRoC playback.** Each backend replays the EuRoC MAV sequences from a
  bag. This isolates SLAM accuracy and cost from the control loop. 10 sequences x
  10 runs = 100 runs per backend.
- **Online HIL.** Each backend runs inside the closed loop described in
  [section 2](#2-the-hil-rig), where its pose estimate feeds a visual servo
  controller that flies a simulated drone to a target. This measures whether the
  backend is usable, not just accurate.

Both families report the same four axes: accuracy (median absolute pose error),
latency, CPU, and reliability.

Sections 1 to 9 cover this SLAM question. The controller benchmark asks a
different one on the same rig, and states it in
[section 10](#10-the-controller-benchmark).

---

## 2. The HIL rig

The physical camera is replaced by a synthetic Unreal Engine scene rendered on a
Windows host and streamed over the network as a ROS 2 image topic. The entire
perception and control pipeline runs on real Raspberry Pi 5 hardware. The
controller's `/cmd_vel` output drives Simulink drone dynamics, which updates the
Unreal scene, which produces the next camera frame. The loop is closed through
real hardware at real timing.

```
MATLAB/Simulink (Windows)
  Unreal scene -> ROS2 Image publisher -> /sim/camera/image_raw
                                          (bgr8, 640x480, ~20 Hz,
                                           BEST_EFFORT + VOLATILE)
                                                v
+---- RPi5  Docker (--net=host --ipc=host --privileged) -----------------+
|                                                                        |
|  sim_camera_bridge    subs /sim/camera/image_raw                       |
|  (C++ node, Core 0)   BGR8 -> NV12 -> writes /ovcam_frames POSIX SHM   |
|                         (matches ovcam_producer wire format exactly)   |
|                               v                                        |
|        +----------------------+-----------------------+               |
|        v                                              v               |
|  ovcam_bridge                                   yolo_producer          |
|  /ovcam/image_raw mono8                         (host, Core 1, Hailo)  |
|        v                                              v               |
|  SLAM backend (Cores 2,3)                       yolo_bridge            |
|  /slam/pose, /point_cloud                       /yolo/detections       |
|        +----------------------+-----------------------+               |
|                               v                                        |
|                   hil_servo_node (Core 0)                              |
|                   target_class = "stop sign"   (min_conf 0.20)         |
|                               v                                        |
|                            /cmd_vel                                    |
+------------------------------------------------------------------------+
                               v
          MATLAB Simulink drone dynamics -> Unreal scene update
```

Only the image source and the servo gains differ from the on-hardware
configuration. The SLAM, `yolo_producer`, `yolo_bridge` and `ovcam_bridge`
binaries are byte-for-byte identical to the live-camera build; behaviour changes
come from launch-file parameter overrides, not source edits.

Mission geometry: drone initial conditions x=0, y=15, z=10 m, yaw=0, pitch=0.
Stop-sign world position, from `/sim/target_pose`, is (35.5, 23.7, 3.2, pi).

Full rig setup, network configuration and MATLAB startup sequence are in
[`SIM_HIL.md`](SIM_HIL.md).

---

## 3. Candidates

| Candidate | Package | Mode | Notes |
|---|---|---|---|
| **ORB-SLAM2** | `src/orbslam2/` | Monocular | Feature-based, standard reference. |
| **ORB-SLAM3** | `src/orbslam3/` | Monocular | Vendored from `Mechazo11/ros2_orb_slam3` v2.0.0 (Jazzy), ORB-SLAM3 V1.0. Upstream publishes no pose at all; a `PublishPose` patch was added for this work. |
| **OV2SLAM-accurate** | `src/ov2slam_ros/` | Monocular | Upstream "accurate" profile. |
| **OV2SLAM-fast** | `src/ov2slam_ros/` | Monocular | Upstream "fast" profile. |
| RTAB-Map | `src/rtabmap_docker/` | **Stereo** | Not a peer row. See [section 7](#7-rtab-map-stereo-reference-point-excluded-from-the-tables). |

All four table candidates run **monocular**. This is what makes them comparable
to each other, and what disqualifies the RTAB-Map runs from sitting in the same
table.

---

## 4. Method

**Scoring window.** Online runs are scored over a unified window from mission
start to arrival, with the initialisation leg excluded for every arm, so no arm
is credited or penalised for the time it spent bootstrapping its map.

**Accuracy.** Median absolute pose error (APE) in metres, after Umeyama
alignment with scale estimated rather than assumed (monocular SLAM has no metric
scale of its own).

**Reliability.** Fraction of runs that completed and produced a usable
trajectory.

**Significance.** Permutation test with Holm correction, applied to median APE.

**Aggregation.** Every paper number is produced by
`scripts/hil_matrix/aggregate_matrix.py` and rendered by
`PAPER_IMAGES_HIL/generate.py`. Numbers are never hand-edited; the tables in
`PAPER_IMAGES_HIL/tables/` regenerate from the bags and CSVs listed in
[section 9](#9-data-provenance).

---

## 5. Measurement caveats (read before the tables)

These are properties of the measurement setup, not of the algorithms. Quoting any
number below without them is misleading.

1. **SLAM is pinned to cores 2 and 3 of 4.** The CPU ceiling in these tables is
   therefore **200%, not 400%**. A backend reporting 164% CPU is using roughly
   82% of the compute it was actually allowed, not 41% of the machine.

2. **Latency figures are "under a 2-core allocation", not intrinsic.** They
   describe what the backend achieved with two cores while detection and control
   competed for the other two. They are not the algorithm's intrinsic frame time
   and will not transfer to a differently provisioned machine.

3. **Minor pinning asymmetry between arms.** OV2SLAM configurations place the
   detector and the controller both on core 0, while the ORB configurations split
   them across cores 0 and 1. The SLAM allocation (cores 2,3) is identical across
   arms, but the non-SLAM contention is not perfectly matched.

4. **OV2SLAM-fast is not directly comparable online.** It initialises at 10.75 m
   against a 3.0 m initialisation leg, so it enters the mission with no map and is
   scored over roughly 26 m where the other arms get roughly 33 m. Its online row
   is reported but should be read as provisional pending its own initialisation
   cycle or a documented exclusion.

5. **ORB-SLAM3 latency comes from added instrumentation.** The
   `frontend/full_tracking` timing category is read for all three backends, but
   for ORB-SLAM3 that instrumentation was added for this work rather than being
   upstream's own.

---

## 6. Results

### 6.1 Offline EuRoC playback

10 sequences x 10 runs = 100 runs per backend.

| Backend | Runs | Reliability | Median APE | Latency | CPU |
|---|---:|---:|---:|---:|---:|
| ORB-SLAM2 | 100 | 100% | 0.0545 m | 39.5 ms | 164.1% |
| ORB-SLAM3 | 100 | **74%** | 0.0546 m | 32.5 ms | 147.9% |
| OV2SLAM-accurate | 100 | 100% | 0.1271 m | n/a | 70.0% |
| OV2SLAM-fast | 100 | 100% | 0.2974 m | n/a | 38.7% |

Source: `PAPER_IMAGES_HIL/tables/t1_offline.tex`, matching the table in
`HANDOFF_SLAM_ABLATION.md`. Latency is not available for the OV2SLAM offline runs.

The headline offline finding is that ORB-SLAM2 and ORB-SLAM3 are accuracy-tied
(0.0545 vs 0.0546 m), but ORB-SLAM3 completed only 74% of runs. OV2SLAM trades
roughly 2.3x (accurate) to 5.5x (fast) the error for roughly 2.3x to 4.2x less
CPU.

### 6.2 Online HIL

| Backend | Runs | Reliability | Median APE | Latency | CPU |
|---|---:|---:|---:|---:|---:|
| ORB-SLAM2 | 10 | 100% | 0.3569 m | 28.4 ms | 80.5% |
| ORB-SLAM3 | 20 | 100% | 0.3365 m | 31.1 ms | 81.7% |
| OV2SLAM-accurate | 10 | 100% | 0.4166 m | 8.5 ms | 37.7% |
| OV2SLAM-fast | 10 | 100% | 0.4560 m | 4.6 ms | 33.9% |

Source: `PAPER_IMAGES_HIL/tables/t2_online.tex`.

> **On the ORB-SLAM3 row (n=20).** This row pools two complete 10-trial sets,
> which `PAPER_IMAGES_HIL/generate.py` treats as statistically indistinguishable
> (permutation p=0.077, identical standard deviation) and therefore pools for a
> more stable median. The two sets individually gave medians of 0.3575 m and
> 0.3162 m. `HANDOFF_SLAM_ABLATION.md` reports the second set alone (median
> 0.3162 m, CPU 74.4%); the pooled figures above are the ones the generator
> produces. Only one of the two sets carries front-end timing, so the latency
> figure comes from those 10 runs while accuracy uses all 20. See
> [section 8](#8-open-issues) for the unresolved CPU discrepancy between the sets.

Initialisation distance, from `HANDOFF_SLAM_ABLATION.md`: ORB-SLAM2 3.40 m,
ORB-SLAM3 3.30 m, OV2SLAM-accurate 3.40 m, OV2SLAM-fast 10.75 m. All four arms
completed 10/10 in their reported sets.

### 6.3 Significance, online median APE

Permutation test with Holm correction.

| A | B | Delta median APE | p (Holm) | Verdict |
|---|---|---:|---:|---|
| ORB-SLAM2 | ORB-SLAM3 | +0.0204 m | 0.3433 | n.s. |
| ORB-SLAM2 | OV2SLAM-accurate | -0.0597 m | 0.0394 | differ |
| ORB-SLAM2 | OV2SLAM-fast | -0.0992 m | 0.0095 | differ |
| ORB-SLAM3 | OV2SLAM-accurate | -0.0801 m | 0.0472 | differ |
| ORB-SLAM3 | OV2SLAM-fast | -0.1195 m | 0.0095 | differ |
| OV2SLAM-accurate | OV2SLAM-fast | -0.0394 m | 0.1720 | n.s. |

Source: `PAPER_IMAGES_HIL/tables/t3_significance.tex`.

The two ORB variants are statistically indistinguishable from each other, and the
two OV2SLAM profiles are indistinguishable from each other, but the ORB family
and the OV2SLAM family separate cleanly.

### 6.4 Reading the four axes together

Every arm held the loop closed at 100% reliability online. The choice is
therefore a Pareto choice on the remaining axes rather than a correctness one:
the ORB family buys roughly 0.06 to 0.12 m of online accuracy for roughly 2.2x
the CPU and roughly 3.3x to 6.2x the latency of the OV2SLAM family. Figures for
each axis are in `PAPER_IMAGES_HIL/figures/` (`f1` accuracy, `f2` reliability,
`f3` cost, `f4` Pareto, `f5` CPU, `f6` latency, `f7` throughput).

---

## 7. RTAB-Map: stereo reference point, excluded from the tables

**Decision, 2026-08-24.** With the supervisor's approval, RTAB-Map Mono is
formally dropped from the benchmark. No monocular RTAB-Map row will be produced.
The section below is retained as the recorded justification.

RTAB-Map was evaluated on EuRoC and is kept as a reference point, but it is **not
a peer row** in the monocular tables above. Three independent reasons, all
verified against the code and data in this repository:

**1. The runs were stereo, not monocular.** The launch file used for every
RTAB-Map EuRoC run, `src/rtabmap_docker/euroc_offline_f2m.launch.py`, sets
`"subscribe_stereo": True` and launches the `stereo_odometry` executable from the
`rtabmap_odom` package, confirmed in `logs/launch.log` for all three batches. A
stereo system has metric scale for free and a fundamentally easier estimation
problem, so it is not a like-for-like row in a monocular table.

Note the comparison this does **not** invalidate: the ORB-SLAM2 batch alongside
it in `trajectory_selection_summary.csv` (`20260530_232822`) is itself stereo
(`"mode": "stereo"`, `Input sensor was set to: Stereo`), so that particular CSV is
stereo against stereo. The mismatch is with the **monocular results tables** in
section 6, whose backends are all monocular.

**2. The summary record is best-of-3, not all-runs.** RTAB-Map's only surviving
summary is a best-run-per-sequence selection in
`/home/amaraly/ORB_SLAM2/images/trajectory_selection_summary.csv`, where
`selection_reason` is `coverage>=90_lowest_rmse` (or `highest_coverage_fallback`)
and `selected_run` is in {1,2,3}. That is a best-of-3 selection, against every
other backend's all-100-runs statistics. The numbers are not on the same footing.

**3. RTAB-Map ships no monocular odometry node,** so a like-for-like monocular
rerun is not simply a configuration change. The `rtabmap_odom` package provides
`rgbd_odometry`, `stereo_odometry` and `icp_odometry`, and nothing else. An
`OdometryMono` class does exist compiled into `librtabmap_core.so`, with
registered `OdomMono/*` parameters, but `Odometry::create` never instantiates it
and it is absent from the documented `Odom/Strategy` values (the shipped help
string lists only `0=F2M` through `14=LIO-SAM`).

It is reachable through one undocumented path: `rtabmap-odometryViewer` tests
`Odom/Strategy` for `-1` and constructs `OdometryMono` directly, bypassing the
factory.

**Tested end to end on 2026-08-23.** A standalone probe
(`benchmarks/rtabmap_mono_probe/`) constructing the class directly was run against
full EuRoC sequences. Method and raw output: `CLAUDE_SCRATCHPAD.md` section 3.

*It crashes as shipped.* The probe aborts on the second frame under OpenCV 4.6
inside `OdometryMono::computeTransform` -> `util3d::solvePnPRansac` ->
`cv::solvePnP`. The cause is an internal inconsistency: `OdometryMono` allocates
its PnP guess vectors `1x3` while the live F2M/F2F path in
`util3d_motion_estimation` allocates them `3x1`, and `useExtrinsicGuess=true`
makes OpenCV adopt the caller's shape verbatim. Both shapes were confirmed by
inspecting the compiled objects of the 0.23.6 install. The crash is
data-independent, reproduced on two sequences.

*With that shape corrected at runtime it does run.* On MH_01_easy it produced
3451 poses over 3682 frames (93.7%). So the class is not inert; it initialises
and tracks given a sequence with adequate parallax.

*But it does not hold up.* Scored with the same `evo_ape -a -s` convention used
for the tables in section 6:

| window from sequence start | poses | median APE [m] | RMSE [m] |
|---|---|---|---|
| first 30 s | 592 | 0.0908 | 0.1579 |
| first 60 s | 1192 | 0.7701 | 1.4647 |
| first 120 s | 2390 | 3.9534 | 4.0473 |
| full 173 s | 3456 | 3.4460 | 3.8444 |

It tracks competitively for roughly 30 seconds, then drifts without bound. Late
in the run the implied velocity reaches 138.8 m/s against a ground-truth maximum
of 2.2 m/s. This is unscaled monocular odometry with no loop closure and no bundle
adjustment: a drifting VO front end, not a SLAM backend. At full sequence length
it is an order of magnitude worse than the weakest candidate in section 6.

*Caveat on the drift numbers.* The table above was produced with the PnP shape
corrected by an `LD_PRELOAD` interposer (`benchmarks/rtabmap_mono_probe/pnp_shim.cpp`)
rather than by a rebuilt library. That shim rebinds the caller's `cv::Mat` headers
through a reference, and a properly patched build was started but never completed
before the investigation was closed. Treat these figures as indicative of the
class's behaviour, not as an audited measurement. Nothing rests on them: reasons
1 and 2 above are read directly off the launch file and the summary CSV, and
reason 3 off the shipped package contents.

**Conclusion.** RTAB-Map's exclusion from the monocular tables is not an
assumption, it is a measured result. There is no shipped monocular odometry node,
and the one orphaned class behind an undocumented flag does not produce a usable
trajectory over a full sequence.

### RTAB-Map stereo results, for reference only

Best-run-per-sequence selection. The ORB-SLAM2 column is drawn from the **same
CSV under the same selection policy**, so the two columns are comparable to each
other, but neither is comparable to the all-runs statistics in
[section 6.1](#61-offline-euroc-playback).

| Sequence | RTAB-Map (stereo) RMSE | GT coverage | ORB-SLAM2 (mono) RMSE | GT coverage |
|---|---:|---:|---:|---:|
| MH_01_easy | 0.053286 m | 96.54% | 0.045605 m | 91.23% |
| MH_02_easy | 0.061772 m | 94.61% | 0.045955 m | 91.47% |
| MH_03_medium | 0.137981 m | 88.36% | 0.045120 m | 94.79% |
| MH_04_difficult | 0.161572 m | 98.63% | 0.090153 m | 96.32% |
| MH_05_difficult | 0.166169 m | 97.90% | 0.098235 m | 69.84% |
| V1_01_easy | 0.059586 m | 97.48% | 0.053935 m | 95.24% |
| V1_02_medium | 0.081081 m | 77.10% | 0.085854 m | 86.48% |
| V1_03_difficult | 0.101461 m | 55.82% | 0.123351 m | 78.92% |
| V2_01_easy | 0.108364 m | 95.63% | 0.058559 m | 96.07% |
| V2_02_medium | 0.293895 m | 60.62% | 0.081644 m | 95.05% |

Source: `/home/amaraly/ORB_SLAM2/images/trajectory_selection_summary.csv`
(batches `20260525_*` for RTAB-Map, `20260530_232822` for ORB-SLAM2).

Note the coverage column: several RTAB-Map selections fall well below the 90%
ground-truth coverage threshold (V1_03_difficult at 55.82%, V2_02_medium at
60.62%, V1_02_medium at 77.10%) and were admitted only by the
`highest_coverage_fallback` rule. Low-coverage RMSE values are computed over a
shorter matched segment and are not comparable to high-coverage ones.

---

## 8. Open issues

**ORB-SLAM3 CPU is unexplained across run sets.** Two n=10 online sets gave
medians 0.3575 m and 0.3162 m (permutation p=0.077, identical sd 0.0546 vs
0.0544), which is not a real accuracy difference; the median is unstable at n=10
because six of ten values clustered inside 8 mm. But CPU differed 89.0% vs 74.4%,
roughly 2.5 sd, and **that remains unexplained**. A third set was planned and not
collected.

**ORB-SLAM3 offline reliability is 74%.** The 26% of runs that failed are not yet
characterised by failure mode.

**OV2SLAM-fast initialisation.** See caveat 4 in
[section 5](#5-measurement-caveats-read-before-the-tables). Needs its own
initialisation cycle or a documented exclusion.

**ORB-SLAM3 upstream provenance.** The vendored tree's upstream commit was not
recorded. Re-clone at a tag and diff before publishing.

**Judgement calls worth challenging.** The scoring window starts at mission start
rather than first pose; `END_PAD = 3`; and `frontend/full_tracking` is read for
all three backends although ORB-SLAM3's is added instrumentation rather than
upstream's.

**Code to audit, highest leverage first**, per `HANDOFF_SLAM_ABLATION.md`:

1. `scripts/hil_matrix/aggregate_matrix.py` - every number originates here. Check
   the `evaluate()` window, `umeyama()` scale handling, and `load_timing()`
   category selection (this previously understated ORB-SLAM2 by 3x).
2. `PAPER_IMAGES_HIL/generate.py` - the globs decide which runs back the results.
   A wrong glob silently changed the numbers twice.
3. `matlab/slam_traj_probe.m` - `PROBE_PHASE`, initialisation leg, cruise.
4. `src/orbslam3/src/common.cpp` - grep `HIL` for the two local patches.
5. `scripts/hil_matrix/run_matrix.ps1` - initialisation gate, preflight.

---

## 9. Data provenance

Offline EuRoC summaries, as globbed by `PAPER_IMAGES_HIL/generate.py`:

| Backend | Summary CSV |
|---|---|
| ORB-SLAM2 | `ORB_SLAM2/results/20260609_092931/experiment_summary.csv` |
| ORB-SLAM3 | `ORBSLAM3_ROS2/results/orbslam_benchmark/20260614_014256/experiment_summary.csv` |
| OV2SLAM-accurate | `results/ov2slam_benchmark_v2/20260612_234141/accurate_mono/experiment_summary.csv` |
| OV2SLAM-fast | `results/ov2slam_benchmark_v2/20260613_102424/fast_mono/experiment_summary.csv` |

Online HIL bag globs:

| Backend | Bag glob |
|---|---|
| ORB-SLAM2 | `bags/run_orbslam2_probe_verso10_*` |
| ORB-SLAM3 | `bags/run_orbslam3_probe_orb3x10_*` and `bags/run_orbslam3_probe_orb3lat_*` (pooled) |
| OV2SLAM-accurate | `bags/run_ov2slam_oracle_accurate_verso10_*` |
| OV2SLAM-fast | `bags/run_ov2slam_oracle_fast_verso10_*` |

The ORB-SLAM3 glob is deliberately not a bare `orb3*`: that also matches aborted
initialisation-only bags from earlier attempts, which inflate n and corrupt the
reliability figure.

RTAB-Map: `/home/amaraly/ORB_SLAM2/images/trajectory_selection_summary.csv` and
`.json`. These carry absolute provenance paths recording where each trajectory
came from; they are a historical record and are left unmodified.

Resolved provenance for the current figures is written to
`PAPER_IMAGES_HIL/data/provenance.json`.

---

## 10. The controller benchmark

A second study runs on the same rig. Where the SLAM benchmark swaps the pose
estimator and holds the controller fixed, this one swaps the **servoing law** and
holds everything else fixed, so any measured difference is the control law alone.

Design and runbook: `benchmarks/CONTROLLER_BENCH_PLAN.md`.

### 10.1 Test subjects

| | Package / node | Servoing law |
|---|---|---|
| **TS1** | `visp_servo` | **IBVS.** ViSP `vpServo`, 4 corner features, interaction matrix and pseudo-inverse: `v = -lambda * L^+ * (s - s*)` |
| **TS2** | `hil_servo` | **Proportional.** Decoupled P-law: `vx = k_fwd*(ratio* - ratio)`, `vy = -k_lat*ex`, `vz = -k_vz*ey` |
| **TS3** | `h_vs_servo` | **Homography-based.** Translational error from the homography decomposition, rotational error from `vex(H - H^T)` |
| **TS4** | `visp_pbvs_servo` | **PBVS.** ViSP `vpServo` on a reconstructed pose rather than image features |

All four are standalone ROS 2 nodes wrapping the **same** `servo_core::ServoFsmNode`:
identical FSM (SEARCHING to APPROACHING to REACQUIRE to REACHED), identical safety
filters (sim-health, clamp, floor, velocity ramp), identical `/bench/state`
instrumentation. The only difference is the injected `IServoController`.

### 10.2 Fairness invariants

Held identical across arms, in `config/hil/bench_fsm.yaml`: camera intrinsics,
image size, velocity clamps, slew-rate limiter, FSM thresholds, target, and start
pose. Per-arm gains live in `config/hil/bench_{ibvs,proportional,h_vs,pbvs}.yaml`.

Depth is **bbox-derived** for every arm, `Z = fy*H/bh`, with no SLAM in the loop.
This is deliberate: it keeps depth from becoming a confound between the control
laws. IBVS can optionally take SLAM depth via `use_slam_depth: true`, but that
flag is off for every run reported here.

### 10.3 The oracle detector

`oracle_detector` projects the known target world point through the sim camera
using ground-truth `/sim/drone_pose` and publishes a perfect box on
`/yolo/detections`, the same topic and type as real YOLO. The box grows as the
drone closes (`size_height = fy*H/Zc`), so it behaves like a real detector, but it
is deterministic and identical every run. This removes the detector as a variable.

### 10.4 Method

- **Closed-loop, not open-loop.** Settling, overshoot and stability cannot be
  measured from a single-step response, so every run flies live against the
  MATLAB plant.
- **Measured from search-exit.** The stopwatch starts at the first
  SEARCHING to APPROACHING edge marked on `/bench/state`. The search phase is
  identical across arms and out of scope.
- **N escalation.** Start at N=1 to validate the pipeline, then N=3, stopping
  early only if arms separate cleanly by `|mu_A - mu_B| > 2*max(sigma_A, sigma_B)`.
  The reported arms all went to N=10.
- **Metrics**, computed offline from the bag by `benchmarks/plot_controller_hil.py`:
  settling time, steady-state error, overshoot, IAE / ITAE / ISE, path efficiency,
  RMS command speed, total variation (smoothness), controller CPU and RSS.
  Error is `e(t) = ||drone - target||_3D - standoff`, standoff about 3.15 m.

---

## 11. Controller results

Medians over N=10 runs per arm, aggregated from `bags/ctrl_*/metrics.csv`.

| | settling [s] | ss error [m] | overshoot [m] | IAE | ITAE | path eff. | RMS cmd | total var. | CPU [%] | RSS [MB] |
|---|---|---|---|---|---|---|---|---|---|---|
| TS1 IBVS | **19.30** | 0.180 | 0.000 | **321.7** | **2047** | 0.890 | 2.438 | 37.81 | 0.81 | 28.3 |
| TS2 Proportional | 33.92 | 0.216 | 0.000 | 517.4 | 5407 | **0.995** | 0.999 | 4.88 | **0.76** | **18.5** |
| TS3 Homography | 35.47 | **0.181** | 0.000 | 516.8 | 5869 | 0.987 | **0.836** | **1.73** | 0.84 | 23.7 |

Cross-run spread (median +/- stdev, N=10):

| | settling [s] | ss error [m] | total variation | CPU [%] |
|---|---|---|---|---|
| TS1 IBVS | 19.30 +/- 0.97 | 0.180 +/- 0.013 | 37.81 +/- 5.03 | 0.811 +/- 0.012 |
| TS2 Proportional | 33.92 +/- 1.31 | 0.216 +/- 0.014 | 4.88 +/- 0.27 | 0.761 +/- 0.019 |
| TS3 Homography | 35.47 +/- 1.59 | 0.181 +/- 0.023 | 1.73 +/- 0.07 | 0.837 +/- 0.026 |

### 11.1 Reading the table

**The trade is speed against smoothness, and it is stark.** IBVS settles in
roughly 19 s where both other laws need 34 to 35 s, and it carries about 40% of
their IAE. It pays for that with a total variation of 37.8 against 4.9 for
proportional and 1.7 for homography, so its command signal is roughly 8x rougher
than TS2 and 22x rougher than TS3. Its path efficiency is also the worst of the
three at 0.890 against 0.995, meaning it reaches the target sooner while taking a
less direct route there.

**Nobody overshoots.** Overshoot is 0.000 m in every run of every arm. The
velocity clamp and slew-rate limiter in `servo_core` are shared, and they appear
to be what dominates the approach envelope.

**Steady-state error is a two-way split, not a three-way one.** IBVS and
homography land at 0.180 and 0.181 m; proportional sits at 0.216 m. Against
stdevs of 0.013 to 0.023 that gap is real, though small in absolute terms.

**CPU did not separate, and the plan predicted it would.**
`CONTROLLER_BENCH_PLAN.md` expected IBVS's 8x6 pseudo-inverse per frame to show
up as a real cost axis against the P-law's handful of multiplies. It does not.
The three arms sit at 0.81%, 0.76% and 0.84%, with stdevs of 0.012 to 0.026.
Applying the plan's own separation criterion, `|0.811 - 0.761| = 0.050` against
`2*max(sigma) = 0.052`, the difference fails to clear the bar. At 20 Hz on a
Cortex-A76 the pseudo-inverse is simply not the bottleneck. Memory does separate
(28.3 vs 18.5 MB), but that is the ViSP dependency, not the arithmetic.

### 11.2 Caveats

1. **TS3 does not produce forward motion from the homography.** For the
   axis-aligned boxes the oracle emits, `getPerspectiveTransform` returns a pure
   affine (`H[2,2] = 1`, so `e_v[2] = 0`). `config/hil/bench_h_vs.yaml` therefore
   falls back to the same proportional `k_fwd` as TS2 for the forward axis, and
   only `lambda_w[1]` (camera yaw to body wz) is active in rotation. TS3 is
   consequently a homography law on the lateral and vertical axes and a
   proportional law on the forward axis, not a pure homography controller. Its
   near-identical settling time to TS2 should be read in that light.
2. **TS4 PBVS is not in the table.** `HANDOFF.md:84` lists it as paused, and the
   data agrees: 17 bags exist under `bags/ctrl_pbvs_*` and none has a
   `metrics.csv`. The runs were recorded and never analysed. The bags are on disk
   and the analysis is one `plot_controller_hil.py` invocation per run; until that
   happens PBVS has no numbers.
3. **The pbvs run names are not a clean N-series.** They include repeated `N1`
   stamps plus `N33` and `N55`, consistent with debugging rather than a
   controlled sweep. Establish which subset is valid before analysing.
4. **`src/h_vs/` is an empty leftover directory.** The real package is
   `src/h_vs_servo/`.

---

## 12. Hardware and prerequisites

| Item | Details |
|---|---|
| **Board** | Raspberry Pi 5 (Cortex-A76 x 4, 16 GB RAM) |
| **Camera (live)** | OV5647 (CSI), 640 x 480 at ~30 fps |
| **Camera (HIL)** | MATLAB/Unreal stream, bgr8 640 x 480 at ~20 Hz |
| **Accelerator** | Hailo-10H AI HAT+ 2, passed through as `/dev/hailo0` |
| **Host OS** | Raspberry Pi OS Trixie (64-bit), **required** |
| **Docker** | Docker Engine >= 24 |
| **ROS 2** | Jazzy, on both the Pi container and the MATLAB side |
| **DDS** | CycloneDDS (`rmw_cyclonedds_cpp`) on both sides, `ROS_DOMAIN_ID=0` |
| **MATLAB** | R2024a or later with ROS Toolbox (CycloneDDS RMW support added in R2024a) |
| **Network** | Same L2 subnet, no NAT between Pi `eth0` and the Windows NIC |

> Raspberry Pi OS Trixie is required. The Hailo-10H AI HAT+ 2 drivers and the
> `libcamera` integration depend on the Trixie release. Bookworm and earlier are
> not supported.

Host packages, outside Docker:

```bash
sudo apt install -y libcamera-dev cmake g++ pkg-config
sudo apt install -y util-linux gettext-base    # taskset, envsubst
```

The benchmark host used for these results is a Pi reachable at
`amaraly@192.168.137.10`, static on `eth0`. Note a timezone trap when correlating
logs: the Windows host is UTC+3 and the Pi is UTC+4 (Asia/Dubai), so bag names
carry Pi time while PowerShell logs carry Windows time, one hour apart.

---

## 13. Running the stack

### Build, first time or after source changes

```bash
./run_stack_hil.sh build              # all stack packages
./run_stack_hil.sh build <pkg>        # a single package
```

### Run a HIL session

1. **On Windows:** follow the MATLAB startup sequence in
   [`SIM_HIL.md`](SIM_HIL.md) until `latest_frame` is publishing and the camera
   publisher timer is running.
2. **On the Pi:** set `network.matlab_host_ip` in your chosen
   `config/hil/stack/<name>.yaml`, then start the stack:

   ```bash
   ./run_stack_hil.sh --config full_ov2slam
   ```
3. **Verify** topics are flowing: `/yolo/detections`, `/cmd_vel`, and
   `/slam/pose` for SLAM configurations.
4. **Benchmark mode** (`ov2slam_oracle`, or any config run with
   `--mode benchmark`): when the script prints `recording ...`, stop and restart
   the Simulink model once for a clean `t=0`. The run records to
   `bags/run_<config>_<stamp>/`.
5. **Stop:**

   ```bash
   ./run_stack_hil.sh stop
   ```

   This tears down the main container, the SLAM sidecar, `yolo_producer` and the
   shared memory segments, and finalises any benchmark bag.

### Stack configurations

Configurations live in `config/hil/stack/`. The ones backing the results above
are the `*_probe` and `*_oracle_*` variants:

```
orbslam2_probe   orbslam3_probe   ov2slam_oracle_accurate   ov2slam_oracle_fast
```

Also present: `default`, `full`, `full_ov2slam`, `h_vs`, `ibvs`, `oracle`,
`pbvs`, `orbslam2_eval`, `orbslam2_scout`, `ov2slam_ibvs`,
`ov2slam_ibvs_slamdepth`, `ov2slam_oracle`, the `*_nopin` variants (identical but
without core pinning, for isolating the pinning effect), and
`ov2slam_fast_clahe_diag`.

### Core pinning

Pinning is applied with `taskset` and is load-bearing for the results. SLAM gets
cores 2 and 3; detection and control share cores 0 and 1. Verify at runtime:

```bash
watch -n1 'ps -eo pid,psr,pcpu,comm --sort=-pcpu | grep -E "PSR|ovcam|yolo|ov2slam|orb|visp|hil_servo" | head -12'
```

> Never start long-running containers with `docker run -it`. The interactive tty
> causes a spin-loop that burns roughly 170% CPU and will corrupt every
> measurement on this page.

### Regenerating the results

```bash
python3 PAPER_IMAGES_HIL/generate.py
```

This rewrites everything in `PAPER_IMAGES_HIL/tables/` and
`PAPER_IMAGES_HIL/figures/` from the bags and CSVs in
[section 9](#9-data-provenance). Do not hand-edit numbers in the generated files.

### Two known process leaks

Both are fixed in `scripts/hil_matrix/run_matrix.ps1`, but check by hand if trials
start failing: MathWorks helper processes (`Get-Process matlab` is an exact match
and misses `matlabwindowhelper`) and `AutoVrtlEnv` (Unreal, roughly 500 MB per
trial; 47 orphans totalling 17 GB were observed). Preflight now clears both and
refuses to start below 8 GB free.

```powershell
scripts/hil_matrix/check_clean.ps1 [-Fix]
```

---

## 14. Repository layout

```
ROS2-PROJECT-SPRING2026/
|
|-- README.md                       # <- this file
|-- SIM_HIL.md                      # HIL rig setup: network, MATLAB, run order
|-- HANDOFF_SLAM_ABLATION.md        # Primary results record and open issues
|-- HANDOFF.md  GOTCHAS.md          # Working notes
|-- Dockerfile                      # Ubuntu 24.04 + ROS 2 Jazzy
|-- run_stack_hil.sh                # Main entry point for HIL sessions
|
|-- src/
|   |-- orbslam2/                   # ORB-SLAM2 backend
|   |-- orbslam3/                   # ORB-SLAM3 backend (vendored, PublishPose patch)
|   |-- ov2slam_ros/                # OV2SLAM backend (accurate + fast profiles)
|   |-- rtabmap_docker/             # RTAB-Map stereo EuRoC harness (see section 7)
|   |   |-- euroc_offline_f2m.launch.py    # stereo_odometry + rtabmap
|   |   |-- run_euroc_f2m_eval.py          # Run driver
|   |   |-- host_benchmarker_rtabmap       # Host-side Docker wrapper
|   |   |-- eval.py  postprocess.py        # Evaluation and aggregation
|   |   |-- mapPath_to_tum.py  odom_to_tum.py
|   |   |-- experiment_config_stereo*.yaml
|   |   |-- images/                        # Per-sequence trajectory plots
|   |   \-- results/ run_logs/ tmp_lib/    # Bulk output (git-ignored)
|   |
|   |-- sim_camera_bridge/          # /sim/camera/image_raw -> /ovcam_frames SHM
|   |-- ovcam_producer/             # Live camera capture, runs on HOST
|   |-- ovcam_bridge/               # /ovcam_frames SHM -> /ovcam/image_raw
|   |-- yolo_producer/              # Hailo NPU inference, runs on HOST
|   |-- yolo_bridge/                # /yolo_shm -> /yolo/detections
|   |-- yolo_msgs/  yolo_ros/       # Message definitions and Python nodes
|   |-- hil_servo/                  # HIL visual servo controller
|   |-- h_vs/  h_vs_servo/          # Hybrid visual servoing
|   |-- visp_servo/  visp_pbvs_servo/   # ViSP IBVS and PBVS controllers
|   |-- servo_core/                 # Shared controller primitives
|   |-- init_gate/                  # Initialisation gate and cycle
|   \-- oracle_detector/            # Ground-truth detector, isolates SLAM from YOLO
|
|-- PAPER_IMAGES_HIL/               # Paper artefacts, all generated
|   |-- generate.py                 # ONLINE/OFFLINE globs = full provenance
|   |-- tables/                     # t1_offline, t2_online, t3_significance
|   |-- figures/                    # f1..f7
|   \-- data/provenance.json
|
|-- benchmarks/                     # Benchmark drivers and analysis
|   |-- eval_slam_hil.py  compare_slam_hil.py  aggregate_slam_results.py
|   |-- slam_eval/                  # Evaluation library
|   \-- ...                         # Controller, IPC, YOLO and pinning benches
|
|-- scripts/hil_matrix/             # Trial matrix orchestration
|   |-- aggregate_matrix.py         # Every paper number originates here
|   |-- run_matrix.ps1              # Windows-side trial driver
|   \-- check_clean.ps1             # Process-leak preflight
|
|-- config/hil/                     # Stack configs and CycloneDDS profiles
|   |-- stack/*.yaml
|   \-- cyclonedds_hil.xml
|
|-- matlab/                         # Simulink model helpers and probes
|-- camera_calib/                   # Calibration YAMLs, live and sim
|-- models/                         # yolo26n_10h.hef, compiled for Hailo-10H
|-- docs/reports/                   # Written comparison reports
|
|-- bags/                           # Recorded HIL runs      (git-ignored)
|-- results/                        # Benchmark output       (git-ignored)
\-- build/ install/ log/            # colcon output          (git-ignored)
```

### A note on the RTAB-Map dataset payload

`src/rtabmap_docker/` holds the harness only. Its dataset payload deliberately
lives **outside** the repository at `/home/amaraly/RTAB_Docker/`:

```
/home/amaraly/RTAB_Docker/
|-- datasets/euroc/hil_full_20260606_002324_0.mcap    (1.6 GB)
|-- MH_01_easy.db3                                    (2.5 GB)
\-- MH_01_easy.txt                                    (ground truth)
```

`run_euroc_f2m_eval.py` derives its default bag and ground-truth paths from the
script's own directory, so those defaults no longer resolve after the harness
move. Pass the bag and ground-truth paths explicitly, or point them at the
location above.

---

## 15. Contributors

| Name | Role / Contribution |
|---|---|
| Amar Aly | SLAM-HIL rig, benchmark design and execution, SLAM backend integration (OV2SLAM, ORB-SLAM2, ORB-SLAM3, RTAB-Map), ROS 2 environment, camera calibration |
| Ahmad Dhaoudi | YOLOv26n conversion to `.hef` for Hailo-10H, YOLO detection and setup, ViSP integration |

---

*Last updated: 2026-08-23*
