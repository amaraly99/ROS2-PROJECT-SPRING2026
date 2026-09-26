# Hardware-in-the-Loop Benchmarks: SLAM and Visual Servoing

**Author:** Amar Aly
**Branch:** `benchmarks/slam-hil`
**Last updated:** 2026-09-26

> Visual SLAM backends, monocular and stereo, and visual-servoing control laws
> evaluated on a Raspberry Pi 5 driven by a MATLAB/Simulink + Unreal Engine
> flight simulator.
> ROS 2 Jazzy - Ubuntu 24.04 (Docker) - CycloneDDS (wired) / FastRTPS (Wi-Fi)

---

## Table of Contents

1. [Executive summary of this branch](#1-executive-summary-of-this-branch)
2. [Results](#2-results)
3. [Observed behaviour](#3-observed-behaviour)
4. [The HIL rig](#4-the-hil-rig)
5. [Candidates](#5-candidates)
6. [Method](#6-method)
7. [Measurement caveats](#7-measurement-caveats)
8. [RTAB-Map: stereo peer, excluded from the monocular tables](#8-rtab-map-stereo-peer-excluded-from-the-monocular-tables)
9. [The controller benchmark](#9-the-controller-benchmark)
10. [Open issues](#10-open-issues)
11. [Data provenance](#11-data-provenance)
12. [Hardware and prerequisites](#12-hardware-and-prerequisites)
13. [Running the stack](#13-running-the-stack)
14. [Repository layout](#14-repository-layout)
15. [Contributors](#15-contributors)

---

## 1. Executive summary of this branch

`benchmarks/slam-hil` extends the August monocular SLAM and controller benchmarks
with **stereo** SLAM in the HIL loop, plus a monocular re-run on the same flight.

**What the branch adds**

- **Stereo for all five backends in HIL.** OV2SLAM (accurate and fast), ORB-SLAM2,
  ORB-SLAM3 and RTAB-Map all take the Simulink stereo pair. ORB-SLAM3's stereo path
  was patched to publish its pose (`src/orbslam3/src/common.cpp`, `// HIL`). The
  ORB stereo wrappers published nothing before FIX-028. RTAB-Map runs
  `stereo_odometry` + `rtabmap` behind a small bridge
  (`src/rtabmap_docker/hil_stereo.launch.py`, `hil_stereo_bridge.py`).
- **A stereo baseline sweep.** 0.11, 0.36, 0.42, 0.54 and 0.61 m, 10 trials per
  backend per baseline. At 0.11 m there are three sets: two over Wi-Fi and one over
  Ethernet.
- **A monocular re-run** of the four monocular candidates on the same flight, so
  mono and stereo can be compared on identical trajectories.
- **Calibration and plumbing fixes**, documented in `docs/fixlog/018` to `028`.
  The two that change numbers:
  - FIX-020: the servo focal length is 1200 px, not 554.
  - FIX-027: the Simulink *stereo* blocks were never moved off 554 px. The stereo
    calibration therefore uses fx = 554; using 1200 had produced a scale of about
    0.46 and a roughly 2x depth bias.

**Headline**

- With a baseline of 0.36 m or more, **every stereo backend recovers metric scale**:
  the Sim(3) scale factor is 0.99 to 1.02, against 2.6 to 47 for monocular.
- **OV2SLAM-accurate has the lowest error at every baseline** (0.09 to 0.10 m APE
  RMSE). It costs 8 ms per frame, against 30 to 82 ms for ORB-SLAM3, ORB-SLAM2 and
  RTAB-Map.
- Between 0.36 and 0.61 m the baseline barely matters for OV2SLAM. RTAB-Map
  improves steadily (0.220 to 0.174 m). ORB-SLAM2 and ORB-SLAM3 move up and down
  within their spread.
- **0.11 m is where stereo breaks down.** Every backend degrades and scale inflates
  to 1.1 to 1.3. The three 0.11 m sets also disagree with each other: ORB-SLAM3
  diverges to metres over Wi-Fi but not over Ethernet. See
  [section 3](#3-observed-behaviour).
- In the monocular re-run, OV2SLAM-fast never initialises (0/10, and 0/20 with
  CLAHE) and ORB-SLAM2 initialises only 5 times in 10.

The August results (monocular EuRoC, monocular online HIL, controller laws) are
unchanged and are kept below as **Phase 1**. The September stereo and monocular
runs are **Phase 2**. The two phases used different scenes, transports, pinning
and metrics, so **their numbers are not comparable with each other** (see
[section 7](#7-measurement-caveats)).

---

## 2. Results

### 2.1 Phase 2: stereo baseline sweep (September 2026)

APE RMSE in metres, mean +/- sample SD over 10 trials. All trials are valid, the
transport is Wi-Fi, and the scene is `parking_lot_far` at 2.0 m/s over a scored
path of about 33.7 m.

| Backend | 0.36 m | 0.42 m | 0.54 m | 0.61 m |
|---|---:|---:|---:|---:|
| **OV2SLAM-accurate** | **0.100 +/- 0.035** | **0.096 +/- 0.016** | **0.096 +/- 0.008** | **0.092 +/- 0.015** |
| OV2SLAM-fast | 0.155 +/- 0.038 | 0.152 +/- 0.026 | 0.142 +/- 0.032 | 0.140 +/- 0.033 |
| ORB-SLAM2 | 0.217 +/- 0.022 | 0.190 +/- 0.032 | 0.255 +/- 0.054 | 0.206 +/- 0.039 |
| ORB-SLAM3 | 0.207 +/- 0.043 | 0.233 +/- 0.030 | 0.243 +/- 0.033 | 0.197 +/- 0.030 |
| RTAB-Map | 0.220 +/- 0.041 | 0.218 +/- 0.041 | 0.193 +/- 0.035 | 0.174 +/- 0.029 |

At 0.61 m the medians are 0.089, 0.133, 0.211, 0.183 and 0.174 m, and the mean
Sim(3) scale is 1.003 to 1.016 for every backend.

**Cost.** These are means over the 40 trials per backend at 0.36 to 0.61 m. SLAM
is **unpinned**, so CPU is a share of 400%.

| Backend | Frame time [ms] | p95 [ms] | Capacity [Hz] | Input delivered [Hz] | CPU mean [%] | CPU max [%] |
|---|---:|---:|---:|---:|---:|---:|
| OV2SLAM-fast | 3.3 | 6.7 | 302 | 10.9 | 38 | 58 |
| OV2SLAM-accurate | 8.1 | 13.6 | 123 | 10.9 | 56 | 94 |
| ORB-SLAM3 | 30.2 | 39.0 | 33 | 11.0 | 67 | 131 |
| ORB-SLAM2 | 42.1 | 53.0 | 24 | 11.0 | 85 | 144 |
| RTAB-Map | 81.9 | 100.6 | 12 | 7.8 | 116 | 272 |

**The 0.11 m sets.** Each cell gives mean / median APE RMSE in metres, with valid
trials in brackets.

| Backend | Wi-Fi (09-20) | Wi-Fi re-run (09-20) | Ethernet (09-20/21) |
|---|---:|---:|---:|
| OV2SLAM-accurate | 1.612 / 0.868 (10/10) | 0.669 / 0.668 (10/10) | 0.227 / 0.208 (10/10) |
| OV2SLAM-fast | 1.585 / 1.017 (10/10) | 0.847 / 0.820 (10/10) | 0.309 / 0.277 (10/10) |
| ORB-SLAM2 | 0.695 / 0.650 (10/10) | 0.915 / 0.813 (10/10) | 0.603 / 0.645 (10/10) |
| ORB-SLAM3 | **6.036 / 7.130** (10/10) | **8.694 / 10.234** (10/10) | 0.621 / 0.618 (10/10) |
| RTAB-Map | 0.276 / 0.286 (10/10) | 0.269 / 0.291 (9/10) | 0.534 / 0.532 (10/10) |

Delivered input rate at 0.11 m was 10 Hz over Wi-Fi (5.5 Hz for RTAB-Map) and
**6.5 Hz over Ethernet** for every backend. Scale was 1.12 to 1.33 for ORB-SLAM
and RTAB-Map, and 0.96 to 1.14 for OV2SLAM.

### 2.2 Phase 2: monocular re-run (September 2026)

Same scene, speed and scoring as 2.1, over Wi-Fi.

| Backend | Valid | APE RMSE mean +/- SD [m] | Median [m] | Sim(3) scale | Frame time [ms] | CPU mean [%] | Input [Hz] |
|---|---:|---:|---:|---:|---:|---:|---:|
| **OV2SLAM-accurate** | 10/10 | **0.106 +/- 0.027** | **0.095** | 2.6 | 6.4 | 48 | 16.0 |
| ORB-SLAM2 | **5/10** | 0.194 +/- 0.122 | 0.152 | 39.9 | 25.4 | 91 | 16.1 |
| ORB-SLAM3 | 10/10 | 0.375 +/- 0.474 | 0.212 | 46.6 | 28.5 | 86 | 15.9 |
| OV2SLAM-fast | **0/10** | n/a | n/a | n/a | n/a | n/a | n/a |
| OV2SLAM-fast + CLAHE | **0/20** (two sets) | n/a | n/a | n/a | n/a | n/a | n/a |

Every failed trial aborted with *"did not initialise during the init cycle
(tracking_state=1), mission not flown"*.

**Mono against stereo on the same flight** (APE RMSE mean, m):

| Backend | Mono | Stereo 0.36 | Stereo 0.61 |
|---|---:|---:|---:|
| OV2SLAM-accurate | 0.106 | 0.100 | 0.092 |
| ORB-SLAM2 | 0.194 (5/10) | 0.217 | 0.206 |
| ORB-SLAM3 | 0.375 | 0.207 | 0.197 |

After Sim(3) alignment, monocular *shape* error is close to stereo error. What
stereo adds is metric scale (scale about 1.0, against 2.6 to 47), reliable
initialisation, and a much tighter tail for ORB-SLAM3.

### 2.3 Phase 1: monocular benchmark (August 2026)

These results are unchanged from the 2026-08-23 README. The metric is **median APE**
(not RMSE), SLAM is **pinned to cores 2 and 3** (a CPU ceiling of 200%), and the
transport is **CycloneDDS over Ethernet**.

**Offline EuRoC playback**, 10 sequences x 10 runs = 100 runs per backend:

| Backend | Runs | Reliability | Median APE | Latency | CPU |
|---|---:|---:|---:|---:|---:|
| ORB-SLAM2 | 100 | 100% | 0.0545 m | 39.5 ms | 164.1% |
| ORB-SLAM3 | 100 | **74%** | 0.0546 m | 32.5 ms | 147.9% |
| OV2SLAM-accurate | 100 | 100% | 0.1271 m | n/a | 70.0% |
| OV2SLAM-fast | 100 | 100% | 0.2974 m | n/a | 38.7% |

Source: `PAPER_IMAGES_HIL/tables/t1_offline.tex`.

**Online HIL**:

| Backend | Runs | Reliability | Median APE | Latency | CPU |
|---|---:|---:|---:|---:|---:|
| ORB-SLAM2 | 10 | 100% | 0.3569 m | 28.4 ms | 80.5% |
| ORB-SLAM3 | 20 | 100% | 0.3365 m | 31.1 ms | 81.7% |
| OV2SLAM-accurate | 10 | 100% | 0.4166 m | 8.5 ms | 37.7% |
| OV2SLAM-fast | 10 | 100% | 0.4560 m | 4.6 ms | 33.9% |

Source: `PAPER_IMAGES_HIL/tables/t2_online.tex`. The ORB-SLAM3 row pools two
10-trial sets that a permutation test cannot tell apart (p = 0.077): medians
0.3575 and 0.3162 m. Only one of the two sets has front-end timing.
Initialisation distances were 3.40 / 3.30 / 3.40 / 10.75 m.

**Significance of the online median APE** (permutation test, Holm correction):

| A | B | Delta median APE | p (Holm) | Verdict |
|---|---|---:|---:|---|
| ORB-SLAM2 | ORB-SLAM3 | +0.0204 m | 0.3433 | n.s. |
| ORB-SLAM2 | OV2SLAM-accurate | -0.0597 m | 0.0394 | differ |
| ORB-SLAM2 | OV2SLAM-fast | -0.0992 m | 0.0095 | differ |
| ORB-SLAM3 | OV2SLAM-accurate | -0.0801 m | 0.0472 | differ |
| ORB-SLAM3 | OV2SLAM-fast | -0.1195 m | 0.0095 | differ |
| OV2SLAM-accurate | OV2SLAM-fast | -0.0394 m | 0.1720 | n.s. |

Source: `PAPER_IMAGES_HIL/tables/t3_significance.tex`. The ORB family and the
OV2SLAM family separate from each other, but the variants within each family do
not. Every arm held the loop at 100% online, so the choice was a Pareto trade: the
ORB family was about 0.06 to 0.12 m more accurate, for about 2.2x the CPU and 3.3x
to 6.2x the latency. The figures are in `PAPER_IMAGES_HIL/figures/` (`f1` to
`f7`).

### 2.4 Phase 1: controller benchmark (August 2026)

Medians over N = 10 runs per arm, from `bags/ctrl_*/metrics.csv`. The design is in
[section 9](#9-the-controller-benchmark).

| | settling [s] | ss error [m] | overshoot [m] | IAE | ITAE | path eff. | RMS cmd | total var. | CPU [%] | RSS [MB] |
|---|---|---|---|---|---|---|---|---|---|---|
| TS1 IBVS | **19.30** | 0.180 | 0.000 | **321.7** | **2047** | 0.890 | 2.438 | 37.81 | 0.81 | 28.3 |
| TS2 Proportional | 33.92 | 0.216 | 0.000 | 517.4 | 5407 | **0.995** | 0.999 | 4.88 | **0.76** | **18.5** |
| TS3 Homography | 35.47 | **0.181** | 0.000 | 516.8 | 5869 | 0.987 | **0.836** | **1.73** | 0.84 | 23.7 |

Cross-run spread (median +/- SD, N = 10):

| | settling [s] | ss error [m] | total variation | CPU [%] |
|---|---|---|---|---|
| TS1 IBVS | 19.30 +/- 0.97 | 0.180 +/- 0.013 | 37.81 +/- 5.03 | 0.811 +/- 0.012 |
| TS2 Proportional | 33.92 +/- 1.31 | 0.216 +/- 0.014 | 4.88 +/- 0.27 | 0.761 +/- 0.019 |
| TS3 Homography | 35.47 +/- 1.59 | 0.181 +/- 0.023 | 1.73 +/- 0.07 | 0.837 +/- 0.026 |

TS4 PBVS has no row: 17 bags were recorded under `bags/ctrl_pbvs_*` and none has
been analysed.

---

## 3. Observed behaviour

These are things seen in the runs, as opposed to design choices.

**Stereo**

1. **Scale is observable, and it breaks at short baselines.** With a baseline of
   0.36 m or more, Sim(3) alignment finds a scale of 0.99 to 1.02 for every
   backend, so stereo depth is metric. At 0.11 m the scale drifts to 1.12 to 1.33
   for ORB-SLAM2, ORB-SLAM3 and RTAB-Map. The 0.61 m baseline was adopted on
   2026-09-13 after the short baseline produced "thousands of exactly-singular
   triangulations per trial" on this scene's far background (see the note in
   `camera_calib/hil_sim_ov2slam_stereo.yaml`).
2. **The 0.11 m sets do not reproduce.** The two Wi-Fi sets, run hours apart with
   the same configuration, differ by more than 2x for OV2SLAM (1.61 against
   0.67 m). ORB-SLAM3 diverges to 6 to 10 m in both Wi-Fi sets but stays at 0.62 m
   over Ethernet. RTAB-Map goes the other way: 0.27 m over Wi-Fi, 0.53 m over
   Ethernet. Treat 0.11 m as unstable, not as a ranking.
3. **Ethernet delivered fewer frames than Wi-Fi.** Stereo input over Ethernet was
   6.5 Hz, against about 10 to 11 Hz over Wi-Fi and 16 Hz for monocular. Stereo
   doubles the image bandwidth, and the delivered rate depends on the transport
   more than on the backend.
4. **RTAB-Map runs closest to its limit.** It sustains about 12 Hz and receives
   5.5 to 7.8 Hz, the lowest input rate of any arm. It is also the heaviest by
   CPU: 116% on average, with peaks of 272% of 400%. Every other backend has at
   least 2x headroom over its input.
5. **OV2SLAM is flat across baselines.** From 0.36 to 0.61 m, OV2SLAM-accurate
   stays at 0.092 to 0.100 m and OV2SLAM-fast at 0.140 to 0.155 m. RTAB-Map is the
   only backend that improves steadily as the baseline widens.

**Monocular**

6. **OV2SLAM-fast never gets out of initialisation** on this flight: 0/10, and
   0/20 with CLAHE. The aggregation notes attribute the CLAHE failures to a
   reproducible OV2SLAM segfault, a race between `SlamManager::reset()` and
   `LoopCloser::run()` under `use_fast: 1`, which is identical with and without
   CLAHE. Both CLAHE sets are excluded as dead.
7. **ORB-SLAM2 initialises only 5 times in 10.** ORB-SLAM3 always initialises but
   has a heavy tail: a mean of 0.375 m against a median of 0.212 m, with a maximum
   of 1.68 m.
8. **Monocular scale is arbitrary**: 2.6 for OV2SLAM, and 40 to 47 for ORB-SLAM.
   This is expected, and it is why every APE here is Sim(3)-aligned.

**Rig**

9. **The ground-truth pose can freeze mid-run.** In stereo + oracle trials,
   `/sim/drone_pose` has been seen to stop changing while it keeps publishing
   (IDEA-004, `docs/TODO.md`). This was last recorded as open on 2026-09-06, and
   the Phase 2 sets have not been re-checked for it. The oracle detector and the
   servo FSM read that topic with arrival-time-only staleness checks, so a frozen
   value would pass them.
10. **A wrong baseline or focal length fails silently.** Nothing raises an error:
    the depth scale is simply wrong. FIX-027 (fx 1200 against 554) showed up only
    as a scale of about 0.46.

---

## 4. The HIL rig

The physical camera is replaced by a synthetic Unreal Engine scene rendered on a
Windows host and streamed over the network as ROS 2 image topics. The entire
perception and control pipeline runs on real Raspberry Pi 5 hardware.

```
MATLAB/Simulink (Windows)
  Unreal scene -> ROS2 Image publisher(s) -> /sim/camera/image_raw  (+ right eye
                                             when STEREO_ON=1, set_stereo.m)
                                             bgr8, 640x480
                                                v
+---- RPi5  Docker (--net=host --ipc=host --privileged) -----------------+
|  sim_camera_bridge -> /ovcam_frames SHM -> ovcam_bridge (mono8)        |
|                                                v                      |
|  SLAM backend (mono or stereo)  -> /slam/pose                          |
|  oracle_detector (GT projection) or yolo_producer/yolo_bridge (Hailo)  |
|                                                v                      |
|  servo FSM node  -> /cmd_vel      (held with --hold-fsm in Phase 2)    |
+------------------------------------------------------------------------+
                               v
          MATLAB Simulink drone dynamics -> Unreal scene update
```

- **Phase 1** closed the loop: the servo flew the drone to the stop sign using
  YOLO or oracle detections, over Ethernet with CycloneDDS and core pinning.
- **Phase 2** isolated SLAM. The FSM was held (`--hold-fsm`), a scripted probe
  mission (`slam_traj_probe.m`, in the Windows host's copy of `matlab/`) flew the drone at 2.0 m/s, the oracle
  detector stood in for YOLO, and SLAM was unpinned. Transport was FastRTPS over
  Wi-Fi (Pi `192.168.1.60`, MATLAB `192.168.1.201`) or over Ethernet (Pi
  `192.168.137.10`, MATLAB `192.168.137.1`).
- The stereo interface is specified in `docs/STEREO_INTERFACE_CONTRACT.md`. The
  rig setup and the MATLAB startup order are in [`SIM_HIL.md`](SIM_HIL.md).

---

## 5. Candidates

| Candidate | Package | Phase 1 | Phase 2 |
|---|---|---|---|
| ORB-SLAM2 | `src/orbslam2/` | Mono | Mono + Stereo |
| ORB-SLAM3 | `src/orbslam3/` (vendored `Mechazo11/ros2_orb_slam3` v2.0.0, ORB-SLAM3 V1.0, `PublishPose` patch on both paths) | Mono | Mono + Stereo |
| OV2SLAM-accurate | `src/ov2slam_ros/` | Mono | Mono + Stereo |
| OV2SLAM-fast | `src/ov2slam_ros/` | Mono | Mono (never initialised) + Stereo |
| RTAB-Map | `src/rtabmap_docker/` | Stereo, EuRoC reference only | **Stereo peer** |

Stack configurations for Phase 2: `config/hil/stack/*_stereo_oracle_nopin.yaml`
(stereo) and `*_probe_nopin_wifi.yaml` / `ov2slam_oracle_*_nopin_wifi.yaml`
(mono).

---

## 6. Method

### Phase 2 (stereo and monocular re-run)

- **Orchestration.** The Windows matrix driver runs one backend (an "arm") for 10
  trials in round-robin, with a fail-fast policy: no retries, and the first bad
  step stops the run. On the Pi each trial runs
  `run_stack_hil.sh --config <cfg> --mode benchmark --hold-fsm --run-tag <TAG>`.
- **Baseline switching.** `~/sync_baseline.py <m>` on the Pi rewrites the five
  files that encode the baseline and reads each back to verify:
  - `hil_sim_ov2slam_stereo{,_fast}.yaml`: `body_T_cam1` x
  - ORB-SLAM2 `hil_sim.yaml`: `Camera.bf = 554 * b`
  - ORB-SLAM3 `HIL_SIM.yaml`: `Stereo.b`
  - `hil_stereo_bridge.py`: `BASELINE_M`

  The Simulink camera mount must match. `*.baseline061.bak` holds the 0.61 m
  versions.
- **Scoring window.** From `max(first non-zero /slam/pose, mission start)` to 3
  samples after ground-truth speed falls below 0.10 m/s (`end_pad = 3`; the pad0,
  pad3 and pad10 values agree to within 1 mm).
- **Accuracy.** `evo_ape tum -a -s`: Sim(3) Umeyama alignment with scale solved and
  a 0.050 s match tolerance. The reported figure is APE RMSE. The scale factor is
  reported separately, as a metric-scale check for stereo.
- **Cost.** Front-end frame time (mean and p95) from each backend's timing CSV;
  capacity = 1000 / frame time; process CPU mean and max.
- **Aggregation.** The per-trial `results.csv` is written by
  `scripts/hil_matrix/aggregate_matrix.py` (a copy sits in each result directory).
  Per-arm rows are the mean over `status == ok` trials; the SD reported is the
  sample SD.

### Phase 1 (monocular, August)

Unified window from mission start to arrival, with the initialisation leg
excluded. Median APE after Umeyama alignment with scale. Reliability = fraction of
runs that completed with a usable trajectory. Permutation test with Holm
correction. Numbers are produced by `scripts/hil_matrix/aggregate_matrix.py` and
rendered by `PAPER_IMAGES_HIL/generate.py`, and are never edited by hand.

---

## 7. Measurement caveats

**Phase 2**

1. **SLAM is unpinned.** CPU is a share of 400%, not the 200% ceiling of Phase 1.
2. **Phase 2 measures SLAM alone.** The controller is not in the loop
   (`--hold-fsm`), so these runs say nothing about whether a stereo pose holds a
   servo loop together (TODO-AS).
3. **The 0.11 m sets ran on 09-20/21, after the 0.36 to 0.61 m sets (09-14 to 09-17)**, and do not
   reproduce between themselves ([section 3](#3-observed-behaviour), item 2).
4. **The Wi-Fi re-run set was first labelled "Ethernet" by mistake.** It was
   renamed to `wifi_rerun` once the recorded interface was checked. Its `run.log`
   header still says `--transport ethernet`; go by the directory name.
5. **No significance tests** have been run on the stereo arms yet.
6. **Focal lengths differ by design.** Stereo calibration uses fx = 554 because the
   Simulink stereo blocks are at 554 (FIX-027). The servo and monocular
   `cam_fx` is 1200 (FIX-020).
7. **The results were produced from `6e7bf48` plus uncommitted changes.** Each
   result directory records `git_commit.txt` and `git_status.txt`. To reproduce,
   commit the working tree that matches those records.
8. The SDs here are sample SDs. `~/baseline_sweep.tex` used population SDs, so its
   figures differ in the third decimal place.

**Phase 1** (unchanged)

1. SLAM was pinned to cores 2 and 3, so the CPU ceiling is 200% and the latency
   figures describe a 2-core allocation.
2. OV2SLAM configurations put the detector and the controller together on core 0;
   the ORB configurations split them across cores 0 and 1.
3. OV2SLAM-fast initialised at 10.75 m against a 3.0 m leg, so it was scored over
   about 26 m where the others were scored over about 33 m. Treat its row as
   provisional.
4. ORB-SLAM3's `frontend/full_tracking` timing is instrumentation added for this
   work, not upstream's.

---

## 8. RTAB-Map: stereo peer, excluded from the monocular tables

In Phase 2, RTAB-Map is a full **stereo** peer: same flight, same scoring, n = 10
per baseline.

It stays **out of every monocular table**. That decision was made on 2026-08-24
with the supervisor's approval, for three reasons:

1. **The EuRoC runs were stereo.** `src/rtabmap_docker/euroc_offline_f2m.launch.py`
   subscribes to stereo and runs `stereo_odometry`.
2. **The only EuRoC summary is a best-of-3 selection.** It is
   `/home/amaraly/ORB_SLAM2/images/trajectory_selection_summary.csv`, which is not
   comparable with all-runs statistics.
3. **RTAB-Map ships no monocular odometry node.** The orphaned `OdometryMono` class
   can only be reached through `rtabmap-odometryViewer` with `Odom/Strategy -1`,
   and it crashes as shipped under OpenCV 4.6 because of a PnP guess-vector shape
   mismatch (1x3 against 3x1). With a runtime shim
   (`benchmarks/rtabmap_mono_probe/`, in the `ROS2-PROJECT-SPRING2026` checkout)
   it tracks MH_01 for about 30 s (median APE 0.09 m), then drifts without bound
   (3.4 m median APE over the full 173 s). It is a VO front end, not a SLAM
   backend.

The EuRoC stereo reference (best-of-3, with ORB-SLAM2 from the same CSV) is kept in
that CSV. Its batches are `20260525_*` for RTAB-Map and `20260530_232822` for
ORB-SLAM2.

---

## 9. The controller benchmark

This is the Phase 1 study. It swaps the **servoing law** and holds everything else
fixed. The design and runbook are in `benchmarks/CONTROLLER_BENCH_PLAN.md`.

| | Package / node | Servoing law |
|---|---|---|
| **TS1** | `visp_servo` | IBVS: ViSP `vpServo`, 4 corner features, `v = -lambda * L^+ * (s - s*)` |
| **TS2** | `hil_servo` | Proportional: `vx = k_fwd*(ratio* - ratio)`, `vy = -k_lat*ex`, `vz = -k_vz*ey` |
| **TS3** | `h_vs_servo` | Homography: translation from the H decomposition, rotation from `vex(H - H^T)` |
| **TS4** | `visp_pbvs_servo` | PBVS: `vpServo` on a reconstructed pose |

- **Same node for every arm.** All four wrap the same `servo_core::ServoFsmNode`:
  the same FSM, safety filters and `/bench/state` instrumentation. Only the injected
  `IServoController` differs.
- **Shared parameters.** Intrinsics, clamps, slew limiter, FSM thresholds, target
  and start pose are shared in `config/hil/bench_fsm.yaml` (now `cam_fx = 1200`,
  FIX-020). Gains are per arm in `config/hil/bench_{ibvs,proportional,h_vs,pbvs}.yaml`.
- **Depth comes from the bounding box** (`Z = fy*H/bh`) for every arm; no SLAM is
  in the loop.
- **Oracle detector.** It projects the known target through the sim camera using
  ground-truth pose, giving a deterministic box on `/yolo/detections`.
- **Measurement.** Closed loop, timed from the first SEARCHING to APPROACHING edge.
  Metrics come from `benchmarks/plot_controller_hil.py`, with
  `e(t) = ||drone - target|| - standoff` and a standoff of about 3.15 m.

**Reading the results.**
- IBVS settles about 1.8x faster, but its command signal is 8x to 22x rougher.
- No arm overshoots: the shared clamp and slew limiter set the approach envelope.
- Steady-state error splits two ways: IBVS and homography about 0.18 m,
  proportional 0.216 m.
- CPU does **not** separate (0.76 to 0.84%). The pseudo-inverse is not the
  bottleneck at 20 Hz on a Cortex-A76.

**Caveats.**
- TS3 falls back to proportional `k_fwd` on the forward axis, because
  axis-aligned oracle boxes give a pure affine H.
- The PBVS run names are not a clean N-series.
- `src/h_vs/` is an empty leftover; the real package is `src/h_vs_servo/`.

---

## 10. Open issues

**Phase 2**

- **Ground-truth pose freeze (IDEA-004).** It is still open, and the Phase 2 sets
  have not been audited for it.
- **The 0.11 m sets do not reproduce.** The Wi-Fi and Ethernet sets disagree in
  direction, not only in size. A controlled Wi-Fi/Ethernet pair on the same day
  would settle it.
- **Ethernet delivered stereo at 6.5 Hz, below Wi-Fi's 10 to 11 Hz.** This is
  unexplained.
- **No stereo controller-in-the-loop run yet** (TODO-AS).
- **OV2SLAM-fast monocular never initialises.** The segfault race under
  `use_fast: 1` is root-caused but not fixed.
- **Scripts that exist only outside the repository.**
  - on the Pi: `~/sync_baseline.py`, `~/aggregate_*.py` and `~/baseline_sweep.tex`
  - on the Windows host: `run_matrix.*`, `check_clean.ps1` and the
    `config/hil/matrix/only_*_stereo_*` configs
- **Planned but not run:** an RTAB-Map RGB-D smoke test on TUM `fr1_desk`
  (`~/TUM_RGBD/`), using ROS 2 nodes fed from a converted bag.

**Phase 1** (still open)

- ORB-SLAM3's CPU differs between its two online sets (89.0% against 74.4%) and
  this is unexplained.
- ORB-SLAM3's offline failures (26%) are not characterised.
- The upstream commit of the vendored ORB-SLAM3 tree was not recorded.
- The judgement calls worth challenging are the window start, `END_PAD = 3`, and
  the ORB-SLAM3 timing instrumentation.

---

## 11. Data provenance

**Phase 2.** Results are in `results/` (gitignored, local to this worktree):

```
results/stereo/alicia_baseline_<011|036|042|054|061>_<wifi|wifi_rerun|eth>_<arm>_<stamp>_n10/
results/mono/alicia_mono_wifi_<arm>_<stamp>_n10/
    results.csv  report.txt  manifest.json  run.log  git_commit.txt  git_status.txt
    configs/  logs/  aggregate_matrix.py  count_poses.py
results/_superseded/   RTAB-Map n=3 sweep runs, replaced by the n=10 repeats
results/_failed/       aborted matrices
results/archive/       earlier matrices, 2026-09-04 to 09-14
results/_*_undo.sh     undo the 2026-09-20/21 renames back to matrix_<stamp>
```

- Bags are under `bags/run_<config>_<RUNTAG>_<stamp>/`, and the run tag
  (`BASELINE036`, `BASELINE011_ETH`, `ALICIA_mono_*`, ...) links each trial to its
  set.
- Per-baseline aggregates are in `~/alicia_aggregates/` and
  `alicia_aggregates/alicia_{mono,stereo}_aggregate.csv`.
- **`results/` and `bags/` are not in git.** Deleting this worktree deletes them.
  Back them up before any worktree cleanup.

**Phase 1.**

| Backend | Offline summary CSV | Online bag glob |
|---|---|---|
| ORB-SLAM2 | `ORB_SLAM2/results/20260609_092931/experiment_summary.csv` | `bags/run_orbslam2_probe_verso10_*` |
| ORB-SLAM3 | `ORBSLAM3_ROS2/results/orbslam_benchmark/20260614_014256/experiment_summary.csv` | `bags/run_orbslam3_probe_orb3x10_*` + `..._orb3lat_*` |
| OV2SLAM-accurate | `results/ov2slam_benchmark_v2/20260612_234141/accurate_mono/experiment_summary.csv` | `bags/run_ov2slam_oracle_accurate_verso10_*` |
| OV2SLAM-fast | `results/ov2slam_benchmark_v2/20260613_102424/fast_mono/experiment_summary.csv` | `bags/run_ov2slam_oracle_fast_verso10_*` |

The resolved provenance is in `PAPER_IMAGES_HIL/data/provenance.json`. The
ORB-SLAM3 glob is deliberately not a bare `orb3*`, because that pattern also
matches aborted initialisation-only bags.

---

## 12. Hardware and prerequisites

| Item | Details |
|---|---|
| **Board** | Raspberry Pi 5 (Cortex-A76 x 4, 16 GB RAM) |
| **Camera (live)** | OV5647 (CSI), 640 x 480 at ~30 fps |
| **Camera (HIL)** | MATLAB/Unreal, bgr8 640 x 480; mono ~16 Hz, stereo ~11 Hz (Wi-Fi) / ~6.5 Hz (Ethernet) delivered |
| **Accelerator** | Hailo-10H AI HAT+ 2 (`/dev/hailo0`) |
| **Host OS** | Raspberry Pi OS Trixie (64-bit), required for the Hailo and libcamera stack |
| **Docker** | Docker Engine >= 24 |
| **ROS 2** | Jazzy on both sides |
| **DDS** | Phase 1: CycloneDDS over Ethernet. Phase 2: FastRTPS, to match MATLAB's `RMW_IMPLEMENTATION` |
| **MATLAB** | R2024a or later with ROS Toolbox |

```bash
sudo apt install -y libcamera-dev cmake g++ pkg-config util-linux gettext-base
```

The wired Pi is `amaraly@192.168.137.10`; over Wi-Fi it is `192.168.1.60`.
Watch the timezones when correlating logs: Windows runs on UTC+3 and the Pi on
UTC+4, so bag names carry Pi time.

---

## 13. Running the stack

```bash
./run_stack_hil.sh build [<pkg>]                        # build all, or one package
./run_stack_hil.sh --config ov2slam_stereo_oracle_nopin \
    --mode benchmark --hold-fsm --run-tag MYTAG          # one Phase 2 stereo trial
./run_stack_hil.sh stop                                 # tear down, finalise bag
```

1. **On Windows**, follow [`SIM_HIL.md`](SIM_HIL.md). For stereo, set
   `STEREO_ON=1` (`set_stereo.m`).
2. **On the Pi**, choose the network profile. Wired configs point at
   `192.168.137.1`; the `*.wifibak` copies and the `*_wifi.yaml` configs point at
   `192.168.1.201`. For stereo, run `~/sync_baseline.py <m>` so the five
   calibration files agree with the Simulink mount.
3. **Verify** that `/slam/pose` and `/yolo/detections` are flowing (the latter
   from the oracle in Phase 2).

**Core pinning (Phase 1 configs).** SLAM runs on cores 2 and 3, detection and
control on cores 0 and 1. The `*_nopin` configs drop pinning.

```bash
watch -n1 'ps -eo pid,psr,pcpu,comm --sort=-pcpu | grep -E "PSR|ovcam|yolo|ov2slam|orb|rtabmap|visp|hil_servo" | head -12'
```

> Never start long-running containers with `docker run -it`. The tty spin-loop
> burns about 170% CPU and corrupts every measurement.

**Regenerate the Phase 1 tables:** `python3 PAPER_IMAGES_HIL/generate.py`.

**Watch for process leaks on Windows.** MathWorks helpers and `AutoVrtlEnv`
(about 500 MB each) can be orphaned. The Windows preflight clears them and refuses
to start with less than 8 GB free.

---

## 14. Repository layout

```
ROS2-PROJECT-SPRING2026/           (this worktree: ROS2-slam-hil, branch benchmarks/slam-hil)
|-- README.md  SIM_HIL.md  GOTCHAS.md
|-- run_stack_hil.sh                # Main entry point for HIL sessions
|-- Dockerfile                      # Ubuntu 24.04 + ROS 2 Jazzy
|-- src/
|   |-- orbslam2/  orbslam3/  ov2slam_ros/     # SLAM backends (mono + stereo)
|   |-- rtabmap_docker/
|   |   |-- hil_stereo.launch.py  hil_stereo_bridge.py   # HIL stereo (Phase 2)
|   |   |-- euroc_offline_f2m.launch.py  eval.py  host_benchmarker_rtabmap  # EuRoC stereo harness
|   |   \-- mapPath_to_tum.py  odom_to_tum.py
|   |-- sim_camera_bridge/  ovcam_producer/  ovcam_bridge/
|   |-- yolo_producer/  yolo_bridge/  yolo_msgs/  yolo_ros/
|   |-- servo_core/  hil_servo/  h_vs_servo/  visp_servo/  visp_pbvs_servo/
|   |-- init_gate/  oracle_detector/
|-- camera_calib/                   # mono + stereo calib; *.baseline061.bak = 0.61 m versions
|-- config/hil/stack/               # *_stereo_oracle_nopin, *_nopin_wifi, *.wifibak, *_TEST
|-- scripts/hil_matrix/             # aggregate_matrix.py, export_bag_csv.py, stereo tooling
|-- matlab/                         # Simulink model helpers (slam_traj_probe.m lives on the Windows host)
|-- PAPER_IMAGES_HIL/               # Phase 1 tables and figures (generated)
|-- benchmarks/                     # drivers and analysis
|-- docs/fixlog/  docs/ideas_to_check/  docs/TODO.md  docs/STEREO_INTERFACE_CONTRACT.md
|-- bags/  results/                 # run data (git-ignored)
\-- build/  install/  log/          # colcon output (git-ignored)
```

`HANDOFF*.md` session notes are local only and ignored by git. The EuRoC payload
for the RTAB-Map harness lives outside the repository, at
`/home/amaraly/RTAB_Docker/`.

---

## 15. Contributors

| Name | Role / Contribution |
|---|---|
| Amar Aly | SLAM-HIL rig, benchmark design and execution, SLAM backend integration (OV2SLAM, ORB-SLAM2, ORB-SLAM3, RTAB-Map), stereo HIL, ROS 2 environment, camera calibration |
| Ahmad Dhaoudi | YOLOv26n conversion to `.hef` for Hailo-10H, YOLO detection and setup, ViSP integration |

---

*Last updated: 2026-09-26*
