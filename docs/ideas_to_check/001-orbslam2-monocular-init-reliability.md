---
id: IDEA-001
title: ORB-SLAM2 monocular init reliability under orbslam2_eval
date: 2026-07-02
status: GRADUATED → docs/fixlog/011-slam-init-gate.md (opt-in SLAM-init gate implemented)
component: config/hil/stack/orbslam2_eval.yaml, src/servo_core/src/servo_fsm_node.cpp
---

> **GRADUATED 2026-07-02.** The candidate fix below (gate `SEARCHING → APPROACHING`
> behind SLAM readiness) was implemented as an opt-in blind-search gate — see
> `docs/fixlog/011-slam-init-gate.md`. Design decisions differ slightly from the
> sketch here (chosen after critic + KISS review): the gate ignores detections
> entirely during a BLIND phase (rather than tweaking the lock-on condition), adds a
> post-init RELOCATE re-sweep, and a `max_blind_strafes` FAILED abort. TODO-Q/TODO-R
> are closed by FIX-011; remaining follow-ups are TODO-S/T/U there. Kept for the
> investigation trail (CPU-pinning falsification, H1/H3 analysis).

## Symptom

`orbslam2_eval` benchmark runs produce wildly inconsistent SLAM quality run-to-run,
despite an (assumed) fixed/deterministic scenario. Five runs, same config family:

| Run | CPU layout (det/ctrl/slam) | duration_s | eval_path_m | full_path_m | umeyama_scale | ATE_RMSE_m |
|---|---|---|---|---|---|---|
| historical (pre-session) | pinned "2,3" | 63.9 | 23.5 | 95.3 | ~1–2 | 0.017 |
| tag33 | 0 / 0 / unpinned | 9.18 | 2.12 | 95.21 | 2.23 | 0.085 |
| tag35 | 0 / 0 / unpinned | 46.66 | 17.17 | 94.10 | **9.91** | 0.493 (max 8.01) |
| tag37 | 0 / 3 / "1,2" | 11.51 | 2.59 | 93.50 | 2.56 | 0.178 |
| tag38 | 0 / 3 / "1,2" | 31.30 | 30.48 | 93.81 | **30.93** | 0.203 |
| tag39 | 0 / 3 / "1,2" | 27.48 | 13.75 | 93.79 | **17.08** | 0.063 |

`full_path_length_m` is consistent (~94–95m) across all runs, confirming this is the
*same* scenario replayed — the variance is entirely in how ORB-SLAM2's monocular
initializer lands, not in the flight itself.

## Hypotheses tested

**H1 — CPU contention (unpinned SLAM competing with detector/controller on core 0).**
Tested by fully separating all three onto dedicated cores (tag37/38/39: det=0,
ctrl=3, slam=1,2) — strictly *more* isolated than the original pinned "2,3" config.
Variance did not improve (durations 11.5–31.3s, scale 2.6–30.9 still swinging wildly).
**FALSIFIED as primary cause.** CPU pinning may be a minor contributor but is not
the dominant driver.

**H2 — reduce `startup_delay_sec: 15`.**
Rejected without testing: the delay exists specifically to prevent ORB-SLAM2's own
near-zero-parallax auto-exit bug (drone must already be moving when the node comes
up). Shrinking it risks re-triggering that failure mode. Not attempted.

**H3 — forward-dominant motion → weak monocular initializer conditioning.** **SUPPORTED.**
- tag33: GT trajectory is dominant motion along x (0→32m) with small y/z components —
  near-degenerate geometry for two-view initialization (needs baseline *perpendicular*
  to the optical axis, not along it). `timing_events.csv` showed 287/490 frames (58%)
  stuck in `tracking_state=1` (`NOT_INITIALIZED`) before finally succeeding.
- tag35: explicit reset sequence found in `timing_events.csv` — attempt #1 failed after
  9 frames, attempt #2 failed after 273 frames (~15s), attempt #3 succeeded in a rushed
  19 frames (~1.1s). The rushed 3rd attempt is consistent with a marginal frame pair
  that barely passed RANSAC acceptance — explains the huge `umeyama_scale=9.91` and the
  `8.01m` ATE outlier (bad map scale baked in early, self-consistently tracked afterward
  since ORB-SLAM2 doesn't re-scale a mono map later without loop closure).

## Key architectural finding: SEARCHING already has the fix, but oracle skips past it

`servo_fsm_node.cpp::build_search_command()` runs a fixed step sequence:
`FULL_ROTATE (yaw-only) → YAW_RIGHT_60 → YAW_LEFT_60 → YAW_CENTER → STRAFE_RIGHT (real
lateral cmd.linear.y translation!) → loops back to YAW_RIGHT_60 → ...`

`STRAFE_RIGHT` is exactly the perpendicular-parallax motion the initializer needs. But
`update_state_on_detection()`'s `SEARCHING → APPROACHING` lock-on fires as soon as
`consecutive_dets_ >= lockon_consec_` — with the **oracle** detector (perfect,
noise-free, instant), that's within ~1-2 seconds of boot. The search sequence never
reaches `STRAFE_RIGHT` before locking on and committing to the forward-dominant
`APPROACHING` motion. This is *why* the trajectory is forward-dominant — not because
the search pattern lacks lateral motion, but because oracle short-circuits past the
part that has it.

## Candidate fix (NOT implemented — needs a decision)

Gate `SEARCHING → APPROACHING` behind an opt-in condition — e.g. "at least one full
search cycle through `STRAFE_RIGHT` has completed" or "SLAM reports ready" — so the
flight naturally picks up lateral parallax before committing to the approach.

**Must be opt-in, default off.** `update_state_on_detection()` is shared with
`controller_bench.launch.py` (the N=10 reported controller-RMSE benchmark, where SLAM
never runs). An unconditional gate — especially one keyed on "SLAM ready" — would
either change reported-benchmark behavior or deadlock it outright (the ready signal
would never arrive). Same pattern as the existing `use_slam_depth`/`use_slam_pose`
opt-in booleans (default `false`, wired only through SLAM-eval configs).

Rejected sibling idea: literally "blind" the detector (suppress all detections) to
hold the drone in `SEARCHING` until SLAM is ready. Doesn't work as stated —
`FULL_ROTATE`/`YAW_*` steps are pure rotation (zero parallax), so a naive blind-hold
would likely get stuck spinning before ever reaching `STRAFE_RIGHT`, unless the gate
specifically forces progression *through* the full step sequence rather than just
freezing on "no detection."

## Also unresolved

**Drone starting position** lives in `matlab/hil_closed_loop.slx` (Simulink model,
binary/zipped — not text-searchable or editable from the Pi/ROS2 side). Not the
initializer script (`matlab/hil_ros_init_LT.m` only *publishes* `/sim/drone_pose`,
doesn't set it). Any trajectory-shape change requires opening the model in Simulink.

## Open TODOs

- **TODO-Q**: decide whether to implement the opt-in SEARCHING-gate fix (full SECOND
  BRAIN protocol — touches shared FSM code) vs. accept run-to-run variance and
  multi-run for a usable sample vs. report the variance itself as a SLAM-sanity finding.
- **TODO-R**: if pursuing the gate fix, confirm parameter default is `false` and
  verify `controller_bench.launch.py` / N=10 benchmark behavior is provably unchanged
  before merging.
