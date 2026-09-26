---
id: FIX-020
title: cam_fx_/cam_fy_ stale at unverified 554 vs the project's own measured-and-fixed 1200 -- corrected the value, collapsed the redundant camera_fx_ parameter that had actually already been correct
date: 2026-09-01
status: resolved
component: src/servo_core/src/servo_fsm_node.cpp, .hpp, config/hil/bench_fsm.yaml
supersedes: none (first attempt at this fix was rejected before being applied -- see below)
critic_verdict: wrong_fix (round 1, rejected before applying) -> open_concern (round 2, applied)
kiss_verdict: simple (round 1 verdict N/A, rejected) -> ok (round 2)
open_todos: [TODO-AH, TODO-AI, TODO-AJ]
---

## Symptom

Mono run on benchmarks/slam-hil: drone did an almost-full-circle spin at HIL
start despite the target being nearly dead-ahead -- described by the user as
qualitatively different from controller-benchmark, which "never tilted away
so horribly."

## Root cause -- and a real misdiagnosis en route, worth recording in full

**Round 1 (WRONG, caught before applying, not committed):** Found `servo_core`
had two ROS parameters both meaning "camera focal length in pixels":
`camera_fx_` (1200.0, used only in `bearing_from_ex()` for SEARCH-state yaw)
and `cam_fx_`/`cam_fy_` (554.0, sourced from `bench_fsm.yaml`, used for
depth-from-bbox and every controller's `ServoInputs.fx/fy`). Verified live
(`ros2 param get /visp_servo_node camera_fx` = 1200.0, `cam_fx` = 554.0) and
cross-checked against a real `[lockon-bias]` log line's math -- both matched
fx=1200 being in use. **Concluded, wrongly: `cam_fx_`=554 must be the
trustworthy pre-existing value, `camera_fx_`=1200 must be a stray stereo
default introduced during migration.** Proposed deleting `camera_fx_` and
pointing `bearing_from_ex()` at `cam_fx_` instead.

**Critic round 1 rejected this outright (`wrong_fix`)**, and re-verifying it
myself confirmed the critic exactly: `git show 32d8ea7` (a same-day commit on
`feat/stereo-hil`'s `visp_servo_node.cpp`, the file `servo_core`'s version was
ported from) introduced `camera_fx_=1200.0` specifically to fix "a critical
issue where the FOV assumed in the SEARCH stage... causes a wrong drift -->
overshoot the target and be in a loop" -- this exact symptom -- with an inline
comment "the 554 in this file is supposedly not really used." `git show
11fa007`'s full commit message goes further: the Simulink camera was
independently measured TWICE (a warp-correlation test, 0.94 vs 0.41
correlation; and stereo triangulation against ground-truth depth, giving
fx=1236, 3% off 1200) and concludes "The Simulink camera renders at fx=1200,
not 554. Every config saying 554 was wrong by 2.17x. 554 was never measured
-- it was derived from an assumption." That commit explicitly lists files
across the project still wrongly carrying 554, not yet swept -- `bench_fsm.yaml`
matches that same stale pattern exactly, just wasn't on the list by name.

**So the diagnosis was backwards: `camera_fx_`=1200 was the correct,
twice-verified value; `cam_fx_`/`cam_fy_`=554 was the stale, never-measured
one.** Applying round 1's fix would have overwritten the correct value with
the disproven one in the exact calculation whose wrong-FOV bug had already
been fixed once on the sibling file.

**Round 2 (applied):** reversed direction. `cam_fx_`/`cam_fy_` (and
`bench_fsm.yaml`'s `cam_fx`/`cam_fy`) were the ones needing correction, to
1200, matching the project's own measured value -- not `camera_fx_`.

## Diff

`config/hil/bench_fsm.yaml`: `cam_fx: 554.0` -> `1200.0`, `cam_fy: 554.0` ->
`1200.0` (cam_cx/cam_cy, the principal point, untouched -- unrelated to focal
length).

`src/servo_core/src/servo_fsm_node.cpp`:
- `declare_parameters()`: changed `cam_fx`/`cam_fy` defaults 554.0 -> 1200.0;
  deleted the now-fully-redundant `declare_parameter("camera_fx", 1200.0)`.
- `load_parameters()`: deleted `camera_fx_ = get_parameter("camera_fx")
  .as_double();`
- `bearing_from_ex()`: `camera_fx_` -> `cam_fx_` (now correctly 1200 via the
  yaml -- numerically a no-op for this function specifically, since both
  values converge on 1200; the actual behavior change is entirely in the
  depth-from-bbox and controller-input paths that were reading the stale 554).

`src/servo_core/include/servo_core/servo_fsm_node.hpp`: deleted the
`double camera_fx_;` member.

Not touched: `config/hil/bench_oracle.yaml` (also `cam_fx/cam_fy: 554.0`,
governs a different node, `oracle_detector`); `feat/stereo-hil` /
`controller-benchmark` (source branches); `benchmarks/controller-hil`
(sibling worktree, confirmed to have the byte-identical bug at identical line
numbers, not fixed here).

## Critic verdict & concerns

Round 1: **wrong_fix**, confirmed and acted on (see Root cause above).
Round 2: **open_concern**, proceeded per protocol. Live-verified the diff's
starting state matched reality, and confirmed `bench_oracle.yaml`'s stale
554 doesn't leak into servo nodes (each node only layers `bench_fsm.yaml` +
its own `bench_<ctrl>.yaml`). Real objection, documented in-repo rather than
speculative: correcting the intrinsics changes the effective closed-loop
behavior of controllers whose gains were empirically tuned around the wrong
554.
- `config/hil/bench_h_vs.yaml`'s own comment documents a sensitivity
  assumption of `140px offset / fx(554) ~ 0.25`; post-fix the same offset
  gives `140/1200 ~ 0.12` -- roughly HALVING TS3's (h_vs) lateral/vertical
  response for the same `lambda_v`, with no retune, and that comment is now
  stale.
- TS1 (IBVS, `visp_servo`) moves the opposite way: `ibvs_controller.cpp`'s
  interaction matrix scales as `1/Z`, and `Z` (depth-from-bbox) is now
  computed ~2.17x LARGER (`Z = fy*H/bh`, fy 554->1200) for the same real
  geometry -- so commanded velocities for a given pixel error scale up
  ~2.17x, meaning more frequent saturation against `max_linear=3.0`, an
  untested behavior change.
- Critic's falsifying/validating experiment (not yet run): log TS1/TS3
  `vel.vy`/`vel.vz` magnitude vs pixel error before/after, on real HIL
  trials.

## KISS verdict

Round 2: **ok**. Simplification agent confirmed combining the value-fix and
the parameter-collapse into one change is provably low-risk, not just
probably fine: once `cam_fx_`/`cam_fy_` become 1200 via the yaml, redirecting
`bearing_from_ex()` from `camera_fx_` to `cam_fx_` is a value-neutral rename,
not a second independent behavior change -- so if anything regresses after
rebuild, it can only be attributed to the depth/controller-input value
change, not the collapse. Also independently re-confirmed the
`benchmarks/controller-hil` sibling-worktree bug (byte-identical, same line
numbers) and the `bench_oracle.yaml` divergence this fix newly introduces
(previously both files agreed, wrongly, at 554; now they'll silently
disagree, 1200 vs 554).

## Verification

- `./run_stack_hil.sh build`: 12/12 packages finished clean, `servo_core`
  [28.9s], no errors.
- Grepped the whole `src/servo_core/` tree for `camera_fx` post-edit: only
  hit is the explanatory comment left in `declare_parameters()` -- zero
  remaining parameter/member references.
- Verified via a throwaway probe instance (`ros2 run visp_servo
  visp_servo_node --ros-args -r __node:=verify_probe_node`, same params
  files, killed immediately after, never touched the live/idle session):
  `ros2 param get cam_fx` = `1200.0` (was `554.0`), `cam_fy` = `1200.0`,
  `ros2 param get camera_fx` = `Parameter not set` (confirms it's genuinely
  gone, not just unused). Probe's own log/pgrep confirmed clean termination.
- NOT yet verified: an actual live HIL trial with the corrected depth
  estimate -- both because the camera pipeline was idle at fix time (no
  MATLAB feed to test against) and because the critic's flagged gain-tuning
  concern means a live trial is exactly what's needed next, not just a
  parameter-readback check.
- NOT resolved: the original symptom (almost-full-circle spin at HIL start).
  This fix corrects a real, independently-confirmed stale-intrinsics bug
  that predates the migration, but was never established as the spin's
  cause -- that diagnosis remains open.

## Open TODOs

- TODO-AH: run a live HIL trial to check whether TS1 (IBVS) now saturates
  against `max_linear` more often post-fix (commanded velocity now scales
  ~2.17x for the same pixel error, per critic's `1/Z` interaction-matrix
  analysis) and whether TS3 (h_vs) has become sluggish (response roughly
  halved per `bench_h_vs.yaml`'s own documented sensitivity assumption,
  now stale). Retune `lambda`/`lambda_v`/`k_fwd` if either shows up.
- TODO-AI: `config/hil/bench_oracle.yaml` still carries `cam_fx/cam_fy:
  554.0`, feeding the `oracle_detector` node, whose own header comment says
  intrinsics "MUST match bench_fsm.yaml" -- previously both files were
  wrong together (554/554); now they silently disagree (1200/554). Fix
  `bench_oracle.yaml` to match, or explicitly document why it's exempt.
- TODO-AJ: `benchmarks/controller-hil` (sibling worktree,
  /home/amaraly/ROS2-controller-hil) has the byte-identical camera_fx/cam_fx
  split and stale-554 bug at identical line numbers -- not fixed here, out
  of scope until that branch is actually used for SLAM/depth-sensitive work.
- Still open from before this fix: the actual root cause of tonight's
  SEARCH-stage spin. Not established to be this bug -- needs its own fresh
  investigation.
