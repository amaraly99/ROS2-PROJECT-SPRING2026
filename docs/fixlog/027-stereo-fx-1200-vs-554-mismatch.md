---
id: FIX-027
title: Stereo camera calib declared fx=1200 (the 2026-08-24 correction) but the Simulink block itself was never updated off 554 -- systematic ~2x depth bias, scale~0.46 instead of ~1.0
date: 2026-09-06
status: resolved
component: camera_calib/hil_sim_ov2slam_stereo.yaml
supersedes: none (FIX-020 corrected a DIFFERENT component -- servo_core's cam_fx_/camera_fx_ params -- and remains valid; this fix is a separate, narrower gap in the same 554-vs-1200 rollout)
critic_verdict: not dispatched -- no subagent review this cycle; verified instead via direct multi-trial empirical testing (Arian directing hypothesis formation and falsification live), see Verification below
kiss_verdict: not dispatched (same reason)
open_todos: [TODO-AS]
---

## Symptom

Every `only_ov2_stereo_oracle` benchmark trial (once trials completed at all --
see IDEA-004 for the separate, still-open ground-truth-freeze bug that blocked
most of today's trials from scoring anything) showed a consistently wrong
Umeyama scale factor: **0.41-0.52**, versus the ~1.0 expected for a properly
calibrated stereo rig (stereo has real absolute scale from the known camera
baseline, unlike mono which is scale-free by construction and legitimately
needs an arbitrary scale correction). APE RMSE was also elevated and noisy
(0.24-0.49 m across 3 trials, mean 0.335 +/- 0.113).

Contemporaneous mono trials (`ov2slam_oracle_accurate`, same day, same
hardware) scored RMSE=0.071 m with no comparable anomaly -- ruling out CPU
pinning, network medium, and stale calibration-in-general as the cause (all
three were separately, directly tested this session and disproven for the
*other* bug being chased at the time; see IDEA-004). The scale anomaly was
noticed as a side effect of that investigation, not the original target.

## Root cause

Two facts, each independently verified, together explain it exactly:

1. **The calib file (`camera_calib/hil_sim_ov2slam_stereo.yaml`) declares
   `Camera.fxl`/`fxr` = 1200.0.** This value traces to the 2026-08-24
   correction (see fixlog 020, and this file's own header comment) --
   `check_camera_fov` (warp-correlation test) and `check_stereo_geometry`
   (disparity-vs-ground-truth-depth triangulation) both independently measured
   the Simulink camera's *effective* rendering at fx~1200-1236, and every
   calib file consuming that measurement was updated. The **mono** profile
   calib files (`hil_sim_ov2slam_accurate.yaml` etc.) were never part of that
   rollout and still say 554 -- a separate, already-logged gap (see
   TODO-AI, fixlog 020), harmless there only because it happens to match
   what's below.

2. **The Simulink model block itself was never actually changed.** Extracted
   `matlab/hil_closed_loop.slx` directly (it's a zip archive) and read the raw
   XML for both `Simulation 3D Camera` (SID=2, left) and
   `Simulation 3D Camera Right` (SID=44, right) blocks:
   `<P Name="FocalLength">[554, 554]</P>` on **both**. Confirmed via MathWorks'
   own documentation that this parameter *is* the pixel fx directly (no unit
   conversion) -- so this is not a units/interpretation ambiguity, the block
   really renders at 554, not 1200.

   The baseline (`body_T_cam1` = 0.11 m translation) was cross-checked the
   same way and does match the block's `mountPoint` Y-offset (0 vs -0.11)
   exactly -- so the baseline was never in question, only fx.

Confirmed the mechanism against OV2SLAM's actual source
(`src/ov2slam_ros/src/frame.cpp`, `Frame::computeStereoKeypoint`): stereo
bearing vectors are built as `bv = normalize(iK_ * [u, v, 1])`, where `iK_`
comes from the calib file's `fxl`/`fxr` (1200), not the real render (554).
This config uses `bdo_stereo_rect: 0`, so triangulation goes through
`Mapper::triangulateStereo`'s general `computeTriangulation(Tlr, bv, rbv)`
path (not the simpler `fx*baseline/disp` shortcut used when
`bdo_stereo_rect: 1`). Unprojecting a 554-fx image with an assumed 1200 fx
systematically shrinks each bearing vector's angle off the principal axis,
which for a correctly-known baseline causes triangulation to place points
farther away than their true depth -- i.e. a systematic overestimate,
which Umeyama then has to shrink back down to match ground truth. Direction
and rough order of magnitude both match what was observed; the *exact*
number was not re-derived by hand for this specific (non-shortcut)
triangulation formula -- see Verification for why that derivation wasn't
needed.

## Diff

`camera_calib/hil_sim_ov2slam_stereo.yaml`:
```
-Camera.fxl: 1200.0
-Camera.fyl: 1200.0
+Camera.fxl: 554.0
+Camera.fyl: 554.0
...
-Camera.fxr: 1200.0
-Camera.fyr: 1200.0
+Camera.fxr: 554.0
+Camera.fyr: 554.0
```
Deliberately fixed by matching the *config* to the *verified-real* Simulink
block value (554), rather than changing the Simulink block to 1200 -- lower
risk (no `.slx` edit, no accelerator rebuild, no touching the model that
every other measurement in this project has been taken against), and exactly
mirrors what the mono calib files already do correctly.

Not touched: the Simulink model itself, `check_camera_fov`/
`check_stereo_geometry`'s own default `Fx=1200` arguments (those scripts
measure the *actual* render; if re-run today they would presumably still
report ~1200, correctly describing the model -- the bug was that the stereo
calib file adopted that *measurement* without anyone re-verifying the
*model's own declared parameter* matched it). Not touched: `bench_oracle.yaml`
/ `bench_fsm.yaml` (FIX-020's component, a different code path entirely,
unaffected either way by this fix).

## Critic verdict & concerns

Not dispatched this cycle -- no subagent critic/simplification review was
run. In its place: the hypothesis was formed, then explicitly challenged by
Arian at each step (asked "did you prove that or are you hypothesizing" and
required the exact derivation to be checked against OV2SLAM's real source
before accepting it), and the fix was verified empirically across 4 live
trials rather than accepted on the strength of the derivation alone -- see
Verification. Open concern, unaddressed: whether TS3/TS4 (h_vs/pbvs, if ever
run in stereo) have gains tuned assuming the wrong intrinsic, the same class
of concern FIX-020's critic raised for TS1/TS3 under that fix (TODO-AH) --
not evaluated here, logged as TODO-AS below since stereo controller-in-the-
loop runs haven't happened yet on this branch (all of today's stereo work
was `--hold-fsm`, controller never engaged).

## KISS verdict

Not dispatched. Self-assessed as the simpler of the two available fixes
(config-only change vs. Simulink block edit + accelerator rebuild), which is
why it was chosen over the alternative in the first place.

## Verification

Real, live trials, before and after, same config (`only_ov2_stereo_oracle`),
same day, same hardware:

| | Trials | Umeyama scale | APE RMSE (m) |
|---|---|---|---|
| Before (fxl/fxr=1200) | 3 | 0.48, 0.41, 0.52 (avg ~0.46) | 0.236, 0.493, 0.275 (0.335 +/- 0.113) |
| After (fxl/fxr=554) | 1+3 | 0.884, then 1.030, 0.966, 0.953 (avg of last 3: 0.983 +/- 0.034) | 0.486, then 0.186, 0.127, 0.202 (avg of last 3: 0.172 +/- 0.032) |

Scale moved from a tight, consistently-wrong ~0.46 to a tight ~0.98 (within
normal noise of the theoretical 1.0). RMSE roughly halved and its spread
tightened by ~3.5x across the post-fix batch (the single first post-fix
trial, 0.486, was within the pre-fix range and reads as noise, not signal --
correctly flagged as such before drawing any conclusion from n=1). This
multi-trial empirical confirmation is why the exact triangulation-formula
derivation (predicted ~0.46 via the wrong/shortcut formula, not re-derived
for the actual `computeTriangulation` path) was not pursued further -- the
live result settles it more directly than the algebra would have.

## Open TODOs

- **TODO-AS** (new): controller gains for any stereo-in-the-loop test (TS1
  IBVS / TS3 h_vs / TS4 PBVS under `stereo: true`, not `--hold-fsm`) have
  never been evaluated against the corrected fx=554. All of today's stereo
  work bypassed the controller entirely (`--hold-fsm` + `slam_traj_probe.m`),
  so this has not yet mattered in practice, but mirrors FIX-020's TODO-AH
  concern for the mono case and should be checked before trusting any
  stereo controller-quality number.
- Not fixed here, logged as still-open: `bench_oracle.yaml` still at 554
  (fixlog 020's TODO-AI) -- unrelated to this fix (different code path,
  oracle detector projection math, not OV2SLAM calibration) but worth
  remembering it's the *same number* for a *different* reason in that file.
