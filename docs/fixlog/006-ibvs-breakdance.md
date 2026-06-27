---
id: FIX-006
title: IBVS (TS1) breakdances — saturates velocity commands, oscillates around sign
date: 2026-06-27
status: partial
component: visp_servo / config/hil/bench_ibvs.yaml
critic_verdict: partial
kiss_verdict: simplify-recommended
open_todos: [TODO-I, TODO-J]
---

## Symptom

ViSP IBVS controller (TS1) saturates `max_linear=3.0` and `max_angular=0.8` repeatedly
during APPROACHING, causing the drone to oscillate ("breakdance") instead of converging
on the sign. Observed during `--config ov2slam_ibvs` integration test.

## Root cause

Two candidates — critic rated saturation windup as more likely than interaction matrix noise:

**Candidate A (high likelihood): lambda-saturation windup.**
`lambda=0.5` produces unclamped commands that exceed FSM hard clamps (`max_linear=3.0`).
When the drone moves less than commanded, the next tick sees a larger residual error →
even larger command → oscillates. Classic integrator-free bang-bang from saturation + lag.
`vpServo::CURRENT` vs `MEAN` does not affect this loop.

**Candidate B (low likelihood): `vpServo::CURRENT` interaction matrix noise.**
YOLO bbox is noisy → L recomputed from noisy corners every tick → `L⁺` flips direction
→ `v = -λ·L⁺·e` swings tick-to-tick. Secondary contributor, not root cause.

**Secondary contributor confirmed: `Z_cur` noise.**
Bbox-ratio-derived depth is ±20% noisy → every element of L contains `1/Z`, so Z noise
propagates directly into the velocity command magnitude regardless of CURRENT vs MEAN.

## Diff

Step 1 applied (yaml-only, zero code change):
```diff
# config/hil/bench_ibvs.yaml
-    lambda: 0.5
+    lambda: 0.3
```
Pi result: oscillation less severe but persisted — confirmed Candidate A partial, Candidate B still active.

Step 2 applied (TODO-I resolved):
```diff
# src/visp_servo/src/ibvs_controller.cpp
-    servo_.setInteractionMatrixType(vpServo::CURRENT);
+    servo_.setInteractionMatrixType(vpServo::MEAN);
```

## Critic verdict & concerns

**Uncertain** — fix is directionally correct but targets the least likely root cause.
Main concern: saturation windup (Candidate A) is the dominant driver of breakdancing.
`MEAN` reduces linearization noise but does not prevent the FSM hard-clamp from breaking
the servo's motion model. Recommended: diagnose saturation rate first (lambda reduction),
then apply MEAN if oscillation persists.

Additional concern: `vpServo::MEAN` can be worse than `CURRENT` at large initial errors
(when L(current) and L(desired) differ substantially). Also: confirm `s_tl_d_.set_Z(Z_des)`
is called with a sensible value — if ViSP defaulted `Z=1.0` on desired features, MEAN
would blend a reasonable L(cur) with a garbage L(des).

## KISS verdict

Simplify recommended. The lambda reduction (yaml only) is the simpler first diagnostic
step and was skipped. Correct order: lambda first → MEAN if needed → not both at once.

## Open TODOs

- **TODO-I**: ~~RESOLVED~~ Pi test confirmed lambda=0.3 reduced severity but oscillation
  persisted → `vpServo::MEAN` applied (step 2). Retest on Pi pending.
- **TODO-J**: Verify `s_tl_d_` desired features have `Z_des` explicitly set (not ViSP
  default Z=1.0). Check `ibvs_controller.cpp` `update_corner_features` call for desired
  corners — `Z_des` is computed via `depth_for_box_height(cam_, known_height_, des.y2-des.y1)`.
  Confirm `known_target_height` is set correctly in `bench_fsm.yaml`.
