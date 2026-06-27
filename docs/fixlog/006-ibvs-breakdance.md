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
  persisted → `vpServo::MEAN` applied (step 2). Oscillation persisted → decoupled fix
  applied (step 3, see below). Retest on Pi pending.
- **TODO-J**: ~~RESOLVED~~ `Z_des = Z_cur` in decoupled fix; Z is no longer computed
  from desired box height at all.

### Step 3 — Decoupled approach (applied after MEAN+lambda still oscillated)

**Root cause confirmed from Pi logs**: at ratio=0.042, `vx=+152 m/s` (13× size mismatch
dominates L^+), `wz=+0.000` throughout 18s approach (`yaw=6.50` constant across all
diag ticks). IBVS assigns near-zero weight to Wy (yaw) channel because size error vector
is orthogonal to yaw in L^+ — not a noise problem, a structural one.

**Fix (critic: do not merge wz override → re-diagnose → decouple)**:
- Change desired features from `centered_box(target_bbox_ratio)` → `centered_box(in.bbox_ratio)`:
  desired = same size as current, centred → feature error is CENTERING only, zero size error.
  L^+ is now well-conditioned; vy/vz/wz get proper budget.
- `vel.vx = k_fwd_ * max(0, target_bbox_ratio - bbox_ratio)`: proportional range control,
  decoupled from IBVS centering.
- `k_fwd: 3.0` added to `bench_ibvs.yaml`; `lambda` default baked to 0.3 in constructor.
- `Z_des = Z_cur` (trivially, since desired box = current size).
