---
id: FIX-003
title: Remove rotation feature (ftu_), zero wz, reduce lambda
date: 2026-06-26
status: superseded
component: src/visp_pbvs_servo/src/pbvs_controller.cpp, config/hil/bench_pbvs.yaml
supersedes: FIX-002
critic_verdict: partial
kiss_verdict: simple
open_todos: [TODO-D, TODO-B]
---

## Symptom

After FIX-001 + FIX-002, drone moves toward sign (correct direction) but zigzags
severely and loses focus on arrival across all runs.

## Root cause

1. **`ftu_` rotation feature is oscillatory:** solvePnP on 4 coplanar symmetric bbox
   corners is geometrically degenerate for rotation. FIX-002 removed the *systematic*
   Rx(180°) but not the *stochastic* rotation noise.
2. **`vel.wz = -v[5]` unsafe with translation-only task:** with only `ft_` registered,
   the 3×6 Jacobian's pseudo-inverse puts non-zero minimum-norm rotational velocity in
   v[5] — uncontrolled yaw noise.
3. **lambda=0.5 amplifies solvePnP translation jitter** into visible zigzag.

## Diff

```diff
# init():
  servo_.addFeature(ft_);
- servo_.addFeature(ftu_);   // removed — noisy rotation from degenerate bbox PnP
# computeApproach():
- vel.wz = -v[5];            // pseudo-inverse garbage with 3-row task
+ vel.wz = 0.0;              // yaw locked
# bench_pbvs.yaml:
- lambda: 0.5
+ lambda: 0.3
```

## Critic verdict & concerns

**Partial.** Critic confirmed v[5] is NOT near-zero with translation-only task (KISS
agent was wrong about this — explicit `vel.wz = 0.0` is necessary). But primary zigzag
driver is likely solvePnP *translation* noise, which lambda reduction only damps.

## KISS verdict

Minimal — 3 trivial changes. (Note: KISS agent's v[5]=0 claim was overridden by critic.)

## Open TODOs

- **TODO-D:** Log raw solvePnP tvec; if tx still jitters, solvePnP geometry is the
  fundamental problem → bypass it (→ done in FIX-004).
- **TODO-B:** ey_norm altitude heuristic still decoupled (→ resolved in FIX-004).
