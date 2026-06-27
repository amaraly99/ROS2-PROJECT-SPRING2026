---
id: FIX-002
title: Cancel phantom Rx(180°) in cdMc (zigzag + yaw overshoot)
date: 2026-06-26
status: superseded
component: src/visp_pbvs_servo/src/pbvs_controller.cpp
supersedes: FIX-001
critic_verdict: partial
kiss_verdict: simple
open_todos: [TODO-B, TODO-C]
---

## Symptom

After FIX-001 the drone moves TOWARD the sign (lateral fixed), but:
- Excessive zigzagging throughout APPROACHING — much worse than TS1/TS2.
- On arrival, overshoots and looks further LEFT — sign exits FOV.

## Root cause

`object_pts` uses Y-up (`top-left = (-w/2, +h/2, 0)`). OpenCV solvePnP is Y-down, so
for a face-on sign it can only fit the corners by recovering R = Rx(180°). This
phantom rotation is baked into every `cMo_current`. Since `cMo_desired_` was built
with R=I, `cdMc` inherits the full Rx(180°); `vpFeatureThetaU(cdRc)` then sees
θu ≈ [π,0,0] as a permanent angular error → oscillatory cross-coupled velocity
(zigzag), and near the target the rotational term dominates → yaw overshoot.

## Diff

Absorb the convention at the desired-pose level so both sides of cdMc carry the same
Rx(180°) and it cancels (1 number vs. reordering 4 corner rows):

```diff
+ #include <cmath>
- cMo_desired_.buildFrom(0, 0, standoff, 0, 0, 0);
+ cMo_desired_.buildFrom(0, 0, standoff, M_PI, 0, 0);
```

## Critic verdict & concerns

**Partial.** Y-up diagnosis correct, fix cancels the systematic rotation. Remaining:
- `vel.vy = -v[0]` mapping re-flagged (counter: validated convergent in FIX-001).
- v[1] altitude hybrid still active (TODO-B).

## KISS verdict

Simplest possible — one number changed. Agent noted disabling `ftu_` entirely
(translation-only PBVS) as a fallback if rotation noise persists (→ taken in FIX-003).

## Open TODOs

- **TODO-B:** map v[1]→vz or drop ftu_ (→ FIX-003 dropped ftu_).
- **TODO-C:** verify zigzag gone next run (→ it was NOT; stochastic PnP noise remained → FIX-003/004).
