---
id: FIX-001
title: cdMc matrix multiplication order inverted
date: 2026-06-26
status: resolved
component: src/visp_pbvs_servo/src/pbvs_controller.cpp
critic_verdict: partial
kiss_verdict: simplify-recommended
open_todos: [TODO-A, TODO-B]
---

## Symptom

During PBVS APPROACHING, the drone actively moves AWAY from the sign:
- Sign is LEFT of image center (ex = -0.18 → -0.79, growing more negative)
- Drone commanded RIGHT (vy = -0.174 → -0.710, increasingly negative)
- Drone yaws RIGHT (wz = -0.050 → -0.210, increasingly negative)
- Sign exits FOV after ~4s → REACQUIRE

Every correction amplifies the error — classic diverging feedback loop.

## Root cause

`cdMc` is the pose-error matrix fed into ViSP's PBVS feature objects
(`vpFeatureTranslation::cdMc`, `vpFeatureThetaU::cdRc`). The ViSP convention is:

```
cdMc = cdMo * cMo.inverse() = cMo_desired * cMo_current.inverse()
```

The code had it inverted (`cMo_desired.inverse() * cMo_current`), which computes a
garbage frame chain and flips the sign on all translation errors. Sign LEFT
(tx_current < 0) → wrong formula gives +tx → servo commands move RIGHT → diverges.

## Diff

```diff
- vpHomogeneousMatrix cdMc = cMo_desired_.inverse() * cMo_current;
+ vpHomogeneousMatrix cdMc = cMo_desired_ * cMo_current.inverse();
```

## Critic verdict & concerns

**Partial.** Matrix order is mathematically right, but two concerns remain:
1. **object_pts Y-axis mismatch (medium):** 3D points use Y-up; OpenCV solvePnP is
   Y-down → recovers a phantom Rx(180°) corrupting `cMo_current` rotation. (→ FIX-002)
2. **v[1] discarded; altitude hybrid (high):** `vel.vz = -0.5*ey_norm` bypasses ViSP's
   computed altitude, creating a conflicting hybrid controller.

## KISS verdict

Almost simple. Recommended renaming `cMo_desired_` → `cdMo_` so the line transcribes
the ViSP doc formula directly. (Rendered moot by FIX-004, which removed ViSP.)

## Open TODOs

- **TODO-A:** Verify object_pts Y convention (→ addressed in FIX-002).
- **TODO-B:** Decide ViSP-handled altitude vs. ey_norm term (→ resolved in FIX-004).
