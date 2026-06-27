---
id: FIX-004
title: Replace solvePnP with depth-from-bbox; drop ViSP servo entirely
date: 2026-06-27
status: superseded
component: src/visp_pbvs_servo/ (controller.hpp + controller.cpp)
supersedes: FIX-003
critic_verdict: correct
kiss_verdict: simple
open_todos: [TODO-E]
---

## Symptom

After FIX-001/002/003, drone still zigzags and loses focus. Lambda reduction and
zeroing wz had no effect on the oscillation.

## Root cause

`cv::solvePnP` on 4 coplanar symmetric bbox corners is **geometrically degenerate** —
multiple valid pose solutions, and the solver switches between them frame-to-frame →
`tx` jitters → `vy` oscillates. A bad pose estimator cannot be fixed by gain/feature
tuning.

## Diff

Bypass solvePnP. Estimate depth + lateral position directly from the bbox (pinhole
model). The ViSP translation-only servo law reduces exactly to `vc = -λ·[tx,ty,tz]`,
so ViSP itself is removed.

```diff
# header: removed OpenCV + all ViSP includes, vpServo, vpFeature*, cMo_desired_
+ double sign_height_; double standoff_;

# computeApproach() (full rewrite):
+ double depth = in.fy * sign_height_ / in.bh;
+ double tx    = (in.cx - in.cx0) / in.fx * depth;
+ vel.vx =  lambda_ * (depth - standoff_);
+ vel.vy = -lambda_ * tx;
+ vel.vz = -0.5 * in.ey_norm;
+ vel.wz =  0.0;
```

## Critic verdict & concerns

**Correct, high confidence.** Depth formula assumes sign roughly parallel to image
plane; degrades gracefully at moderate angles, only breaks edge-on. Not a benchmark
concern.

## KISS verdict

This IS the KISS fix. `vpServo + vpFeatureTranslation` with R=I + translation-only is
mathematically identical to `vel = -λ·error`; keeping ViSP was false complexity.

## Open TODOs

- **TODO-E:** At long range bh is small (20-30 px); 1-px jitter → ~4% depth/tx jitter.
- **TODO-B resolved:** solvePnP + ViSP removed; vz heuristic is now the only altitude
  controller (no dual loop).

## Note

This made TS4 functionally equivalent to TS2 (proportional) but with **metric
depth-scaled** lateral gain vs. pixel-space gain. The unbounded `vel.vx` here caused
the next failure → see FIX-005.
