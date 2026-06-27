---
id: FIX-005
title: vx unbounded → overshoot; vy still diverges (PAUSED)
date: 2026-06-27
status: paused
component: src/visp_pbvs_servo/, config/hil/bench_pbvs.yaml
supersedes: FIX-004
critic_verdict: partial
kiss_verdict: simple
open_todos: [TODO-F, TODO-G, TODO-H]
---

## Symptom

After FIX-004, drone moves correct direction but overshoots at full speed:
- `vx=+16.176 m/s` on first APPROACHING tick (sign at 35m, 8.5° off-nose).
- `ex=-0.18 → -0.84` over 4.9s — sign bleeds off left edge despite correct vy.
- REACQUIRE after 4.9s.

## Root cause

`vel.vx = lambda*(depth-standoff)` is **unbounded**: at depth=35m, lambda=0.5,
vx = 0.5*32 = 16 m/s. The off-center sign's bearing drifts faster than vy (≈1.85 m/s)
can compensate. (Pi yaml still had lambda=0.5 — the FIX-003 0.3 was never pushed.)
vy was also unbounded at long range (ex=-0.9 → tx≈18m → vy≈5.4 m/s).

## Diff

```diff
# bench_pbvs.yaml:
- lambda: 0.5   (+ removed sign_width)
+ lambda: 0.3   # lateral depth-scaled gain
+ k_fwd:  3.0   # forward bbox-ratio gain
# computeApproach():
- vel.vx = lambda_ * (depth - standoff_);                 // 16 m/s at 35m
+ vel.vx = k_fwd_ * std::max(in.target_bbox_ratio - in.bbox_ratio, 0.0);  // bounded
- vel.vy = -lambda_ * tx;
+ vel.vy = std::clamp(-lambda_ * tx, -1.5, 1.5);
# header: removed init(), standoff_, sign_height_
```

## Critic verdict & concerns

**Partial.** Fixes the vx explosion + caps vy. Concerns: vx cap (k_fwd*0.55 ≈ 1.65)
may still be aggressive vs TS2 ~1.5; Z guard at 0.5m may be redundant with FSM gating.

## KISS verdict

Simple — bbox-ratio formula is cleaner than a capped metric term; class ~25 lines.

## Status: PAUSED — vy still diverges

Despite the caps, ex still drifts -0.18 → -0.85 over 8s (bearing 2.7° → 30.3°). vx is
now correct (~1.5 m/s) but with **wz=0 locked the heading never tracks the sign**, so
an off-center target bleeds off the edge geometrically as range closes. Flagged and
parked.

## Open TODOs

- **TODO-H (active, next session):** Either (a) re-enable yaw to track bearing
  (`vel.wz = -k_yaw * in.ex_norm`) so the nose stays on the sign during approach, OR
  (b) diagnose why TS2 survives the same geometry without wz.
- **TODO-F:** try k_fwd=1.5 (slower forward → more time for lateral correction).
- **TODO-G:** check vy clamp ±1.5 is enough when sign starts far off-center.
