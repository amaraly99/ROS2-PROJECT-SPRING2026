# Fix Log — "SECOND BRAIN" archive

One markdown file per non-trivial fix, ADR-style (Architecture Decision Record).
These are **not** auto-loaded into the LLM context — they are read on demand when a
fix is relevant. This keeps `CLAUDE.md` small while preserving the full reasoning
trail for any human or LLM who comes after.

The protocol for *making* a fix (including the critic + simplification agents) lives
in [`../../CLAUDE.md`](../../CLAUDE.md). This directory is the *output* of that protocol.

## Naming

`NNN-short-kebab-title.md` — `NNN` zero-padded, monotonic. Newest at top of the index.

## Frontmatter template

```markdown
---
id: FIX-NNN
title: <short title>
date: YYYY-MM-DD
status: resolved | partial | paused | superseded
component: <package or file>
supersedes: FIX-NNN        # optional
critic_verdict: correct | partial | wrong
kiss_verdict: simple | simplify-recommended
open_todos: [TODO-x, TODO-y]   # optional
---

## Symptom
## Root cause
## Diff
## Critic verdict & concerns
## KISS verdict
## Open TODOs
```

## Index (newest first)

| ID | Date | Title | Status |
|----|------|-------|--------|
| [015](015-slam-eval-evo-modular.md) | 2026-07-05 | SLAM eval → modular per-SLAM package (evo ATE, paper plots) + OV2SLAM front-end timing + fast/accurate | **resolved** |
| [014](014-ov2slam-init-gate-support.md) | 2026-07-05 | INITIALIZER_GATE support for OV2SLAM — bvision_init_ republished as /slam/tracking_state | **resolved** |
| [013](013-slam-coverage-after-fsm.md) | 2026-07-04 | slam_coverage_after_fsm_pct — SLAM tracking-coverage sanity check post-FSM-start | **resolved** |
| [012](012-stale-orbslam2-wrapper-binary.md) | 2026-07-02 | Stale ORB-SLAM2 ROS2 wrapper binary — /slam/tracking_state was never published | **resolved** |
| [011](011-slam-init-gate.md) | 2026-07-02 | SLAM-init gate — blind search until ORB-SLAM2 initializes (opt-in) | **resolved** |
| [010](010-slam-pose-integration.md) | 2026-06-29 | SLAM pose → IBVS controller (staleness gate + ORB-SLAM2 confidence + bag stop fix) | **done** |
| [009](009-slam-depth-ibvs.md) | 2026-06-28 | Wire any SLAM pose+map-points into ViSP IBVS depth (dead path revived) | **partial** |
| [008](008-cpu-pinning-regression.md) | 2026-06-27 | CPU pinning regression — yolo_bridge co-located with controller | resolved |
| [007](007-deployment-bugs.md) | 2026-06-27 | Sidecar deployment — taskset list prefix + missing --entrypoint | resolved |
| [006](006-ibvs-breakdance.md) | 2026-06-27 | IBVS breakdances — saturates, oscillates (3 steps, partial) | **partial** |
| [005](005-vx-unbounded.md) | 2026-06-27 | vx unbounded → overshoot; vy still diverges | **paused** |
| [004](004-depth-from-bbox.md) | 2026-06-27 | Replace solvePnP with depth-from-bbox; drop ViSP | superseded by 005 |
| [003](003-remove-rotation-feature.md) | 2026-06-26 | Remove ftu_, zero wz, reduce lambda | superseded by 004 |
| [002](002-phantom-rotation.md) | 2026-06-26 | Cancel phantom Rx(180°) in cdMc | superseded by 003 |
| [001](001-cdmc-order.md) | 2026-06-26 | cdMc matrix multiply order inverted | resolved |

## Open TODOs across all fixes

- **TODO-AB/AC** (015, active): fast/accurate OV2SLAM configs omit upstream
  dop3p/fkf_filtering_ratio differences (AB); the two calib files duplicate ~133
  hand-synced lines (AC).
- **TODO-Y/Z** (014, active): stereo OV2SLAM + init_gate would deterministically fail every
  run (Y, no config exists yet); TRANSIENT_LOCAL QoS for guaranteed late-joiner delivery on
  OV2SLAM's tracking_state topic (Z, currently relies on per-frame republish instead).
- **TODO-V/W** (013, active): `slam_coverage_after_fsm_pct` methodology risks —
  raw path-length jitter bias + reused-global-scale on a re-localized SLAM
  segment could mask exactly the degenerate case it's meant to catch (V); no
  shared upper time bound between GT/SLAM windows compared (W).
- **TODO-S/T/U** (011, active): SLAM-init gate follow-ups — verify 3-strafe budget vs
  15s startup on the Pi (S), evaluate the RELOCATE full-rotate cost / mono-map stability (T),
  distinguish "SLAM never up" from "SLAM up but failed to init" in the FAILED marker (U).
- **TODO-P** (010, deferred): Option 3 weighted fusion — monocular scale drift makes raw translation
  blending unreliable. Revisit after GT-anchored scale correction (TODO-L). Only yaw is safe to blend.
- **TODO-L/M/O** (009, active): SLAM-depth IBVS caveats — mono scale (depth_scale can't model drift),
  background-vs-target bias in bbox-median depth, and A/B cloud-load fairness.
  TODO-N (staleness) is closed by FIX-010.
- **TODO-K** (006, active): Residual oscillation after decoupled fix — diagnose if lambda needs further tuning or if close-range detection dropout is the remaining driver.
- **TODO-H** (005, active): re-enable wz to track bearing, OR diagnose why TS2 survives
  the same geometry without yaw.
- TODO-A/B/C/D/E/F/G — see individual entries; most superseded by the depth-from-bbox
  rewrite (004) which removed solvePnP and ViSP entirely.
