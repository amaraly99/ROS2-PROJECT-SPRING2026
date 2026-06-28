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

- **TODO-L/M/N/O** (009, active): SLAM-depth IBVS caveats — mono scale (depth_scale can't model drift),
  background-vs-target bias in bbox-median depth, warmup Z-step + tracking-lost staleness, and A/B
  cloud-load fairness. Run the bbox-vs-SLAM-depth A/B and resolve scale before trusting numbers.
- **TODO-K** (006, active): Residual oscillation after decoupled fix — diagnose if lambda needs further tuning or if close-range detection dropout is the remaining driver.
- **TODO-H** (005, active): re-enable wz to track bearing, OR diagnose why TS2 survives
  the same geometry without yaw.
- TODO-A/B/C/D/E/F/G — see individual entries; most superseded by the depth-from-bbox
  rewrite (004) which removed solvePnP and ViSP entirely.
