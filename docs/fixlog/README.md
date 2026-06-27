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
| [005](005-vx-unbounded.md) | 2026-06-27 | vx unbounded → overshoot; vy still diverges | **paused** |
| [004](004-depth-from-bbox.md) | 2026-06-27 | Replace solvePnP with depth-from-bbox; drop ViSP | superseded by 005 |
| [003](003-remove-rotation-feature.md) | 2026-06-26 | Remove ftu_, zero wz, reduce lambda | superseded by 004 |
| [002](002-phantom-rotation.md) | 2026-06-26 | Cancel phantom Rx(180°) in cdMc | superseded by 003 |
| [001](001-cdmc-order.md) | 2026-06-26 | cdMc matrix multiply order inverted | resolved |

## Open TODOs across all fixes

- **TODO-H** (005, active): re-enable wz to track bearing, OR diagnose why TS2 survives
  the same geometry without yaw.
- TODO-A/B/C/D/E/F/G — see individual entries; most superseded by the depth-from-bbox
  rewrite (004) which removed solvePnP and ViSP entirely.
