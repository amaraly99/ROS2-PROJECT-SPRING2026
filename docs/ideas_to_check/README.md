# Ideas to Check

Open hypotheses and candidate fixes that came out of a discussion but were **not**
implemented yet — distinct from [`docs/fixlog/`](../fixlog/), which only holds fixes
that were actually applied (SECOND BRAIN protocol, `CLAUDE.md`).

Use this directory to park a line of investigation mid-thought: what was tried, what
got ruled out, what's still open, and what the next concrete step would be. When an
idea here graduates to an actual code change, run it through the SECOND BRAIN
protocol and move the writeup to `docs/fixlog/` (or delete the entry here if it was a
dead end).

## Naming

`NNN-short-kebab-title.md` — same numbering convention as fixlog, separate sequence.

## Index (newest first)

| ID | Date | Title | Status |
|----|------|-------|--------|
| [002](002-searching-yaw-settle-oscillation.md) | 2026-07-02 | SEARCHING yaw-settle oscillation — YAW_RIGHT_60/LEFT_60/CENTER never converges, blocks FIX-011 end-to-end | **open** |
| [001](001-orbslam2-monocular-init-reliability.md) | 2026-07-02 | ORB-SLAM2 monocular init reliability (CPU pinning falsified, FSM gating idea) | **graduated → fixlog/011** |
