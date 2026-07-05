---
id: FIX-013
title: slam_coverage_after_fsm_pct — SLAM tracking-coverage sanity check post-FSM-start
date: 2026-07-04
status: resolved
component: benchmarks/eval_slam_hil.py
critic_verdict: partial
kiss_verdict: simple
open_todos: [TODO-V, TODO-W]
---

## Symptom

Not a bug — a gap. `eval_slam_hil.py` already reports ATE RMSE as a % of two
path-length denominators (`ATE_RMSE_full_pct`, `ATE_RMSE_eval_pct`), but nothing
catches the case where a low ATE RMSE is **deceptive**: SLAM loses tracking early
in the real mission (post-`INITIALIZER_GATE` warmup) and only produces a handful
of points that happen to align tightly with GT — RMSE looks great, but SLAM
"only walked 5% of the path." User's own framing: a coarse sanity check, not a
publication-grade metric.

## Root cause

No existing metric distinguishes "SLAM tracked accurately" from "SLAM tracked a
tiny fraction of the flight accurately." The bag also starts recording *before*
the FSM launches (the `INITIALIZER_GATE` mirror-warmup happens first, ~15-20s),
so any raw "did SLAM move" check needs to explicitly exclude that pre-mission
window or it's comparing the wrong thing.

## Diff

1. `read_bag()` now also collects `/bench/state` (`std_msgs/String`) as
   `(t, data)` tuples — 3-tuple return instead of 2.
2. New `find_fsm_start(bench_states)`: first `/bench/state` message *containing
   a comma*. The FSM (`servo_fsm_node.cpp:812-824`, `publish_bench_state`)
   always publishes `"<STATE>,<sim_time>,<controller>,<dist>"`; `init_gate`
   (`init_gate/cycle.py`) only ever publishes bare `SLAM_READY` /
   `SLAM_INIT_FAILED` (no commas) before handoff. Verified no other
   `/bench/state` publisher exists in the codebase. Returns `None` if the FSM
   never launched (e.g. bag only has a failed gate).
3. In `main()`, after Umeyama alignment (which already produces scale `s`):
   restrict `gt_arr`/`slam_arr` (full unmasked bag arrays) to `t >= fsm_start_t`,
   compute `gt_path_length_fsm` (GT distance actually flown post-FSM-start) and
   `slam_path_length_fsm` (SLAM's own raw path length over that window, scaled
   by `s` into metric units — same `sum(‖Δ‖)` idiom already used twice in this
   file for the other two path lengths). `slam_coverage_pct =
   slam_path_length_fsm / gt_path_length_fsm * 100`.
4. Guard added after critic review (see below): if `gt_path_length_fsm < 0.05m`
   (FSM marker lands at/near the last GT sample), report `None` instead of
   dividing — the original `max(gt_path_length_fsm, 1e-6)` floor only prevented
   a crash, not a nonsensical blown-up percentage when GT≈0 but SLAM≠0 in that
   window.
5. Three new `slam_metrics.csv` / printed fields: `fsm_gt_path_length_m`,
   `fsm_slam_path_length_m`, `slam_coverage_after_fsm_pct` (all `None` together
   if no FSM marker or degenerate window, with a one-line printed reason).

Validated against 3 real Pi bags: known-good 13cm-ATE run →
`slam_coverage_after_fsm_pct=106.22` (plausible — SLAM's scaled path length
tracks slightly over real GT distance, consistent with normal per-frame
jitter, not a red flag); `gatetest1` → `64.13` (plausible partial-tracking
signal); `gatetest2` → both GT and SLAM path length ≈0 in-window, now
correctly reports `None` with a printed reason (previously reported a
coincidental `0.0`, which was itself a symptom of the bug fixed in point 4).

## Critic verdict & concerns

**Verdict: directionally right, one real bug (fixed), two open methodological
concerns (logged as TODOs, not blocking).**

Fixed during review: the GT-path-near-zero-but-SLAM-nonzero case could blow the
ratio up to an arbitrarily large, meaningless percentage rather than the clean
"both zero" case the tested bags happened to hit — only one of the two
zero-denominator code paths had actually been exercised. Now guarded (point 4
above).

Not fixed, logged as open concerns:
- **TODO-V**: `slam_path_length_fsm` is a *raw, unsmoothed* sum of consecutive
  monocular pose deltas — noisy pose streams accumulate spurious path length
  from jitter alone (coastline-paradox effect), independent of real tracked
  motion. This is scaled by a single Umeyama `s` fit over the *entire* ATE
  window, not the post-FSM segment specifically — if SLAM re-localizes with a
  new arbitrary scale mid-mission, the reused global `s` has no reason to be
  correct for that segment. In the worst case a degenerate/frozen post-loss
  pose stream could still read near 100% coverage, which would defeat the
  purpose of this exact metric. Not fixed now because doing so properly
  (resampling both streams to common timestamps, and/or per-segment scale
  refitting) is a meaningfully bigger job than the "coarse gut-check" the user
  asked for — but a *very* degenerate real failure (near-zero real motion)
  should still read as clearly low, since jitter alone is unlikely to fabricate
  tens of meters of apparent path length.
- **TODO-W**: `gt_fsm`/`slam_fsm_raw` are each bounded only from below
  (`t >= fsm_start_t`), with no shared upper bound — if the two streams' tails
  end at different wall-clock times (SLAM node outlives GT recording or vice
  versa), the two path lengths being ratioed aren't strictly over the same
  span. Low practical impact for well-behaved bags (tail misalignment is
  normally sub-second), not fixed now.

Also confirmed clean during review (no action needed): the comma heuristic is
currently safe (only two `/bench/state` publishers exist, structurally
distinct formats); non-gated configs degrade correctly (FSM's first message is
near bag start, `slam_coverage_after_fsm_pct` ≈ the old full-path behavior);
CSV `None` handling is harmless (blank cell, no crash, no existing aggregator
reads this file yet).

## KISS verdict

**Simple — no changes recommended.** Reuses the existing path-length idiom
(`np.sum(np.linalg.norm(np.diff(...), axis=1))`) already used twice in this
file rather than inventing a new method. The comma-heuristic is right-sized —
the two message formats are structurally stable (defined in
`servo_fsm_node.cpp`, not user input), and a stricter allowlist/field-count
check would add no real safety. Threading `/bench/state` through `read_bag()`'s
existing single-pass topic loop (rather than a second bag scan) is the
minimal-diff choice. Three near-identical `None`-guarded metrics-dict lines is
fine at this scale — no helper warranted.

## Open TODOs

- **TODO-V** (this entry): raw path-length jitter bias + reused-global-scale
  risk on `slam_coverage_after_fsm_pct` — could mask exactly the degenerate
  case this metric exists to catch, if SLAM re-localizes to a different scale
  mid-mission. Revisit if a real run ever shows a suspiciously high coverage %
  alongside other signs of lost tracking (e.g. very low front-end Hz, high ATE
  max).
- **TODO-W** (this entry): no shared upper time bound between the GT and SLAM
  windows used for `slam_coverage_after_fsm_pct` — low practical impact,
  revisit only if a real bag shows a large SLAM/GT tail misalignment.
