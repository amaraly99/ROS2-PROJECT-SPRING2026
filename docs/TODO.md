# TODO — 2026-07-04

Reviewed list per user request. Not a fixlog entry (nothing here is applied/verified
yet) — see `HANDOFF.md` at repo root for the full session context behind each item.

---

## 1. GT recording must start when the FSM starts, not before

**Current state:** `start_bag_recording()` runs right after `start_slam_sidecar()`,
*before* `INITIALIZER_GATE` even begins — this was a deliberate fix earlier this
session so a failed gate still leaves a real bag to diagnose (previously a failed
gate left only `meta.txt`, no bag at all). Side effect: `/sim/drone_pose` in the bag
now includes the entire gate warmup+SLAM-init window (~15-20s) before the FSM/
detector/controller ever launch, plus (already patched, see below) a handful of
Simulink's zero-initialized placeholder samples before the sim model is actually
running.

**Already patched today:** `eval_slam_hil.py` now filters exact-`(0,0,0)` GT samples
(Simulink's pre-run placeholder — no real telemetry is ever exactly zero given
non-zero ICs). This fixed a ~20m phantom jump in the trajectory plot.

**Still open — the actual ask:** the *bag* itself still records from before FSM
start, not from FSM start. Options to resolve, needs a decision:
- (a) Add an explicit marker for "FSM actually started" — e.g. the FSM's own first
  `/bench/state` message (format `"SEARCHING,<sim_t>,<controller>,..."`, distinct
  from the gate's one-shot `SLAM_READY`/`SLAM_INIT_FAILED` string) — and have
  `eval_slam_hil.py` report a *third* path-length/ATE window keyed to that marker,
  alongside the existing full/eval pair. Doesn't touch recording behavior, keeps the
  gate-failure diagnostic bag intact.
- (b) Actually delay `start_bag_recording()` until the FSM launches, and accept losing
  bag-based diagnostics on gate failure (back to the original problem this session
  fixed) unless something else covers that case.
- (c) Something else — needs your call, this wasn't decided, only diagnosed.

---

## 2. Commit changes

Checked fresh: turns out almost everything from this session is **already committed**
(7 incremental commits, same message "Added a INITIALIZER_GATE for online SLAM
benchmarking, pending test", both Windows and Pi HEAD = `16bb178`, in sync). Only
`benchmarks/eval_slam_hil.py` (today's zero-GT-sample filter, trajectory/CPU plot
restyle to match `SLAM_benchmark/images/`, new front-end-timing-per-frame plot) is
still uncommitted, on both machines identically (I `scp`'d it to the Pi directly to
test). One more commit + push (Windows) + pull (Pi) closes this out.

Also on the Pi only, deliberately left alone: an uncommitted `detector.cpu`/
`controller.cpu` pin in `config/hil/stack/orbslam2_eval.yaml` (user's own manual
edit — do NOT touch per standing instruction) and the usual rebuilt-ORB-SLAM2-binary
diffs (`src/orbslam2/build_stereo/...`, expected, unrelated).

Plot/CSV outputs sitting at repo root (`fig_slam_*.png`, `traj_xy_*.png`,
`slam_metrics.csv`) — per user's explicit call, NOT to be committed, just local.

`bags/` is gitignored on both sides — safe regardless of any commit/pull sequence.
Do not run `git clean -x` on either side (nukes bags/; already verified once this
session, see HANDOFF.md §Sync).

---

## 3. Run tagging for 10x benchmarking (needs Opus / a fresh, well-resourced session)

`run_stack_hil.sh` already has a `--run-tag` flag (from an earlier session,
`add7c90`). What's still missing: a **multi-run comparison tool** for SLAM
benchmarks — the HIL equivalent of `benchmarks/compare_controllers.py` (which
already aggregates N controller-benchmark runs into `cmp_*.png` + `summary.csv`).

This is directly motivated by today's investigation into `SLAM_benchmark/images/`
(an *offline, EuRoC, 10-sequence, multi-backend* statistical study —
`manifest.json` confirms the source). Most of that folder's plot types
(`*_rmse_per_sequence`, `*_by_sequence`, `pairwise_*`, `comparison_*_all_algorithms`)
are structurally impossible for a single HIL run — they need multiple runs (ideally
~10x, per the user's own framing) to be statistically meaningful, the same way the
EuRoC study needed 10 sequences. Building this properly would let those richer
aggregate plot types finally apply to the live HIL SLAM experiments, not just the
offline dataset ones.

Scope for the next session: design + build `benchmarks/compare_slam_hil.py`
(name TBD), reading N tagged `orbslam2_eval`-style bag runs, producing RMSE-across-runs,
CPU-across-runs, and (if multiple SLAM backends are run) pairwise comparison plots,
styled consistently with the now-restyled `eval_slam_hil.py` output.

---

## 4. Paper writing

Circle back once the above stabilizes. Existing groundwork already in `docs/`:
`oracle_projection_math.tex`, `homography_explained.tex`, `metrics_explained.tex`.
