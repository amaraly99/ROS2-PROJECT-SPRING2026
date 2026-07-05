---
id: FIX-015
title: SLAM HIL eval → modular per-SLAM package (evo ATE, paper plots), OV2SLAM front-end timing + fast/accurate
date: 2026-07-05
status: resolved
component: benchmarks/slam_eval/, src/ov2slam_ros/, run_stack_hil.sh, camera_calib/, config/hil/stack/
critic_verdict: partial
kiss_verdict: simple
open_todos: [TODO-AB, TODO-AC]
---

## Symptom / motivation (user asks)

1. `eval_slam_hil.py` computed ATE with a **hand-rolled Umeyama** — the offline
   ORB-SLAM2 / OV2SLAM studies used **evo** (`evo_ape --align --correct_scale`).
   Inconsistent methodology, and not the trusted/canonical tool.
2. The evaluator was a **monolith** trying to be SLAM-generic. User wants it
   **hardcoded per-SLAM** (one file each) with a shared base skeleton, split into
   a package, "developer-friendly for someone who barely knows ROS."
3. OV2SLAM produced **no front-end timing** in HIL — `eval_slam_hil.py`'s timing
   came from ORB-SLAM2's `timing_events.csv` (via `ORB_BENCH_TIMING_CSV`), which
   OV2SLAM ignores. So the front-end plot had no data source for OV2SLAM.
4. Plots must match `SLAM_benchmark/images/` (generator:
   `src/orbslam2/tools/generate_paper_images.py`) — for a single HIL run, the
   per-sequence bars collapse to 1-column bars.
5. OV2SLAM must support its **fast** and **accurate** modes (the offline study
   benchmarked them as two separate algorithms, `ov2_fast` / `ov2_accurate`).

## Root cause

Pre-existing gaps, not a regression: the HIL eval predated evo adoption; OV2SLAM's
per-frame front-end cost lives only in its internal `Profiler` (`ov2slam_timings.csv`,
written on graceful `writeResults()` which a HIL `docker stop` never reaches); and
only one HIL OV2SLAM calib existed.

## Diff (four coordinated changes)

1. **OV2SLAM per-frame front-end timing CSV (C++)** — mirrors ORB-SLAM2's
   `ORB_BENCH_TIMING_CSV`/`ScopedBenchmarkTimer`:
   - `src/ov2slam_ros/include/profiler.hpp`: `Profiler::LogEvent(category, ms,
     frame_id)` — lazily opens the file named by env `OV2_BENCH_TIMING_CSV`,
     writes header `wall_ts,category,duration_ms,frame_id`, appends+flushes one
     row per call (docker-stop-safe), no-op if the env var is unset. Own
     `event_csv_`/`event_mutex_`, independent of the existing timing mutex.
   - `src/ov2slam_ros/src/visual_front_end.cpp`: measure the `0.Full-Front_End`
     region with a cheap `steady_clock` (independent of the debug/log_timings
     gate) and `LogEvent("frontend/full_tracking", …, pcurframe_->id_)`.
2. **`run_stack_hil.sh`**: benchmark mode now exports BOTH `ORB_BENCH_TIMING_CSV`
   (`timing_events.csv`) and `OV2_BENCH_TIMING_CSV` (`ov2slam_timing_events.csv`)
   into the sidecar; each backend picks up only its own.
3. **fast/accurate configs**: `camera_calib/hil_sim_ov2slam_accurate.yaml`
   (canonical base = proven `hil_sim_ov2slam.yaml`, debug off) +
   `hil_sim_ov2slam_fast.yaml` (differs only in `use_fast/use_singlescale_detector/
   nmaxdist/use_clahe`). Stack configs `config/hil/stack/ov2slam_oracle_{fast,accurate}.yaml`.
4. **`benchmarks/slam_eval/` package**: `base.py` (`SlamEvaluator` — bag read →
   evo ATE → timing/CPU → paper-style plots → `slam_metrics.csv`), tiny
   `orbslam2.py` / `ov2slam.py` subclasses (declare label/colour/timing-file;
   ov2slam picks fast/accurate label+colour from the config name), `__main__.py`
   dispatch by `slam_type`. ATE uses **evo's Python API** directly
   (`sync.associate_trajectories` → `align(correct_scale=True)` → `APE`), i.e.
   evo's own Umeyama, not a copy. evo + deps live in `benchmarks/.evo_venv`
   (PEP-668, can't install system-wide on the Pi); `eval_slam_hil.py` is now a
   thin re-exec shim into that venv. Old monolith (~470 lines) fully replaced.

## Verification (all on the Pi, real data — not assumed)

- Rebuilt OV2SLAM; played 514 real `/ovcam/image_raw` frames through the node →
  **466 rows** in the timing CSV, correct header, sane 2–18 ms durations.
- Ran the new package against the known-good ORB-SLAM2 bag
  `run_orbslam2_eval_20260704_013420`: **ATE via evo = 0.130901 m** vs the old
  hand-rolled-Umeyama **0.129932 m** — matches to <1 mm, confirming evo does the
  same Sim3 alignment (the ~1 mm delta is evo's nearest-GT association vs the old
  code's GT interpolation). Coverage 106.22% and scale 30.27 both match the old
  code. All 6 paper-style plots generated and visually checked against
  `SLAM_benchmark/images/` (purple `#6a4c93`, 5/10cm + 20Hz-budget reference
  lines, GT-dashed/SLAM-solid overlay, ORB-only thread roles).

## Critic verdict & concerns

**Directionally right; one real (bounded) bug found and FIXED, plus caveats.**

- **FIXED during review**: OV2SLAM's `pcurframe_->id_` restarts at 0 on a SLAM
  reset (`SlamManager::reset()` → `Frame::reset()` sets `id_=-1`; resets fire from
  `visual_front_end.cpp` and `mapper.cpp` *during* flight, not only pre-init), so
  `frame_id` is not monotonic across a run. The eval originally sorted the
  front-end timeseries by `frame_id` → a reset scrambled the plot. Impact was
  bounded (RMSE/mean/std/CPU/coverage are all order-independent — only the
  timeseries plot). Fixed by sorting on the always-monotonic `wall_ts` column and
  plotting against a sample index. (The two backends still don't share a
  `frame_id` contract — ORB's `mnId` never resets — but nothing now depends on
  frame_id being monotonic or cross-comparable.)
- Confirmed sound: evo alignment/APE use positions only (identity quats are
  harmless); `max_diff=0.05` association is safe at 50/16 Hz; env/filename wiring
  has no crosswiring; evo's `align(correct_scale=True)` scale has the same
  SLAM→GT-metric convention the coverage metric assumes; the un-closed flushed
  `ofstream` loses nothing on `docker stop`; no lock-ordering deadlock.
- Open caveat **TODO-AB**: the fast/accurate configs flip only the 4 core
  feature-extractor knobs, not `dop3p` / `fkf_filtering_ratio` (which also differ
  in upstream's fast vs accurate). Defensible (HIL-tuned shared values) but the
  labels slightly overclaim — documented in both config headers, flagged here.

## KISS verdict

**Simplest correct.** Single well-sectioned `base.py` + tiny declarative
subclasses is right-sized for a fixed small set of backends (a separate `plots.py`
would add a seam for no gain). C++ sink correctly lives on the existing Profiler
singleton (mirrors `ExportCSV`). Re-exec shim is the simplest venv bridge.
Cleanups applied: removed the redundant `frontend_category` override from both
subclasses (it's the base default); documented the calib-duplication sync risk in
both headers and named one canonical base config (was inconsistent). Nothing
useful dropped from the old monolith (its per-axis error plot isn't in the paper
set; its hand-rolled Umeyama was the thing being removed).

## Open TODOs

- **TODO-AB** (015): fast/accurate configs omit the upstream `dop3p` /
  `fkf_filtering_ratio` differences — decide whether to flip them for full
  fidelity or keep the HIL-tuned shared values (then drop the caveat).
- **TODO-AC** (015): the two OV2SLAM calib files duplicate ~133 lines that must be
  hand-synced (YAML can't include across files; the calib is read by the OV2SLAM
  binary, not our parser). Revisit if they drift.
