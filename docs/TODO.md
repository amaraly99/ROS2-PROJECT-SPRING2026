# TODO — 2026-07-04

Reviewed list per user request. Not a fixlog entry (nothing here is applied/verified
yet) — see `HANDOFF.md` at repo root for the full session context behind each item.

---

## 0. (2026-09-08) Overnight session -- "freeze" closed as a false alarm; stereo integration of 4 backends

Pointers, not duplicates: `HANDOFF_2026-09-08.md` (Windows checkout root, narrative),
`docs/fixlog/028-orb-stereo-wrappers-published-nothing.md` (the one real bug fixed),
`docs/ideas_to_check/004-*.md` (now CLOSED -- false alarm, resolution appended),
`docs/REVERT_2026-09-08_stereo_integration.md` + `scripts/revert_stereo_integration_2026-09-08.sh`
(every created/modified path, one-shot revert; nothing committed).

### The "ground-truth freeze" (IDEA-004) is NOT a bug -- closed
The drone is deliberately held still twice per run (spawn hold while SLAM starts; post-init-leg
hold while the orchestrator waits for `/slam/tracking_state == 2` before flying the mission).
Verified on 16/16 stereo bags of 09-06: the hold always ends 0.0-0.3 s before the scorer's
`t_init`, so RMSE was never touched. A week of A/B tests compared a stationary drone with a
stationary drone. Lesson encoded as four skills in `.claude/skills/` (`rule-out-by-design`,
`read-the-driver-first`, `occams-razor-before-escalate`, `second-brain` w/ Stage-0 design gate).

### Stereo readiness after tonight (live `run_matrix.py --trials 1`, oracle, --hold-fsm) -- ALL FIVE READY
| arm | APE RMSE | err %/m | scale | track ms | pose Hz | matrix dir |
|---|---|---|---|---|---|---|
| OV2SLAM accurate (unchanged) | 0.172 +/- 0.032 (09-06, 3 trials, ~15 m path) | 1.16 | 0.98 | -- | -- | `results/matrix_20260906_210036` |
| OV2SLAM fast | 0.257 | 0.76 | 0.960 | 4.2 | 11.7 | `results/matrix_20260908_101113` |
| RTAB-Map | 0.509 | 1.51 | 1.102 | 82.8 | 7.0 | `results/matrix_20260908_100013` |
| ORB-SLAM2 | 0.619 | 1.84 | 1.119 | 44.8 | 11.6 | `results/matrix_20260908_100347` |
| ORB-SLAM3 | 0.700 | 2.07 | 1.223 | 33.0 | 11.6 | `results/matrix_20260908_100722` |
n = 1 per new arm, all scored over the full 33.7 m cruise (compare err %/m, not raw RMSE, against
the 15 m-path 09-06 numbers). Scale readings are single-trial; run the 10-trial matrix before
reading anything into them.

### Done tonight, all uncommitted (your call when to commit; revert script exists)
- FIX-028: ORB-SLAM3 `StereoMode::ProcessStereoPair` and the ORB-SLAM2 stereo wrapper never
  published `/slam/pose` / `/slam/tracking_state` (HIL port existed only for mono). Fixed,
  critic + simplification reviewed (both `open_concern`, none blocking).
- Builds now present in THIS worktree: `src/orbslam3/install` (+ Pangolin, Vocabulary copied
  from `~/ROS2-PROJECT-SPRING2026`), `src/orbslam2/{lib,install_stereo}` (+ Vocabulary copy).
- New stack configs: `ov2slam_stereo_oracle_fast`, `orbslam3_stereo_oracle`,
  `orbslam2_stereo_oracle`, `rtabmap_stereo_oracle`; matching Windows matrix configs
  `only_*_stereo_oracle*.yaml`; calibs/settings: `camera_calib/hil_sim_ov2slam_stereo_fast.yaml`,
  `src/orbslam3/orb_slam3/config/Stereo/HIL_SIM.yaml`, `src/orbslam2/ros2-ORB_SLAM2/src/stereo/hil_sim.yaml`.
- RTAB-Map live-HIL integration from scratch: `src/rtabmap_docker/hil_stereo_bridge.py` +
  `hil_stereo.launch.py`.
- `run_matrix.py:809`: `== "orbslam3"` -> `.startswith("orbslam3")` (untracked file! pristine
  copy in `trash/run_matrix.py.orig-2026-09-08`).
- Test tooling: `scripts/hil_matrix/synthetic_stereo_pub.py`, `scripts/hil_matrix/stereo_wiring_test.sh`
  (launches any stack config's sidecar exactly as `run_stack_hil.sh` would and probes the contract).

### Open, new
- **TODO-AT** (028): ORB-SLAM2 stereo -- use `ORB_SLAM2::Converter::toQuaternion` instead of the
  copied quaternion block (KISS); rebuild + wiring re-test.
- **TODO-AU** (028): mono ORB-SLAM matrix configs still point `pi_repo` at `~/ROS2-PROJECT-SPRING2026`;
  the mono builds never existed in `~/ROS2-slam-hil`. Move mono arms here or document the split.
- **TODO-AV** (028): ORB-SLAM2 publishes poses on LOST frames, ORB-SLAM3 gates on state==2 --
  decide whether the aggregator should drop `tracking_state != 2` samples for every arm.
- 6/16 stereo OV2SLAM trials on 09-06 never reached SLAM-ready in budget -- a stereo
  initialisation-time question, not a pose bug; untouched tonight.
- Housekeeping: the `_TEST` configs from 09-06 and the 09-06 tooling (`export_bag_csv.py`,
  `pose_watch.py`) are still untracked.

---

## 0b. (2026-09-06) Tonight's session -- stereo fx bug resolved, GT-freeze bug paused

Two separate threads investigated, both now fully written up. See
`docs/fixlog/027-stereo-fx-1200-vs-554-mismatch.md` and
`docs/ideas_to_check/004-ground-truth-pose-frozen-in-bag.md` for the
full technical detail -- this section is a pointer, not a duplicate.

### Resolved and verified tonight: stereo scale/RMSE bug (FIX-027)
Stereo OV2SLAM was scoring a consistently wrong Umeyama scale (~0.46 instead
of ~1.0) and elevated RMSE. Root cause: the stereo calib file
(`camera_calib/hil_sim_ov2slam_stereo.yaml`) declared fx=1200 (the
2026-08-24 correction) but the Simulink camera blocks themselves (both
eyes, confirmed via raw .slx XML) were never actually updated off 554 --
so OV2SLAM's triangulation was unprojecting a 554-fx image with an assumed
1200 fx. Fixed by matching the calib file back to the verified-real 554
(lower risk than editing/rebuilding the .slx). Verified across 4 live
trials: scale settled at ~0.98 +/- 0.03, RMSE ~0.17 +/- 0.03 (down from
0.335 +/- 0.113). Staged, not committed yet -- your call when to commit.
New open item: **TODO-AS** -- no stereo controller-in-the-loop run
(TS1/TS3/TS4 under `stereo:true`, not `--hold-fsm`) has been evaluated
against the corrected fx yet; everything tonight was `--hold-fsm` only.

### Still open, paused not abandoned: ground-truth-pose-frozen-in-bag
`/sim/drone_pose` freezes mid-run in every stereo+oracle benchmark trial
(9/9 this session), corrupting RMSE/scale scoring for those runs. Ten
black-box variables individually ruled out via direct live test (mission
script, MATLAB-side staleness, message reuse, array layout, RMW/DDS
implementation, generic bandwidth, topic-specific staleness via a
duplicate publisher, QoS reliability, timer BusyMode, network medium
WiFi-vs-wired). Reframed tonight: real stereo rendering alone does NOT
reproduce it (3/3 non-empty scout-mode stereo bags are healthy) -- leading
hypothesis is real stereo rendering AND `slam_traj_probe.m`'s tight
busy-loop TOGETHER, since neither alone reproduces it. Remaining candidate
space is MATLAB ROS Toolbox's own closed `ros2publisher`/`send()` or the
DDS library underneath (RMW itself already ruled out). Concrete next
experiment, not yet run: isolate `--hold-fsm` from the busy-loop pattern
specifically. A fallback workaround (MATLAB writes `sim_pose` to a local
file, `aggregate_matrix.py` reads GT from that instead of the bag) is
designed but not implemented -- your call whether to isolate first or go
straight to the workaround. **Deliberately not pursued further tonight
past this write-up** -- next step is running SLAM itself, per your
explicit direction to wait.

New tooling this session (untracked): `scripts/hil_matrix/export_bag_csv.py`
(bag-to-CSV GT+SLAM extractor) and a live freeze-watchdog
(`/tmp/pose_watch.py` on the Pi) -- both make future diagnosis of pose-bag
issues cheaper without needing a full trial cycle each time.

New safety finding, flagged not fixed: `oracle_detector_node.py` and
`servo_fsm_node.cpp` both read `/sim/drone_pose` continuously for real
control/safety logic outside `--hold-fsm`, with no guard against a
frozen-but-still-publishing value (their staleness checks are arrival-time
only, not value-change). Latent so far since nothing non-`--hold-fsm` was
run tonight, but should be addressed before any real reactive stereo
mission.

---

## 0b. (2026-09-04) Tonight's session — where things stand

Stereo SLAM went from "produces nothing, ever" to "produces real poses, real
`tracking_state==2`, real end-to-end automated trial" tonight, through a chain of
five real fixes (FIX-024/025/026, plus the `set_stereo`/remap gaps found along the
way). **Everything below is either done-but-uncommitted, or a genuinely new,
still-open item found along the way.**

### Done tonight, staged but NOT committed (your call when to commit)
- FIX-024: right-eye shared memory cleanup gap (`run_stack_hil.sh`) — resolved,
  verified end-to-end with real `/slam/pose` output.
- FIX-025: OV2SLAM never reported stereo "ready" (`mapper.cpp`) — resolved,
  verified live (`tracking_state` reached 2, real trial flew).
- FIX-026: automated init-check gave up too early for stereo's slower ramp-up
  (`run_matrix.py`) — resolved, verified live.
- New Pi-side stack config `ov2slam_stereo_oracle.yaml` + new Windows-side matrix
  config `only_ov2_stereo_oracle.yaml` — both working, one real trial completed.
- New Windows worktree `C:\Users\homie\Desktop\ROS2-slam-hil`, matching the Pi's
  branch, for MATLAB automation — working, confirmed in active use.

### Open, not started, not blocking anything above
- **TODO-AQ** (fixlog 024): if the right-eye writer crashes *mid-run* (not just at
  stop/restart) the fix doesn't catch it — no liveness check exists yet, only an
  existence check at startup. Not observed happening, just unguarded.
- **TODO-AR** (fixlog 025): stereo has no way to *un-flag* "ready" if its first
  reading was wrong (mono can; stereo can't yet). Not observed happening yet.
- **IDEA-003**: on a *manual* (non-matrix) stereo run, SLAM stalled forever ~50s
  in with a repeating `Throw img0 -- Sync error`. Separate from tonight's matrix
  work — found on an earlier, different run. Not root-caused. See
  `docs/ideas_to_check/003-ov2slam-sync-error-stalls-mid-run.md`. Possibly the
  same class of bug as TODO-AQ (a mid-run stall) — worth checking together.

### New, found just now — the live blocker for a full 10-trial matrix run
- **The drone itself never moved** in the one real trial completed tonight, even
  though SLAM worked perfectly and the mission script ran its full 22 seconds.
  Traced to `matlab/slam_traj_probe.m`'s "mission" phase — it computes how far to
  fly as `(a desired distance) / (speed)`, and that desired-distance value looks
  like it's coming out to zero for this oracle+stereo+no-warm-up setup
  specifically. Not yet root-caused — the exact reason that value is zero here
  hasn't been found. This is what's actually stopping the 10-trial run right now.

  **Update 2026-09-06: this specific blocker no longer applies as originally
  described.** Tonight's session ran real stereo+oracle trials that DID fly and
  score (the fx investigation above depended on real, non-zero trajectories) —
  so whatever zero-distance issue existed 2026-09-04 was resolved or worked
  around between then and now, separately from tonight's two documented threads.
  Not re-investigated tonight; flagged here so it isn't mistaken for still-open.

### Housekeeping, noticed along the way, not urgent
- A Simulink build-cache folder (`matlab/slprj`) was accidentally committed to git
  on this branch at some point — should never be tracked. Not fixed, just flagged.
- On the Pi, `git status` shows local, unstaged edits to files nobody touched
  tonight (`config/hil/bench_fsm.yaml`, `config/hil/stack/ov2slam_ibvs.yaml`,
  `src/servo_core/...`, `src/ov2slam_ros/CMakeLists.txt`). Pre-existing, not
  investigated — worth a quick look before assuming they're harmless.
- Scout/record-mode priority work (from earlier in this session) — explicitly
  deferred by you until the controller-benchmarking phase. Not touched tonight,
  not forgotten.

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
