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
| [028](028-orb-stereo-wrappers-published-nothing.md) | 2026-09-08 | ORB-SLAM3 + ORB-SLAM2 stereo wrappers tracked frames but never published /slam/pose or /slam/tracking_state (HIL port was mono-only) -- both fixed, built in this worktree, verified by live stereo trials (RMSE 0.62 m / 0.70 m over ~34 m) | **resolved** |
| [027](027-stereo-fx-1200-vs-554-mismatch.md) | 2026-09-06 | Stereo camera calib declared fx=1200 (the 2026-08-24 correction) but the Simulink block itself stayed at 554 -- systematic depth bias, scale~0.46; fixed by matching config to the verified real render | **resolved** |
| [026](026-init-check-no-patience-for-slow-init.md) | 2026-09-04 | run_matrix.py's SLAM-ready check accepted the first real reading as final -- no patience for stereo's gradual ramp-up; verified live, real trial flew | **resolved** |
| [025](025-stereo-tracking-state-never-ready.md) | 2026-09-04 | OV2SLAM stereo never reports tracking_state==2 -- the only code path that sets it was mono-only; added a stereo path in mapper.cpp -- confirmed live via FIX-026's real trial | **resolved** |
| [024](024-ovcam-right-shm-orphaned-mapping.md) | 2026-09-04 | ovcam_bridge_right reads a deleted/orphaned shm segment -- fixed missing right-eye cleanup in run_stack_hil.sh, confirmed end-to-end: 66 real /slam/pose messages | **resolved** |
| [023](023-new-record-mode.md) | 2026-09-03 | new record mode -- scout behavior + bag recording, no CPU/thread profiling | **resolved** |
| [022](022-slam-startup-delay-15s-to-2s.md) | 2026-09-02 | ov2slam_stereo startup_delay_sec 15s → 2s — copy-pasted default, real auto-exit guard is a 50s silence watchdog | **resolved** |
| [021](021-slam-sidecar-log-lost-on-teardown-scout-mode.md) | 2026-09-01 | SLAM sidecar console output unrecoverable after teardown in scout mode — extended benchmark-mode log persistence to scout | **resolved** |
| [020](020-camera-fx-stale-554-vs-verified-1200.md) | 2026-09-01 | cam_fx_/cam_fy_ stale at unverified 554 vs the project's own measured 1200 -- corrected value, collapsed redundant camera_fx_ | **resolved** |
| [019](019-opengv-not-linked-in-ov2slam.md) | 2026-09-01 | opengv silently not linked into OV2SLAM — one-line CMakeLists.txt PATHS hint + two more missing worktree artifacts | **resolved** |
| [018](018-ov2slam-never-built-on-new-worktrees.md) | 2026-09-01 | OV2SLAM never built on benchmarks/slam-hil (+ controller-benchmark) — copied working build + custom OpenCV from feat/stereo-hil | **resolved** |
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
- **TODO-AT/AU/AV** (028, active): ORB-SLAM2 stereo.cpp should use
  ORB_SLAM2::Converter::toQuaternion instead of the copied quaternion block (AT);
  the mono ORB-SLAM matrix configs still point pi_repo at ~/ROS2-PROJECT-SPRING2026
  because the mono builds never existed in this worktree (AU); ORB-SLAM2 publishes
  poses on LOST frames while ORB-SLAM3 gates on state==2 -- decide whether the
  aggregator should drop tracking_state != 2 samples for every arm (AV).
- **TODO-AR** (025, active, not blocking): stereo has no recovery path if
  its first ready reading turns out wrong -- unlike mono, which can
  revert via mapper.cpp's existing reset-on-bad-first-keyframe check. Both
  review agents independently flagged this. See fixlog 025.
- **FIX-026** (resolved): run_matrix.py's SLAM-ready check now has a real,
  configurable patience budget instead of accepting the first reading as
  final. See fixlog 026.
- **TODO-AQ** (024, active): right-eye readiness gate (added by TODO-AP)
  only checks shm *existence* at launch, not *liveness* during a run --
  critic review confirmed sim_camera_bridge_right has no respawn/exit
  handling in the launch file, so a genuine mid-run crash of that node
  would go uncaught. Not yet confirmed to happen in practice; not yet
  implemented. See fixlog 024.
- **TODO-AL/AM/AN/AO** (024, active, lower priority): remaining fix options
  from the original diagnosis, superseded in priority by TODO-AP (resolved)
  for the mechanism that was actually confirmed. TODO-AM (self-healing
  reconnect) would also close TODO-AQ's gap as a side effect if
  implemented. See fixlog 024.
- **TODO-AK** (023, active): final run-summary block still has a stale
  benchmark-only check for where the SLAM sidecar log lives, not updated
  when FIX-021 gave scout/record modes their own persisted log path.
- **TODO-AH/AI/AJ** (020, active): TS1/TS3 controller gains were tuned
  around the now-corrected 554 intrinsic and may need retuning post-fix (AH);
  bench_oracle.yaml still carries the stale 554, now silently disagreeing
  with bench_fsm.yaml where before both agreed (wrongly) (AI);
  benchmarks/controller-hil has the byte-identical bug, unfixed (AJ).
- **TODO-AF/AG** (019, active): no before/after trajectory comparison exists
  between the OpenCV-fallback geometry path (every run before this fix) and
  the now-active opengv path (AF); libopencv_*.so.406 vs .so.410 linker
  warning on the ov2slam rebuild, not chased down (AG).
- **TODO-AD/AE** (018, active): benchmarks/controller-hil has the identical
  never-built-ov2slam gap if it ever needs SLAM (AD, currently out of scope);
  /slam/pose publishing under a live camera feed is not yet verified, only
  clean node startup (AE, next live run's job).

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
