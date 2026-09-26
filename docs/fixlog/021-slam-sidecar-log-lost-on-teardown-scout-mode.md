---
id: FIX-021
title: OV2SLAM sidecar's own console output unrecoverable after container teardown in scout mode -- extended the existing benchmark-mode log-persistence mechanism to scout mode
date: 2026-09-01
status: resolved
component: run_stack_hil.sh (start_slam_sidecar)
supersedes: none
critic_verdict: wrong_fix (round 1) -> ok (round 2, after fix)
kiss_verdict: ok
open_todos: []
---

## Symptom

Tried to check whether SLAM's own pose estimate was sane for the 22:30
stereo test run tonight. `docker logs slam_ov2slam` returned "No such
container" -- the sidecar had already been torn down, and there was no
persisted copy of its console output anywhere (`/tmp/ov2slam_hil.log` did
not exist for this run). The evidence needed to judge SLAM quality was
gone, unrecoverably.

## Root cause

`run_stack_hil.sh` already has a proven mechanism for exactly this --
persisting the SLAM sidecar's console output to a file that survives
container removal -- but it only fires when `MODE == "benchmark"`:
```bash
SLAM_LOG_REDIRECT=""  # no-op for scout mode (no RUNREL to write into)
...
if [[ "$MODE" == "benchmark" ]]; then
    SLAM_LOG_REDIRECT=" >>/workspace/${RUNREL}/slam_sidecar.log 2>&1"
fi
```
Its own comment even names the exact failure mode being fixed here:
*"previously only existed in `docker logs`, which is lost the moment the
container is stopped/removed -- exactly the evidence needed to diagnose an
init failure after the fact."* Scout mode (what every interactive/manual
run uses, including tonight's stereo test) has no `RUNREL` (per-run results
directory -- a benchmark-mode-only concept) to write into, so the redirect
was left as a deliberate no-op for that mode. Not a bug in the mechanism --
it just was never turned on for the mode actually used tonight.

## Diff

`run_stack_hil.sh`, the `SLAM_LOG_REDIRECT` block: added an `else` branch
so scout mode also redirects, to a fixed `/tmp` path (mirroring how
`/tmp/hil_launch.log` and `/tmp/yolo_producer.log` already handle
mode-agnostic debug logs in this same script) instead of the benchmark-only
`RUNREL` path. Benchmark-mode branch is untouched.
```bash
else
    SLAM_LOG_REDIRECT=" >/tmp/slam_sidecar_${SLAM_CONTAINER}.log 2>&1"
fi
```
Also updated the "SLAM sidecar started" log line to report the new
persisted path in scout mode too, mirroring the existing benchmark-mode
message.

## Critic verdict & concerns

Round 1: **wrong_fix**. Original proposal used one single fixed path
(`/tmp/slam_sidecar.log`) for every scout-mode SLAM sidecar. Critic found a
real collision: `SLAM_CONTAINER` differs per SLAM *backend*
(`slam_ov2slam`, `slam_orbslam2`, `slam_orbslam3`) but is identical across
worktrees for the same backend (confirmed: `ROS2-slam-hil` and
`ROS2-controller-hil` both use `slam_ov2slam`). A single shared path opened
with truncate (no `O_APPEND`) risked two different-backend scout runs
overlapping and silently corrupting/interleaving each other's log via
independently-advancing file offsets into the same inode -- worse than no
log at all, since it would look like valid output. Also confirmed (and
this part held up): `stop` never deletes anything under `/tmp` that would
undermine the fix, and `/tmp` is already bind-mounted into the sidecar
container, so no new plumbing was needed.
Round 2 (after scoping the filename by `${SLAM_CONTAINER}`): **ok**.

## KISS verdict

**ok.** Simplification agent confirmed the fixed-`/tmp`-path + truncate
approach already matches this script's own established precedent
(`yolo_producer.log`: fixed path, truncate; `hil_launch.log`: fixed path,
though it appends forever with no rotation -- flagged as an existing,
separate wart elsewhere, not something to copy here). Confirmed scout mode
deliberately has no per-run directory concept to piggyback on instead
(not an oversight). Confirmed editing the shared `run_stack_hil.sh`
function is the right scope -- the goal is backend-agnostic, so a
stereo-config-only or wrapper-script alternative would under-deliver or
duplicate the whole `docker run` invocation for a 3-line difference.

## Verification

Ran the real thing, not just a syntax check:
- `bash -n run_stack_hil.sh` clean, on both the edited local copy and after
  deploying to the Pi.
- `./run_stack_hil.sh --config ov2slam_stereo`: startup log shows the new
  message verbatim -- `SLAM sidecar started -- log: docker logs
  slam_ov2slam  (persisted: /tmp/slam_sidecar_slam_ov2slam.log)`.
- While the sidecar was still running: `/tmp/slam_sidecar_slam_ov2slam.log`
  existed, 126 lines, real OV2SLAM startup content (`OV²SLAM is ready to
  process incoming images!`, estimator/loop-closer/mapper init messages).
- `./run_stack_hil.sh stop`: confirmed via `docker ps -a --filter
  name=slam_ov2slam` that the container is fully gone (not just stopped --
  no row at all).
- **After** that removal: the log file still exists, still 126 lines,
  content unchanged. This is the actual thing that was broken before --
  confirmed fixed, with real before/after evidence, not assumed.

## Open TODOs

None outstanding for this fix. (Simplification agent's one non-blocking
note -- also echoing the persisted-log path in the final run-summary block,
for symmetry with hil_launch.log/yolo_producer.log there -- folded into the
startup message instead, which already covers the same information; not
worth a second edit for the same fact restated twice.)
