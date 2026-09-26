---
id: FIX-018
title: OV2SLAM never built on benchmarks/slam-hil (and controller-benchmark) -- copied working build + custom OpenCV from feat/stereo-hil
date: 2026-09-01
status: resolved
component: src/ov2slam_ros, opencv (custom worktree-local build), docker slam_ov2slam sidecar
critic_verdict: wrong_fix (on the original diff)
kiss_verdict: simplify-recommended (on the original diff)
open_todos: [TODO-AD, TODO-AE]
---

## Symptom

Live mono run on `benchmarks/slam-hil` (worktree `/home/amaraly/ROS2-slam-hil`):
the `slam_ov2slam` docker sidecar restarted 10 times in a single session,
every restart failing immediately with `ExitCode=0`:
```
Package 'ov2slam' not found
```
`docker inspect` confirmed `RestartCount=10`. Masked as "alive" in `docker ps`
the whole time because of `restart: unless-stopped`.

## Root cause

Two independent, stacked gaps, neither specific to stereo vs mono:

1. `ov2slam` (ROS2 package name; directory `src/ov2slam_ros`) was never
   colcon-built on this worktree. `install/` had no `ov2slam` entry and there
   was no colcon build log for it at all -- not "failed to build", never
   *attempted*. Checked `ROS2-controller-benchmark`'s own worktree too: same
   gap. The only worktree with a working build is the original
   `ROS2-PROJECT-SPRING2026` (`feat/stereo-hil`) -- `build/`/`install/`/`log/`
   are per-worktree, git-ignored directories that `git worktree add` does not
   populate, and the original migration's Phase 5 build verification
   (`optimized-growing-creek.md`) only ran colcon for the controller/stereo
   package list, never for the SLAM sidecar.
2. Found mid-fix, not initially: `ov2slam_ros` also dynamically links against
   a custom-built OpenCV 4.10 (with contrib modules, specifically
   `xfeatures2d` for its feature front-end) living at a worktree-local
   `opencv/build/lib/` -- built by a repo-root `build_opencv.sh` that was also
   never run on this worktree. `ldd` on the copied binary, even with the
   correct `LD_LIBRARY_PATH=/workspace/opencv/build/lib`, showed 7 missing
   `.so.410` libs (only system OpenCV 4.6 was present). This one is silent for
   every other package in the stack (visp_servo, sim_camera_bridge, etc. all
   have the same `LD_LIBRARY_PATH` entry in their launch env) because none of
   them link `xfeatures2d`/OpenCV-contrib -- only OV2SLAM does.

## Diff

No source diff. Original plan (rebuild from scratch: `git submodule update
--init` for the empty `Thirdparty/opengv`, run `build_thirdparty.sh`, then
`colcon build --packages-select ov2slam`) was rejected by both review agents
-- see verdicts below. Actual fix, on the Pi, copying already-built,
already-verified-working artifacts from `feat/stereo-hil`'s worktree (both
worktrees mount to the identical `/workspace` inside the identical
`ros2_perception_stack:latest` image, confirmed via `docker images`, so
container-relative paths transfer cleanly):

```
sudo rsync -a ROS2-PROJECT-SPRING2026/build/ov2slam/      ROS2-slam-hil/build/ov2slam/
sudo rsync -a ROS2-PROJECT-SPRING2026/install/ov2slam/    ROS2-slam-hil/install/ov2slam/
      rsync -a ROS2-PROJECT-SPRING2026/src/ov2slam_ros/Thirdparty/  ROS2-slam-hil/src/ov2slam_ros/Thirdparty/
      rsync -a ROS2-PROJECT-SPRING2026/opencv/              ROS2-slam-hil/opencv/
sudo chown -R amaraly:amaraly ROS2-slam-hil/build/ov2slam ROS2-slam-hil/install/ov2slam
```
(`build/`, `install/` were root-owned -- created by the container running as
root -- hence `sudo` for those two; `Thirdparty/` and `opencv/` were already
user-owned in the source worktree.)

## Critic verdict & concerns

**wrong_fix.** Ran the original step 1 live and it failed outright:
`git submodule update --init` errors `fatal: No url found for submodule path
'...opengv' in .gitmodules` -- confirmed `.gitmodules` does not exist
*anywhere in this repo's history*, in any worktree, even though
opengv/Sophus/ceres-solver/ibow_lcd/obindex2/backward-cpp are all real
gitlinks (mode 160000) in the tree. The submodule step could never have
worked, in any worktree, not just this one. Critic also flagged that opengv
might be baked into the docker image itself (root-owned dir on
`feat/stereo-hil`, matching an overlayfs path) -- which would have made the
empty-Thirdparty-opengv diagnosis non-causal for the original symptom.
Resolved by the final fix sidestepping the question entirely: no source
recompilation happens, so where opengv originally came from doesn't matter.

## KISS verdict

**simplify-recommended**, and the simplification actually applied. Live-
verified that colcon's `CMAKE_INSTALL_PREFIX` and the install-space symlinks
(`--symlink-install`) are all `/workspace/...`-relative, not host-absolute,
and both worktrees mount to the same `/workspace`. A from-scratch rebuild
(Ceres-solver alone measured ~11 min compile time on this Pi 5 previously,
plus obindex2/ibow_lcd/Sophus/ov2slam itself, 25-40+ min total estimated) was
unnecessary when a same-disk copy (~1.6 GB total across ov2slam build+install,
Thirdparty, and the OpenCV tree) takes well under a minute and reuses a
binary already proven to run correctly (real keyframe/front-end/back-end
processing observed in its log on `feat/stereo-hil`).

## Verification

- `docker restart slam_ov2slam`, watched for 35s: `RestartCount` stayed `0`
  (was climbing to 10 before), `Status=running`.
- Log tail shows genuine OV2SLAM startup, not the old error: `Estimator
  Object is created!`, `Mapper is ready to process Keyframes!`, `LoopCloser
  is ready to process Keyframes!`, `OV²SLAM is ready to process incoming
  images!`.
- `ros2 pkg list` inside the container now lists `ov2slam` (previously
  absent).
- `ldd` on the copied `ov2slam_node`, with the real launch `LD_LIBRARY_PATH`,
  resolves cleanly -- zero `not found` entries (previously 7).
- `/slam/pose` and `/ovcam/image_raw` both showed zero traffic in a 5s
  `ros2 topic hz` check -- expected and not a defect: MATLAB was not actively
  simulating at check time (the earlier mono run had ended), so there was no
  camera feed for OV2SLAM to process. Actual pose-publishing under a live
  camera feed is NOT yet verified -- that's the next live run's job.

## Open TODOs

- TODO-AD: `ROS2-controller-hil` (the other new worktree) has the identical
  gap (no `ov2slam` build) if it ever needs SLAM -- currently out of scope,
  its whole purpose is the controller-benchmark campaign, not SLAM
  comparison. Apply the same copy if that changes.
- TODO-AE: confirm `/slam/pose` actually publishes end-to-end under a live
  camera feed (this fix only proves the node loads and initializes cleanly,
  not that it produces correct output against live frames).
