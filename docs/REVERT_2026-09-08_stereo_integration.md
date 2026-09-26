# REVERT manifest — stereo integration work of 2026-09-08

Scope: making OV2SLAM-fast, ORB-SLAM3, ORB-SLAM2 and RTAB-Map runnable as **stereo**
arms of the HIL benchmark (`run_matrix.py` → `run_stack_hil.sh --config <stack> --hold-fsm`).
Plan approved by Arian on 2026-09-08 (`~/.claude/plans/shiny-crunching-hanrahan.md` on the
Windows side).

**Nothing was committed or pushed. No downloads. All images used were already present.**
Claude Code's own "rewind" can undo the Windows-side files but cannot see anything done
over SSH — this manifest + the script below are the revert path for the Pi side.

## One-shot revert (Pi side)

```bash
cd ~/ROS2-slam-hil && ./scripts/revert_stereo_integration_2026-09-08.sh
```
It moves every created path into `trash/stereo_integration_2026-09-08_<ts>/` (never deletes,
per the standing rule) and `git checkout --`s the one modified tracked file. Re-runnable.

## Windows side (by hand, in `C:\Users\homie\Desktop\ROS2-PROJECT-SPRING2026`)

```
git checkout -- scripts/hil_matrix/run_matrix.py      # one-line edit, see below
move config\hil\matrix\only_ov2_stereo_oracle_fast.yaml trash\
move config\hil\matrix\only_orbslam3_stereo_oracle.yaml trash\
move config\hil\matrix\only_orbslam2_stereo_oracle.yaml trash\
move config\hil\matrix\only_rtabmap_stereo_oracle.yaml  trash\
```

## Baseline

`git status --short` of `~/ROS2-slam-hil` right before the first change is saved at
`/tmp/git_baseline_2026-09-08.txt` on the Pi (the script diffs against it at the end).
Notable pre-existing state, NOT touched by this work: staged-uncommitted FIX-023..027 content
(`run_stack_hil.sh`, `src/ov2slam_ros/src/mapper.cpp`, `camera_calib/hil_sim_ov2slam_stereo.yaml`,
`docs/fixlog/*`), plus untracked `_TEST` configs from the 2026-09-06 session.

## Modified files (exactly three)

| Machine | File | Change | Revert |
|---|---|---|---|
| Pi | `src/orbslam2/ros2-ORB_SLAM2/src/stereo/stereo.cpp` | HIL publishers added (`slam_pose`, `slam_tracking_state`) + `rclcpp::remove_ros_arguments` so the appended `--ros-args` doesn't trip the argc check — the same two things `mono.cpp`/`monocular-slam-node.cpp` already had (71+/3−) | `git checkout -- <file>` |
| Pi | `src/orbslam3/src/common.cpp` | `StereoMode::ProcessStereoPair`: capture `TrackStereo`'s return and call the existing `PublishPose(Tcw, timestamp)` — the mono callback already did this at `:671`, the stereo path discarded the pose (2+/1−) | `git checkout -- <file>` |
| Windows | `scripts/hil_matrix/run_matrix.py` line 809 | `arm["name"] == "orbslam3"` → `arm["name"].startswith("orbslam3")` | **`scripts/hil_matrix/` is UNTRACKED on `feat/stereo-hil-posefix`, so `git checkout` cannot restore it.** Pristine pre-edit copy saved as `trash/run_matrix.py.orig-2026-09-08` (copy it back), or change that one line back by hand. |

## Created paths (Pi, `~/ROS2-slam-hil`)

OV2SLAM fast stereo
- `camera_calib/hil_sim_ov2slam_stereo_fast.yaml` (derived from the accurate stereo calib: header + 4 keys)
- `config/hil/stack/ov2slam_stereo_oracle_fast.yaml`

ORB-SLAM3 stereo
- `src/orbslam3/Pangolin/` — copied from `~/ROS2-PROJECT-SPRING2026/src/orbslam3/Pangolin` (148 MB)
- `src/orbslam3/orb_slam3/Vocabulary/` — copied from the same checkout (47 MB)
- `src/orbslam3/build/`, `src/orbslam3/install/`, `src/orbslam3/log/`, `src/orbslam3/build_stereo_2026-09-08.log` — colcon build output
- `src/orbslam3/orb_slam3/config/Stereo/HIL_SIM.yaml`
- `config/hil/stack/orbslam3_stereo_oracle.yaml`

ORB-SLAM2 stereo
- `src/orbslam2/Vocabulary/ORBvoc.txt` — copied from `~/ROS2-PROJECT-SPRING2026/src/orbslam2/Vocabulary/ORBvoc.txt` (318 MB; `Thirdparty/` here holds only DBoW2/g2o, and a symlink into the other checkout would dangle inside the container mount)
- `src/orbslam2/build/`, `src/orbslam2/lib/`, `src/orbslam2/build_stereo/`, `src/orbslam2/install_stereo/`, `src/orbslam2/log/`, `src/orbslam2/build_stereo_2026-09-08.log` — core + wrapper build output (README_HIL.md recipe, minus `pip install evo`)
- possibly `src/orbslam2/Thirdparty/{DBoW2,g2o}/{build,lib}` if the core build created them (script moves them only if untracked)
- `src/orbslam2/ros2-ORB_SLAM2/src/stereo/hil_sim.yaml`
- `config/hil/stack/orbslam2_stereo_oracle.yaml`

RTAB-Map stereo
- `src/rtabmap_docker/hil_stereo_bridge.py`
- `src/rtabmap_docker/hil_stereo.launch.py`
- `config/hil/stack/rtabmap_stereo_oracle.yaml`

Tooling / docs
- `src/orbslam2/build_stereo_2026-09-08.sh` (the container build recipe actually run)
- `scripts/hil_matrix/synthetic_stereo_pub.py` (wiring-test publisher: ovcam-identical topics/QoS/stamps, correct-sign disparity)
- `scripts/hil_matrix/stereo_wiring_test.sh` (launches a sidecar exactly as `run_stack_hil.sh` would, from the real stack config via `parse_stack.py`, and probes the contract)
- `scripts/revert_stereo_integration_2026-09-08.sh`
- `docs/fixlog/028-orb-stereo-wrappers-published-nothing.md`, `HANDOFF_2026-09-08.md` (Pi copy of the Windows handoff)
- doc edits (not reverted by the script -- they are the record of what happened): `docs/TODO.md` (new
  top section), `docs/fixlog/README.md` (028 row + TODO-AT/AU/AV), `docs/fixlog/_active.yaml`,
  `docs/ideas_to_check/004-*.md` + its `README.md` row (IDEA-004 closed as a false alarm)
- `*.matrixbak` copies of the four `calib:` files, only if `run_matrix.py` aborted mid-matrix (script moves them if present)
- Windows: `scripts/hil_matrix/logs/live_chain_2026-09-08.{ps1,log}`, `scripts/hil_matrix/logs/live_only_*_stereo_oracle*.log` (trial logs), `trash/run_matrix.py.orig-2026-09-08`
- `docs/REVERT_2026-09-08_stereo_integration.md` (this file)
- `bags/run_*_stereo_oracle_2026090*` + `results/matrix_2026090*` produced by verification trials — left in place (bags are gitignored; they are evidence, not code)

## Created paths (Windows, `ROS2-PROJECT-SPRING2026`)
- `config/hil/matrix/only_ov2_stereo_oracle_fast.yaml`
- `config/hil/matrix/only_orbslam3_stereo_oracle.yaml`
- `config/hil/matrix/only_orbslam2_stereo_oracle.yaml`
- `config/hil/matrix/only_rtabmap_stereo_oracle.yaml`

## Ephemeral / not persisted
- Build containers (`orbslam3_build`, `orbslam2_build`) run with `--rm`; libs copied into them do not survive.
- Smoke-test containers/processes are stopped after each test; `/tmp/*` scratch on the Pi.
