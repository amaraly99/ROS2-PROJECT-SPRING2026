---
id: FIX-028
title: ORB-SLAM3 and ORB-SLAM2 stereo wrappers tracked frames but never published /slam/pose or /slam/tracking_state -- the HIL pose contract had only been ported to the mono paths
date: 2026-09-08
status: resolved
component: src/orbslam3/src/common.cpp (StereoMode::ProcessStereoPair), src/orbslam2/ros2-ORB_SLAM2/src/stereo/stereo.cpp
supersedes: none
critic_verdict: open_concern (root cause confirmed; objections recorded below, none blocking)
kiss_verdict: simplify-recommended (ORB-SLAM2 half only -- see KISS verdict)
open_todos: [TODO-AT, TODO-AU, TODO-AV]
---

## Symptom

Stage 0 (design gate, `rule-out-by-design`) came first: is "a stereo sidecar
publishes nothing" something the code intends? Read the drivers -- no. The
stack's SLAM contract is backend-agnostic (`/slam/pose` PoseStamped +
`/slam/tracking_state` Int32, polled by `run_matrix.py:845-903`), and the
vendored ORB-SLAM3 README lists "capture the TrackMonocular return, publish
/slam/pose and /slam/tracking_state" as the HIL modification -- applied to
`MonocularMode::Img_callback` (`common.cpp:669-671`) only. The stereo path was
simply never ported. Verified, not assumed:

- ORB-SLAM3: `scripts/hil_matrix/stereo_wiring_test.sh orbslam3_stereo_oracle`
  (synthetic stereo pair on the real `/ovcam/*` topics, sidecar launched with the
  exact `parse_stack.py`-derived command/image/ld_prefix of the stack config):
  `tracking_state samples over ~45s: none none none none none none`,
  `/slam/pose:` empty -- while the sidecar log showed `handshake ACKed`,
  `New Map created with 318 points`, and `timing_events.csv` grew to 4558 rows.
  Tracking ran every frame; nothing was published. Cause by reading:
  `StereoMode::ProcessStereoPair` (`common.cpp:773-812`) called
  `pAgent->TrackStereo(...)` at `:805` and discarded the return; `PublishPose`
  is never called on that path.
- ORB-SLAM2: found by reading `stereo.cpp` before any test. The upstream
  wrapper discards `TrackStereo`'s result and has no publishers at all (the mono
  node `monocular-slam-node.cpp:97-150` has both). Additionally `main()` checked
  raw `argc != 3`, but `run_stack_hil.sh:236` appends `--ros-args ${SLAM_REMAPS}`
  to every sidecar command (`mono.cpp` already strips them with
  `rclcpp::remove_ros_arguments`), so the stock stereo binary would have printed
  its usage line and exited under the stack even if it had publishers. The
  `orbslam2_fixed` image ships no `stereo` executable at all, and this worktree
  had no `install_stereo/`, `lib/` or `Vocabulary/` (same class of gap as
  FIX-018 -- the mono ORB-SLAM matrix configs quietly point `pi_repo` at the
  other checkout, `~/ROS2-PROJECT-SPRING2026`, where those builds live).

## Root cause

An unfinished HIL port: the pose/tracking-state publishing added for the
benchmark lived only in the two mono callbacks. Tier 2 (project source) per
`occams-razor-before-escalate`; nothing deeper was involved.

## Diff

`src/orbslam3/src/common.cpp`, `StereoMode::ProcessStereoPair` (2+/1-):
```
-    pAgent->TrackStereo(left_cv_ptr->image, right_cv_ptr->image, timestamp);
+    const Sophus::SE3f Tcw = pAgent->TrackStereo(left_cv_ptr->image, right_cv_ptr->image, timestamp);  // HIL: was discarded
     const auto after_track = std::chrono::steady_clock::now();
+    PublishPose(Tcw, timestamp);  // HIL: same contract as MonocularMode::Img_callback
     PublishMapPointCloud(timestamp);
```
`PublishPose` is the existing `OrbSlamNodeBase` method (`common.cpp:529`) the
mono callback already uses; it publishes `slam_pose` and `slam_tracking_state`
(relative names, remapped by the stack yaml). A first version of this patch
duplicated the `PublishMapPointCloud` call that already followed `after_track`;
the simplification agent flagged it and the duplicate was removed before the
final rebuild (verified: `git diff --stat` = 2 insertions, 1 deletion).

`src/orbslam2/ros2-ORB_SLAM2/src/stereo/stereo.cpp` (71+/3-): two publishers
(`slam_pose`, `slam_tracking_state`) on the wrapper's node; after
`TrackStereo`, publish `GetTrackingState()` every frame and, when `Tcw` is a
valid 4x4, the `Twc = Tcw.inv()` pose with the mono node's rotation->quaternion
block copied verbatim; `main()` uses `rclcpp::remove_ros_arguments` like
`mono.cpp`. Everything else (message_filters approximate-time pairing of
`camera/left` + `camera/right`, benchmark timers, trajectory dump) untouched.

Built: ORB-SLAM3 with colcon in `orbslam3_container` (`src/orbslam3/{build,install}`,
Pangolin + Vocabulary copied from the other checkout), ORB-SLAM2 via the
documented `README_HIL.md` recipe (`src/orbslam2/build_stereo_2026-09-08.sh`,
minus its `pip install evo` download). Every created path is listed in
`docs/REVERT_2026-09-08_stereo_integration.md`.

## Critic verdict & concerns

Critic agent (delegation template verbatim, both diffs): **open_concern**, not
wrong_fix. Root cause independently confirmed in the source (`common.cpp:805`
discards `TrackStereo`; publishers created in the base ctor `:121-122`;
`PublishPose` protected; `OK=2` in `Tracking.h:130`; `remove_ros_arguments`
keeps argv[0] so `size()==3` is right).

Strongest objection: `stereo.cpp`'s `message_filters::Subscriber`s use the
default RELIABLE QoS, so a BEST_EFFORT image publisher would never match and
the symptom would look identical to "fix didn't work". Checked: not a live
risk -- `ovcam_bridge_node.cpp:54-55` publishes RELIABLE KeepLast(2), and the
wiring-test publisher (`synthetic_stereo_pub.py`) publishes RELIABLE depth 2
by construction (mirrors the bridge). Recorded here so the next person who
feeds ORB-SLAM2 from a best-effort source (e.g. `scripts/stereo_sync_test.py`)
knows why it goes silent.

Secondary concerns:
1. duplicate `PublishMapPointCloud` -- see Diff, fixed before the final build;
2. contract divergence: ORB-SLAM3 publishes a pose only when state==2, the
   ORB-SLAM2 half publishes whenever `Tcw` is non-empty, and `Tracking.cc:793/901`
   set `mTcw` before failure paths, so LOST frames can emit stale/motion-model
   poses. This matches the mono ORB-SLAM2 node exactly (all existing ORB-SLAM2
   results were scored that way) -- kept for comparability, logged as TODO-AV;
3. ORB-SLAM3 `CreateMapInAtlas()` after a LOST with >10 KFs starts a new origin
   under the same `frame_id="map"` -- pre-existing in mono, unchanged;
4. the agent could not see `config/Stereo/HIL_SIM.yaml` on the Windows mirror
   -- it exists on the Pi and is the settings file the wiring test and the
   live trial used.

## KISS verdict

Simplification agent: **open_concern / simplify-recommended**.
1. Strongest objection: the duplicated `PublishMapPointCloud` in the first
   ORB-SLAM3 patch (full-map PointCloud2 built twice per frame when map-point
   publishing is on). Fixed before the final rebuild, see Diff.
2. ORB-SLAM3 half is the simplest correct fix (mirrors the mono callback).
3. ORB-SLAM2: the copied 40-line quaternion block could be
   `ORB_SLAM2::Converter::toQuaternion(R)` (`include/Converter.h:52`, already
   linked -- `System::SaveKeyFrameTrajectoryTUM` uses it). Not applied: the
   copied block is byte-identical to the mono node's proven code, and swapping
   it would have meant rebuilding the binary that was already queued for its
   live trial. Logged as TODO-AT.
4. Its config concern (no `Camera.bf`/`ThDepth`, no stereo stack yaml) refers to
   files the agent could not see on the Windows mirror; both exist on the Pi
   (`stereo/hil_sim.yaml`: `Camera.bf: 60.94` = 554 x 0.11 per FIX-027 -- not
   the stale 1200 x 0.11 the agent quoted from `ov2slam_stereo.yaml` --
   `ThDepth: 60`; `config/hil/stack/orbslam2_stereo_oracle.yaml` with the
   `camera/left`/`camera/right` remaps).

## Verification

Wiring tests (`scripts/hil_matrix/stereo_wiring_test.sh`, final binaries):
- ORB-SLAM3: `tracking_state samples: 2 2 2 2 2 2`, `/slam/pose average rate:
  13.0-15.0 Hz`, `New Map created with 358 points`, timing CSV growing.
- ORB-SLAM2: `tracking_state samples: 2 2 2 2 2 2`, `/slam/pose 15.0 Hz`,
  `New map created with 414 points`, `timing_events.csv` 4953 rows
  (`wrapper/callback_total`, `frontend/full_tracking`).

Live HIL trials (`run_matrix.py --trials 1`, orchestrator-driven MATLAB,
`set_stereo(1)`, oracle detector, `--hold-fsm`, 2026-09-08, WiFi profile):

| arm | bag | init gate | scored poses | APE RMSE | scored path | scale | track ms | pose Hz |
|---|---|---|---|---|---|---|---|---|
| `orbslam2_stereo` | `run_orbslam2_stereo_oracle_20260908_100539` | YES (`tracking_state=2`, 2.9 s) | 282 (581 non-zero in bag) | **0.619 m** (1.84 %/m) | 33.63 m | 1.119 | 44.8 | 11.6 |
| `orbslam3_stereo` | `run_orbslam3_stereo_oracle_20260908_100914` | YES via `handshake ACKed` path, `tracking_state=2` (8.3 s) | 271 (652 non-zero in bag) | **0.700 m** (2.07 %/m) | 33.76 m | 1.223 (drift 1.9 %) | 33.0 | 11.6 |

Both `VALID 1/1` in `aggregate_matrix.py` (`results/matrix_20260908_100347`,
`results/matrix_20260908_100722` on the Pi); orchestrator logs
`scripts/hil_matrix/logs/live_only_orbslam{2,3}_stereo_oracle.log` (Windows).
The ORB-SLAM3 arm exercised the `run_matrix.py` `startswith("orbslam3")` change
("SLAM ready (handshake ACKed)"). Scale 1.12 / 1.22 rather than ~1.0 is a
first-trial number, not a calibration verdict (n=1; OV2SLAM's accurate stereo
arm also read 0.88 on its first post-FIX-027 trial before settling at 0.98);
worth watching across a 10-trial matrix before drawing conclusions.

## Open TODOs

- **TODO-AT**: replace the copied quaternion block in `stereo.cpp` with
  `ORB_SLAM2::Converter::toQuaternion` (KISS); needs a rebuild + wiring re-test.
- **TODO-AV**: ORB-SLAM2 (mono and now stereo) publishes a pose on LOST frames
  (stale/motion-model `mTcw`), ORB-SLAM3 does not (state==2 gate). Decide
  whether the aggregator should drop `tracking_state != 2` samples for all arms
  so the two ORB backends are scored on the same rule.
- **TODO-AU**: the mono ORB-SLAM matrix configs (`only_orbslam2.yaml`,
  `only_orbslam3.yaml`) still point `pi_repo` at `~/ROS2-PROJECT-SPRING2026`
  (and the old wired address); the mono builds have never existed in
  `~/ROS2-slam-hil`. Decide whether mono arms should move to this worktree (the
  stereo builds made tonight already give ORB-SLAM3 its `mono_node_cpp` here).
