# orbslam3 — vendored ROS 2 wrapper for ORB-SLAM3

## Provenance (this matters for the benchmark — record it in any writeup)

| | |
|---|---|
| ROS 2 wrapper | `Mechazo11/ros2_orb_slam3` **v2.0.0** (Jazzy branch) |
| upstream URL | https://github.com/Mechazo11/ros2_orb_slam3 |
| SLAM core | **ORB-SLAM3 V1.0** (UZ-SLAMLab) |
| ROS 2 distro | Jazzy — the wrapper README states v2.0.0 is **not** Humble-compatible |
| vendored from | `~/ORBSLAM3_ROS2/src/` on the Pi, 2026-08-03 |
| upstream commit | UNKNOWN — the source tree had no git remote or .gitmodules |

**The commit is unrecorded.** The tree was copied without version control, so the
exact upstream revision cannot be recovered. If this backend goes into a
published comparison, re-clone from upstream at a pinned tag and diff against
this tree before trusting the identification above.

## Why this needed modification

Upstream is built for **offline dataset playback**, not live cameras:

- `scripts/mono_driver_node.py` reads JPEGs from a folder and feeds the C++ node
  over a custom handshake (`/experiment_settings` -> `/exp_settings_ack`, then
  `/img_msg` + `/timestep_msg` per frame). There is no subscription to a live
  image topic.
- `src/common.cpp:570` calls `pAgent->TrackMonocular(...)` and **discards the
  returned `Sophus::SE3f`**. The node publishes only a config-ack String and a
  PointCloud2 of map points — no pose topic at all. Upstream's trajectory output
  is the `CameraTrajectory_*.txt` file written at shutdown.

The HIL stack's backend-agnostic contract is `/slam/pose` (PoseStamped) +
`/slam/tracking_state` (Int32), so both gaps had to be closed. See the HIL
section below for what was added on top of upstream.

## Local modifications on top of upstream

(kept deliberately small and marked with `HIL:` comments so they can be
re-applied if the wrapper is ever re-vendored)

1. `src/common.cpp` — capture the `TrackMonocular` return, publish
   `/slam/pose` and `/slam/tracking_state`, and emit per-frame
   `frontend/full_tracking` timing so latency is comparable with OV2SLAM and
   ORB-SLAM2 (upstream has no instrumentation).
2. `scripts/live_adapter_node.py` — subscribes to the live camera topic and
   drives upstream's handshake, replacing the bag/folder reader. Modelled on the
   existing `rosbag_adapter_node.py`, which already implements the handshake and
   shutdown state machine correctly.
