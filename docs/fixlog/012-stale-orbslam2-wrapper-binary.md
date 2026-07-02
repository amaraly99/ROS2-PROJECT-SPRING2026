---
id: FIX-012
title: Stale ORB-SLAM2 ROS2 wrapper binary — /slam/tracking_state was never published
date: 2026-07-02
status: resolved
component: src/orbslam2 (install_stereo build artifact)
supersedes:
critic_verdict: n/a (deployment fix, not a design change — see note)
kiss_verdict: n/a
open_todos: []
---

## Symptom

First live test of FIX-011 (the SLAM-init gate): `orbslam2_eval --run-tag gatetest1`.
`/bench/state` went `SLAM_BLIND` → `SLAM_INIT_FAILED` after exactly 2 blind strafes —
the gate's designed fail-safe fired, holding the drone at zero. User: "it did 2
strafes and killed itself."

## Root cause

**Not a gate-logic bug.** Traced the bag (`ros2 bag` read via a throwaway
`ros2_perception_stack` container, since the persistent container had already been
torn down by `stop`) and cross-referenced against the SLAM sidecar's
`timing_events.csv` (its own wall-clock, offset ~4h from the bag's — container
timezone mismatch, confirmed and corrected for):

- `SLAM_BLIND` (gate start): bag t+0.00s
- ORB-SLAM2 reached `tracking_state==2` (OK) internally: bag-aligned **t+32.18s**
- `SLAM_INIT_FAILED` declared: bag t+**51.05s**

**ORB-SLAM2 had already initialized 18.87s before the gate gave up.** The FSM's new
`/slam/tracking_state` subscription (added in FIX-011) never received a message.

Static binary inspection nailed it:

```
install_stereo/orbslam/lib/orbslam/mono   — built Jun 28 22:39
ros2-ORB_SLAM2/src/monocular/monocular-slam-node.cpp — last modified Jun 29 10:15
```

The **source** containing the `tracking_state_pub_` publisher (`monocular-slam-node.
cpp:32,96-97`) was edited ~12 hours **after** the deployed `mono` binary was last
built. `strings -a` on the deployed binary confirmed: `slam_pose` present (pose
publishing works, matches FIX-010's success), `slam_tracking_state` **absent** —
the publisher code was never compiled in. (The CSV logging that made
`timing_events.csv` look complete and healthy is separate: it lives in
`libORB_SLAM2.so`, the core library, which *had* been rebuilt more recently — so the
CSV was accurate and up to date while the ROS2 wrapper executable was not. Two
different build artifacts, two different staleness states — easy to miss.)

`src/orbslam2/` has its own dedicated build path (`./start_container`, documented in
`HANDOFF.md` §5) separate from `./run_stack_hil.sh build` (whose `STACK_PKGS` list
does not include `orbslam`). Nothing in FIX-011's session touched
`src/orbslam2/ros2-ORB_SLAM2/` — the publisher code already existed from an earlier
session; it just never got rebuilt into the deployed artifact before now.

## Fix

Ran `cd src/orbslam2 && ./start_container` on the Pi: rebuilds the ORB-SLAM2 core
lib, refreshes runtime `.so`s, and rebuilds the ROS2 wrapper
(`colcon build --packages-select orbslam --install-base install_stereo`) — exactly
the step that compiles `monocular-slam-node.cpp` fresh. Verified post-rebuild:
`mono` binary mtime now Jul 2 18:46, `strings -a` finds `slam_tracking_state`
(previously absent), `slam_pose` still present (no regression).

No source change was needed — the fix is a deployment/build-freshness issue, not a
design or logic issue in FIX-011's gate.

## Critic / KISS note

Skipped the two-subagent review for this entry: SECOND BRAIN's critic+KISS step is
for design decisions with room for a wrong call; this is a mechanical rebuild of
already-existing, already-reviewed source with a fully evidenced root cause (binary
mtime older than source mtime, absent symbol confirmed via `strings`, present after
rebuild). Logged per the fixlog spirit for traceability, not skipped out of
laziness — if the live re-test (next step) shows a *different* failure, that
warrants full protocol.

## Open TODOs

- None yet — awaiting the next live `orbslam2_eval` run to confirm FIX-011's gate
  now sees `/slam/tracking_state` and transitions `BLIND → RELOCATE → DONE` as
  designed. If it still fails, re-open with fresh bag tracing (the method used here
  — bag + timing_events.csv cross-reference — is now a proven diagnostic pattern,
  worth keeping as a reusable playbook rather than re-deriving next time).
