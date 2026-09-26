---
id: IDEA-003
title: OV2SLAM stereo pairing stalls forever partway through a run ("Throw img0 -- Sync error", no recovery)
date: 2026-09-04
status: open — not implemented, root cause not fully confirmed
component: src/ov2slam_ros/src/ov2slam_node.cpp (sync_process())
---

## Symptom

Confirmed on a real recorded run (`bags/run_ov2slam_stereo_20260904_115958`,
the run that also proved FIX-024 fixed): `/slam/pose` had 66 messages, but
all of them landed in the *first* ~50 seconds of a 155-second run. Ground
truth (`/sim/drone_pose`) shows the drone genuinely stationary for that
first ~50s window (matches distance dist=0.000m between the pose at SLAM's
first message and the pose at SLAM's last message) — so the 66 messages
matching "barely moved" isn't wrong, it's a real stationary window. What's
actually wrong: for the remaining ~100 seconds, where the ground truth
shows the drone flying a real ~13m displacement, SLAM produced nothing at
all.

## Evidence

Persisted SLAM console log for that exact run (`/tmp/slam_sidecar_slam_
ov2slam.log`, FIX-021's persistence mechanism): right after a completely
normal frame-processing cycle (clean timing summary, no errors), the very
next line is:
```
Throw img0 -- Sync error : -65743
```
...and it never recovers — the exact same message (with slowly drifting
numbers, e.g. `-65804` a couple hundred lines later) repeats for the rest
of the log, thousands of times, until the log ends with the run.

## One confirmed code fact (not yet proven to be the full explanation)

Read `sync_process()` in `ov2slam_node.cpp` directly this session:
```cpp
double time0 = (double)img0_buf.front()->header.stamp.sec + 1e-9*(double)img0_buf.front()->header.stamp.nanosec;
double time1 = (double)img1_buf.front()->header.stamp.sec + 1e-9*(double)img0_buf.front()->header.stamp.nanosec;
//                                                                  ^^^^^^^^ uses img0_buf's nanoseconds, not img1_buf's
```
This is upstream/vendored OV2SLAM code, not something this project wrote —
a real copy-paste mistake: `time1`'s whole-seconds come from `img1_buf` but
its nanoseconds come from `img0_buf`. Worked through the algebra: because
of this bug, the img0_buf-nanosecond terms in `time0 - time1` cancel out
exactly, so the printed "Sync error" value always collapses to just
`img0_buf.front().sec - img1_buf.front().sec` — a whole number of seconds,
which matches the suspiciously round-looking printed values (`-65743`,
`-65804`).

Under this project's own pair-atomicity design (both eyes carry the
identical MATLAB source timestamp when synced correctly — confirmed this
session that BOTH `sim_camera_bridge` and `sim_camera_bridge_right` use
`use_source_stamp: true` for a stereo run), this bug should normally be
harmless: if both `.sec` fields genuinely match, `time0 - time1` is exactly
0 either way. The fact that it's printing large, persistent, non-zero
whole-second differences means the two buffers' `.sec` fields are
genuinely disagreeing by a large amount, repeatedly, with no recovery —
which is NOT explained by this bug alone. The bug's effect is on making
the failure total and unrecoverable (see next paragraph), not on causing
the initial disagreement.

**Why it can't be a graceful correction and instead becomes total,
permanent silence**: the corrupted comparison reduces to comparing whole
seconds only — `img0.sec < img1.sec` throws away img0 and pops it,
`img0.sec > img1.sec` throws away img1 and pops it, and because of the
bug neither branch ever properly re-evaluates using img1's real
nanoseconds, so once the two `.sec` fields diverge, the loop has no path
back to the matched branch. This matches the observed pattern exactly
(forever repeating, never one-off).

## What's NOT yet confirmed

- Which side (left ring, right ring) actually falls behind first, and why
  — a real stall (matching this exact same session's FIX-024 mechanism,
  possibly recurring on a different node or mid-run instead of at a
  stop/restart boundary — see `docs/fixlog/024-...md`'s open TODO-AQ,
  which flagged exactly this kind of mid-run stall as unguarded)? Or some
  other cause entirely (a genuine clock/timestamp source mismatch,
  a queue-depth/backpressure issue, something else)?
- Whether fixing the `time1` copy-paste bug alone (using img1_buf's own
  nanoseconds) would let the pairing recover on its own once frames
  resync, or whether the real fix has to be further upstream (why do the
  `.sec` fields diverge at all).

## Next concrete step (not started)

Cross-reference the exact moment the sync error starts against the
sim_camera_bridge / ovcam_bridge process states for *that* run (was either
side's shm reader mapping showing `(deleted)` at that timestamp, the same
signal that caught FIX-024?). If yes, this is very likely FIX-024's exact
mechanism recurring mid-run rather than a new, separate bug — which would
mean TODO-AQ (mid-run liveness gap, already logged in fixlog 024) is the
real fix needed, not a SLAM-internal change at all.
