---
id: FIX-025
title: OV2SLAM stereo mode never reports /slam/tracking_state==2 -- the only code path that sets it is mono-only, permanently, regardless of time or motion
date: 2026-09-04
status: resolved  # confirmed live via FIX-026 real trial: tracking_state reached 2, mission flew, 86 real poses recorded
component: src/ov2slam_ros/src/mapper.cpp (vendored/adapted OV2SLAM, ov2slam_ros package)
supersedes: none
critic_verdict: open_concern (both agents independently found the same gap -- see below)
kiss_verdict: open_concern (same gap; placement/threshold both independently confirmed justified)
open_todos: [TODO-AR]
---

## Symptom

Live automated stereo trial (run_matrix.py, only_ov2_stereo_oracle,
following FIX-024's shm fix and the set_stereo(1)/remap fixes from the same
session): SLAM genuinely processing both camera feeds (confirmed: real
per-frame timing-events count climbing 4->12 in the bag's own timing CSV),
but `/slam/tracking_state` stayed permanently at `1` ("not ready") for the
entire run -- not slow, not eventually reaching 2, *permanently* 1,
regardless of elapsed time (~7s init-cycle probe + ~15s of retries) or
drone motion (drone was near-stationary the whole window, which stereo
should not need motion to overcome). `run_matrix.py`'s own "verify SLAM
initialised" gate reads this and aborts the trial before ever flying the
mission -- correctly, per its own logic, but on a false signal.

## Root cause

Confirmed directly from source (not inferred): `/slam/tracking_state`
reports `pslam_->pslamstate_->bvision_init_ ? 2 : 1`
(`src/ov2slam_ros/src/ov2slam_node.cpp:85`). The ONLY place in the entire
codebase that ever sets `bvision_init_ = true` is
`src/ov2slam_ros/src/visual_front_end.cpp:109`, inside a block gated
`if( pslamstate_->mono_ && !pslamstate_->bvision_init_ )` -- a motion-based
2D-2D parallax bootstrap (`checkReadyForInit()`) that mono needs to
triangulate initial structure from camera motion. In stereo mode `mono_`
is false, so this entire block -- and therefore `bvision_init_` -- never
executes, for the whole life of any stereo run, unconditionally. Not a
timing issue; would never resolve no matter how long a trial ran.

Stereo does not need this motion-based bootstrap at all: it triangulates
real 3D points directly from the fixed, known camera baseline on every
keyframe, via `mapper.cpp`'s `triangulateStereo(*pnewkf)` call, which is
unconditional on `mono_`/`stereo_` and already running correctly (confirmed
via the ">>> (AFTER STEREO TRIANGULATION)" debug log firing normally).
Stereo was already functionally working -- proven earlier this session by
a real recorded bag with 66 genuine `/slam/pose` messages -- it simply had
no path to ever report "ready" under the signal this project's own
init-gate/matrix-runner infrastructure depends on.

## Diff

`src/ov2slam_ros/src/mapper.cpp`, one new block inserted immediately before
the pre-existing, untouched mono-only "bad init, reset" check:

```cpp
            // Stereo mode has no motion-based bootstrap step (unlike mono's
            // visual_front_end.cpp checkReadyForInit() gate) -- every keyframe
            // triangulates real 3D points directly from the fixed camera
            // baseline above, so "ready" just means the first keyframe with
            // enough of those points has been produced. Non-blocking: unlike
            // the mono block below, this never resets or skips a keyframe,
            // it only flips a flag once.
            //
            // Threshold (30) reuses the SAME number visual_front_end.cpp:495
            // already uses for stereo specifically (`stereo_ && nb3dkps_ > 30`,
            // there deciding whether to prefer 3D keypoints for epipolar
            // filtering) -- a real stereo precedent, not the mono kfid_==1
            // reset bar a few lines below, which is a different mode and a
            // different, motion-dependent point count. Still: nothing in this
            // codebase establishes that bar transfers to meaning "ready" --
            // that's reused, not independently validated for this purpose.
            //
            // Known gap, not fixed here (flagged in fixlog TODO-x): mono has
            // a recovery path if its first "ready" reading turns out wrong
            // (mapper.cpp:129 below, kfid_==1 && nb3dkps_<30 -> reset()),
            // gated to mono_ and untouched by this change. Stereo has no
            // equivalent -- once this flips true here, it never reverts for
            // the rest of the run, even if that first batch of points was
            // bad (e.g. a calibration issue, not just per-point noise -- this
            // project has shipped one of those before, see commits 11fa007 /
            // 4839666). Each point is still cheirality- and reprojection-
            // filtered by triangulateStereo() above, so this isn't reading
            // raw noise -- but it has no defense against a systematic error
            // that would pass those same per-point checks.
            if( pslamstate_->stereo_ && !pslamstate_->bvision_init_
                && pnewkf->nb3dkps_ >= 30 )
            {
                pslamstate_->bvision_init_ = true;
            }
```

Placement in `mapper.cpp` (not alongside the mono block in
`visual_front_end.cpp`) is structurally required, not stylistic -- see KISS
verdict below.

## Critic verdict & concerns

**open_concern** (not `wrong_fix`). Independently re-verified the root
cause directly (read both files live on the Pi, confirmed license/vendored
provenance via `ov2slam_ros/README.md`, grepped every consumer of
`bvision_init_`/`nb3dkps_`). Confirmed `nb3dkps_` is not raw/noisy --
`triangulateStereo` rejects each candidate on cheirality and per-image
reprojection error before counting it. Confirmed the reused threshold of
30 has real, better precedent than the diff's original comment claimed
(fixed in the polish pass below).

**Real gap found, root-caused precisely**: `ov2slam_node.cpp`'s own
existing comment explains that `init_gate`'s `READY_DEBOUNCE=2` exists
specifically because `bvision_init_` is expected to be able to flip back
to false if mono's post-init sanity check (the `mono_`-gated block just
below this diff) rejects a bad first keyframe. This diff's stereo path has
no equivalent revert mechanism -- once true, always true for the rest of
the run. Not a new failure mode this diff introduces (stereo never had
this safety net; the gap was simply unreachable before, since `bvision_init_`
never went true at all) -- but the diff is what first makes the gap
consequential.

## KISS verdict

**open_concern**, same finding as critic, converged independently.
Additionally confirmed: placement in `mapper.cpp` (not `visual_front_end.cpp`,
where the mono block lives) is structurally necessary, not a style choice --
`map_manager.cpp`'s `addKeyframe()` copies the current frame into an
independent `Frame` object (`pnewkf`) that only `mapper.cpp`'s thread
triangulates; `visual_front_end.cpp`'s per-frame function runs
asynchronously to that and does not have this keyframe's fresh stereo
point count available without adding new cross-thread synchronization --
a bigger, riskier change than this fix. Also independently found a better
precedent for the `30` threshold than the diff originally cited
(`visual_front_end.cpp:495`, an existing stereo-specific use of the same
number) -- folded into the deployed comment. Minor nit (folded in): the
guard now reads `pslamstate_->stereo_` instead of the deployed diff's
original `!pslamstate_->mono_`, for clarity (`mono_`/`stereo_` are
independent config bools in this codebase, not a complementary pair, even
though this project's configs only ever set one).

## Verification

1. `bash`/C++ compile: rebuilt via `./run_stack_hil.sh build ov2slam` --
   clean, 44.2s, only pre-existing unrelated OpenCV library-version link
   warnings (same warnings present before this change, confirmed by their
   content referencing `libopencv_*.so.406` vs `.410` version conflicts,
   unrelated to this diff).
2. **Not yet done**: a real live trial confirming `/slam/tracking_state`
   actually reaches `2` and the mission flies past the init-cycle gate.
   Requires the user to run `run_matrix.py` again (matches this session's
   established pattern -- live trials are user-initiated, not run by the
   assistant). `status: applied-pending-live-verification` reflects this;
   will update to `resolved` once a real run confirms it, or back to
   investigation if it doesn't.

## Open TODOs

- **TODO-AR** (active, not blocking): stereo has no recovery path if its
  first "ready" reading is wrong -- unlike mono, which can revert via
  `mapper.cpp:129`'s `mono_`-gated reset-on-bad-first-keyframe check. Both
  review agents independently flagged this. Options for later, not decided:
  (a) generalize the existing mono reset block's `kfid_==1` check to also
  cover `stereo_`, giving stereo the same one-time revert path; (b)
  explicitly document that stereo's readiness signal is intentionally
  weaker than mono's on this one axis, since `init_gate`/this project's
  matrix runner treat both as equivalent through the same `tracking_state`
  contract. Not fixed now -- flagged, not blocking, since each stereo point
  is still per-point filtered (cheirality + reprojection) even without this
  additional safety net, and the actual failure mode (a systematic
  calibration bug slipping through) has occurred before in this project but
  is not known to be currently present.
