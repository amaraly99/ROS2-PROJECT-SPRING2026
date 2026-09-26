---
id: FIX-022
title: ov2slam_stereo.yaml's startup_delay_sec reduced 15s -> 2s -- copy-pasted value, never tuned for OV2SLAM, real auto-exit guard is a silence watchdog not a time gate
date: 2026-09-02
status: resolved
component: config/hil/stack/ov2slam_stereo.yaml
supersedes: none
critic_verdict: open_concern
kiss_verdict: n/a (single-value change, no simplification dimension -- agent not dispatched)
open_todos: []
---

## Symptom

A real stereo test run (Sep 1, 22:30) was stopped before OV2SLAM ever
received a camera frame -- its persisted log (FIX-021) showed clean
initialization through "OV2SLAM is ready to process incoming images!" and
then nothing. Root cause of *that*: `startup_delay_sec: 15` means OV2SLAM
does not even start running for the first 15 seconds after its sidecar
container launches -- easy to stop a run in that window mistaking silence
for "nothing is happening" (correct -- nothing was, yet, by design).

## Root cause / investigation

The delay exists on purpose, per `run_stack_hil.sh`'s own comment: SLAM is
started a beat after the drone begins moving so it has some translation to
build parallax from before it starts receiving frames, else it hits a
documented auto-exit failure mode.

Critic verified this claim against OV2SLAM's actual source
(`src/ov2slam_ros/src/ov2slam.cpp`, `SlamManager::run()`, ~line 208-215)
rather than trusting the comment at face value, and found the real
mechanism is different from what the comment implies: it's a **camera-
silence watchdog** (exits after `100x cam_delay` with no new frame), not a
parallax/motion check -- and `cam_delay` is floored at 0.5s specifically
"so intermittent streams... cannot trigger auto-exit until 50s of complete
silence" (per that function's own comment, added in commit `80afa9f`,
confirmed present in the currently-running binary by file mtime). So the
15s value was guarding against a failure mode that tolerates up to 50
seconds of silence -- a 2s startup delay is nowhere near that threshold.

Also found via `git log -p` on this exact config file: `startup_delay_sec:
15` was copy-pasted verbatim across the "Migrate stereo capability"
commits -- never independently tuned or tested for OV2SLAM specifically.
Not a carefully-chosen number; a default that rode along.

One separate, unrelated fact surfaced along the way (informational, not
part of this fix): this stack's own SEARCH behavior is 13s of pure rotation
(`search_full_rotate_sec_`) before any translating strafe, so parallax-
readiness was never actually gated by this config value in the first place
-- SLAM starting at t=2s or t=15s lands in a similarly near-zero-
translation window either way. Not touched here; noted for whoever next
looks at SLAM init timing relative to the search pattern.

## Diff

`config/hil/stack/ov2slam_stereo.yaml`, line 58:
```diff
-  startup_delay_sec: 15
+  startup_delay_sec: 2
```
Single value, single file.

## Critic verdict & concerns

**open_concern**, proceeded per protocol. The one gap it flagged: the
silence-watchdog floor-fix being already in the running binary was inferred
from file timestamps (source file mtime vs. binary mtime), not confirmed
live -- so it asked for a real test run before calling this resolved, not
just trusting the source-reading.

## KISS verdict

Not applicable -- single-value change, no alternative form to weigh, so the
simplification agent was not dispatched for this one (noted explicitly to
the user rather than silently skipped).

## Verification

Did the real test the critic asked for, not just applied-and-assumed:
- Launched the real stack with the new value. Sidecar container came up,
  `docker ps` confirmed "Up" almost immediately (well inside the old 15s
  window).
- Persisted log (`/tmp/slam_sidecar_slam_ov2slam.log`, FIX-021) showed the
  full clean init sequence through "OV2SLAM is ready to process incoming
  images!" -- same successful init as before, just reached far sooner.
- Waited 15 more seconds past that point specifically to check for the
  silence-watchdog firing late: `docker inspect` showed `RestartCount=0`,
  `Status=running` the whole time. No auto-exit, no crash-loop.
- Stopped the stack cleanly afterward via `./run_stack_hil.sh stop`.
- Not verified in this pass: real frame processing / keyframe tracking
  under a live MATLAB feed (no MATLAB session was running during this
  specific test) -- this fix only verifies SLAM survives its own startup
  at the shorter delay, not full end-to-end tracking quality. That's the
  next real run's job.

## Open TODOs

None new from this fix. The unrelated SEARCH-pattern-vs-parallax-timing
observation above is worth a note somewhere if the actual `init_gate`
wiring for this stack (still pending, separate task) ends up caring about
it, but nothing to track as its own item yet.
