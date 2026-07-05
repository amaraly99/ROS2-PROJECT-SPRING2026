---
id: FIX-014
title: INITIALIZER_GATE support for OV2SLAM — bvision_init_ republished as /slam/tracking_state
date: 2026-07-05
status: resolved
component: src/ov2slam_ros/, src/init_gate/, run_stack_hil.sh, config/hil/stack/
critic_verdict: partial
kiss_verdict: simple
open_todos: [TODO-Y, TODO-Z]
---

## Symptom

`INITIALIZER_GATE` (`src/init_gate/`) only worked with `slam.type=orbslam2` —
`run_stack_hil.sh` hard-`die`d if `init_gate.enabled=true` was combined with any other
backend. User asked: why, and can OV2SLAM get the exact same gate.

## Root cause

The gate's readiness check (`slam_readiness.py`) subscribes to `/slam/tracking_state`
(`std_msgs/Int32`, `2`=ready), which only ORB-SLAM2's wrapper published — a thin passthrough
of its own upstream `Tracking::eTrackingState` enum via a pre-existing getter,
`System::GetTrackingState()`. OV2SLAM has no equivalent persistent enum or getter; it only
has two low-level booleans inside `SlamParams` (`bvision_init_`, `breset_req_`), never wired
to any topic. `bvision_init_` (set `true` once mono init succeeds, `visual_front_end.cpp:98-107`;
reset to `false` by `SlamParams::reset()`, `slam_params.cpp:175-179`, independently verified
by reading the file directly) is a live, correctly-toggling equivalent to ORB-SLAM2's OK
state — just never exposed.

A first attempt at this (made by the user mid-session, in `ov2slam.cpp`/`ov2slam.hpp`) tried
to add a publisher + timer directly inside `SlamManager`, which is not a ROS node and has no
`create_publisher`/`create_wall_timer` — would not compile. It also had a stale `void` return
type omission, set the "ready" flag at the top of `run()` (thread start, not real init), and
captured the flag by value in `std::bind` (frozen at construction time, never updated). Fully
reverted — none of it reusable.

## Diff

- **`src/ov2slam_ros/src/ov2slam_node.cpp`**: `SensorsGrabber` (the class whose
  `sync_process()` already runs a continuous loop on its own thread, dereferencing
  `pslam_->pslamstate_` elsewhere) gets a new `tracking_state_pub_` (`std_msgs/Int32`,
  constructed via the file-global `nh`, already valid at construction time — verified) and a
  `publishTrackingState()` helper, called once per frame actually handed to OV2SLAM
  (`addNewMonoImage`/`addNewStereoImages` call sites), publishing `2` if `bvision_init_` else
  `1`. Mirrors ORB-SLAM2's own per-frame publish cadence exactly.
- **`run_stack_hil.sh:313-321`**: relaxed `[[ "$SLAM_TYPE" == "orbslam2" ]]` to a `case`
  accepting `orbslam2|ov2slam` (matches this file's existing `case "$MODE" in ...` idiom four
  lines above).
- **`config/hil/stack/ov2slam_oracle.yaml`**: added the same `slam.remap` entry
  (`slam_tracking_state:=/slam/tracking_state`) and `init_gate: enabled: true` block
  `orbslam2_eval.yaml` already has. `orbslam2_eval.yaml`'s own comment updated (was stale —
  said OV2SLAM had no equivalent).
- **`src/init_gate/init_gate/slam_readiness.py`**: comment-only update. **Zero logic
  changes** — the gate only ever depends on the topic contract, not on which backend
  publishes it, confirmed by both review agents independently.

## Critic verdict & concerns

**Verdict: directionally right, two real issues found — one fixed, one logged as a TODO.**

**Fixed during review**: the first implementation published unconditionally every ~1ms
`sync_process()` poll tick (not gated on whether a frame was actually processed), to fix a
separately-discovered late-subscriber bug (below). Critic found this created a *second*,
more serious problem: `mapper.cpp:129-135` (verified directly) rejects the first keyframe
after `bvision_init_` goes true if it has `nb3dkps_ < 30`, triggering a full reset — on a
**third thread** (the Mapper thread) neither the original design nor its comments accounted
for. At a 1kHz artificial publish rate, `init_gate`'s `READY_DEBOUNCE=2` check could latch in
~2ms — nowhere near enough real time for that reset-check to run before the gate hands off to
the main flight stack. Fixed by moving the publish call to fire only on frames OV2SLAM
actually processes (matching ORB-SLAM2's real per-frame cadence exactly), giving the
Mapper's reset-check a realistic wall-clock window comparable to what ORB-SLAM2 implicitly
already has, without touching `cycle.py`/the gate's own one-shot-latch design.

**Also verified and fixed independently, before the critic pass**: an earlier "publish only
on state *change*" version (to reduce message volume) was live-tested on the actual Pi target
and found broken — the topic's QoS is `Durability: VOLATILE` (confirmed via
`ros2 topic info -v`), so a late-subscribing `init_gate` could permanently miss a one-shot
transition that happened before it connected. Verified live: a subscriber connecting 8s after
node startup received nothing under change-only publishing, and correctly received the
current value under unconditional publishing. The per-frame-cadence fix above preserves this
property (a late subscriber still gets the current value within ~1 real frame interval).

**Logged as TODO, not fixed** (see Open TODOs): `bvision_init_` is only ever set inside
`if(pslamstate_->mono_ && !pslamstate_->bvision_init_)` (`visual_front_end.cpp:98`) — for a
stereo OV2SLAM config it would stay `false` forever, and the relaxed `run_stack_hil.sh` guard
checks backend name only, not mono/stereo. Any future stereo-OV2SLAM + `init_gate.enabled`
config would deterministically time out and fail every single run. Not fixed now: no such
config exists in this repo today (the only OV2SLAM config with the gate,
`ov2slam_oracle.yaml`, is mono), and a real runtime guard would need YAML-parsing plumbing
that doesn't currently exist in `run_stack_hil.sh`'s bash validation. Also flagged, lower
priority: switching the publisher to `TRANSIENT_LOCAL` QoS (depth 1) would make late-joiner
delivery a guarantee rather than something that depends on the node never stopping — not
done now since the per-frame-cadence fix already resolved the concrete bug found live.

**Also confirmed clean during review** (no action needed): the constructor/lifetime ordering
(`nh` assigned before `SensorsGrabber` is constructed) checks out against the actual `main()`
sequence; non-gated / ORB-SLAM2 configs are unaffected; the "same informal risk as
`bis_on_`/`bexit_required_`" framing for the unsynchronized cross-thread bool read is not the
real risk here (a torn/stale read costs ~1ms at worst) — the actual risk was the non-monotonic
reset-shortly-after-init behavior above, which `std::atomic<bool>` would not have fixed.

## KISS verdict

**Simplest correct fix.** Reusing ORB-SLAM2's own `1`/`2` numbering (rather than a fresh
boolean topic or custom message type) was confirmed as the right call — it's what keeps
`slam_readiness.py` (which hardcodes `TRACKING_OK = 2`) at zero changes, which was the actual
goal. Constructing the publisher via the pre-existing global `nh` inside `SensorsGrabber`'s
constructor, and calling the new helper from the existing per-frame processing call sites
(rather than a new dedicated `rclcpp::TimerBase`, or hooking directly into
`visual_front_end.cpp` where `bvision_init_` changes — the latter is exactly the on-change
design already tried and found broken under `VOLATILE` QoS), adds zero new threads/timers and
reuses infrastructure that already runs for the node's full lifetime. `run_stack_hil.sh`'s
`case` statement matches an existing four-line-away precedent in the same file, not a
stylistic detour. One doc-drift nit found and fixed: `orbslam2_eval.yaml`'s `init_gate`
comment was stale (still said OV2SLAM had no equivalent) — updated in the same pass.

## Gotcha: config sync clobbered a Pi-only manual edit

While syncing changed files to the Pi via `scp`, `config/hil/stack/orbslam2_eval.yaml`
overwrote the Pi's own manual `detector.cpu`/`controller.cpu` pins (`"0"`/`"1"`) with the
Windows copy's original `""`/`""` values — a standing "do not touch without being asked"
edit documented in `HANDOFF.md`. Caught immediately after by explicitly diffing the Pi's file
post-sync (not assumed clean), and restored via a targeted two-line `sed` on the Pi only,
without bringing the pin back into the Windows copy (the divergence between the two machines
on this specific file is intentional). Lesson: `scp`-ing a whole file that's known to have
machine-local divergences risks silently clobbering them — diff first, or patch only the
specific lines that actually changed, rather than syncing the whole file wholesale.

## Open TODOs

- **TODO-Y** (this entry): stereo OV2SLAM + `init_gate.enabled=true` would deterministically
  fail every run (`bvision_init_` never leaves `false` outside mono mode). No such config
  exists today; if one is ever proposed, either extend `bvision_init_`'s semantics to stereo
  in `visual_front_end.cpp`, or add a real guard (needs YAML-parsing plumbing not currently
  in `run_stack_hil.sh`).
- **TODO-Z** (this entry): consider `TRANSIENT_LOCAL` QoS (depth 1) on
  `tracking_state_pub_` for a guaranteed (not just currently-true) late-joiner delivery
  property, if the per-frame-cadence fix ever turns out insufficient in practice.
