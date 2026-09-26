---
id: IDEA-004
title: Ground-truth pose (/sim/drone_pose) freezes mid-run in stereo+oracle benchmark trials only -- ten black-box variables individually disproven, root cause not found
date: 2026-09-06
status: CLOSED 2026-09-08 -- false alarm, not a bug (see RESOLUTION at the end)
component: matlab/hil_ros_init_LT.m (publish_state_LT / pose_pub), MATLAB ROS Toolbox ros2publisher/send() (suspected, unverified -- see below)
---

## Symptom

Every `only_ov2_stereo_oracle` benchmark-mode trial run this session (9/9,
all `--hold-fsm`, oracle detector, `slam_traj_probe.m`-driven, git_sha
`6e7bf48`) showed `/sim/drone_pose` in the recorded bag going stale
partway through the run: the topic keeps publishing at its normal 20 Hz
rate (message *count* looks healthy), but the *value* stops changing for
tens of seconds at a time, then sometimes resumes. This silently corrupts
downstream RMSE/scale scoring in `aggregate_matrix.py` (Umeyama Sim(3)
alignment against a ground truth that isn't actually moving is meaningless
for the frozen segment). Confirmed via bag decode (`export_bag_csv.py`,
built this session) and via a live watchdog (`pose_watch.py`, built this
session, subscribes to the topic directly and flags >3s of unchanged
value) — both agree on the pattern: frozen at spawn for the first ~0-50s,
tracks real motion for a window in the middle, frozen again afterward.

MATLAB's own internal `sim_pose` (read directly via `evalin('base',
'sim_pose')` inside debug instrumentation added to
`publish_state_LT()`) is confirmed healthy and changing throughout the
*entire* run, every tick, with no gaps -- the freeze is not a MATLAB-side
computation problem. It happens strictly between "MATLAB has the right
number in memory" and "the Pi's recorded bag shows that number changing."

## What was ruled out, one at a time, with a real live test each

All ten below were individually reproduced/disproven this session, each
with its own direct evidence (not inferred):

1. **Mission-script logic bug** (`slam_traj_probe.m` computing a bad
   setpoint) -- ruled out: MATLAB's own `sim_pose` read via `evalin`
   changes correctly every tick throughout, independent of what the bag
   shows.
2. **MATLAB-side timer/workspace staleness** -- ruled out: debug-logging
   instrumentation inside `publish_state_LT()` itself confirmed the read
   side is always healthy, every tick, matching (1).
3. **Reused `ros2message` object never refreshed** -- ruled out: tried a
   fresh `ros2message` call every tick instead of reusing `pose_msg`, froze
   identically.
4. **Unpopulated `Float64MultiArray.layout.dim`** -- ruled out: populated
   it explicitly, froze identically.
5. **RMW/DDS implementation** (`rmw_fastrtps_cpp` vs `rmw_cyclonedds_cpp`)
   -- ruled out: swapped, froze identically under both.
6. **Generic bandwidth/publisher-count contention** -- ruled out: a
   throwaway high-bandwidth dummy publisher (~18.4 MB/s, mono mode, real
   `slam_traj_probe.m` busy-loop) ran clean with no freeze; conversely
   real stereo bandwidth alone (scout-mode bags, no busy-loop) also never
   froze -- see reframe below.
7. **Topic-specific staleness** (something specific to
   `/sim/drone_pose`'s own publisher/history) -- ruled out: built a
   byte-identical duplicate publisher (`pose_pub2`/`pose_msg2` publishing
   the same `sim_pose` data to `/sim/drone_pose_dup_TEST`) -- froze
   identically and at the same time as the original. Also tried a fully
   self-contained fake publisher (`pose_pub3`, `persistent fake_ctr`
   counter, no dependency on `sim_pose`/`evalin` at all) -- did not freeze
   in that one trial, but inconclusive (see Errors below -- this test's
   result was contaminated by a forgotten revert on the *next* trial, and
   was not re-run clean afterward).
8. **QoS reliability policy** (`reliable` vs `besteffort` on `pose_pub`)
   -- ruled out: swapped, froze identically under both.
9. **Timer `BusyMode`** (`drop` vs `queue` on `pitch_pub_timer_LT`) --
   ruled out: swapped to `queue` explicitly, froze identically. Also
   noted the premise was never even sound: `controller-benchmark`'s own
   `hil_ros_init_LT.m` has the identical timer block (`fixedRate`,
   default `'drop'`, never set explicitly) and never froze in any
   mono/scout run this session -- so `'drop'` was already present in
   every *working* case too.
10. **Network medium** (WiFi vs wired Ethernet) -- ruled out: Arian
    physically brought up the Pi's `eth0` (previously down/no-carrier all
    session), configured a direct wired link
    (192.168.137.1 Windows <-> 192.168.137.10 Pi), verified with a real
    ping (0% loss, 1-2ms RTT) and an ARP-table MAC-address cross-check
    (confirmed genuine, not spoofed/cached). Built a throwaway stack
    config (`ov2slam_stereo_oracle_wired_TEST.yaml`, one line changed:
    `network.matlab_host_ip`) so `run_stack_hil.sh`'s own `ip route get`
    auto-detection would resolve the Pi's local IP/interface to `eth0`
    instead of `wlan0`. Ran clean end-to-end: identical freeze signature
    (frozen 0-27s, tracks motion 30-38s, frozen again 38-62s) as every
    WiFi trial.

Every variable inside this project's own code or configuration has now
been swapped without changing the outcome.

## The reframe (2026-09-06, prompted by Arian)

Arian asked directly whether earlier **scout-mode** stereo runs (real
IBVS controller, real YOLO detector, NOT `--hold-fsm`, NOT
`slam_traj_probe.m`) had working `/sim/drone_pose`. Pulling the four
pre-oracle stereo bags and decoding them fully:

| Bag | Unique GT samples | Verdict |
|---|---|---|
| `run_ov2slam_stereo_20260904_115958` | 116/307 | WORKING |
| `run_ov2slam_stereo_20260903_233539` | 7374/10249 | WORKING (spot-check earlier this session undersold it; full decode shows healthy almost throughout) |
| `run_ov2slam_stereo_20260903_232002` | 194/337 | WORKING |
| `run_ov2slam_stereo_20260903_224123` | 0 messages | different failure mode entirely (never started -- not the freeze pattern, not counted either way) |

3/3 non-empty scout-mode stereo runs -- **real stereo rendering, same
git_sha, same hardware** -- show a healthy, continuously-changing ground
truth. So "stereo breaks ground truth" is the wrong frame: real stereo
rendering, by itself, does not reproduce the freeze.

This also resolves an apparent loose end from the bandwidth-isolation
test (line 6 above): that dummy-publisher test DID run through
`slam_traj_probe.m`'s tight busy-loop (same harness) and worked fine.
Combined with the scout-mode result (real stereo, no busy-loop, also
fine) -- **neither factor alone is sufficient.** The leading hypothesis
is that it requires **both together**: real dual-camera Unreal rendering
AND `slam_traj_probe.m`'s `send_cmd()` -> `pause(0.05)` -> repeat busy-loop
(a fundamentally different MATLAB execution pattern than the
callback-driven `/cmd_vel` subscriber the scout runs use), running
simultaneously for the whole mission. Every failing trial this session
used both together; no trial isolating `--hold-fsm` from
`slam_traj_probe.m` specifically (i.e., driving the drone under
`--hold-fsm` some other way, without a tight MATLAB busy-loop) has been
run yet.

One real, structural, mono/stereo-neutral fact was also confirmed but
explicitly does NOT explain stereo-vs-mono by itself (retracted after
Arian pointed out "it literally doesn't justify stereo vs mono at all"):
`matlab/read_cmdvel_live_interp.m` writes `sim_pose` via
`assignin('base', 'sim_pose', ...)` from inside a MATLAB Function block
in the Simulink model, using `coder.extrinsic('assignin','evalin')` --
MathWorks' own docs confirm this forces an interpreter round-trip "at
each time step" even in compiled/Accelerator-mode Simulink. This
mechanism is real and is a legitimate source of interpreter contention,
but it's byte-identical code across mono/stereo/scout modes -- it cannot
alone explain why only stereo+oracle+busy-loop freezes. It may still be
part of the real mechanism (interpreter contention under load), just not
the differentiator on its own.

## Where this leaves the search

The remaining candidate space is entirely in code neither this project
nor Claude can directly inspect or patch: MATLAB ROS Toolbox's own
(shipped, closed-source) `ros2publisher`/`send()` implementation, or the
DDS library underneath it (Fast DDS / Cyclone DDS internals) -- both
already swapped at the RMW level (line 5) without effect, which pushes
suspicion toward something inside the toolbox's own publish path rather
than the DDS layer itself, though this has not been proven, only
inferred from elimination.

**Concrete next experiment, not yet run**: isolate `--hold-fsm` from
`slam_traj_probe.m`'s specific busy-loop execution pattern -- drive the
drone under `--hold-fsm` some other way (e.g. a callback-driven
`/cmd_vel` publisher on the Windows side instead of a tight
`pause(0.05)` loop) and see whether the freeze still occurs. This would
tell us whether the trigger is "no reactive FSM" (`--hold-fsm` itself) or
specifically "MATLAB spends the whole mission in a tight synchronous
loop instead of returning to its event loop between commands."

**Fallback workaround (not yet implemented, reopened by this reframe)**:
have MATLAB write `sim_pose` to a local file each tick (bypassing the ROS2
bag path entirely for ground truth) and have `aggregate_matrix.py` read
GT from that file instead of the bag. This doesn't depend on which of the
two factors is the real trigger, and would unblock benchmark scoring
immediately. Awaiting Arian's call on whether to isolate the two factors
first or proceed straight to this workaround.

## Live-coupling safety risk (separate finding, flagged not fixed)

`src/oracle_detector/oracle_detector/oracle_detector_node.py` subscribes
to `/sim/drone_pose` continuously and its entire function is projecting
the known target through this ground truth to fake a detection; it has a
`pose_stale_sec` guard (default 0.3s) but it checks *arrival time only*,
not *value change* -- it would NOT catch this exact freeze (the topic
keeps publishing at 20 Hz, just with a stale value). Same blind spot in
`src/servo_core/src/servo_fsm_node.cpp`'s `on_drone_pose()` callback,
used for SEARCH-state yaw control and a safety altitude clamp
(`drone_z_ < safety_min_altitude_`). Neither node launches under
`--hold-fsm` (used for 100% of today's stereo work), so this has been
completely latent so far -- but should be fixed or at least documented
before any real reactive (non-`--hold-fsm`) stereo mission runs on this
branch. `src/init_gate/init_gate/cycle.py` was confirmed NOT to depend on
this topic at all (deliberately mirrors `/cmd_vel` instead, per its own
in-code comment about a world/body-frame sign-inversion risk at yaw=pi).

## Tooling added this session (untracked, not yet committed)

- `scripts/hil_matrix/export_bag_csv.py` -- standalone GT+SLAM bag-to-CSV
  extractor, reuses `aggregate_matrix.py`'s own decode logic, verified
  against multiple bags.
- `pose_watch.py` (Claude's scratchpad, copied to `/tmp/pose_watch.py` on
  the Pi) -- live watchdog, subscribes to `/sim/drone_pose` inside the
  running stack container, flags a value that hasn't changed in >3s.
  Demonstrated working correctly on a live trial. Exists to make any
  future diagnosis of this bug cheaper (no need to wait for a full trial
  + bag decode just to know a run froze).

## Mistakes made while investigating (logged so they aren't repeated)

- Edited `run_stack_hil.sh` on the wrong machine once (the Windows mirror
  checkout, not the Pi) -- had no effect, cost a confusing "why did this
  suddenly pass" moment before being caught.
- Edited `scripts/record_bag.sh` once when the actually-relevant file for
  benchmark-mode bag topics is `run_stack_hil.sh`'s own inline
  `BAG_TOPICS` variable -- caught via the bag's own `bag_record.log`
  showing the new topic was never subscribed to.
- Left test instrumentation in `hil_ros_init_LT.m` un-reverted across one
  follow-up trial (caught via `git status --short` after Arian asked what
  the latest change was) -- both trials that ran on top of it are not
  reliable evidence for or against anything and are excluded from the
  counts above.

## Open follow-ups (not blocking, not urgent)

- Separate, unrelated, intermittent: mono Umeyama "Degenerate covariance
  rank" failures (2/4 mono trials this session). Calib and CPU pinning
  both directly ruled out. Cause unidentified.
- `run_ov2slam_stereo_20260903_224123` produced *zero* messages on
  `/sim/drone_pose` (not frozen -- entirely absent). Different failure
  mode, not investigated, possibly a run that crashed before
  `publish_state_LT` ever started. Flagged so it isn't confused with the
  freeze bug later.
- `/sim/camera/right/image_raw` (besteffort) has no matching override in
  `config/hil/bag_qos_overrides.yaml` -- noticed, not investigated.
- TODO-AQ (fixlog 024) and TODO-AR (fixlog 025) remain open, unrelated,
  not blocking.


---

## RESOLUTION 2026-09-07/08 -- FALSE ALARM. Not a bug. Closed.

**Status changed: open -> closed (no code change).** Everything above the line
stands as a record of what was tested, but its framing was wrong, and the
correction matters more than the investigation:

The "freeze" is the drone genuinely holding still, twice, on purpose, every run:

1. **Spawn hold (t = 0 to ~20 s, exactly at the spawn pose).** The bag recorder
   starts before the mission script does (`docs/TODO.md` item 1). Nothing is
   commanding the drone yet -- SLAM/gate startup. Genuinely stationary.
2. **Post-init hold (~8-23 s, always ending at y ~ 16.88).** `matlab/slam_traj_probe.m`
   flies its Phase 0 init leg 3.0 m sideways (`INIT_LATERAL = 3.0`, `INIT_VY = -0.8`)
   from y = 20 -> y ~ 17, then executes `send_cmd([0;0;0;0;0]); pause(INIT_SETTLE)`
   ("let the mapper close out its first keyframes"), then RETURNS to the
   orchestrator, which polls `/slam/tracking_state` until the backend reports
   ready (`run_matrix.py:839-931`, `init_patience_sec`) before calling the
   script again with `PROBE_PHASE = 'mission'`. Nothing commands the drone during
   that poll. The hold's length varies with how long SLAM takes to become ready.

Proof, 16/16 stereo+oracle bags of 2026-09-06 decoded (`export_bag_csv.py`):
the 10 runs that flew the mission all show the second hold ending 0.0-0.3 s
before `aggregate_matrix.py`'s `t_init` (first x-displacement > 0.15 m), i.e.
exactly when the mission leg starts commanding motion; the 6 earlier runs where
the hold never ended are runs where SLAM never reported ready and the mission
was never flown. `y ~ 16.88 = 20 - 3` is the script's own number, not a
coincidence. The same "hold at spawn / hold when parked" shape appears in the
`controller-benchmark` bags (`ctrl_ibvs_N1_20260623_190358` etc.), which have
none of the suspected trigger conditions.

Why the RMSE was never affected: the scorer's window starts at `t_init`
(mission start) by design, which is after both holds. The "risk" noted above of
the freeze "leaking into the scored window" was likewise unfounded -- the drone
does not stop during the mission leg.

Why ten variables were "disproven": none of them was ever the cause. Every test
compared a stationary drone against a stationary drone. The premise -- "the
recorded pose is stale" -- was never checked against "is the drone supposed to
be moving right now", which one read of `slam_traj_probe.m` would have answered
on day one. That failure is now encoded as skills (`rule-out-by-design`,
`read-the-driver-first`, `occams-razor-before-escalate`) and as Stage 0 of the
`second-brain` protocol.

What remains genuinely useful from this entry: `export_bag_csv.py`,
`pose_watch.py`, and the observation that 6/16 runs never reached SLAM-ready
(a SLAM-initialisation-time question for the stereo arm, not a pose-pipeline
question -- tracked under the stereo benchmarking work, not here).
