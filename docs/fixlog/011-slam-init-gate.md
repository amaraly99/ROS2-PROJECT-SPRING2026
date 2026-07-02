---
id: FIX-011
title: SLAM-init gate — blind search until ORB-SLAM2 initializes (opt-in)
date: 2026-07-02
status: resolved
component: src/servo_core (servo_fsm_node), config/hil/stack/orbslam2_eval.yaml, launch/parse wiring
supersedes:
critic_verdict: partial
kiss_verdict: simplify-recommended
open_todos: [TODO-S, TODO-T, TODO-U]
graduated_from: docs/ideas_to_check/001-orbslam2-monocular-init-reliability.md
---

## Symptom

`orbslam2_eval` benchmark runs produce wildly inconsistent monocular ORB-SLAM2
trajectory quality run-to-run on the *same* replayed scenario (`full_path_length_m`
≈ 93–95 m every time): eval-window coverage 2–32 %, `umeyama_scale` 2.2–30.9×,
ATE 0.017–0.49 m. CPU pinning was tested three ways over five runs and falsified as
the cause (FIX-008 / IDEA-001).

## Root cause

The shared FSM's `SEARCHING` step sequence is
`FULL_ROTATE → YAW_RIGHT_60 → YAW_LEFT_60 → YAW_CENTER → STRAFE_RIGHT → loop`.
Only `STRAFE_RIGHT` produces translation (`cmd.linear.y`); the rest are pure
rotation. Monocular SLAM needs a **translational baseline** to initialize its first
map (two-view triangulation is degenerate under rotation-only or forward-only
motion). But with the **oracle** detector (perfect, instant), lock-on
(`SEARCHING → APPROACHING`, `update_state_on_detection()`) fires within ~1–2 s of
boot — before the search ever reaches `STRAFE_RIGHT`. The drone commits straight
into forward-dominant `APPROACHING` motion, which is near-degenerate for init, so
ORB-SLAM2 initializes late, on a marginal frame pair, with badly-conditioned scale —
and that bad scale is baked in for the rest of the (mono, no-loop-closure) run.

## Fix

Opt-in **SLAM-init gate** in `servo_fsm_node`, enabled only by benchmark stack
configs (default off → scout mode and the isolated N=10 controller benchmark are
provably unaffected — `controller_bench.launch.py` never sets the params).

New params: `blind_until_slam_ready` (bool, default `false`) and
`max_blind_strafes` (int, default `3`).

New runtime state machine `SlamGate { BLIND, RELOCATE, DONE, FAILED }` (init `DONE`
== transparent; `BLIND` when the gate is enabled), latch `slam_ready_`, counter
`blind_strafe_count_`, and a subscription to `/slam/tracking_state` (`std_msgs/Int32`,
created only when enabled; QoS(10) matches the ORB-SLAM2 publisher).

- **BLIND**: `on_detections()` ignores the target and routes to
  `handle_no_detection()`, so `build_search_command()` free-runs the search pattern
  and its `STRAFE_RIGHT` leg generates parallax. On each completed strafe
  (`advance_search_step()` `STRAFE_RIGHT` case): if `slam_ready_` → **RELOCATE**;
  else if `++blind_strafe_count_ >= max_blind_strafes_` → **FAILED**.
- **RELOCATE**: one full `FULL_ROTATE` re-sweep (via `reset_search()`) to re-find the
  target's true bearing after the blind strafe(s) drifted the drone sideways;
  completes in the `FULL_ROTATE` case → **DONE**.
- **DONE**: normal FSM — detections processed, lock-on/approach as usual.
- **FAILED**: zero-hold enforced in `publish_cmd_vel()` (the single cmd chokepoint,
  so the `watchdog_tick()` path can't re-drive search), `RCLCPP_FATAL`, and a distinct
  `/bench/state` marker `SLAM_INIT_FAILED`.

`publish_bench_state()` emits `SLAM_BLIND` / `SLAM_RELOCATE` / `SLAM_INIT_FAILED`
while the gate is active (greppable; no spurious `SEARCHING→APPROACHING` edge during
the hold). `handle_sim_restart()` re-arms the gate (`BLIND`, clear `slam_ready_`,
reset counter) so a persistent FSM node doesn't lose the fix or deadlock across runs.

### Why this makes "the lengths right"

**Not** via `/bench/state` t=0 keying — the critic correctly established that
`eval_slam_hil.py` does *not* read `/bench/state`; its eval window is purely the
temporal intersection of `/slam/pose` ∩ `/sim/drone_pose` (`eval_slam_hil.py:243-245`).
The real mechanism: initializing SLAM **before** the forward-dominant approach means
SLAM tracks continuously through the whole approach, so the eval window covers the
full flight (`eval_path_length_m` ≈ `full_path_length_m`) with a well-conditioned
initial map (scale ≈ 1–2), consistently — instead of missing most of the flight and
baking in a random scale.

### Files

- `src/servo_core/include/servo_core/servo_fsm_node.hpp` — `SlamGate` enum, params,
  runtime state, `Int32` alias/include, `on_slam_tracking_state` decl, subscription.
- `src/servo_core/src/servo_fsm_node.cpp` — declare/load params, conditional
  subscription, callback, `on_detections` gate, `advance_search_step` transitions,
  `publish_cmd_vel` FAILED zero-hold, `publish_bench_state` string override,
  `handle_sim_restart` re-arm.
- `scripts/parse_stack.py` — `CONTROLLER_BLIND_UNTIL_SLAM_READY` / `_MAX_BLIND_STRAFES`.
- `run_stack_hil.sh` — append `blind_until_slam_ready:=` / `max_blind_strafes:=`.
- `src/sim_camera_bridge/launch/hil_simulation.launch.py` — read args, pass to
  controller param dict, `DeclareLaunchArgument` ×2.
- `config/hil/stack/orbslam2_eval.yaml` — `blind_until_slam_ready: true`,
  `max_blind_strafes: 2`, `slam.startup_delay_sec: 8` (see "Timing update" below).

### Timing update (same session, post-review)

`max_blind_strafes` dropped `3 → 2` and `slam.startup_delay_sec` dropped `15 → 8`,
at the user's request, after checking a specific interaction:

`slam.startup_delay_sec: 15` predates this fix and is documented in `HANDOFF.md` §5
as load-bearing — "exists so the drone is already moving (accumulating parallax)
*before* ORB-SLAM2's node comes online, avoiding a near-zero-parallax auto-exit bug."
That rationale was written for the **pre-gate** FSM, where lock-on fires in ~1-2s and
the drone is already translating (`APPROACHING`) well before t=15s. **Under this
fix's gate, that assumption no longer holds regardless of the delay value**: the
search sequence spends `search_full_rotate_sec_` (13s) + 3 yaw-settle legs
(rotation only, zero translation) before the *first* `STRAFE_RIGHT` even begins
(~t=17-19s). So SLAM's node boots into the same near-zero-*translational*-parallax
window whether `startup_delay_sec` is 15 or 8 — shrinking it doesn't newly expose
that risk, it was already present the moment the gate started ignoring detections
during `FULL_ROTATE`/`YAW_*`. What shrinking it *does* buy: more alive/warm-up time
(vocab load, `SYSTEM_NOT_READY → NO_IMAGES_YET → NOT_INITIALIZED`) before the first
real translational parallax event, which should improve first-strafe init odds and
justifies dropping `max_blind_strafes` to 2. No static grep of the vendored
ORB-SLAM2/wrapper source turned up an explicit "auto-exit on no motion" code path —
the bug is presumed empirical (observed on real Pi runs, not necessarily reproducible
by reading source), so this is a reasoned bet, not a verified-safe change. **Watch
`docker logs slam_orbslam2` on the first Pi run under these values** for any
early-exit signature; if it recurs, it's evidence the bug is a startup-time
condition independent of translation, not specifically tied to the delay length.

## Critic verdict & concerns

**PARTIAL** (core mechanism sound; stated rationale was wrong; several real risks —
all addressed before commit):
- **Falsified the original "eval t=0 keying" rationale.** `eval_slam_hil.py` uses the
  SLAM∩GT temporal intersection, not `/bench/state` edges. Rationale rewritten (above).
- **tracking_state wiring**: the `slam_tracking_state:=/slam/tracking_state` remap only
  applies to the SLAM sidecar; the FSM must subscribe to the absolute topic itself.
  Done (hardcoded `/slam/tracking_state`, same contract as `/slam/pose` + `/slam/cloud`).
- **Timing (KEY)**: `startup_delay_sec: 15` + ~13 s `FULL_ROTATE` means the first strafe
  lands ~t≈20 s with SLAM barely warmed up; `max_blind_strafes: 1` would likely
  spurious-FAIL. → default raised to **3** and kept as a param (empirically-uncertain,
  hardware-dependent value = exactly when a param is justified).
- **`handle_sim_restart` didn't reset the gate** → deadlock/lose-fix on re-run. Fixed.
- **FAILED leaked through `watchdog_tick`** → zero-hold moved to `publish_cmd_vel`
  chokepoint. Fixed.
- Single-threaded-executor assumption documented (no mutex; must not move to
  MultiThreadedExecutor).

## KISS verdict

**simplify-recommended**. Adopted: hardcode-vs-param debate resolved in favor of
keeping `max_blind_strafes` (critic's timing analysis overrides KISS's "hardcode 1");
`RESWEEP` renamed `RELOCATE` (collided with `State::REACQUIRE`); FAILED enforced at one
chokepoint. Declined: folding `DONE` into a bool (the 4-value enum is honest and reads
clearly — `DONE` just *is* the disabled/transparent init value, no extra member); and
reusing `seed_search_from_last_bearing()` for the re-sweep (it needs `have_last_bearing_`,
which is never set during BLIND since detections are ignored before that line — it would
fall back to a full rotate anyway, and a full cycle is what was explicitly requested).

## Open TODOs

- **TODO-S**: Verify on the Pi that ORB-SLAM2 reliably reaches `tracking_state==2`
  within 2 blind strafes given the 8 s startup delay + vocab load. If it often needs
  more, raise `max_blind_strafes`/`startup_delay_sec` in `orbslam2_eval.yaml`; watch
  `docker logs slam_orbslam2` on the first run for the near-zero-parallax auto-exit
  bug HANDOFF.md §12 warned about — see "Timing update" above for why 8s vs 15s
  likely doesn't change exposure to it (both land inside the gate's rotation-only
  prefix), but this hasn't been Pi-verified yet.
- **TODO-T**: The RELOCATE full 13 s `FULL_ROTATE` (both agents flagged the cost) is a
  pure rotation while SLAM is freshly initialized — verify it doesn't destabilize the
  fresh mono map at `search_spin_speed_ 0.5 rad/s`, and consider a bounded re-bearing
  (small yaw toward the target) instead of a full circle if the 13 s is wasteful.
- **TODO-U**: If SLAM never publishes (sidecar crash), `slam_ready_` stays false and the
  gate correctly FAILs after `max_blind_strafes` — but confirm this is distinguishable in
  logs from "SLAM up but couldn't init" (both show `SLAM_INIT_FAILED`). Consider latching
  a `slam_seen_` flag on first tracking_state message to tell the two apart.
