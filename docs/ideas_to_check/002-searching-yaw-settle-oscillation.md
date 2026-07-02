---
id: IDEA-002
title: SEARCHING yaw-settle oscillation (YAW_RIGHT_60/YAW_LEFT_60/YAW_CENTER never converges)
date: 2026-07-02
status: open — not implemented
component: src/servo_core/src/servo_fsm_node.cpp (build_search_command / advance_search_step)
---

## Symptom

Live-testing FIX-011 (the opt-in SLAM-init gate that holds the FSM in
`SEARCHING` and ignores detections until ORB-SLAM2 initializes — see
`docs/fixlog/011-slam-init-gate.md`), the drone rolled around continuously for
145+ seconds and never reached `STRAFE_RIGHT` even once (`vy` stayed `0.000` the
entire run). This happened *after* FIX-012 (stale ORB-SLAM2 wrapper binary) was
already fixed and `/slam/tracking_state` confirmed flowing correctly — so it's a
separate, second-layer problem underneath the gate itself.

## Evidence

Bag tracing (`orbslam2_eval_gatetest2`, `ros2 bag` read via a throwaway
`ros2_perception_stack` container, fine-grained `/sim/drone_pose` +
`/cmd_vel` extraction):

| Time | Yaw | wz command |
|---|---|---|
| t+29.85s | 686° | 0.000 (just entered `YAW_RIGHT_60`, right after `FULL_ROTATE` completed) |
| t+31.5s | **726° (peak)** | -0.8 |
| t+36.2s | **570° (trough)** | +0.8 |
| t+41.0s | **719° (peak again)** | -0.8 |
| t+43.5s | ~621° (trough again) | +0.8 |

A genuine, undamped **~150° limit cycle**, same amplitude across multiple
cycles, no decay. `wz` saturates at ±0.8 rad/s (`max_angular`) and reverses, but
the observed yaw keeps moving in the *old* direction for ~1.6s after the command
flips — the simulated drone has real rotational lag/inertia that a pure
proportional controller with no damping term cannot track without oscillating.

## Root cause

`build_search_command()`'s shared `YAW_RIGHT_60`/`YAW_LEFT_60`/`YAW_CENTER` case
(`servo_fsm_node.cpp`, ~line 700):

```cpp
double err = wrap_to_pi(yaw_target_rad_ - drone_yaw_rad_);
if (std::abs(err) < search_yaw_arrive_tol_rad_) {
    ...settle and advance...
} else {
    cmd.angular.z = clamp_vel(k_search_yaw_ * err, max_angular_);
}
```

Pure-P, no derivative/rate-feedback term, chasing a hard step target
(`search_yaw_target_deg_ = 60.0°`) to a tight arrival tolerance
(`search_yaw_arrive_tol_rad_ = 0.05` rad ≈ 2.9°) against a plant with real
response lag. Classic setup for a saturated-P limit cycle: the controller
commands max output, the plant overshoots due to inertia, the controller
reverses to max output the other way, the plant overshoots again — never
settling inside a 2.9° window.

By contrast, `FULL_ROTATE` and `STRAFE_RIGHT` — the only two other steps in
this exact state machine — are **purely elapsed-time-based**
(`if elapsed >= duration: advance`), no arrival/settling check at all. Only
the three `YAW_*` steps use position-arrival, which is the likely source of
the instability.

## Why this was never seen before

Confirmed via a historical pre-FIX-011 bag (`orbslam2_eval_39`,
2026-07-02 00:50): `SEARCHING` lasted only **11.40s** before lock-on — *less*
than `FULL_ROTATE`'s own 13.0s duration (`search_full_rotate_sec`). Lock-on has
always fired before the drone ever left `FULL_ROTATE` in every prior run, since
the oracle detector locks on almost instantly (this is the exact mechanism
FIX-011 was built to work around). The `YAW_RIGHT_60`/`YAW_LEFT_60`/
`YAW_CENTER` code path had **never been exercised to completion** in any run
before FIX-011's gate held `SEARCHING` open long enough to reach it. This is a
pre-existing latent bug, not something FIX-011 introduced.

## Codebase research (no fix precedent to reuse)

- No PD/damping/rate-limiting control pattern exists anywhere in this repo.
  Every other controller that fought yaw oscillation historically either
  **eliminated the DOF** (`wz = 0.0`, used by TS2 proportional and PBVS — see
  `docs/fixlog/002-phantom-rotation.md` / `003-remove-rotation-feature.md`) or
  **reduced gain + respected saturation** (`docs/fixlog/006-ibvs-breakdance.md`)
  — never added a derivative term. A fix here would be new territory for this
  codebase's control style, not a reuse of an existing pattern.
- Deployed params (`config/hil/bench_fsm.yaml`) are unmodified code defaults —
  `k_search_yaw=1.0`, `search_yaw_arrive_tol_rad=0.05`, `max_angular=0.8`,
  `search_yaw_target_deg=60.0` — nobody has ever tuned these, because nobody
  ever needed the settle logic to run to completion before.
- No file anywhere in the repo (`matlab/*.m`, config yamls, docs) quantifies the
  simulated drone's rotational inertia/damping/response lag — this is
  unmeasured, only inferable from bag behavior like the trace above.

## Candidate fix ideas (NOT implemented — needs a decision)

1. **Max-dwell timeout fallback** on the three `YAW_*` steps, matching how
   `FULL_ROTATE`/`STRAFE_RIGHT` already work: keep the existing arrival+settle
   check as the fast path, but force-advance regardless once a generous
   elapsed-time budget is exceeded. Precise heading isn't actually required for
   the search sweep's purpose (looking for the target / generating strafe
   parallax), so accepting "wherever it ended up" on timeout should be fine.
   Needs a new param (`search_yaw_max_dwell_sec`; ~8s proposed as a starting
   point based on the observed ~8-10s limit-cycle half-period, unverified) and
   a new per-step entry timestamp in `servo_fsm_node.hpp`/`.cpp`. Should apply
   unconditionally (not gated behind a new opt-in flag) since bounding
   worst-case time is strictly better than the current unbounded oscillation
   for every consumer — but this is a shared-FSM-code change touching every
   controller/config, so run the full SECOND BRAIN protocol (critic + KISS
   agents, per `CLAUDE.md`) before applying, and spot-check
   `controller_bench.launch.py` (the N=10 benchmark) is unaffected, same
   caution as FIX-011's TODO-R.
2. **Config-only experiment** (zero code change): loosen
   `search_yaw_arrive_tol_rad` and/or shrink `search_yaw_target_deg` in
   `config/hil/bench_fsm.yaml`. Cheaper and fully reversible to try first, but
   the observed ~150° oscillation amplitude is much larger than a modest
   tolerance loosening would plausibly fix on its own — untested.

## Open TODOs

- **TODO-V**: Decide between candidate fix 1 (code, general robustness fix) and
  candidate fix 2 (config-only experiment, try first since it's free) — or try
  2 as a quick diagnostic even if 1 ends up being the real fix.
- **TODO-W**: If pursuing fix 1, follow SECOND BRAIN protocol fully (this idea
  was investigated and documented, but the fix itself was deliberately not
  implemented this session per user direction — "we already stretched
  [the FSM]" today with FIX-011 and the FIX-012 rebuild).
- **TODO-X**: Once resolved, re-run `orbslam2_eval` (`blind_until_slam_ready:
  true`) and confirm via `/bench/state` + `/cmd_vel` bag tracing (same method
  used to diagnose this) that `STRAFE_RIGHT` actually executes and the gate
  reaches `SLAM_RELOCATE`/`DONE` — FIX-011 itself is still unproven end-to-end
  until this blocker clears.
