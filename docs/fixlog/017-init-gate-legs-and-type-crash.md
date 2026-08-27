---
id: FIX-017
title: init_gate crashed silently on int-valued YAML params; legs was never wired at all
date: 2026-07-18
status: applied, verified in-container (repro'd the exact crash + the fix)
component: scripts/parse_stack.py, run_stack_hil.sh, src/init_gate/init_gate/init_gate_node.py
critic_verdict: n/a (deterministic reproduced crash, not an ambiguous why - no critic needed per user's bandwidth rule)
kiss_verdict: simple
open_todos: []
---

## Symptom

User set `init_gate.params.legs: [up, down]` in `orbslam2_eval.yaml` (WS2's new
per-run override mechanism) and reported "I want to control the legs... why is it
not working." Two live run attempts (`gustave_1`, 2026-07-18 17:59 and 18:09)
both produced a bag with ZERO `/bench/state`, `/cmd_vel`, or `/yolo/detections`
messages ever recorded - despite the gate node normally publishing at least one
`/bench/state` message even on a plain timeout. Both containers were still up
minutes later with no `run_stack_hil.sh` or `init_gate_node` process running -
i.e. the launcher script had already exited, silently, before the gate did
anything observable.

## Root cause (two separate bugs, both confirmed by direct repro, not guessed)

1. **Type-mismatch crash.** The same config also had `timeout_sec: 30` (no
   decimal) - a bare YAML int. `init_gate_node.py` declares
   `timeout_sec` via `declare_parameter("timeout_sec", -1.0)` (a DOUBLE default).
   ROS2 parameters are strictly typed by default: overriding a DOUBLE param with
   an INTEGER-typed command-line value raises
   `rclpy.exceptions.InvalidParameterTypeException` INSIDE `declare_parameter()`,
   before the node constructs any publisher. Reproduced directly:
   `declare_parameter("timeout_sec", -1.0)` + `-p timeout_sec:=30` ->
   `InvalidParameterTypeException: Trying to set parameter 'timeout_sec' to '30'
   of type 'INTEGER', expecting type 'DOUBLE'`. This explains the empty bag -
   the node never got far enough to publish `/bench/state`.
2. **`legs` was never wired.** WS2 (2026-07-18, earlier this session) built the
   module/leg_speed/leg_duration_sec/timeout_sec/ready_debounce override pipeline,
   but never added a config-level override for WHICH LEGS RUN - `legs:` in the
   yaml was silently parsed by PyYAML and simply never read by
   `parse_stack.py`, so it had zero effect (no error, no warning - just ignored).
   This was a real gap in what WS2 shipped, not a user mistake.

## Fix

- `scripts/parse_stack.py`: added `get_num(d, path, caster)` - forces
  `INIT_GATE_LEG_SPEED` / `INIT_GATE_LEG_DURATION` / `INIT_GATE_TIMEOUT`
  (float) and `INIT_GATE_READY_DEBOUNCE` (int) to the CORRECT type before
  emitting, so a bare YAML int can never again reach ROS2 as the wrong param
  type. Added `INIT_GATE_LEGS` = comma-joined `init_gate.params.legs` list.
- `run_stack_hil.sh`: passes `-p legs:=${INIT_GATE_LEGS}` to the gate node
  alongside the existing scalar overrides; also records legs/leg_speed/
  leg_duration_sec/timeout_sec/ready_debounce into `run_config.yaml` (previously
  only `module` was recorded there - a related gap, also fixed here).
- `init_gate_node.py`: declares `legs` (string param). Added `NAMED_DIRECTIONS`
  (left/right/up/down/forward/backward - matches the existing servo_core body-frame
  convention). When `legs` is set, each comma-separated name is looked up; unknown
  names are logged and skipped (never crash); if that empties the list, falls back
  to the profile's own `LEGS` entirely (gate must never run with zero legs). Legs
  are OUTBOUND directions only - `cycle.py` still auto-mirrors each for the return
  leg (unchanged design), so `legs: [up]` alone already includes the "come back
  down" motion; listing `down` separately adds a second, largely redundant leg
  (up-out+back, then down-out+back) - not wrong, just worth knowing.

## Verification (in-container, exact repro of the user's config)

- Repro'd the crash standalone: `declare_parameter(-1.0)` + `-p timeout_sec:=30`
  -> `InvalidParameterTypeException` (confirmed BEFORE any fix applied).
- After fix: constructed `InitGateNode` with the EXACT args
  `module:=orbslam2 legs:=up,down leg_speed:=0.6 leg_duration_sec:=1.0
  timeout_sec:=30.0 ready_debounce:=2` (what `run_stack_hil.sh` now emits for
  the current `orbslam2_eval.yaml`) - no crash, `legs=['up','down']`,
  `leg_speed=0.6`, `timeout_sec=30.0`, all correct.
- Unknown-name fallback tested: `legs:=sideways,up` -> `['up']` (bad name
  skipped, good one kept). `legs:=bogus1,bogus2` -> falls back to profile
  default `['left','right','up']` (never empty).
- `colcon build --packages-select init_gate` clean; `bash -n run_stack_hil.sh`
  clean; `parse_stack.py --emit-env` against the live config emits the correct
  typed values.

## Not yet done

Full live-flight validation (does the gate actually complete a `legs:=up,down`
cycle end-to-end with MATLAB running) is still pending - the user's next run
attempt IS that validation.
