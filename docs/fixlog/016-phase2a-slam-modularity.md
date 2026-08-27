---
id: FIX-016
title: Phase 2A - SLAM/mode/init-gate modularity cleanup (prep for ORBSLAM2 flight + ORBSLAM3)
date: 2026-07-18
status: applied (live-run validation pending in Phase 4)
component: run_stack_hil.sh, scripts/parse_stack.py, src/init_gate/**, config/hil/stack/orbslam2_eval.yaml
critic_verdict: n/a (lightweight process per user bandwidth constraint; verified via in-container tests)
kiss_verdict: simple
open_todos: [TODO-P2A-1, TODO-P2A-2, TODO-P2A-3]
---

## Symptom / motivation

Modularity concerns blocking clean multi-SLAM benchmarking (see HANDOFF 2026-07-18):
1. `mode` (benchmark|scout) secretly controlled THREE things - recording, FSM
   engagement, AND a Simulink t=0 sim-reset wait - tangled with `init_gate.enabled`.
2. The init_gate was a single hard-coded cycle; each SLAM may need its own warmup
   motion, with no config-level swap or per-run param control.
3. ORBSLAM2 launched via a raw binary path (justified by a FALSE 'rclpy missing
   librcl_action.so' claim - that lib IS present); the direct-binary launch was the
   root of the earlier HANDOFF Sec.1 regression.
4. No per-run record of the RESOLVED effective config ("what actually happened").

## Fixes (Workstreams)

- **WS1 (mode KISS)** `run_stack_hil.sh`: `MODE` now controls RECORDING ONLY. FSM
  always engages scout-style (`benchmark_mode:=false`); engagement timing is the
  init_gate's job. Gated ORBSLAM2 already ran benchmark_mode:=false -> byte-identical.
- **WS2 (swappable init_gate)** new `src/init_gate/init_gate/profiles/<name>.py`
  (default, orbslam2, ov2slam) each holding LEGS + scalar DEFAULTS. `init_gate_node`
  reads ROS params `module` + `leg_speed/leg_duration_sec/timeout_sec/ready_debounce`
  (each <0 => use profile default). `parse_stack.py` emits `INIT_GATE_MODULE` + the
  params; `run_stack_hil.sh` passes them via `--ros-args -p ...`. `cycle.py` made
  generic (takes legs+params). default/orbslam2 preserve the exact pre-Phase-2 values
  (legs left/right/up, 1.2/1.0/30/2). params.py deprecated (orphaned).
- **WS3/Fix5 (ros2 run)** `orbslam2_eval.yaml` command -> `ros2 run orbslam mono ...`
  (verified on the Pi: works AND preserves taskset affinity on the child binary).
- **WS4 (depth wiring)** verified SlamDepthSource is backend-agnostic (subscribes
  /slam/pose + /slam/cloud, any SLAM remaps to these) and works in both modes; only
  ViSP/IBVS consumes it today. TODO comment added at the use_slam_depth declaration.
- **WS5 (run_config.yaml)** each benchmark run dir now gets a `run_config.yaml` with
  the resolved effective config (mode/detector/controller/slam/init_gate/network +
  git sha). meta.txt kept as an eval compat shim.

## Verification (in-container, no live flight yet)

- `bash -n run_stack_hil.sh` clean.
- `parse_stack.py orbslam2_eval.yaml --emit-env` emits INIT_GATE_MODULE=orbslam2 +
  correct SLAM_COMMAND (ros2 run ...).
- `colcon build --packages-select init_gate` OK; profiles import OK; typo->default
  fallback OK.
- init_gate_node constructed with `-p module:=ov2slam -p leg_speed:=0.6 -p
  ready_debounce:=5`: picked ov2slam profile (legs=['up']), applied overrides, and
  fell back to profile DEFAULT for the unset timeout (30.0). PARAM PIPELINE OK.

## Open TODOs

- TODO-P2A-1: wire SLAM depth into non-ViSP controllers (proportional/h_vs/pbvs).
- TODO-P2A-2: Phase 4 - pin the Julien-exact ORBSLAM2 init_gate leg_speed
  (HANDOFF notes 0.6 for OV2SLAM's gate vs the 1.2 currently in the orbslam2 profile);
  set via config init_gate.params.leg_speed, no code change.
- TODO-P2A-3: full end-to-end live-flight validation of the new mode/gate wiring
  (deferred to Phase 4). ORBSLAM3 port (WS6/7) deferred until user confirms.
