---
id: FIX-007
title: Two sidecar deployment bugs — taskset list prefix + missing --entrypoint
date: 2026-06-27
status: resolved
component: run_stack_hil.sh / hil_simulation.launch.py
critic_verdict: correct
kiss_verdict: simple
open_todos: []
---

## Symptom

After Phase A implementation, running `--config full_ov2slam` produced two failure modes:

**Bug A**: Controller and detector nodes failed to spawn. Log showed
`FileNotFoundError: [Errno 2] No such file or directory: 'taskset-c0'`.
OV2SLAM never received frames → never initialized. Drone never moved.

**Bug B** (after Bug A fixed): `slam_ov2slam` container was in `Restarting (0) 3 seconds ago`
loop. OV2SLAM logs showed the image's banner/info entrypoint running and exiting 0
repeatedly, never reaching the `sleep 15; ov2slam_node` command.

## Root cause

**Bug A**: `pfx` lambda in `hil_simulation.launch.py` returned a list:
```python
pfx = lambda cpu: ['taskset', '-c', cpu] if cpu else None
```
`launch_ros` concatenates list prefixes **without spaces** → `taskset-c0`, not a real command.
The existing inline `prefix='taskset -c 2,3'` on the OV2SLAM node was already a string —
proof the string form is correct. List form is silently wrong.

**Bug B**: `ros2_perception_stack` image has a custom entrypoint (banner/health-check script).
Without `--entrypoint ""`, `docker run` executes the entrypoint instead of the
`bash -lc "sleep 15; ...ov2slam_node..."` command. The entrypoint exits 0 → `--restart`
loops it forever.

## Diff

**Bug A**:
```diff
# hil_simulation.launch.py
-pfx = lambda cpu: ['taskset', '-c', cpu] if cpu else None
+pfx = lambda cpu: f'taskset -c {cpu}' if cpu else None
```

**Bug B**:
```diff
# run_stack_hil.sh — SLAM sidecar block
-sudo docker run -d \
+sudo docker run -d \
+    --entrypoint "" \
     --name "$SLAM_CONTAINER" \
```

## Critic verdict & concerns

Both fixes are mechanically correct and match the existing usage patterns in the codebase
(inline `prefix='taskset -c 2,3'` for string; `--entrypoint ""` on the main container).
No concerns raised.

## KISS verdict

Both are one-liner fixes at the exact site of the bug. No simpler form exists.

## Verification

Pi retest after both fixes:
- `slam_ov2slam Up 22 seconds`, `ros2_perception_stack Up 27 seconds`
- OV2SLAM log: `"OV²SLAM is ready to process incoming images! Starting the measurements reader thread!"`
- PIDs confirmed for controller and detector nodes; zero `taskset-c` errors in launch log.
