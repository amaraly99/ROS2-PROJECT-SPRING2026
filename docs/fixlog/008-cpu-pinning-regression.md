---
id: FIX-008
title: CPU pinning regression — yolo_bridge co-located with controller on core 0
date: 2026-06-27
status: resolved
component: config/hil/stack/*.yaml
critic_verdict: correct
kiss_verdict: simple
open_todos: []
---

## Symptom

After Phase A CPU-pinning feature was introduced, detection rate from yolo_bridge jittered
(12–17 Hz instead of stable ~20 Hz), contributing to noisy bbox → jittery controller
input during `--config full_ov2slam` run.

## Root cause

All stack configs were written with `detector.cpu: "0"` (yolo_bridge inside the container
pinned to core 0). The controller (`hil_servo_node`) is also on core 0. They competed for
the same core every tick.

The Pi 5 core map comment said "core 0 = bridges + controller" — I mistakenly treated
`yolo_bridge` as one of the "bridges" (like `sim_camera_bridge` / `ovcam_bridge`).
Those are lightweight DMA/SHM copiers. `yolo_bridge` is a hot 30+ Hz SHM consumer.

The correct assignment: `yolo_bridge` on core 1, co-located with `yolo_producer`
(host-side Hailo NPU process). Both share `/dev/shm` — same core improves cache locality
and removes contention from the controller core.

## Diff

Applied to all 6 yolo configs (`full_ov2slam`, `ov2slam_ibvs`, `default`, `ibvs`, `h_vs`, `full`):
```diff
 detector:
   type:     yolo
-  cpu:      "0"
+  cpu:      "1"
   host_cpu: "1"
```

Oracle configs (`ov2slam_oracle`, `oracle`) keep `cpu: "0"` — oracle node is a lightweight
Python ROS node with no SHM reads; core 0 contention with the controller is negligible.

Also corrected the core map comment in `full_ov2slam.yaml`:
```diff
-#   core 0    bridges + controller
-#   core 1    detector host side (yolo_producer needs the Hailo NPU)
+#   core 0    controller (+ lightweight bridges: sim_camera_bridge, ovcam_bridge)
+#   core 1    detector — yolo_producer (host, Hailo NPU) + yolo_bridge (container, reads shm)
```

## Critic verdict & concerns

Fix is correct. Co-locating yolo_bridge and yolo_producer on core 1 matches the
shared-memory access pattern and removes false sharing with the controller.

## KISS verdict

One-line change per config file, no code change.
