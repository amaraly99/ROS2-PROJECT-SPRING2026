# Troubleshooting

### yolo_producer reports 0 detections

Not a bug if nothing detectable is in frame — point the camera at a COCO object. If objects are
clearly present and nothing fires, lower `--conf` to 0.1. If that still yields nothing on the NPU
but the CPU/ONNX path detects fine, you are looking at a postprocess parser bug, not a bad model:
the Hailo NMS-BY-CLASS output is **variable-length per class**, and parsing it with a fixed stride
silently drops every class after the first.

### yolo_bridge cannot map the shared memory

Start `yolo_producer` **before** `yolo_bridge` — the producer creates `/yolo_shm`, the bridge only
reads it. If you restart the producer while the bridge is up, restart the bridge too; it is holding
a stale mmap of the old segment.

### The container cannot see shared memory at all

Docker must be started with `--ipc=host`, and `ovcam_producer` must be running before
`ovcam_bridge`. `start_container.sh` handles the former.

### "Lost communication with the server" from Hailo

The Python process crashed mid-inference. Restart `yolo_producer.py`. If it persists, check that
`/dev/hailo0` exists and `hailortcli fw-control identify` succeeds.

If the NPU needs a reset, **escalate gently**: try re-running first, then a soft reset. Do not start
with `hailortcli --reset-type chip` — it can wedge the vDMA firmware upload and force a full reboot.

### OV²SLAM never initializes ("Parallax X.X < threshold")

It needs lateral translation, not rotation. Pan the camera side to side. If it still will not
initialize, lower `finit_parallax` (see [TUNING.md](TUNING.md)).

### Foxglove shows ~10 Hz

Not a bug. Each frame goes through the full SLAM pipeline before the next is accepted; at 640×480
with KLT and BA, ~10 Hz is expected. Set `force_realtime: 1` so stale frames are dropped rather than
queued.

### OV²SLAM does not receive images

Check that `Camera.topic_left` in the YAML matches a real topic (`ros2 topic list | grep image`).
For the live camera it should be `/ovcam/image_raw`.

### YOLO rate halves when SLAM is running

Known and fixed. `yolo_producer` and `ovcam_bridge` used to share the POSIX semaphore
`/ovcam_ready`, so each consumer stole wakeups from the other. The producer now polls `write_seq`
instead, which restored YOLO from 7 Hz to 18 Hz. If you add a third consumer of the camera segment,
do not have it wait on that semaphore.

### Core dumps filling the disk

SLAM crashes leave `core.*` files. They are git-ignored; delete them with `rm -f core.*`, or prevent
them with `ulimit -c 0` before launching.

### Build error: `Sophus/se3.hpp not found`

The vendored third-party libraries are not built yet:

```bash
cd src/ov2slam_ros/Thirdparty && ./build_thirdparty.sh
```

### Build error: SuiteSparse not found

Expected. The Dockerfile deliberately builds Ceres without SuiteSparse; it does not affect
functionality.

### OV²SLAM cannot find OpenCV contrib symbols (`xfeatures2d`)

OpenCV is built from source at `/workspace/opencv/build` inside the container, **not** the system
OpenCV. `src/ov2slam_ros/CMakeLists.txt` needs `link_directories("/workspace/opencv/build/lib")`.

### The HIL sim deadlocks when the supervisor restarts the model

The supervisor's poll callback must never block. It can fire re-entrantly from *inside* the Simulink
3D engine's step, so a blocking wait for the model to stop prevents that step from finishing, which
prevents the model from ever stopping. `matlab/hil_run_supervisor.m` is a non-blocking state machine
for exactly this reason — do not "simplify" it back into a blocking wait.
