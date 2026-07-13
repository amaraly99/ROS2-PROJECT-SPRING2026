# ovcam_producer

Captures NV12 frames from the OV5647 via libcamera and writes them into a POSIX shared-memory ring
buffer (`/ovcam_frames`). **Runs natively on the host, not in Docker** — it touches the camera
hardware.

It is the head of the pipeline: `ovcam_bridge` and `yolo_producer` both read the segment it creates,
so it must be started first.

## Build

On the host, not in the container:

```bash
cd src/ovcam_producer
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```

Needs `libcamera-dev`.

## Run

```bash
taskset -c 0 src/ovcam_producer/build/ovcam_producer --width 640 --height 480 --fps 30
```

Expected:

```
[shm] created '/ovcam_frames': 4 slots x ...
[cam] actual: 640x480 stride=640
```

## The shared-memory protocol

`include/ovcam_shm.hpp` defines the layout, shared verbatim with `ovcam_bridge` and
`yolo_producer`. A seqlock ring buffer: the writer bumps `write_seq` before and after each frame, so
a reader can detect a torn read and retry rather than lock. At 921 KB per frame this is roughly 235×
faster than DDS at the median.

**Consumers must poll `write_seq`, not wait on a semaphore.** There used to be a shared
`/ovcam_ready` semaphore, and because both `yolo_producer` and `ovcam_bridge` waited on it, each
stole wakeups from the other — halving YOLO's rate from 18 Hz to 7 Hz whenever SLAM was running. If
you add a consumer, poll.
