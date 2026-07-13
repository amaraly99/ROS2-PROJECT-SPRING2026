# ovcam_bridge

Reads camera frames from the `/ovcam_frames` shared-memory ring and republishes them as ROS 2
`sensor_msgs/Image` (mono8, ~30 Hz) on `/ovcam/image_raw`, which is what OV²SLAM consumes.

Runs **in Docker**. `ovcam_producer` must already be running on the host — this node only reads the
segment, it does not create it. The container needs `--ipc=host` to see it at all.

## Build

```bash
# inside the container
cd /workspace
colcon build --packages-select ovcam_bridge --symlink-install
source install/setup.bash
```

## Run

```bash
taskset -c 0,1 ros2 run ovcam_bridge ovcam_bridge_node
```

Expected: `[INFO] [ovcam_bridge]: shm mapped 640x480`

## Topics

| Topic | Type | QoS |
|---|---|---|
| `/ovcam/image_raw` | `sensor_msgs/Image` (mono8) | best_effort, depth 1 |

If you restart `ovcam_producer`, restart this node too — it holds an mmap of the old segment.
