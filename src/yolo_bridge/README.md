# yolo_bridge

Reads detections from the `/yolo_shm` shared-memory segment and republishes them as ROS 2 topics.
The counterpart to `yolo_producer`, which runs on the host; this node runs **in Docker**.

## Build

```bash
# inside the container
cd /workspace
colcon build --packages-select yolo_msgs yolo_bridge --symlink-install
source install/setup.bash
```

`yolo_msgs` must be built first — the message definitions come from there.

## Run

```bash
taskset -c 0,1 ros2 run yolo_bridge yolo_bridge_node
```

Expected: `[INFO] [yolo_bridge]: yolo_shm mapped: 640x480, 4 slots`

## Topics

| Topic | Type | QoS |
|---|---|---|
| `/yolo/detections` | `yolo_msgs/DetectionArray` | best_effort, depth 1 |
| `/yolo/annotated` | `sensor_msgs/Image` (bgr8) | best_effort, depth 1 |

`/yolo/annotated` is only useful for debugging and costs 921 KB per frame — leave it unsubscribed on
the HIL rig.

Start `yolo_producer` before this node (it creates the segment), and restart this node if you restart
the producer.
