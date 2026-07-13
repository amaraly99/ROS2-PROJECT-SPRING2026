# yolo_msgs

ROS 2 message definitions for detector output. Built by `yolo_bridge` and consumed by `visp_servo`.

| Message | Fields |
|---|---|
| `Detection.msg` | `class_name`, `confidence`, `center_x`, `center_y`, `size_width`, `size_height` — all in pixels |
| `DetectionArray.msg` | `std_msgs/Header header` + `Detection[] detections` |

Detector-agnostic on purpose: YOLO, MOG2 and OWL-ViT all publish this same type, which is what makes
the detector swappable without touching the controller.

```bash
colcon build --packages-select yolo_msgs --symlink-install
```

Build this before `yolo_bridge` or `visp_servo`.
