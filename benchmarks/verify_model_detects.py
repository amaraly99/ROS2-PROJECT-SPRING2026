#!/usr/bin/env python3
"""
verify_model_detects.py — quick offline check that a model actually detects.

Runs ONE model (NPU HEF or CPU ONNX) on a still image and prints the detected
class names + confidences. Use it to confirm a HEF is healthy (detects a stop
sign, not just 'person') BEFORE spending an hour on a HIL sweep.

This is the tool that exposed Finding 9 (yolov8/v11 HEFs only detect 'person').

Usage:
    # NPU HEF:
    python3 benchmarks/verify_model_detects.py yolov8n npu /tmp/stop1.jpg
    # CPU ONNX:
    python3 benchmarks/verify_model_detects.py yolov8n cpu /tmp/stop1.jpg

A healthy stop-sign image should yield: "stop sign  conf=0.9x".
"""
import sys
import os

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "src", "yolo_producer"))

import cv2  # noqa: E402
from yolo_runtime_engine import build_predictor  # noqa: E402
from yolo_producer import COCO_NAMES  # noqa: E402


def main():
    if len(sys.argv) < 4:
        print("usage: verify_model_detects.py <model_id> <npu|cpu> <image> [conf]")
        sys.exit(2)
    model, backend, img_path = sys.argv[1], sys.argv[2], sys.argv[3]
    conf = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25

    bgr = cv2.imread(img_path)
    if bgr is None:
        print(f"ERROR: cannot read image: {img_path}")
        sys.exit(1)
    img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    p = build_predictor(backend, model_id=model, conf=conf, iou=0.45)
    dets = p.infer(img, timing={})
    print(f"{model} [{backend}] conf={conf} on {os.path.basename(img_path)}: "
          f"{len(dets)} detections")
    for (x1, y1, x2, y2, c, cf) in sorted(dets, key=lambda d: -d[5])[:12]:
        name = COCO_NAMES[c] if c < len(COCO_NAMES) else f"cls{c}"
        print(f"   {name:<16} conf={cf:.3f}  box=({int(x1)},{int(y1)},{int(x2)},{int(y2)})")
    try:
        p.close()
    except Exception:
        pass
    # exit non-zero if nothing detected — handy in shell loops
    sys.exit(0 if dets else 3)


if __name__ == "__main__":
    main()
