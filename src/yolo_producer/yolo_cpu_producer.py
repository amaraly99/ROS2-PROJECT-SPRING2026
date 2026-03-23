#!/usr/bin/env python3
"""
yolo_cpu_producer — YOLO inference on ARM CPU (Ultralytics PyTorch).

Drop-in replacement for yolo_producer.py that bypasses the Hailo NPU
and runs YOLO26 (n/s/m/l/xl) directly on the ARM CPU via Ultralytics.

Reads camera frames from ovcam SHM, runs CPU inference, and writes
detections to yolo SHM — same interface as yolo_producer.py so the
rest of the pipeline (yolo_bridge → visp_servo) is unchanged.

Usage:
    # Benchmark mode (no SHM output, just timing):
    python3 yolo_cpu_producer.py --model yolo26n.pt --benchmark --frames 200

    # Live mode (writes to SHM for full pipeline):
    python3 yolo_cpu_producer.py --model yolo26n.pt --no-image

    # Compare all sizes:
    for size in n s m l; do
        python3 yolo_cpu_producer.py --model yolo26${size}.pt --benchmark --frames 100
    done

Config comparisons enabled by this script:
    Config A (baseline):   yolo_producer.py   — YOLO26n on Hailo NPU (INT8)
    Config B:              yolo_cpu_producer.py --model yolo26n.pt  — CPU FP32
    Config C/D/E:          --model yolo26s/m/l.pt — larger CPU models

Prerequisites:
    pip install ultralytics torch torchvision
"""

import argparse
import os
import signal
import sys
import time
from typing import List

import cv2
import numpy as np

# ── Import SHM classes from existing yolo_producer ───────────────────────────
# OvcamReader + YoloShmWriter are module-level classes in yolo_producer.py
# Python only runs the if __name__ == "__main__" block when invoked directly.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from yolo_producer import OvcamReader, YoloShmWriter  # noqa: E402

# ── Signal handling ───────────────────────────────────────────────────────────
g_stop = False

def _sig(signum, frame):
    global g_stop
    g_stop = True

signal.signal(signal.SIGINT,  _sig)
signal.signal(signal.SIGTERM, _sig)


# ── Inference wrapper ─────────────────────────────────────────────────────────

class CPUInference:
    """Wraps Ultralytics YOLO for synchronous CPU inference."""

    def __init__(self, model_path: str, img_size: int = 640,
                 conf_thresh: float = 0.25, iou_thresh: float = 0.45,
                 device: str = "cpu"):
        from ultralytics import YOLO
        print(f"[yolo_cpu] loading model: {model_path}  device={device}")
        self.model = YOLO(model_path)
        self.img_size = img_size
        self.conf = conf_thresh
        self.iou  = iou_thresh
        self.device = device

        # Warm-up: one dummy pass to initialise torch / JIT
        dummy = np.zeros((img_size, img_size, 3), dtype=np.uint8)
        self.model.predict(dummy, imgsz=img_size, conf=conf_thresh,
                           iou=iou_thresh, device=device,
                           verbose=False)
        print(f"[yolo_cpu] model ready")

    def infer(self, rgb_frame: np.ndarray) -> List[dict]:
        """
        Run inference on an RGB frame.
        Returns list of dicts: {x1, y1, x2, y2, class_id, class_name, confidence}
        Coordinates are in source image pixel space.
        """
        results = self.model.predict(
            rgb_frame,
            imgsz=self.img_size,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )
        dets = []
        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            boxes_xyxy = r.boxes.xyxy.cpu().numpy()
            confs      = r.boxes.conf.cpu().numpy()
            cls_ids    = r.boxes.cls.cpu().numpy().astype(int)
            cls_names  = [r.names[c] for c in cls_ids]
            for i in range(len(boxes_xyxy)):
                dets.append({
                    "x1":        float(boxes_xyxy[i][0]),
                    "y1":        float(boxes_xyxy[i][1]),
                    "x2":        float(boxes_xyxy[i][2]),
                    "y2":        float(boxes_xyxy[i][3]),
                    "class_id":  int(cls_ids[i]),
                    "class_name": cls_names[i],
                    "confidence": float(confs[i]),
                })
        return dets


# ── Timing stats helper ───────────────────────────────────────────────────────

class TimingAccumulator:
    def __init__(self, keys):
        self._acc = {k: 0.0 for k in keys}
        self.count = 0

    def add(self, **kwargs):
        for k, v in kwargs.items():
            self._acc[k] += v
        self.count += 1

    def mean(self, key):
        n = self.count if self.count > 0 else 1
        return self._acc[key] / n

    def summary(self):
        n = self.count if self.count > 0 else 1
        return {k: v / n for k, v in self._acc.items()}


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="YOLO CPU inference producer (Ultralytics)")
    p.add_argument("--model",    default="yolo26n.pt",
                   help="Ultralytics .pt model file (default: yolo26n.pt)")
    p.add_argument("--conf",     type=float, default=0.25,
                   help="Confidence threshold (default: 0.25)")
    p.add_argument("--iou",      type=float, default=0.45,
                   help="IoU threshold for NMS (default: 0.45)")
    p.add_argument("--img-size", type=int, default=640,
                   help="Model input size (default: 640)")
    p.add_argument("--device",   default="cpu",
                   help="torch device: 'cpu' or '0' for GPU (default: cpu)")
    p.add_argument("--no-image", action="store_true",
                   help="Do not write annotated image to yolo SHM")
    p.add_argument("--benchmark", action="store_true",
                   help="Benchmark mode: time inference only, skip SHM write")
    p.add_argument("--frames",   type=int, default=200,
                   help="Number of frames for benchmark mode (default: 200)")
    p.add_argument("--csv",      default="",
                   help="Path to write per-frame timing CSV")
    return p.parse_args()


def main():
    global g_stop
    args = parse_args()

    # ── Model ────────────────────────────────────────────────────────
    model_path = args.model
    if not os.path.isabs(model_path):
        # Try models/ directory relative to project root
        project_root = os.path.dirname(os.path.dirname(_HERE))
        candidate = os.path.join(project_root, "models", model_path)
        if os.path.exists(candidate):
            model_path = candidate

    infer = CPUInference(model_path, img_size=args.img_size,
                         conf_thresh=args.conf, iou_thresh=args.iou,
                         device=args.device)

    # ── Camera SHM ───────────────────────────────────────────────────
    reader = OvcamReader("/ovcam_frames", "/sem.ovcam_ready")
    print(f"[yolo_cpu] camera: {reader.w}x{reader.h}, {reader.slot_count} slots")

    # ── YOLO SHM writer (skip if benchmark mode) ──────────────────────
    writer = None
    if not args.benchmark:
        writer = YoloShmWriter("/yolo_shm", "/sem.yolo_ready",
                                img_w=reader.w, img_h=reader.h,
                                n_slots=4, include_image=not args.no_image)
        print(f"[yolo_cpu] yolo SHM writer ready")

    # ── Optional CSV ─────────────────────────────────────────────────
    csv_file = None
    csv_writer_obj = None
    if args.csv:
        import csv as _csv
        csv_file = open(args.csv, "w", newline="")
        csv_writer_obj = _csv.writer(csv_file)
        csv_writer_obj.writerow(
            ["frame", "read_ms", "preprocess_ms", "inference_ms",
             "postprocess_ms", "total_ms", "n_dets"])

    timing = TimingAccumulator(
        ["read_ms", "preprocess_ms", "inference_ms", "postprocess_ms", "total_ms"])

    frame_count = 0
    t_start = time.monotonic()
    t_window = t_start
    target_frames = args.frames if args.benchmark else 10**9

    print(f"[yolo_cpu] starting — model={os.path.basename(model_path)} "
          f"device={args.device} benchmark={args.benchmark}")

    while not g_stop and frame_count < target_frames:
        # 1. Read frame
        t0 = time.monotonic()
        try:
            rgb, t_ns = reader.wait_frame(timeout_s=1.0)
        except TimeoutError:
            continue
        t1 = time.monotonic()
        read_ms = (t1 - t0) * 1000.0

        # 2. Preprocess (resize to model input if needed)
        t_pre = time.monotonic()
        if rgb.shape[:2] != (args.img_size, args.img_size):
            input_frame = cv2.resize(rgb, (args.img_size, args.img_size))
        else:
            input_frame = rgb
        t_pre_end = time.monotonic()
        preprocess_ms = (t_pre_end - t_pre) * 1000.0

        # 3. CPU Inference (includes postprocess in Ultralytics pipeline)
        t_infer = time.monotonic()
        dets = infer.infer(input_frame)
        t_infer_end = time.monotonic()
        inference_ms = (t_infer_end - t_infer) * 1000.0

        # Scale bboxes back to source image space (if resize was applied)
        t_post = time.monotonic()
        if rgb.shape[:2] != (args.img_size, args.img_size):
            sx = rgb.shape[1] / args.img_size
            sy = rgb.shape[0] / args.img_size
            for d in dets:
                d["x1"] *= sx; d["x2"] *= sx
                d["y1"] *= sy; d["y2"] *= sy
        # Convert to yolo_producer SHM format (x1/y1/x2/y2 already present)
        # Rename class_name → class_name (already correct key)
        t_post_end = time.monotonic()
        postprocess_ms = (t_post_end - t_post) * 1000.0

        total_ms = (t_post_end - t0) * 1000.0

        # 4. Write to SHM (skip in benchmark mode)
        if writer is not None:
            writer.write(dets, rgb if not args.no_image else None, t_ns)

        # Accumulate stats
        timing.add(read_ms=read_ms, preprocess_ms=preprocess_ms,
                   inference_ms=inference_ms, postprocess_ms=postprocess_ms,
                   total_ms=total_ms)

        if csv_writer_obj is not None:
            csv_writer_obj.writerow([
                frame_count + 1,
                f"{read_ms:.3f}", f"{preprocess_ms:.3f}",
                f"{inference_ms:.3f}", f"{postprocess_ms:.3f}",
                f"{total_ms:.3f}", len(dets)])

        frame_count += 1

        # Print progress every 50 frames
        if frame_count % 50 == 0:
            now = time.monotonic()
            fps = frame_count / (now - t_start)
            s = timing.summary()
            print(f"[yolo_cpu] {frame_count:4d} frames  {fps:5.1f} fps | "
                  f"infer={s['inference_ms']:.1f}ms  "
                  f"total={s['total_ms']:.1f}ms  "
                  f"dets={len(dets)}")

    # ── Final summary ─────────────────────────────────────────────────
    elapsed = time.monotonic() - t_start
    avg_fps = frame_count / elapsed if elapsed > 0 else 0
    s = timing.summary()

    print()
    print("═" * 60)
    print(f"  Model        : {os.path.basename(model_path)}")
    print(f"  Device       : {args.device}")
    print(f"  Frames       : {frame_count}  ({elapsed:.1f}s)")
    print(f"  Avg FPS      : {avg_fps:.1f}")
    print(f"  Inference    : {s['inference_ms']:.1f} ms/frame  (incl. Ultralytics NMS)")
    print(f"  Preprocess   : {s['preprocess_ms']:.2f} ms/frame")
    print(f"  Total E2E    : {s['total_ms']:.1f} ms/frame")
    print("═" * 60)

    # Compare note vs NPU baseline
    NPU_BASELINE_MS = 25.1  # NPU inference baseline (from Session 3 measurement)
    ratio = s["inference_ms"] / NPU_BASELINE_MS if NPU_BASELINE_MS > 0 else 0
    print(f"  vs NPU baseline ({NPU_BASELINE_MS:.1f} ms): {ratio:.1f}×  "
          f"({'FASTER' if ratio < 1 else 'SLOWER'})")
    print()

    if csv_file:
        csv_file.close()
        print(f"  Timing CSV   : {args.csv}")


if __name__ == "__main__":
    main()
