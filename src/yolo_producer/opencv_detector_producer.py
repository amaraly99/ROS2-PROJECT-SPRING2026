#!/usr/bin/env python3
"""
opencv_detector_producer — classical CV SHM detector (no DNN, no NPU).

Reads camera frames from /ovcam_frames and writes the standard /yolo_shm
detection schema (YoloDet array + seqlock + /yolo_ready sem_post) so
yolo_bridge and the controller see no difference from a YOLO producer.
Demonstrates the framework's plug-and-play modularity claim with the maximum
possible contrast to DNN detection.

Method: MOG2 background subtraction → morphology (OPEN 3x3, CLOSE 5x5) →
contour extraction → bounding boxes. class_id=0 for every detection
("object" — map it in yolo_bridge via class_name_map:='0:object' and point
visp at target_class:='object'; run_stack_hil.sh --detector opencv does both).

Confidence: contour_area / bbox_area (fill ratio, 0..1). NOTE: this deviates
from the original spec (contour_area / IMG_area) deliberately — an object
filling 10% of the frame would score 0.10 and be silently dropped by the
controller's min_confidence (0.20-0.30) floor, leaving the loop open. Fill
ratio lands in 0.5-0.9 for compact objects, which behaves like a confidence.

Telemetry: same 7-stamp format as yolo_shm_producer; for this detector
  preprocess  = (nothing — frame is used as delivered)        ~0 ms
  inference   = MOG2 apply + morphology + findContours
  nms         = area filter + sort + truncate to max-detections

Warmup: MOG2 needs ~50 frames to stabilise its background model; detections
are suppressed during warmup ("MOG2 warming up (N/50)" on stderr).

CLI:
    python3 opencv_detector_producer.py [--conf 0.5] [--min-area 500]
            [--max-detections 10] [--benchmark] [--frames N]
            [--telemetry_csv PATH]
"""

import argparse
import os
import sys
import time

import cv2

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from detector_producer_base import run_producer  # noqa: E402

WARMUP_FRAMES = 50


def _now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


class MOG2Predictor:
    """Predictor-contract wrapper around MOG2 + contours (see
    detector_producer_base for the infer()/timing contract)."""

    def __init__(self, conf: float, min_area: int, max_detections: int):
        self.conf = conf
        self.min_area = min_area
        self.max_detections = max_detections
        self.mog2 = cv2.createBackgroundSubtractorMOG2(
            history=200, varThreshold=16, detectShadows=False)
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.frames_seen = 0

    def infer(self, rgb, timing=None):
        if timing is None:
            timing = {}
        timing["pre_done"] = _now_ns()      # no preprocessing needed
        timing["inf_start"] = timing["pre_done"]

        fg = self.mog2.apply(rgb)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, self.kernel_open)
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, self.kernel_close)
        contours, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        timing["inf_done"] = _now_ns()

        self.frames_seen += 1
        if self.frames_seen <= WARMUP_FRAMES:
            if self.frames_seen % 10 == 0 or self.frames_seen == WARMUP_FRAMES:
                print(f"MOG2 warming up ({self.frames_seen}/{WARMUP_FRAMES})",
                      file=sys.stderr)
            timing["nms_done"] = _now_ns()
            return []

        cands = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            fill = area / float(max(w * h, 1))
            if fill < self.conf:
                continue
            cands.append((area, (x, y, x + w, y + h, 0, min(fill, 1.0))))
        cands.sort(key=lambda t: t[0], reverse=True)
        dets = [d for _, d in cands[:self.max_detections]]
        timing["nms_done"] = _now_ns()
        return dets

    def close(self):
        pass


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="OpenCV MOG2 SHM detector producer")
    ap.add_argument("--conf", type=float, default=0.5,
                    help="min fill ratio (contour_area/bbox_area) — confidence proxy")
    ap.add_argument("--min-area", dest="min_area", type=int, default=500,
                    help="min contour area in px^2")
    ap.add_argument("--max-detections", dest="max_detections", type=int, default=10)
    ap.add_argument("--benchmark", action="store_true",
                    help="print a timing summary on exit")
    ap.add_argument("--frames", dest="max_frames", type=int, default=0,
                    help="stop after N frames (0 = unlimited)")
    ap.add_argument("--telemetry_csv", default=None,
                    help="per-frame 7-stamp telemetry CSV path")
    ap.add_argument("--with-image", dest="no_image", action="store_false",
                    default=True, help="also write the frame into /yolo_shm")
    return ap.parse_args(argv)


def main(argv=None):
    cfg = parse_args(argv)
    if cfg.benchmark and not cfg.telemetry_csv:
        cfg.telemetry_csv = "/tmp/opencv_detector_telemetry.csv"

    predictor = MOG2Predictor(cfg.conf, cfg.min_area, cfg.max_detections)
    run_producer(
        cfg, predictor, label="opencv-mog2",
        startup_line=("Producer: opencv-mog2, resolution: {w}x{h}, "
                      "target: object (class_id=0), backend: cpu"))

    if cfg.benchmark and cfg.telemetry_csv and os.path.exists(cfg.telemetry_csv):
        import csv as _csv
        with open(cfg.telemetry_csv) as f:
            rows = list(_csv.DictReader(f))
        if rows:
            det_ms = sorted(float(r["inference_ms"]) for r in rows)
            e2e_ms = sorted(float(r["e2e_capture_to_commit_ms"]) for r in rows)
            n = len(det_ms)
            print(f"[benchmark] {n} frames  "
                  f"detect p50={det_ms[n//2]:.2f}ms p95={det_ms[int(n*0.95)]:.2f}ms  "
                  f"capture→commit p50={e2e_ms[n//2]:.2f}ms "
                  f"p95={e2e_ms[int(n*0.95)]:.2f}ms")


if __name__ == "__main__":
    main()
