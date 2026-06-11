#!/usr/bin/env python3
"""
detector_producer_base — shared plumbing for non-YOLO SHM detector producers.

GenericShmDetectorProducer is YoloShmProducer with the predictor injected
instead of built from the model registry. Everything else — OvcamReader,
TelemetryYoloShmWriter (7-stamp telemetry inside the seqlock), per-frame CSV
mirror, run loop, shutdown — is inherited unchanged, so opencv/vlm producers
write the exact same /yolo_shm schema and telemetry format as the YOLO path.

A predictor must provide:
    infer(rgb, timing: dict) -> list[(x1, y1, x2, y2, class_id, conf)]
        filling timing with absolute CLOCK_MONOTONIC ns at keys
        pre_done / inf_start / inf_done / nms_done (same contract as
        yolo_runtime_engine.BaseYoloPredictor)
    close()
"""

import argparse
import csv
import os
import sys
import threading
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from yolo_producer import OvcamReader  # noqa: E402
from yolo_shm_producer import (  # noqa: E402
    YoloShmProducer, TelemetryYoloShmWriter, TELEM_FIELDS,
)


def _now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


class GenericShmDetectorProducer(YoloShmProducer):
    """YoloShmProducer with an externally-supplied predictor."""

    def __init__(self, cfg: argparse.Namespace, predictor, label: str):
        # Mirrors YoloShmProducer.__init__ minus the registry/build_predictor
        # step (our predictor is handed in). Inherits step()/_log_frame()/
        # run_forever()/close() so the SHM write path and telemetry CSV stay
        # byte-identical with the YOLO producer.
        self.cfg = cfg
        self.model_id = label
        self.predictor = predictor

        self.reader = OvcamReader()
        self.writer = TelemetryYoloShmWriter(
            self.reader.w, self.reader.h,
            include_image=not getattr(cfg, "no_image", True))

        self.csv_file = None
        self.csv_writer = None
        if cfg.telemetry_csv:
            os.makedirs(os.path.dirname(os.path.abspath(cfg.telemetry_csv)),
                        exist_ok=True)
            self.csv_file = open(cfg.telemetry_csv, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["frame", *TELEM_FIELDS,
                 "capture_read_ms", "preprocess_ms", "inference_ms",
                 "nms_ms", "shmwrite_ms", "e2e_capture_to_commit_ms", "n_dets"])

        self.frame_count = 0


def wait_for_camera_shm(timeout_s: float = 60.0):
    """Block until the camera bridge has created /ovcam_frames + its semaphore
    (the bridge may still be starting). Mirrors yolo_bridge's shm_open retry."""
    deadline = time.monotonic() + timeout_s
    while not (os.path.exists("/dev/shm/ovcam_frames")
               and os.path.exists("/dev/shm/sem.ovcam_ready")):
        if time.monotonic() >= deadline:
            raise RuntimeError("/ovcam_frames did not appear — camera bridge down?")
        print("[detector_producer] waiting for /ovcam_frames...", file=sys.stderr)
        time.sleep(1.0)


def run_producer(cfg: argparse.Namespace, predictor, label: str,
                 startup_line: str):
    """Common main(): SIGINT-clean run loop with startup summary.

    startup_line may contain {w}/{h} placeholders — filled in from the camera
    SHM dimensions once the reader is attached."""
    import signal

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    # Make sure the camera SHM is up before constructing the producer
    # (GenericShmDetectorProducer builds its own reader; this is the gate).
    wait_for_camera_shm()

    prod = GenericShmDetectorProducer(cfg, predictor, label)
    print(startup_line.format(w=prod.reader.w, h=prod.reader.h), flush=True)
    try:
        prod.run_forever(stop)
    finally:
        prod.close()
