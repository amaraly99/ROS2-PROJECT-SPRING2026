#!/usr/bin/env python3
"""
yolo_shm_producer — generic, fully-parameterized SHM YOLO producer.

Successor to yolo_cpu_producer.py: a single backend-agnostic host process that

  1. reads NV12 camera frames from /ovcam_frames (POSIX shm seqlock, zero-copy)
     via the verified OvcamReader,
  2. runs inference through yolo_runtime_engine (CPU/onnxruntime or NPU/pyHailoRT,
     selected at runtime),
  3. writes detections (+ optional annotated frame) to /yolo_shm via YoloShmWriter
     for the existing C++ yolo_bridge to consume — UNCHANGED binary layout, and
  4. stamps 7 high-resolution monotonic timestamps per frame for the paper's
     per-stage latency-breakdown figures.

Every choice (backend, model, threads, image on/off) is a CLI flag AND an env var,
so one shell script can sweep the whole matrix on a single branch. An optional
rclpy LifecycleNode wrapper (--ros2) exposes the same knobs as ROS2 parameters for
launch-file integration; the default path stays ROS2/DDS-free (the zero-overhead
host property is itself a paper result).

Telemetry (binary-safe): the 7 stamps are packed with struct.pack("<7Q", ...) into
the spare YoloSlotHeader._reserved[8] (offset +56, 56 of its 64 bytes), INSIDE the
seqlock window. The C++ bridge ignores _reserved, so /yolo_shm stays compatible and
needs no rebuild. The stamps are also mirrored to a per-run CSV.

Usage:
    python3 yolo_shm_producer.py --backend npu --model_variant n --model_family yolo26
    python3 yolo_shm_producer.py --backend cpu --model_id yolov8n --threads 2 \
        --telemetry_csv /tmp/telem.csv
    YOLO_BACKEND=cpu YOLO_MODEL=yolov11s YOLO_CPU_THREADS=2 python3 yolo_shm_producer.py
"""

import argparse
import csv
import hashlib
import os
import signal
import struct
import sys
import threading
import time
from typing import Dict, List, Optional

# Engine + reused SHM plumbing live next to this file.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from yolo_runtime_engine import build_predictor, ModelRegistry  # noqa: E402
# Reuse the verified SHM classes + layout constants (no copy-paste drift).
import yolo_producer as yp  # noqa: E402
from yolo_producer import (  # noqa: E402
    OvcamReader, YoloShmWriter, YOLO_SLOT_HDR, YOLO_IMG_OFFSET, YOLO_MAX_DETS,
)

# Offset of _reserved[8] within YoloSlotHeader (see yolo_shm.hpp):
#   seq(8)+t_ns(8)+det_count(4)+img_w(4)+img_h(4)+img_stride(4)+img_bytes(4)
#   +_pad[5](20) = 56  ->  _reserved[8] starts here. We use 7*8 = 56 of its 64 bytes.
TELEM_RESERVED_OFF = 56
TELEM_FIELDS = (
    "ts_capture",              # frame written by camera driver (from SlotHeader.t_ns)
    "ts_yolo_shm_read",        # YOLO picked the frame up from shm
    "ts_yolo_preprocess_done", # resize/normalize finished
    "ts_inference_start",      # handed to the execution provider
    "ts_inference_done",       # raw tensors returned
    "ts_nms_done",             # bbox parsing / NMS finished
    "ts_yolo_shm_write",       # detections committed back to shm
    "frame_hash",              # blake2b-4 of raw RGB frame; repeated value → frozen camera
)


def _now_ns() -> int:
    """CLOCK_MONOTONIC ns — same epoch as the C++ camera driver's t_ns stamp."""
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


# ═════════════════════════════════════════════════════════════════════════════
# Telemetry-aware shm writer
# ═════════════════════════════════════════════════════════════════════════════
class TelemetryYoloShmWriter(YoloShmWriter):
    """YoloShmWriter that also commits the 7-point timing struct into the slot
    header's reserved bytes, inside the same seqlock window as the detections.

    We override write() wholesale (rather than post-patching) so the telemetry is
    written BEFORE the seqlock closes — otherwise a consumer could observe a 'done'
    slot whose timing bytes are still being written (a torn read).
    """

    def write(self, dets: list, img, t_ns: int,
              telemetry: Optional[Dict[str, int]] = None):
        self._frame_num += 1
        fn = self._frame_num
        idx = (fn - 1) % self.slot_count
        slot_off = 128 + idx * self.slot_size          # after YoloShmGlobal (128 B)

        det_count = min(len(dets), YOLO_MAX_DETS)

        # ── seqlock open (odd) ────────────────────────────────────────────────
        self.mm[slot_off:slot_off + 8] = struct.pack("<Q", fn * 2 - 1)

        # ── slot header meta (unchanged layout) ───────────────────────────────
        write_img = self.include_image and img is not None
        meta = struct.pack("<QIIIIi",
                           t_ns, det_count,
                           self.img_w, self.img_h, self.img_stride,
                           self.img_bytes if write_img else 0)
        self.mm[slot_off + 8:slot_off + 8 + len(meta)] = meta

        # ── detection array ───────────────────────────────────────────────────
        det_off = slot_off + YOLO_SLOT_HDR
        for i in range(det_count):
            d = dets[i]
            self.mm[det_off + i * 16:det_off + i * 16 + 16] = struct.pack(
                "<HHfHHHH", d["class_id"], 0, d["confidence"],
                d["x1"], d["y1"], d["x2"], d["y2"])

        # ── annotated image (optional) ────────────────────────────────────────
        if write_img:
            img_off = slot_off + YOLO_IMG_OFFSET
            img_flat = img.tobytes()
            self.mm[img_off:img_off + len(img_flat)] = img_flat

        # ── telemetry into _reserved[8] (the 7th stamp = the commit itself) ───
        ts_write = _now_ns()
        if telemetry is not None:
            packed = struct.pack(
                "<7Q",
                int(telemetry.get("ts_capture", t_ns)),
                int(telemetry.get("ts_yolo_shm_read", 0)),
                int(telemetry.get("ts_yolo_preprocess_done", 0)),
                int(telemetry.get("ts_inference_start", 0)),
                int(telemetry.get("ts_inference_done", 0)),
                int(telemetry.get("ts_nms_done", 0)),
                int(ts_write),
            )
            # 56-byte window, well within the 64-byte _reserved region.
            self.mm[slot_off + TELEM_RESERVED_OFF:
                    slot_off + TELEM_RESERVED_OFF + 56] = packed
            telemetry["ts_yolo_shm_write"] = ts_write     # report back for CSV

        # ── seqlock close (even) + publish + wake consumer ────────────────────
        self.mm[slot_off:slot_off + 8] = struct.pack("<Q", fn * 2)
        self.mm[64:72] = struct.pack("<Q", fn)
        yp._pt.sem_post(self.sem)


# ═════════════════════════════════════════════════════════════════════════════
# Producer
# ═════════════════════════════════════════════════════════════════════════════
class YoloShmProducer:
    """Camera-shm -> predictor -> yolo-shm, with per-frame telemetry."""

    def __init__(self, cfg: argparse.Namespace):
        self.cfg = cfg

        # Resolve the model id (registry id OR family+variant; --model_path bypasses).
        model_id = cfg.model_id
        if model_id is None and cfg.model_family and cfg.model_variant:
            model_id = ModelRegistry.id_from(cfg.model_family, cfg.model_variant)
        self.model_id = model_id

        # Build the predictor — backend chosen at runtime.
        self.predictor = build_predictor(
            cfg.backend, model_id=model_id, model_path=cfg.model_path,
            conf=cfg.conf, iou=cfg.iou,
            threads=cfg.threads, inter_threads=cfg.inter_threads)

        # Camera reader (zero-copy seqlock) + telemetry-aware writer.
        self.reader = OvcamReader()
        self.writer = TelemetryYoloShmWriter(
            self.reader.w, self.reader.h, include_image=not cfg.no_image)

        # Per-run telemetry CSV (mirror of the in-shm timing struct).
        self.csv_file = None
        self.csv_writer = None
        if cfg.telemetry_csv:
            os.makedirs(os.path.dirname(os.path.abspath(cfg.telemetry_csv)), exist_ok=True)
            self.csv_file = open(cfg.telemetry_csv, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["frame", *TELEM_FIELDS,
                 "capture_read_ms", "preprocess_ms", "inference_ms",
                 "nms_ms", "shmwrite_ms", "e2e_capture_to_commit_ms", "n_dets"])

        self.frame_count = 0
        label = self.model_id or os.path.basename(cfg.model_path or "?")
        print(f"[yolo_shm_producer] backend={cfg.backend} model={label} "
              f"threads={cfg.threads} image={'off' if cfg.no_image else 'on'} "
              f"csv={cfg.telemetry_csv or '-'}")

    # -- one frame ---------------------------------------------------------
    def step(self, timeout_s: float = 1.0) -> bool:
        """Process one frame. Returns False on read timeout."""
        try:
            rgb, ts_capture = self.reader.wait_frame(timeout_s=timeout_s)
        except TimeoutError:
            return False

        stamps: Dict[str, int] = {"ts_capture": int(ts_capture)}
        stamps["frame_hash"] = int.from_bytes(
            hashlib.blake2b(rgb.tobytes(), digest_size=4).digest(), "little")
        stamps["ts_yolo_shm_read"] = _now_ns()

        # Engine fills pre_done / inf_start / inf_done / nms_done (absolute ns).
        timing: Dict[str, int] = {}
        dets_t = self.predictor.infer(rgb, timing=timing)
        stamps["ts_yolo_preprocess_done"] = timing.get("pre_done", 0)
        stamps["ts_inference_start"] = timing.get("inf_start", 0)
        stamps["ts_inference_done"] = timing.get("inf_done", 0)
        stamps["ts_nms_done"] = timing.get("nms_done", 0)

        # canonical tuple -> SHM dict (clamp to uint16 box range)
        dets = [{
            "class_id": int(c), "confidence": float(cf),
            "x1": max(0, int(x1)), "y1": max(0, int(y1)),
            "x2": max(0, int(x2)), "y2": max(0, int(y2)),
        } for (x1, y1, x2, y2, c, cf) in dets_t]

        # commit (writer stamps ts_yolo_shm_write inside the seqlock)
        self.writer.write(dets, rgb if not self.cfg.no_image else None,
                          int(ts_capture), telemetry=stamps)

        self.frame_count += 1
        self._log_frame(stamps, len(dets))
        return True

    def _log_frame(self, s: Dict[str, int], n_dets: int):
        if self.csv_writer is not None:
            def dms(a, b):  # ns delta -> ms
                return f"{(s[a] - s[b]) / 1e6:.4f}" if s.get(a) and s.get(b) else "0.0"
            self.csv_writer.writerow([
                self.frame_count, *[s.get(k, 0) for k in TELEM_FIELDS],
                dms("ts_yolo_shm_read", "ts_capture"),
                dms("ts_yolo_preprocess_done", "ts_yolo_shm_read"),
                dms("ts_inference_done", "ts_inference_start"),
                dms("ts_nms_done", "ts_inference_done"),
                dms("ts_yolo_shm_write", "ts_nms_done"),
                dms("ts_yolo_shm_write", "ts_capture"),
                n_dets])

        if self.frame_count % 100 == 0:
            e2e = (s["ts_yolo_shm_write"] - s["ts_capture"]) / 1e6
            inf = (s["ts_inference_done"] - s["ts_inference_start"]) / 1e6
            print(f"[yolo_shm_producer] {self.frame_count} frames | "
                  f"infer={inf:.1f}ms capture→commit={e2e:.1f}ms dets={n_dets}")

    # -- loops -------------------------------------------------------------
    def run_forever(self, stop: threading.Event):
        max_frames = self.cfg.max_frames
        while not stop.is_set():
            self.step(timeout_s=1.0)
            if max_frames > 0 and self.frame_count >= max_frames:
                print(f"[yolo_shm_producer] reached --max-frames={max_frames}")
                break

    def close(self):
        try:
            self.predictor.close()
        except Exception:
            pass
        if self.csv_file is not None:
            self.csv_file.close()
            print(f"[yolo_shm_producer] telemetry CSV → {self.cfg.telemetry_csv}")
        print(f"[yolo_shm_producer] stopped after {self.frame_count} frames")


# ═════════════════════════════════════════════════════════════════════════════
# Config: CLI flags with env-var fallbacks (so launch files / sweeps can set either)
# ═════════════════════════════════════════════════════════════════════════════
def _env(name: str, default=None):
    v = os.environ.get(name)
    return v if v not in (None, "") else default


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Generic SHM YOLO producer")
    ap.add_argument("--backend", default=_env("YOLO_BACKEND", "npu"),
                    choices=["cpu", "npu"])
    ap.add_argument("--model_id", default=_env("YOLO_MODEL"),
                    help="registry id e.g. yolov8n (overrides family/variant)")
    ap.add_argument("--model_family", default=_env("YOLO_MODEL_FAMILY"),
                    help="v8 | v11 | yolo26")
    ap.add_argument("--model_variant", default=_env("YOLO_MODEL_VARIANT"),
                    help="n | s | m")
    ap.add_argument("--model_path", default=_env("YOLO_MODEL_PATH"),
                    help="explicit .onnx/.hef path (bypasses registry)")
    ap.add_argument("--threads", type=int,
                    default=int(_env("YOLO_CPU_THREADS", "2")),
                    help="onnxruntime intra_op threads (CPU backend)")
    ap.add_argument("--inter_threads", type=int,
                    default=int(_env("YOLO_CPU_INTER_THREADS", "1")))
    ap.add_argument("--conf", type=float,
                    default=float(_env("YOLO_CONF", "0.25")))
    ap.add_argument("--iou", type=float,
                    default=float(_env("YOLO_IOU", "0.45")))
    ap.add_argument("--no-image", dest="no_image", action="store_true",
                    default=_env("YOLO_NO_IMAGE", "") not in ("", "0", "false"),
                    help="skip annotated image in shm (saves ~1.8MB/frame)")
    ap.add_argument("--telemetry_csv", default=_env("YOLO_TELEMETRY_CSV"),
                    help="per-frame 7-stamp telemetry CSV path")
    ap.add_argument("--max-frames", dest="max_frames", type=int,
                    default=int(_env("YOLO_MAX_FRAMES", "0")),
                    help="stop after N frames (0 = unlimited); used by the sweep")
    ap.add_argument("--ros2", action="store_true",
                    help="run as an rclpy LifecycleNode exposing ROS2 parameters")
    return ap.parse_args(argv)


# ═════════════════════════════════════════════════════════════════════════════
# Plain host-process entrypoint (default — no ROS2/DDS)
# ═════════════════════════════════════════════════════════════════════════════
def main_plain(cfg: argparse.Namespace):
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    prod = YoloShmProducer(cfg)
    try:
        prod.run_forever(stop)
    finally:
        prod.close()


# ═════════════════════════════════════════════════════════════════════════════
# Optional rclpy LifecycleNode wrapper (--ros2) for launch-file integration
# ═════════════════════════════════════════════════════════════════════════════
def main_ros2(cfg: argparse.Namespace):
    import rclpy
    from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
    from rclpy.parameter import Parameter

    class YoloShmProducerNode(LifecycleNode):
        """Declares backend/model/threads as ROS2 params; runs the producer loop in
        a worker thread between on_activate and on_deactivate."""

        def __init__(self):
            super().__init__("yolo_shm_producer")
            # ROS2 parameters (overridable via launch file / `ros2 param`)
            self.declare_parameter("backend", cfg.backend)
            self.declare_parameter("model_id", cfg.model_id or "")
            self.declare_parameter("model_family", cfg.model_family or "")
            self.declare_parameter("model_variant", cfg.model_variant or "")
            self.declare_parameter("cpu_threads", cfg.threads)
            self.declare_parameter("telemetry_csv", cfg.telemetry_csv or "")
            self._prod = None
            self._thread = None
            self._stop = threading.Event()

        def _cfg_from_params(self) -> argparse.Namespace:
            g = lambda n: self.get_parameter(n).value
            merged = argparse.Namespace(**vars(cfg))
            merged.backend = g("backend")
            merged.model_id = g("model_id") or None
            merged.model_family = g("model_family") or None
            merged.model_variant = g("model_variant") or None
            merged.threads = int(g("cpu_threads"))
            merged.telemetry_csv = g("telemetry_csv") or None
            return merged

        def on_activate(self, state):
            self.get_logger().info("activating yolo_shm_producer")
            self._stop.clear()
            self._prod = YoloShmProducer(self._cfg_from_params())
            self._thread = threading.Thread(
                target=self._prod.run_forever, args=(self._stop,), daemon=True)
            self._thread.start()
            return TransitionCallbackReturn.SUCCESS

        def on_deactivate(self, state):
            self.get_logger().info("deactivating yolo_shm_producer")
            self._stop.set()
            if self._thread:
                self._thread.join(timeout=3.0)
            if self._prod:
                self._prod.close()
                self._prod = None
            return TransitionCallbackReturn.SUCCESS

        def on_shutdown(self, state):
            self.on_deactivate(state)
            return TransitionCallbackReturn.SUCCESS

    rclpy.init()
    node = YoloShmProducerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    cfg = parse_args(argv)
    if cfg.ros2:
        main_ros2(cfg)
    else:
        main_plain(cfg)


if __name__ == "__main__":
    main()
