#!/usr/bin/env python3
"""
yolo_producer — Host-side YOLO inference on Hailo-10H NPU.

Reads NV12 camera frames from /ovcam_frames (POSIX shm, written by ovcam_producer),
runs YOLO inference via hailort, post-processes detections (DFL bbox decode + NMS),
draws annotations, and writes results to /yolo_shm for yolo_bridge in Docker.

Usage:
    python3 yolo_producer.py [--hef <path>] [--conf <thresh>] [--iou <thresh>]
"""

import argparse
import ctypes
import math
import mmap
import os
import signal
import struct
import sys
import time
from typing import List, Tuple

import cv2
import numpy as np

# ─── Hailo imports ────────────────────────────────────────────────
from hailo_platform import VDevice, FormatType, HailoSchedulingAlgorithm

# ─── POSIX shm / sem via ctypes ──────────────────────────────────
_rt = ctypes.CDLL("librt.so.1", use_errno=True)
_pt = ctypes.CDLL("libpthread.so.0", use_errno=True)

# shm_open / shm_unlink
_rt.shm_open.argtypes  = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
_rt.shm_open.restype   = ctypes.c_int
_rt.shm_unlink.argtypes = [ctypes.c_char_p]
_rt.shm_unlink.restype  = ctypes.c_int

# sem_open / sem_post / sem_timedwait / sem_unlink
_pt.sem_open.argtypes  = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
_pt.sem_open.restype   = ctypes.c_void_p
_pt.sem_post.argtypes  = [ctypes.c_void_p]
_pt.sem_post.restype   = ctypes.c_int
_pt.sem_unlink.argtypes = [ctypes.c_char_p]
_pt.sem_unlink.restype  = ctypes.c_int

class _Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

_pt.sem_timedwait.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Timespec)]
_pt.sem_timedwait.restype  = ctypes.c_int

O_RDONLY  = 0
O_CREAT   = 0o100
O_RDWR    = 0o2

# ─── Constants matching yolo_shm.hpp ─────────────────────────────
YOLO_MAGIC       = 0x594F4C4F
YOLO_SHM_VERSION = 1
YOLO_MAX_DETS    = 64
YOLO_SLOT_HDR    = 128        # sizeof(YoloSlotHeader)
YOLO_DETS_PAD    = YOLO_MAX_DETS * 16   # 1024 bytes
YOLO_IMG_OFFSET  = YOLO_SLOT_HDR + YOLO_DETS_PAD

# ovcam_shm.hpp constants
OVCAM_MAGIC    = 0x4F564342
OVCAM_GLBL_SZ  = 128  # sizeof(ShmGlobal): two 64-byte cache lines

# ─── Signal handling ─────────────────────────────────────────────
g_stop = False
def _on_signal(signum, frame):
    global g_stop
    g_stop = True
signal.signal(signal.SIGINT, _on_signal)
signal.signal(signal.SIGTERM, _on_signal)

# ─── COCO 80 class names ─────────────────────────────────────────
COCO_NAMES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush",
]

# ═════════════════════════════════════════════════════════════════
# ovcam_shm reader
# ═════════════════════════════════════════════════════════════════
class OvcamReader:
    """Reads NV12 frames from /ovcam_frames shared memory."""

    def __init__(self, shm_name: str = "/ovcam_frames",
                 sem_name: str = "/ovcam_ready"):
        fd = _rt.shm_open(shm_name.encode(), O_RDONLY, 0)
        if fd < 0:
            raise RuntimeError(f"shm_open({shm_name}) failed: errno={ctypes.get_errno()}")

        # Read the ShmGlobal header (128 bytes)
        hdr_buf = os.pread(fd, OVCAM_GLBL_SZ, 0)
        (magic, ver, slot_count, _p0,
         slot_size, max_frame_bytes,
         w, h, stride, fourcc) = struct.unpack_from("<IIIIQQIIII", hdr_buf, 0)

        if magic != OVCAM_MAGIC:
            raise RuntimeError(f"ovcam shm magic mismatch: 0x{magic:08X}")

        self.w, self.h, self.stride = w, h, stride
        self.slot_count = slot_count
        self.slot_size  = slot_size
        self.fourcc     = fourcc

        total = OVCAM_GLBL_SZ + slot_count * slot_size
        self.mm = mmap.mmap(fd, total, mmap.MAP_SHARED, mmap.PROT_READ)
        os.close(fd)

        # Open semaphore
        self.sem = _pt.sem_open(sem_name.encode(), 0, 0, 0)
        if self.sem is None or self.sem == ctypes.c_void_p(-1).value:
            raise RuntimeError(f"sem_open({sem_name}) failed")

        self._last_seq = 0
        print(f"[ovcam] opened: {w}×{h} stride={stride} slots={slot_count}")

    def wait_frame(self, timeout_s: float = 0.5) -> Tuple[np.ndarray, int]:
        """Wait for a new frame. Returns (bgr_image, timestamp_ns) or raises TimeoutError."""
        # Wait on semaphore with timeout
        now = time.clock_gettime(time.CLOCK_REALTIME)
        ts = _Timespec(int(now + timeout_s), int(((now + timeout_s) % 1) * 1e9))
        if _pt.sem_timedwait(self.sem, ctypes.byref(ts)) != 0:
            raise TimeoutError("ovcam semaphore timeout")

        # write_seq sits at offset 64 in ShmGlobal (alignas(64) member)
        ws_buf = self.mm[64:72]
        write_seq = struct.unpack("<Q", ws_buf)[0]

        if write_seq == self._last_seq:
            raise TimeoutError("no new frame")

        # Determine slot index
        idx = ((write_seq - 1) % self.slot_count)
        slot_off = OVCAM_GLBL_SZ + idx * self.slot_size

        # Read seqlock (first 8 bytes of SlotHeader)
        s1 = struct.unpack_from("<Q", self.mm, slot_off)[0]
        if s1 & 1:
            raise TimeoutError("slot being written")

        # Read metadata
        hdr = self.mm[slot_off:slot_off + 64]
        (seq, t_ns, w, h, stride, fourcc, bytes_used) = struct.unpack_from(
            "<QQIIIIi", hdr, 0)

        # Read frame data (full NV12: Y + UV)
        y_bytes  = stride * h
        uv_bytes = stride * (h // 2)
        nv12_bytes = y_bytes + uv_bytes
        data_off = slot_off + 64  # after SlotHeader
        nv12_data = bytes(self.mm[data_off:data_off + nv12_bytes])

        # Validate seqlock
        s2 = struct.unpack_from("<Q", self.mm, slot_off)[0]
        if s2 != s1:
            raise TimeoutError("seqlock torn read")

        self._last_seq = write_seq

        # NV12 → BGR: re-pack to width-stride layout so cvtColor always works
        raw = np.frombuffer(nv12_data, dtype=np.uint8)
        y_plane  = raw[:stride * h].reshape(h, stride)[:, :w]
        uv_plane = raw[stride * h:].reshape(h // 2, stride)[:, :w]
        nv12_packed = np.ascontiguousarray(np.concatenate([y_plane, uv_plane], axis=0))
        bgr = cv2.cvtColor(nv12_packed, cv2.COLOR_YUV2BGR_NV12)

        return bgr, t_ns


# ═════════════════════════════════════════════════════════════════
# yolo_shm writer
# ═════════════════════════════════════════════════════════════════
class YoloShmWriter:
    """Creates and writes to /yolo_shm shared memory."""

    def __init__(self, img_w: int, img_h: int,
                 shm_name: str = "/yolo_shm",
                 sem_name: str = "/yolo_ready",
                 n_slots: int = 4):
        self.img_w    = img_w
        self.img_h    = img_h
        img_stride    = img_w * 3  # BGR
        img_bytes     = img_stride * img_h
        slot_payload  = YOLO_SLOT_HDR + YOLO_DETS_PAD + img_bytes
        slot_size     = (slot_payload + 63) & ~63  # 64-byte align
        total         = 128 + n_slots * slot_size  # YoloShmGlobal = 128 bytes

        self.slot_count = n_slots
        self.slot_size  = slot_size
        self.img_stride = img_stride
        self.img_bytes  = img_bytes

        # Create shm
        _rt.shm_unlink(shm_name.encode())
        fd = _rt.shm_open(shm_name.encode(), O_CREAT | O_RDWR, 0o666)
        if fd < 0:
            raise RuntimeError(f"shm_open({shm_name}) failed")
        os.ftruncate(fd, total)
        self.mm = mmap.mmap(fd, total, mmap.MAP_SHARED,
                            mmap.PROT_READ | mmap.PROT_WRITE)
        os.close(fd)

        # Zero out
        self.mm[:total] = b'\x00' * total

        # Write global header
        hdr = struct.pack("<IIIIQII",
                          YOLO_MAGIC, YOLO_SHM_VERSION, n_slots, 0,
                          slot_size, img_w, img_h)
        self.mm[0:len(hdr)] = hdr
        # write_seq at offset 64 (alignas(64))
        self.mm[64:72] = struct.pack("<Q", 0)

        # Create semaphore
        _pt.sem_unlink(sem_name.encode())
        self.sem = _pt.sem_open(sem_name.encode(), O_CREAT, 0o666, 0)
        if self.sem is None or self.sem == ctypes.c_void_p(-1).value:
            raise RuntimeError(f"sem_open({sem_name}) failed")

        self._frame_num = 0
        print(f"[yolo_shm] created: {img_w}×{img_h} slots={n_slots} "
              f"slot_size={slot_size}")

    def write(self, dets: list, annotated_bgr: np.ndarray, t_ns: int):
        """Write detections + annotated frame to next slot."""
        self._frame_num += 1
        fn = self._frame_num
        idx = (fn - 1) % self.slot_count
        slot_off = 128 + idx * self.slot_size  # after YoloShmGlobal

        det_count = min(len(dets), YOLO_MAX_DETS)

        # Seqlock open (odd)
        self.mm[slot_off:slot_off + 8] = struct.pack("<Q", fn * 2 - 1)

        # Write slot header (skip seq, already written)
        meta = struct.pack("<QIIIIi",
                           t_ns, det_count,
                           self.img_w, self.img_h,
                           self.img_stride, self.img_bytes)
        self.mm[slot_off + 8:slot_off + 8 + len(meta)] = meta

        # Write detections
        det_off = slot_off + YOLO_SLOT_HDR
        for i in range(det_count):
            d = dets[i]
            det_data = struct.pack("<HHfHHHH",
                                   d["class_id"], 0, d["confidence"],
                                   d["x1"], d["y1"], d["x2"], d["y2"])
            self.mm[det_off + i * 16:det_off + i * 16 + 16] = det_data

        # Write annotated image
        img_off = slot_off + YOLO_IMG_OFFSET
        img_flat = annotated_bgr.tobytes()
        self.mm[img_off:img_off + len(img_flat)] = img_flat

        # Seqlock close (even)
        self.mm[slot_off:slot_off + 8] = struct.pack("<Q", fn * 2)

        # Update global write_seq
        self.mm[64:72] = struct.pack("<Q", fn)

        # Wake consumer
        _pt.sem_post(self.sem)


# ═════════════════════════════════════════════════════════════════
# YOLO post-processing
# ═════════════════════════════════════════════════════════════════
def sigmoid(x: np.ndarray) -> np.ndarray:
    x = np.clip(x, -88.0, 88.0)
    return 1.0 / (1.0 + np.exp(-x))


def dfl_decode(bbox_raw: np.ndarray, reg_max: int = 16) -> np.ndarray:
    """
    Decode DFL (Distribution Focal Loss) bounding box predictions.
    bbox_raw: shape (N, 4 * reg_max) or (H, W, 4 * reg_max)
    Returns: shape (N, 4) or (H, W, 4) with decoded ltrb distances.
    """
    orig_shape = bbox_raw.shape
    if len(orig_shape) == 3:
        H, W, C = orig_shape
        bbox_raw = bbox_raw.reshape(-1, C)

    N, C = bbox_raw.shape
    bbox_raw = bbox_raw.reshape(N, 4, reg_max)
    # Softmax over reg_max dimension
    bbox_exp = np.exp(bbox_raw - bbox_raw.max(axis=2, keepdims=True))
    bbox_sm  = bbox_exp / bbox_exp.sum(axis=2, keepdims=True)
    # Expected value: sum(softmax * [0, 1, ..., reg_max-1])
    arange   = np.arange(reg_max, dtype=np.float32)
    decoded  = (bbox_sm * arange).sum(axis=2)  # (N, 4)

    if len(orig_shape) == 3:
        decoded = decoded.reshape(H, W, 4)
    return decoded


def postprocess(outputs: dict, img_h: int, img_w: int,
                input_size: int = 640,
                conf_thresh: float = 0.25,
                iou_thresh: float = 0.45,
                grids: dict = None) -> list:
    """
    Post-process YOLO outputs from 6 tensors (3 scales × bbox+classes).
    Returns list of dicts with keys: class_id, confidence, x1, y1, x2, y2.
    """
    names = sorted(outputs.keys())
    bbox_tensors = []
    cls_tensors  = []
    for name in names:
        t = outputs[name]
        if len(t.shape) == 4:
            t = t[0]
        if len(t.shape) != 3:
            continue
        H, W, C = t.shape
        if C <= 4 or C == 64:
            bbox_tensors.append((name, t))
        elif C == 80:
            cls_tensors.append((name, t))

    bbox_tensors.sort(key=lambda x: -x[1].shape[0] * x[1].shape[1])
    cls_tensors.sort(key=lambda x: -x[1].shape[0] * x[1].shape[1])

    all_boxes  = []
    all_scores = []
    all_classes = []

    # Pre-filter threshold in logit space: sigmoid(x) > t  ⟺  x > log(t/(1-t))
    logit_thresh = math.log(conf_thresh / (1.0 - conf_thresh))
    strides = [8, 16, 32]

    for i, ((bn, bbox_t), (cn, cls_t)) in enumerate(zip(bbox_tensors, cls_tensors)):
        H, W, _ = cls_t.shape
        stride = strides[i] if i < len(strides) else input_size // H

        # Fast pre-filter on raw logits (skip sigmoid on 672K values)
        cls_flat = cls_t.reshape(-1, 80)
        max_logits = cls_flat.max(axis=1)
        mask = max_logits > logit_thresh
        if not mask.any():
            continue

        # Sigmoid + argmax only on passing anchors
        passing_cls = cls_flat[mask]
        passing_scores = sigmoid(passing_cls)
        max_scores  = passing_scores.max(axis=1)
        max_classes = passing_scores.argmax(axis=1)

        # DFL decode only on passing anchors
        bbox_flat = bbox_t.reshape(-1, bbox_t.shape[-1])
        bbox_passing = bbox_flat[mask]
        if bbox_passing.shape[1] == 64:
            ltrb = dfl_decode(bbox_passing, reg_max=16)
        elif bbox_passing.shape[1] == 4:
            ltrb = bbox_passing
        else:
            continue

        # Use pre-computed grids
        if grids and (H, W) in grids:
            cx = grids[(H, W)][0][mask]
            cy = grids[(H, W)][1][mask]
        else:
            gy, gx = np.meshgrid(np.arange(H), np.arange(W), indexing='ij')
            cx = (gx.flatten()[mask] + 0.5) * stride
            cy = (gy.flatten()[mask] + 0.5) * stride

        x1 = cx - ltrb[:, 0] * stride
        y1 = cy - ltrb[:, 1] * stride
        x2 = cx + ltrb[:, 2] * stride
        y2 = cy + ltrb[:, 3] * stride

        all_boxes.append(np.stack([x1, y1, x2, y2], axis=1))
        all_scores.append(max_scores)
        all_classes.append(max_classes)

    if not all_boxes:
        return []

    boxes   = np.concatenate(all_boxes, axis=0)
    scores  = np.concatenate(all_scores, axis=0)
    classes = np.concatenate(all_classes, axis=0)

    # NMS per class
    dets = []
    for cls_id in np.unique(classes):
        cls_mask = (classes == cls_id)
        cls_boxes  = boxes[cls_mask]
        cls_scores = scores[cls_mask]

        order = cls_scores.argsort()[::-1]
        cls_boxes  = cls_boxes[order]
        cls_scores = cls_scores[order]

        keep = _nms(cls_boxes, cls_scores, iou_thresh)
        for k in keep:
            dets.append({
                "class_id": int(cls_id),
                "confidence": float(cls_scores[k]),
                "x1": int(cls_boxes[k, 0]),
                "y1": int(cls_boxes[k, 1]),
                "x2": int(cls_boxes[k, 2]),
                "y2": int(cls_boxes[k, 3]),
            })

    dets.sort(key=lambda d: -d["confidence"])
    return dets[:YOLO_MAX_DETS]


def _nms(boxes: np.ndarray, scores: np.ndarray,
         iou_thresh: float) -> list:
    """Standard greedy NMS."""
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    keep = []
    suppressed = np.zeros(len(scores), dtype=bool)

    for i in range(len(scores)):
        if suppressed[i]:
            continue
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[i+1:])
        yy1 = np.maximum(y1[i], y1[i+1:])
        xx2 = np.minimum(x2[i], x2[i+1:])
        yy2 = np.minimum(y2[i], y2[i+1:])
        w = np.maximum(0, xx2 - xx1)
        h = np.maximum(0, yy2 - yy1)
        inter = w * h
        iou   = inter / (areas[i] + areas[i+1:] - inter + 1e-6)
        suppressed[i+1:] |= (iou > iou_thresh)

    return keep


def draw_annotations(img: np.ndarray, dets: list) -> np.ndarray:
    """Draw bounding boxes and labels on image."""
    out = img.copy()
    for d in dets:
        x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
        cls_id = d["class_id"]
        conf   = d["confidence"]
        label  = COCO_NAMES[cls_id] if cls_id < len(COCO_NAMES) else str(cls_id)

        # Color based on class
        color = (
            int((cls_id * 41) % 256),
            int((cls_id * 97 + 50) % 256),
            int((cls_id * 163 + 100) % 256),
        )
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        txt = f"{label} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
        cv2.putText(out, txt, (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out


# ═════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="YOLO Hailo-10H producer")
    parser.add_argument("--hef", default="/workspace/models/yolo26n_10h.hef",
                        help="Path to HEF model file")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45,
                        help="NMS IoU threshold")
    args = parser.parse_args()

    hef_path = args.hef
    if not os.path.exists(hef_path):
        # Try relative to script dir
        hef_path = os.path.join(os.path.dirname(__file__), "..", "..",
                                "models", "yolo26n_10h.hef")
    if not os.path.exists(hef_path):
        print(f"[error] HEF not found: {args.hef}", file=sys.stderr)
        sys.exit(1)

    print(f"[hailo] loading HEF: {hef_path}")

    # ── Init Hailo ────────────────────────────────────────────────
    vdevice = VDevice()
    infer_model = vdevice.create_infer_model(hef_path)

    # Get input info
    input_info = infer_model.input()
    input_shape = input_info.shape  # (H, W, C) typically (640, 640, 3)
    input_h, input_w = input_shape[0], input_shape[1]
    print(f"[hailo] input: {input_w}×{input_h}×{input_shape[2]}")

    # Set output format to float32 for post-processing
    for out in infer_model.outputs:
        out.set_format_type(FormatType.FLOAT32)

    configured = infer_model.configure()

    # Print output info
    out_names = [o.name for o in infer_model.outputs]
    print(f"[hailo] outputs: {out_names}")

    # ── Open camera shm ──────────────────────────────────────────
    reader = OvcamReader()

    # ── Create YOLO shm ──────────────────────────────────────────
    # Annotated image: same size as camera frame
    writer = YoloShmWriter(reader.w, reader.h)

    print(f"[yolo_producer] running — conf={args.conf} iou={args.iou}")

    # ── Pre-allocate inference buffers (reused every frame) ───────
    bindings = configured.create_bindings()
    input_buf = np.empty((input_h, input_w, 3), dtype=np.uint8)
    bindings.input().set_buffer(input_buf)

    output_buffers = {}
    for out_vstream in infer_model.outputs:
        buf = np.empty(out_vstream.shape, dtype=np.float32)
        bindings.output(out_vstream.name).set_buffer(buf)
        output_buffers[out_vstream.name] = buf

    # Pre-compute anchor grids for all 3 scales
    grids = {}
    for H, W, s in [(80, 80, 8), (40, 40, 16), (20, 20, 32)]:
        gy, gx = np.meshgrid(np.arange(H), np.arange(W), indexing='ij')
        grids[(H, W)] = ((gx.flatten() + 0.5) * s, (gy.flatten() + 0.5) * s)

    # Letterbox parameters (constant — camera and model resolution don't change)
    scale = min(input_w / reader.w, input_h / reader.h)
    new_w = int(reader.w * scale)
    new_h = int(reader.h * scale)
    pad_left = (input_w - new_w) // 2
    pad_top  = (input_h - new_h) // 2
    max_x = reader.w - 1
    max_y = reader.h - 1
    print(f"[yolo_producer] letterbox: {reader.w}x{reader.h} → {new_w}x{new_h} "
          f"pad=({pad_left},{pad_top}) scale={scale:.4f}")

    # ── Main loop ─────────────────────────────────────────────────
    frame_count = 0
    t_start = time.monotonic()
    t_window = t_start

    while not g_stop:
        try:
            bgr, t_ns = reader.wait_frame(timeout_s=1.0)
        except TimeoutError:
            continue

        # Preprocess: letterbox resize (preserve aspect ratio) + BGR→RGB
        input_buf[:] = 114  # YOLO standard gray fill
        resized = cv2.resize(bgr, (new_w, new_h))
        input_buf[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
        cv2.cvtColor(input_buf, cv2.COLOR_BGR2RGB, dst=input_buf)

        # Run inference (reuses pre-allocated bindings + buffers)
        configured.run([bindings], timeout=1000)

        # Post-process
        dets = postprocess(output_buffers, bgr.shape[0], bgr.shape[1],
                           input_size=input_w,
                           conf_thresh=args.conf,
                           iou_thresh=args.iou,
                           grids=grids)

        # Scale boxes from letterboxed model space → source image space
        for d in dets:
            d["x1"] = max(0, min(int((d["x1"] - pad_left) / scale), max_x))
            d["y1"] = max(0, min(int((d["y1"] - pad_top)  / scale), max_y))
            d["x2"] = max(0, min(int((d["x2"] - pad_left) / scale), max_x))
            d["y2"] = max(0, min(int((d["y2"] - pad_top)  / scale), max_y))

        # Write to YOLO shm (raw frame, no annotation drawing)
        writer.write(dets, bgr, t_ns)

        frame_count += 1
        if frame_count % 100 == 0:
            now = time.monotonic()
            window_fps = 100.0 / (now - t_window)
            avg_fps = frame_count / (now - t_start)
            t_window = now
            print(f"[yolo_producer] {frame_count} frames, "
                  f"{window_fps:.1f} fps (avg {avg_fps:.1f}), "
                  f"{len(dets)} dets")

    print(f"\n[yolo_producer] stopped after {frame_count} frames")


if __name__ == "__main__":
    main()
