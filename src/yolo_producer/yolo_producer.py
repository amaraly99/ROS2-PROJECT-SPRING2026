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
        self._stride_packed = (self.stride == self.w)
        print(f"[ovcam] opened: {w}×{h} stride={stride} slots={slot_count}")

    def wait_frame(self, timeout_s: float = 0.5) -> Tuple[np.ndarray, int]:
        """Wait for a new frame. Returns (rgb_image, timestamp_ns) or raises TimeoutError."""
        # Poll write_seq instead of consuming the shared /ovcam_ready semaphore.
        # That semaphore is ALSO drained by ovcam_bridge, and each sem_post wakes
        # only ONE waiter — so with two consumers the wakeups split ~50/50 and our
        # frame rate halves whenever ovcam_bridge runs (i.e. SLAM enabled).
        # write_seq (offset 64) is the SHM's authoritative "latest frame" marker,
        # so polling it lets every consumer observe every frame independently with
        # no wakeup stealing. yolo_producer owns its core, so a 1 ms poll is
        # negligible CPU and adds <=1 ms latency at 20 Hz.
        deadline = time.monotonic() + timeout_s
        while True:
            write_seq = struct.unpack("<Q", self.mm[64:72])[0]
            if write_seq != self._last_seq:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("ovcam poll timeout")
            time.sleep(0.001)

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

        # Single efficient copy from mmap (replaces bytes() + frombuffer)
        raw = np.frombuffer(self.mm, dtype=np.uint8,
                            count=nv12_bytes, offset=data_off).copy()

        # Validate seqlock
        s2 = struct.unpack_from("<Q", self.mm, slot_off)[0]
        if s2 != s1:
            raise TimeoutError("seqlock torn read")

        self._last_seq = write_seq

        # NV12 → RGB directly (skip BGR intermediate)
        if self._stride_packed:
            # Fast path: stride == width, NV12 data already contiguous
            nv12_packed = raw.reshape(h * 3 // 2, w)
        else:
            # Slow path: strip stride padding
            y_plane  = raw[:stride * h].reshape(h, stride)[:, :w]
            uv_plane = raw[stride * h:].reshape(h // 2, stride)[:, :w]
            nv12_packed = np.ascontiguousarray(
                np.concatenate([y_plane, uv_plane], axis=0))

        rgb = cv2.cvtColor(nv12_packed, cv2.COLOR_YUV2RGB_NV12)
        return rgb, t_ns


# ═════════════════════════════════════════════════════════════════
# yolo_shm writer
# ═════════════════════════════════════════════════════════════════
class YoloShmWriter:
    """Creates and writes to /yolo_shm shared memory."""

    def __init__(self, img_w: int, img_h: int,
                 shm_name: str = "/yolo_shm",
                 sem_name: str = "/yolo_ready",
                 n_slots: int = 4,
                 include_image: bool = True):
        self.img_w    = img_w
        self.img_h    = img_h
        self.include_image = include_image
        img_stride    = img_w * 3  # RGB
        img_bytes     = img_stride * img_h if include_image else 0
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

    def write(self, dets: list, img, t_ns: int):
        """Write detections (+ optional image) to next slot."""
        self._frame_num += 1
        fn = self._frame_num
        idx = (fn - 1) % self.slot_count
        slot_off = 128 + idx * self.slot_size  # after YoloShmGlobal

        det_count = min(len(dets), YOLO_MAX_DETS)

        # Seqlock open (odd)
        self.mm[slot_off:slot_off + 8] = struct.pack("<Q", fn * 2 - 1)

        # Write slot header — img_bytes=0 when image not included
        write_img = self.include_image and img is not None
        meta = struct.pack("<QIIIIi",
                           t_ns, det_count,
                           self.img_w, self.img_h,
                           self.img_stride,
                           self.img_bytes if write_img else 0)
        self.mm[slot_off + 8:slot_off + 8 + len(meta)] = meta

        # Write detections
        det_off = slot_off + YOLO_SLOT_HDR
        for i in range(det_count):
            d = dets[i]
            det_data = struct.pack("<HHfHHHH",
                                   d["class_id"], 0, d["confidence"],
                                   d["x1"], d["y1"], d["x2"], d["y2"])
            self.mm[det_off + i * 16:det_off + i * 16 + 16] = det_data

        # Write image only when included (saves ~1.8MB/frame when skipped)
        if write_img:
            img_off = slot_off + YOLO_IMG_OFFSET
            img_flat = img.tobytes()
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
    clip_val = 11.0 if x.dtype == np.float16 else 88.0
    x = np.clip(x, -clip_val, clip_val)
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
                grids: dict = None,
                timing: dict = None,
                max_dets: int = YOLO_MAX_DETS,
                float_coords: bool = False) -> list:
    """
    Post-process YOLO outputs from 6 tensors (3 scales × bbox+classes).
    Returns list of dicts with keys: class_id, confidence, x1, y1, x2, y2.

    max_dets / float_coords exist for OFFLINE ACCURACY MEASUREMENT ONLY and
    default to the deployed behaviour, so the live pipeline is unaffected:

      max_dets     the robot keeps 64 (YOLO_MAX_DETS is an SHM ABI constant --
                   YOLO_DETS_PAD = 64*16, mirrored in include/yolo_shm.hpp).
                   COCO evaluates at maxDets=100, so a benchmark must be able
                   to ask for more or it silently truncates the tail of the
                   precision/recall curve.
      float_coords the robot wants integer pixels. int() TRUNCATES, biasing
                   every box inward by ~0.5 px, which depresses mAP75/mAP50-95
                   and confounds NPU-vs-CPU comparisons because only the NPU
                   paths do it. Pass True to keep sub-pixel boxes.
    If timing dict is provided, records per-stage durations (ms).
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

    t_decode_start = time.monotonic()
    t_prefilter = 0
    t_sigmoid = 0
    t_bbox_extract = 0
    t_grid_lookup = 0
    t_coord_rescale = 0

    for i, ((bn, bbox_t), (cn, cls_t)) in enumerate(zip(bbox_tensors, cls_tensors)):
        H, W, _ = cls_t.shape
        stride = strides[i] if i < len(strides) else input_size // H

        # Fast pre-filter on raw logits (skip sigmoid on 672K values)
        _t0 = time.perf_counter_ns()
        cls_flat = cls_t.reshape(-1, 80)
        max_logits = cls_flat.max(axis=1)
        mask = max_logits > logit_thresh
        _t1 = time.perf_counter_ns()
        t_prefilter += (_t1 - _t0)
        if not mask.any():
            continue

        # Sigmoid + argmax only on passing anchors
        _t0 = time.perf_counter_ns()
        passing_cls = cls_flat[mask]
        passing_scores = sigmoid(passing_cls)
        max_scores  = passing_scores.max(axis=1)
        max_classes = passing_scores.argmax(axis=1)
        _t1 = time.perf_counter_ns()
        t_sigmoid += (_t1 - _t0)

        # DFL decode only on passing anchors
        _t0 = time.perf_counter_ns()
        bbox_flat = bbox_t.reshape(-1, bbox_t.shape[-1])
        bbox_passing = bbox_flat[mask]
        if bbox_passing.shape[1] == 64:
            ltrb = dfl_decode(bbox_passing, reg_max=16)
        elif bbox_passing.shape[1] == 4:
            ltrb = bbox_passing
        else:
            t_bbox_extract += (time.perf_counter_ns() - _t0)
            continue
        _t1 = time.perf_counter_ns()
        t_bbox_extract += (_t1 - _t0)

        # Use pre-computed grids
        _t0 = time.perf_counter_ns()
        if grids and (H, W) in grids:
            cx = grids[(H, W)][0][mask]
            cy = grids[(H, W)][1][mask]
        else:
            gy, gx = np.meshgrid(np.arange(H), np.arange(W), indexing='ij')
            cx = (gx.flatten()[mask] + 0.5) * stride
            cy = (gy.flatten()[mask] + 0.5) * stride
        _t1 = time.perf_counter_ns()
        t_grid_lookup += (_t1 - _t0)

        _t0 = time.perf_counter_ns()
        x1 = cx - ltrb[:, 0] * stride
        y1 = cy - ltrb[:, 1] * stride
        x2 = cx + ltrb[:, 2] * stride
        y2 = cy + ltrb[:, 3] * stride

        all_boxes.append(np.stack([x1, y1, x2, y2], axis=1))
        all_scores.append(max_scores)
        all_classes.append(max_classes)
        _t1 = time.perf_counter_ns()
        t_coord_rescale += (_t1 - _t0)

    t_decode_end = time.monotonic()

    if not all_boxes:
        if timing is not None:
            timing["decode_ms"] = (t_decode_end - t_decode_start) * 1000.0
            timing["nms_ms"] = 0.0
            timing["total_ms"] = timing["decode_ms"]
            timing["prefilter_ms"] = t_prefilter / 1e6
            timing["sigmoid_ms"] = t_sigmoid / 1e6
            timing["bbox_extract_ms"] = t_bbox_extract / 1e6
            timing["grid_lookup_ms"] = t_grid_lookup / 1e6
            timing["coord_rescale_ms"] = t_coord_rescale / 1e6
        return []

    boxes   = np.concatenate(all_boxes, axis=0)
    scores  = np.concatenate(all_scores, axis=0)
    classes = np.concatenate(all_classes, axis=0)

    # NMS per class using OpenCV's C++-backed NMSBoxes
    t_nms_start = time.monotonic()
    dets = []
    for cls_id in np.unique(classes):
        cls_mask = (classes == cls_id)
        cls_boxes  = boxes[cls_mask]
        cls_scores = scores[cls_mask]

        # cv2.dnn.NMSBoxes expects [x, y, w, h] format and list of floats
        xywh = np.empty_like(cls_boxes)
        xywh[:, 0] = cls_boxes[:, 0]                           # x
        xywh[:, 1] = cls_boxes[:, 1]                           # y
        xywh[:, 2] = cls_boxes[:, 2] - cls_boxes[:, 0]         # w
        xywh[:, 3] = cls_boxes[:, 3] - cls_boxes[:, 1]         # h

        indices = cv2.dnn.NMSBoxes(
            xywh.tolist(), cls_scores.tolist(),
            conf_thresh, iou_thresh)

        if len(indices) > 0:
            # indices may be [[i]] or [i] depending on OpenCV version
            keep = indices.flatten()
            _c = float if float_coords else int
            for k in keep:
                dets.append({
                    "class_id": int(cls_id),
                    "confidence": float(cls_scores[k]),
                    "x1": _c(cls_boxes[k, 0]),
                    "y1": _c(cls_boxes[k, 1]),
                    "x2": _c(cls_boxes[k, 2]),
                    "y2": _c(cls_boxes[k, 3]),
                })

    t_nms_end = time.monotonic()

    if timing is not None:
        timing["decode_ms"] = (t_decode_end - t_decode_start) * 1000.0
        timing["nms_ms"] = (t_nms_end - t_nms_start) * 1000.0
        timing["total_ms"] = (t_nms_end - t_decode_start) * 1000.0
        timing["prefilter_ms"] = t_prefilter / 1e6
        timing["sigmoid_ms"] = t_sigmoid / 1e6
        timing["bbox_extract_ms"] = t_bbox_extract / 1e6
        timing["grid_lookup_ms"] = t_grid_lookup / 1e6
        timing["coord_rescale_ms"] = t_coord_rescale / 1e6

    dets.sort(key=lambda d: -d["confidence"])
    return dets[:max_dets]


def postprocess_ondevice_nms(output_flat: np.ndarray,
                             img_w: int, img_h: int,
                             input_size: int = 640,
                             pad_left: int = 0, pad_top: int = 0,
                             scale: float = 1.0,
                             num_classes: int = 80,
                             max_per_class: int = 100,
                             conf_thresh: float = 0.25,
                             timing: dict = None,
                             max_dets: int = YOLO_MAX_DETS,
                             float_coords: bool = False) -> list:
    """
    Parse on-device NMS output from Hailo (yolov8/yolov11, HAILO NMS BY CLASS).

    The output is VARIABLE-LENGTH, packed back-to-back per class:
      [count_c][count_c × {y_min, x_min, y_max, x_max, score}] for c = 0..79,
    boxes normalized to [0, 1] relative to the 640×640 input (incl. letterbox).
    The buffer is only zero-padded up to num_classes*(1+max_per_class*5) floats.

    NOTE: this parser previously assumed FIXED 501-float strides per class,
    which coincides with the real layout only for class 0 (person) — every
    other class read a padding zero as its count and was silently dropped.
    That bug masqueraded as "v8/v11 HEFs only detect persons" (old Finding 9).
    """
    t0 = time.perf_counter_ns()
    dets = []

    off = 0
    total = output_flat.shape[0]
    for cls_id in range(num_classes):
        if off >= total:
            break
        count = int(output_flat[off])
        off += 1
        if count == 0:
            continue
        count = min(count, max_per_class)
        for j in range(count):
            y_min, x_min, y_max, x_max, score = output_flat[off + j * 5:
                                                            off + j * 5 + 5]
            if score < conf_thresh:
                continue
            # Convert from normalized [0,1] in 640×640 space → source image pixels
            px1 = (x_min * input_size - pad_left) / scale
            py1 = (y_min * input_size - pad_top) / scale
            px2 = (x_max * input_size - pad_left) / scale
            py2 = (y_max * input_size - pad_top) / scale
            if float_coords:
                dets.append({
                    "class_id": cls_id,
                    "confidence": float(score),
                    "x1": max(0.0, min(float(px1), float(img_w - 1))),
                    "y1": max(0.0, min(float(py1), float(img_h - 1))),
                    "x2": max(0.0, min(float(px2), float(img_w - 1))),
                    "y2": max(0.0, min(float(py2), float(img_h - 1))),
                })
            else:
                dets.append({
                    "class_id": cls_id,
                    "confidence": float(score),
                    "x1": max(0, min(int(px1), img_w - 1)),
                    "y1": max(0, min(int(py1), img_h - 1)),
                    "x2": max(0, min(int(px2), img_w - 1)),
                    "y2": max(0, min(int(py2), img_h - 1)),
                })
        off += count * 5

    t1 = time.perf_counter_ns()
    if timing is not None:
        timing["decode_ms"] = 0.0
        timing["nms_ms"] = 0.0
        timing["total_ms"] = (t1 - t0) / 1e6
        timing["prefilter_ms"] = 0.0
        timing["sigmoid_ms"] = 0.0
        timing["bbox_extract_ms"] = 0.0
        timing["grid_lookup_ms"] = 0.0
        timing["coord_rescale_ms"] = (t1 - t0) / 1e6  # all time is parse+rescale

    dets.sort(key=lambda d: -d["confidence"])
    return dets[:max_dets]


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
    parser.add_argument("--fp16", action="store_true",
                        help="Use float16 for post-processing (precision experiment)")
    parser.add_argument("--no-image", action="store_true",
                        help="Skip writing image to YOLO SHM (saves ~1.8MB/frame)")
    parser.add_argument("--ondevice-nms", action="store_true",
                        help="Use on-device NMS postprocessing (for yolov8m/yolov11m HEFs)")
    parser.add_argument("--csv", type=str, default=None,
                        help="Write per-frame timing CSV to this path")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="Stop after N frames (0 = unlimited)")
    parser.add_argument("--downsample", type=float, default=1.0,
                        help="Downsample factor for resolution experiment (e.g. 2.0 = half res)")
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

    # Set output format for post-processing
    out_dtype = np.float16 if args.fp16 else np.float32
    for out in infer_model.outputs:
        out.set_format_type(FormatType.FLOAT32)  # Hailo always outputs float32
    if args.fp16:
        print("[yolo_producer] float16 post-processing enabled")

    configured = infer_model.configure()

    # Print output info and detect on-device NMS
    out_names = [o.name for o in infer_model.outputs]
    print(f"[hailo] outputs: {out_names}")

    # Auto-detect on-device NMS: single output with "nms_postprocess" in name
    use_ondevice_nms = args.ondevice_nms
    if not use_ondevice_nms and len(infer_model.outputs) == 1:
        if "nms_postprocess" in infer_model.outputs[0].name:
            use_ondevice_nms = True
            print("[hailo] auto-detected on-device NMS output")
    if use_ondevice_nms:
        print(f"[yolo_producer] on-device NMS mode — CPU postprocess skipped")

    # ── Open camera shm ──────────────────────────────────────────
    reader = OvcamReader()

    # ── Create YOLO shm ──────────────────────────────────────────
    # Annotated image: same size as camera frame
    writer = YoloShmWriter(reader.w, reader.h,
                           include_image=not args.no_image)

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

    # Pre-compute anchor grids for all 3 scales (not needed for on-device NMS)
    grids = {}
    if not use_ondevice_nms:
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

    # ── CSV writer (optional) ─────────────────────────────────────
    csv_file = None
    csv_writer = None
    if args.csv:
        import csv
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["frame", "preprocess_ms", "inference_ms",
                             "prefilter_ms", "sigmoid_ms", "bbox_extract_ms",
                             "grid_lookup_ms", "coord_rescale_ms",
                             "decode_ms", "nms_ms", "total_pp_ms", "n_dets"])

    if args.downsample > 1.0:
        print(f"[yolo_producer] downsample={args.downsample:.2f}x "
              f"(effective {int(reader.w/args.downsample)}x{int(reader.h/args.downsample)})")

    # ── Main loop ─────────────────────────────────────────────────
    frame_count = 0
    t_start = time.monotonic()
    t_window = t_start

    # Timing accumulators for benchmarking
    pp_timing = {}
    pp_timing_acc = {"preprocess_ms": 0.0, "inference_ms": 0.0,
                     "decode_ms": 0.0, "nms_ms": 0.0, "postprocess_ms": 0.0,
                     "prefilter_ms": 0.0, "sigmoid_ms": 0.0,
                     "bbox_extract_ms": 0.0, "grid_lookup_ms": 0.0,
                     "coord_rescale_ms": 0.0}
    pp_timing_count = 0

    while not g_stop:
        try:
            rgb, t_ns = reader.wait_frame(timeout_s=1.0)
        except TimeoutError:
            continue

        # Resolution experiment: downsample then upsample to simulate lower-res camera
        if args.downsample > 1.0:
            dh = int(rgb.shape[0] / args.downsample)
            dw = int(rgb.shape[1] / args.downsample)
            rgb = cv2.resize(cv2.resize(rgb, (dw, dh)), (rgb.shape[1], rgb.shape[0]))

        # Preprocess: letterbox resize (already RGB from wait_frame)
        t_pre = time.monotonic()
        input_buf[:] = 114  # YOLO standard gray fill
        resized = cv2.resize(rgb, (new_w, new_h))
        input_buf[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
        t_pre_end = time.monotonic()

        # Run inference (reuses pre-allocated bindings + buffers)
        t_infer = time.monotonic()
        configured.run([bindings], timeout=1000)
        t_infer_end = time.monotonic()

        # Post-process
        pp_timing.clear()
        if use_ondevice_nms:
            # On-device NMS: single flat output, parse directly
            out_tensor = list(output_buffers.values())[0].flatten()
            dets = postprocess_ondevice_nms(
                out_tensor, img_w=reader.w, img_h=reader.h,
                input_size=input_w,
                pad_left=pad_left, pad_top=pad_top, scale=scale,
                conf_thresh=args.conf, timing=pp_timing)
        else:
            # Standard: 6 raw tensors, full CPU postprocess
            if args.fp16:
                pp_inputs = {k: v.astype(np.float16) for k, v in output_buffers.items()}
            else:
                pp_inputs = output_buffers
            dets = postprocess(pp_inputs, rgb.shape[0], rgb.shape[1],
                               input_size=input_w,
                               conf_thresh=args.conf,
                               iou_thresh=args.iou,
                               grids=grids,
                               timing=pp_timing)

        # Accumulate timing
        pp_timing_count += 1
        pp_timing_acc["preprocess_ms"] += (t_pre_end - t_pre) * 1000.0
        pp_timing_acc["inference_ms"] += (t_infer_end - t_infer) * 1000.0
        pp_timing_acc["decode_ms"] += pp_timing.get("decode_ms", 0.0)
        pp_timing_acc["nms_ms"] += pp_timing.get("nms_ms", 0.0)
        pp_timing_acc["postprocess_ms"] += pp_timing.get("total_ms", 0.0)
        pp_timing_acc["prefilter_ms"] += pp_timing.get("prefilter_ms", 0.0)
        pp_timing_acc["sigmoid_ms"] += pp_timing.get("sigmoid_ms", 0.0)
        pp_timing_acc["bbox_extract_ms"] += pp_timing.get("bbox_extract_ms", 0.0)
        pp_timing_acc["grid_lookup_ms"] += pp_timing.get("grid_lookup_ms", 0.0)
        pp_timing_acc["coord_rescale_ms"] += pp_timing.get("coord_rescale_ms", 0.0)

        # Write per-frame CSV row
        if csv_writer is not None:
            csv_writer.writerow([
                frame_count + 1,
                f"{(t_pre_end - t_pre) * 1000.0:.4f}",
                f"{(t_infer_end - t_infer) * 1000.0:.4f}",
                f"{pp_timing.get('prefilter_ms', 0.0):.4f}",
                f"{pp_timing.get('sigmoid_ms', 0.0):.4f}",
                f"{pp_timing.get('bbox_extract_ms', 0.0):.4f}",
                f"{pp_timing.get('grid_lookup_ms', 0.0):.4f}",
                f"{pp_timing.get('coord_rescale_ms', 0.0):.4f}",
                f"{pp_timing.get('decode_ms', 0.0):.4f}",
                f"{pp_timing.get('nms_ms', 0.0):.4f}",
                f"{pp_timing.get('total_ms', 0.0):.4f}",
                len(dets),
            ])

        # Scale boxes from letterboxed model space → source image space
        # (on-device NMS already does this in postprocess_ondevice_nms)
        if not use_ondevice_nms:
            for d in dets:
                d["x1"] = max(0, min(int((d["x1"] - pad_left) / scale), max_x))
                d["y1"] = max(0, min(int((d["y1"] - pad_top)  / scale), max_y))
                d["x2"] = max(0, min(int((d["x2"] - pad_left) / scale), max_x))
                d["y2"] = max(0, min(int((d["y2"] - pad_top)  / scale), max_y))

        # Write to YOLO shm (detections always, image only if --no-image not set)
        writer.write(dets, rgb if not args.no_image else None, t_ns)

        frame_count += 1
        if frame_count % 100 == 0:
            now = time.monotonic()
            window_fps = 100.0 / (now - t_window)
            avg_fps = frame_count / (now - t_start)
            t_window = now
            n = pp_timing_count if pp_timing_count > 0 else 1
            print(f"[yolo_producer] {frame_count} frames, "
                  f"{window_fps:.1f} fps (avg {avg_fps:.1f}), "
                  f"{len(dets)} dets | "
                  f"pre={pp_timing_acc['preprocess_ms']/n:.1f}ms "
                  f"infer={pp_timing_acc['inference_ms']/n:.1f}ms "
                  f"decode={pp_timing_acc['decode_ms']/n:.2f}ms "
                  f"[prefilt={pp_timing_acc['prefilter_ms']/n:.2f} "
                  f"sig={pp_timing_acc['sigmoid_ms']/n:.2f} "
                  f"bbox={pp_timing_acc['bbox_extract_ms']/n:.2f} "
                  f"grid={pp_timing_acc['grid_lookup_ms']/n:.2f} "
                  f"coord={pp_timing_acc['coord_rescale_ms']/n:.2f}] "
                  f"nms={pp_timing_acc['nms_ms']/n:.2f}ms "
                  f"pp_total={pp_timing_acc['postprocess_ms']/n:.2f}ms")

        # Stop after max-frames if set
        if args.max_frames > 0 and frame_count >= args.max_frames:
            print(f"[yolo_producer] reached --max-frames={args.max_frames}")
            break

    if csv_file is not None:
        csv_file.close()
        print(f"[yolo_producer] CSV written to {args.csv}")

    print(f"\n[yolo_producer] stopped after {frame_count} frames")


if __name__ == "__main__":
    main()
