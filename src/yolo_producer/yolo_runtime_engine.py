#!/usr/bin/env python3
"""
yolo_runtime_engine — Plug-and-play YOLO predictor core (OOP).

A single, backend-agnostic interface (`BaseYoloPredictor`) with two concrete
implementations:

    ONNXYoloPredictor       — ARM CPU via onnxruntime (bounded threads, vectorized NMS)
    HailoNPUYoloPredictor   — Hailo-10H NPU via pyHailoRT (reuses the proven
                              yolo_producer.postprocess / postprocess_ondevice_nms)

The caller is completely oblivious to which backend runs underneath:

    from yolo_runtime_engine import build_predictor
    pred = build_predictor("cpu", model_id="yolov8n", threads=2)   # or "npu"
    dets = pred.infer(rgb_image)          # -> [(x1,y1,x2,y2,class_id,conf), ...]

Every model selection, backend toggle and thread count is a runtime parameter,
so a single shell script can sweep the whole experimental matrix on one branch.

`infer(image, timing=dict)` optionally fills `timing` with absolute CLOCK_MONOTONIC
nanosecond stamps ('pre_done','inf_start','inf_done','nms_done'), which the SHM
producer turns into the paper's per-stage latency breakdown.
"""

from __future__ import annotations

import json
import os
import sys
import time
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# Canonical detection: bbox corners (source pixels) + class id + confidence.
Detection = Tuple[float, float, float, float, int, float]

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# repo/models/model_registry.json  (this file lives in repo/src/yolo_producer/)
_DEFAULT_REGISTRY = os.path.normpath(
    os.path.join(_THIS_DIR, "..", "..", "models", "model_registry.json"))

# COCO-80 labels. Kept local so importing the CPU engine never pulls in
# hailo_platform (which yolo_producer.py imports at module scope). Mirrors the
# list in yolo_producer.py.
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
    "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


def _now_ns() -> int:
    """Monotonic nanosecond clock.

    Deliberately CLOCK_MONOTONIC (not perf_counter): the C++ camera driver stamps
    frames with clock_gettime(CLOCK_MONOTONIC), so sharing the same epoch makes the
    cross-process ts_capture -> ts_yolo_shm_read interval physically meaningful.
    """
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -88.0, 88.0)))


# ═════════════════════════════════════════════════════════════════════════════
# Model registry — the ONLY place that knows the on-disk naming quirks
# ═════════════════════════════════════════════════════════════════════════════
class ModelRegistry:
    """Resolves a logical model id (e.g. 'yolov8n') to absolute artifact paths
    plus metadata. Absorbs the cross-family filename inconsistencies
    (yolov11n.hef vs yolo11n.onnx, missing nms_config files, ...)."""

    # Aliases accepted from CLI/launch params -> canonical family token.
    _FAMILY_ALIASES = {
        "v8": "v8", "yolov8": "v8", "8": "v8",
        "v11": "v11", "yolov11": "v11", "yolo11": "v11", "11": "v11",
        "yolo26": "yolo26", "v26": "yolo26", "26": "yolo26",
    }

    def __init__(self, path: Optional[str] = None):
        self.path = path or _DEFAULT_REGISTRY
        with open(self.path, "r") as f:
            self._doc = json.load(f)
        self.root = os.path.dirname(os.path.abspath(self.path))
        self._defaults = self._doc.get("defaults", {})
        self._models = self._doc.get("models", {})

    # -- id helpers --------------------------------------------------------
    @classmethod
    def id_from(cls, family: str, variant: str) -> str:
        """('v8','n') / ('yolo11','s') / ('26','m') -> canonical id."""
        fam = cls._FAMILY_ALIASES.get(str(family).lower())
        if fam is None:
            raise ValueError(f"unknown model_family '{family}' "
                             f"(expected one of {sorted(set(cls._FAMILY_ALIASES))})")
        var = str(variant).lower()
        if var not in ("n", "s", "m", "l", "x"):
            raise ValueError(f"unknown variant '{variant}' (expected n/s/m)")
        # canonical ids: yolov8{v} / yolov11{v} / yolo26{v}
        return {"v8": "yolov8", "v11": "yolov11", "yolo26": "yolo26"}[fam] + var

    def list_ids(self) -> List[str]:
        return list(self._models.keys())

    # -- resolution --------------------------------------------------------
    def _abs(self, rel: Optional[str]) -> Optional[str]:
        return os.path.normpath(os.path.join(self.root, rel)) if rel else None

    def resolve(self, model_id: str) -> Dict:
        """Return a meta dict with absolute paths + merged defaults."""
        if model_id not in self._models:
            raise KeyError(
                f"model id '{model_id}' not in registry. "
                f"Available: {', '.join(self.list_ids())}")
        m = dict(self._models[model_id])           # shallow copy
        meta = dict(self._defaults)                # start from defaults
        meta.update(m)                             # per-model overrides win
        meta["id"] = model_id
        meta["hef"] = self._abs(m.get("hef"))
        meta["onnx"] = self._abs(m.get("onnx"))
        meta["nms_config"] = self._abs(m.get("nms_config"))
        return meta

    def load_nms_config(self, meta: Dict) -> Dict:
        """Load the optional per-model *_nms_config.json sidecar (if present)."""
        p = meta.get("nms_config")
        if p and os.path.exists(p):
            try:
                with open(p, "r") as f:
                    return json.load(f)
            except Exception:
                pass
        return {}


# ═════════════════════════════════════════════════════════════════════════════
# Shared preprocessing — one letterbox path for every backend / layout
# ═════════════════════════════════════════════════════════════════════════════
def letterbox(img_rgb: np.ndarray, size: int,
              pad_value: int = 114) -> Tuple[np.ndarray, float, int, int]:
    """Aspect-preserving resize + center pad to (size, size).

    Returns (canvas_uint8_RGB, scale, pad_left, pad_top). Identical math to the
    live producer so CPU/NPU detections land in the same coordinate frame.
    """
    src_h, src_w = img_rgb.shape[:2]
    scale = min(size / src_w, size / src_h)
    new_w, new_h = int(round(src_w * scale)), int(round(src_h * scale))
    resized = cv2.resize(img_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), pad_value, dtype=np.uint8)
    pad_left = (size - new_w) // 2
    pad_top = (size - new_h) // 2
    canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
    return canvas, scale, pad_left, pad_top


# ═════════════════════════════════════════════════════════════════════════════
# Abstract base — uniform interface; callers never branch on backend
# ═════════════════════════════════════════════════════════════════════════════
class BaseYoloPredictor(ABC):
    """Uniform predictor interface.

    Subclasses isolate ALL backend-specific behaviour (tensor shapes, output
    indexing, runtime init, threading) behind two methods so the caller node /
    benchmarker stays completely backend-agnostic.
    """

    def __init__(self, model_path: str,
                 input_size: int = 640,
                 num_classes: int = 80,
                 conf: float = 0.25,
                 iou: float = 0.45,
                 class_names: Optional[List[str]] = None,
                 **_opts):
        self.model_path = model_path
        self.input_size = int(input_size)
        self.num_classes = int(num_classes)
        self.conf = float(conf)
        self.iou = float(iou)
        self.class_names = class_names or COCO_NAMES
        self.load_model(model_path)

    # -- interface ---------------------------------------------------------
    @abstractmethod
    def load_model(self, path: str) -> None:
        """Initialize the backend runtime and bind I/O for `path`."""

    @abstractmethod
    def infer(self, image_mat: np.ndarray,
              timing: Optional[Dict] = None) -> List[Detection]:
        """RGB HxWx3 uint8 -> [(x1,y1,x2,y2,class_id,conf), ...] in source pixels."""

    # -- shared helpers ----------------------------------------------------
    def class_name(self, class_id: int) -> str:
        return (self.class_names[class_id]
                if 0 <= class_id < len(self.class_names) else str(class_id))

    def _rescale_xyxy(self, x1, y1, x2, y2,
                      scale: float, pad_left: int, pad_top: int,
                      src_w: int, src_h: int):
        """Map boxes from letterboxed input space back to source pixels (vectorized)."""
        x1 = (x1 - pad_left) / scale
        y1 = (y1 - pad_top) / scale
        x2 = (x2 - pad_left) / scale
        y2 = (y2 - pad_top) / scale
        np.clip(x1, 0, src_w - 1, out=x1)
        np.clip(x2, 0, src_w - 1, out=x2)
        np.clip(y1, 0, src_h - 1, out=y1)
        np.clip(y2, 0, src_h - 1, out=y2)
        return x1, y1, x2, y2

    def close(self) -> None:
        """Release backend resources (override where needed)."""
        return None

    # context-manager sugar so callers can `with build_predictor(...) as p:`
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ═════════════════════════════════════════════════════════════════════════════
# ONNX Runtime backend (ARM CPU)
# ═════════════════════════════════════════════════════════════════════════════
class ONNXYoloPredictor(BaseYoloPredictor):
    """CPU inference via onnxruntime with EXPLICIT thread bounding.

    Threading management (critical for the contention experiments):
      onnxruntime defaults to an intra-op pool spanning ALL cores and a spin-wait
      that busy-burns idle threads. On a 4-core Pi5 that tramples the OV2SLAM
      pinned to cores 2-3. We therefore set intra/inter op thread counts from
      explicit params (never the defaults) and disable thread spinning, so that
      `taskset`-pinning YOLO to N cores actually means it uses N cores.
    """

    def __init__(self, model_path: str,
                 intra_threads: int = 2,
                 inter_threads: int = 1,
                 onnx_layout: Optional[str] = None,
                 **kw):
        self.intra_threads = int(intra_threads)
        self.inter_threads = int(inter_threads)
        self.layout_hint = onnx_layout       # 'v8_anchors' | 'yolo26_e2e' | None
        super().__init__(model_path, **kw)

    def load_model(self, path: str) -> None:
        import onnxruntime as ort            # lazy: CPU-only hosts may lack hailo, and vice-versa

        so = ort.SessionOptions()
        so.intra_op_num_threads = self.intra_threads      # within-op (conv/GEMM) parallelism
        so.inter_op_num_threads = self.inter_threads      # across-node; keep 1 for serial graphs
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # Stop ORT worker threads from busy-spinning between inferences (frees the
        # cores for SLAM instead of pinning them at 100%). Guarded: older ORT may
        # not know the key.
        try:
            so.add_session_config_entry("session.intra_op.allow_spinning", "0")
        except Exception:
            pass

        self.sess = ort.InferenceSession(
            path, sess_options=so, providers=["CPUExecutionProvider"])
        self.input_name = self.sess.get_inputs()[0].name
        self.output_names = [o.name for o in self.sess.get_outputs()]

        # Prefer the model's own static input H/W when available, else registry size.
        ishape = self.sess.get_inputs()[0].shape       # e.g. [1, 3, 640, 640]
        try:
            h, w = int(ishape[2]), int(ishape[3])
            if h > 0 and w > 0:
                self.input_size = max(h, w)
        except (TypeError, ValueError, IndexError):
            pass

        out_shapes = [o.shape for o in self.sess.get_outputs()]
        print(f"[onnx] {os.path.basename(path)} in='{self.input_name}'{ishape} "
              f"outputs={self.output_names} shapes={out_shapes} "
              f"threads(intra={self.intra_threads},inter={self.inter_threads})")

    # -- preprocessing -----------------------------------------------------
    def _blob(self, image_rgb: np.ndarray):
        """RGB HxWx3 uint8 -> (NCHW float32 0..1, scale, pad_left, pad_top)."""
        canvas, scale, pad_left, pad_top = letterbox(image_rgb, self.input_size)
        blob = canvas.astype(np.float32, copy=False) * (1.0 / 255.0)
        blob = np.transpose(blob, (2, 0, 1))[None]          # HWC -> NCHW, add batch
        return np.ascontiguousarray(blob), scale, pad_left, pad_top

    # -- output-layout decision (tensor indexing) --------------------------
    def _decide_layout(self, arr: np.ndarray) -> str:
        """Identify the output tensor layout from its shape.

        v8/v11 export a single decoded tensor [1, 4+nc, A] (or transposed) where
        A (~8400) >> channels. YOLO26 is end-to-end and emits [1, N, 6]
        (x1,y1,x2,y2,conf,cls). We key off the channel dim, falling back to the
        registry hint when ambiguous.
        """
        a = arr[0] if arr.ndim == 3 else arr
        ch = 4 + self.num_classes                # 84 for COCO
        if ch in a.shape:
            return "anchors"
        if 6 in a.shape and ch not in a.shape:
            return "e2e"
        if self.layout_hint == "yolo26_e2e":
            return "e2e"
        return "anchors"

    def infer(self, image_mat: np.ndarray,
              timing: Optional[Dict] = None) -> List[Detection]:
        t = timing if timing is not None else {}
        src_h, src_w = image_mat.shape[:2]

        blob, scale, pad_left, pad_top = self._blob(image_mat)
        t["pre_done"] = _now_ns()

        t["inf_start"] = _now_ns()
        outputs = self.sess.run(self.output_names, {self.input_name: blob})
        t["inf_done"] = _now_ns()

        arr = np.asarray(outputs[0])
        if self._decide_layout(arr) == "e2e":
            dets = self._postprocess_e2e(arr, scale, pad_left, pad_top, src_w, src_h)
        else:
            dets = self._postprocess_anchors(arr, scale, pad_left, pad_top, src_w, src_h)
        t["nms_done"] = _now_ns()
        return dets

    # -- Layer 4: unified vectorized post-processing -----------------------
    def _postprocess_anchors(self, raw, scale, pad_left, pad_top,
                             src_w, src_h) -> List[Detection]:
        """v8/v11 decoded-anchor head. Fully vectorized NumPy slicing + the
        C++-backed cv2.dnn.NMSBoxes — no per-box Python loops on the hot path."""
        pred = raw[0] if raw.ndim == 3 else raw
        ch = 4 + self.num_classes
        # Normalize to shape (A, 4+nc): channels must be the LAST axis.
        if pred.shape[0] == ch and pred.shape[1] != ch:
            pred = pred.T
        elif ch not in pred.shape and pred.shape[0] < pred.shape[1]:
            pred = pred.T                                   # ambiguous: smaller dim = channels

        boxes = pred[:, :4]                                 # cx, cy, w, h (input px)
        scores = pred[:, 4:4 + self.num_classes]
        # Ultralytics already applies sigmoid in-graph (scores in [0,1]); guard
        # the rare case of raw logits being exported.
        if scores.size and float(scores.max()) > 1.0 + 1e-3:
            scores = _sigmoid(scores)

        class_ids = scores.argmax(axis=1)
        confs = scores.max(axis=1)
        keep = confs >= self.conf                           # vectorized confidence gate
        if not np.any(keep):
            return []
        boxes = boxes[keep]
        class_ids = class_ids[keep]
        confs = confs[keep]

        # cx,cy,w,h -> x1,y1,x2,y2 in input space (vectorized)
        cx, cy, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = cx - w * 0.5
        y1 = cy - h * 0.5
        x2 = cx + w * 0.5
        y2 = cy + h * 0.5
        x1, y1, x2, y2 = self._rescale_xyxy(x1, y1, x2, y2, scale,
                                            pad_left, pad_top, src_w, src_h)

        # xywh in SOURCE space for cv2.dnn.NMSBoxes (expects [x, y, w, h]).
        xywh = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1)

        # Class-aware NMS: loop only over the (few) classes actually present;
        # each call is a single C++-backed NMSBoxes over a vectorized batch.
        dets: List[Detection] = []
        for c in np.unique(class_ids):
            m = class_ids == c
            idx = cv2.dnn.NMSBoxes(xywh[m].tolist(), confs[m].tolist(),
                                   self.conf, self.iou)
            if len(idx) == 0:
                continue
            cls_boxes = xywh[m]
            cls_conf = confs[m]
            for k in np.asarray(idx).flatten():
                bx, by, bw, bh = cls_boxes[k]
                dets.append((float(bx), float(by), float(bx + bw), float(by + bh),
                             int(c), float(cls_conf[k])))
        dets.sort(key=lambda d: -d[5])
        return dets

    def _postprocess_e2e(self, raw, scale, pad_left, pad_top,
                         src_w, src_h) -> List[Detection]:
        """YOLO26 NMS-free head: output already top-k [N,6] = x1,y1,x2,y2,conf,cls.
        Only confidence-threshold + rescale — NMS already done in-graph."""
        pred = raw[0] if raw.ndim == 3 else raw
        if pred.ndim != 2:
            return []
        if pred.shape[0] == 6 and pred.shape[1] != 6:
            pred = pred.T                                   # -> (N, 6)
        if pred.shape[1] < 6 or pred.size == 0:
            return []

        coords = pred[:, :4].astype(np.float32)
        confs = pred[:, 4].astype(np.float32)
        class_ids = pred[:, 5].astype(np.int32)
        # Some exports emit normalized [0,1] coords; scale up to input pixels.
        if coords.size and float(coords.max()) <= 1.5:
            coords = coords * float(self.input_size)

        keep = confs >= self.conf
        coords, confs, class_ids = coords[keep], confs[keep], class_ids[keep]
        if coords.shape[0] == 0:
            return []

        x1, y1, x2, y2 = self._rescale_xyxy(
            coords[:, 0].copy(), coords[:, 1].copy(),
            coords[:, 2].copy(), coords[:, 3].copy(),
            scale, pad_left, pad_top, src_w, src_h)

        dets = [(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]),
                 int(class_ids[i]), float(confs[i])) for i in range(len(confs))]
        dets.sort(key=lambda d: -d[5])
        return dets


# ═════════════════════════════════════════════════════════════════════════════
# Hailo-10H NPU backend (pyHailoRT)
# ═════════════════════════════════════════════════════════════════════════════
class HailoNPUYoloPredictor(BaseYoloPredictor):
    """NPU inference via pyHailoRT.

    Reuses the verified post-processing already shipping in yolo_producer.py
    (DFL raw-tensor path + on-device-NMS parser) so NPU results are bit-identical
    to the production HIL pipeline — only the HEF path changes per model.
    """

    def __init__(self, model_path: str,
                 hef_postproc: Optional[str] = None,
                 **kw):
        self.hef_postproc = hef_postproc     # 'ondevice_nms' | 'raw_tensor' | None
        self._vdevice = None
        self._configured = None
        super().__init__(model_path, **kw)

    def load_model(self, path: str) -> None:
        from hailo_platform import VDevice, FormatType         # lazy import

        # Reuse the production post-processing. Import lazily AND from this dir so
        # CPU-only hosts (no hailo_platform) never trip over it.
        if _THIS_DIR not in sys.path:
            sys.path.insert(0, _THIS_DIR)
        from yolo_producer import postprocess, postprocess_ondevice_nms
        self._pp = postprocess
        self._pp_ondevice = postprocess_ondevice_nms

        self._vdevice = VDevice()
        infer_model = self._vdevice.create_infer_model(path)

        in_info = infer_model.input()
        self.input_h, self.input_w = in_info.shape[0], in_info.shape[1]
        self.input_size = self.input_w

        for out in infer_model.outputs:
            out.set_format_type(FormatType.FLOAT32)          # Hailo upconverts INT8 -> f32
        self._configured = infer_model.configure()

        # Auto-detect on-device NMS exactly like yolo_producer / hef_bench: single
        # output whose name carries 'nms_postprocess'. Registry hint is the fallback.
        outs = infer_model.outputs
        self.use_ondevice_nms = (len(outs) == 1 and "nms_postprocess" in outs[0].name)
        if not self.use_ondevice_nms and self.hef_postproc == "ondevice_nms":
            self.use_ondevice_nms = True

        # Pre-allocate + bind I/O buffers once; reused every frame (zero realloc).
        self._bindings = self._configured.create_bindings()
        self._input_buf = np.empty((self.input_h, self.input_w, 3), dtype=np.uint8)
        self._bindings.input().set_buffer(self._input_buf)
        self._output_buffers = {}
        for ov in outs:
            buf = np.empty(ov.shape, dtype=np.float32)
            self._bindings.output(ov.name).set_buffer(buf)
            self._output_buffers[ov.name] = buf

        # Anchor grids for the raw-tensor (DFL) path (3 scales: stride 8/16/32).
        self._grids = {}
        if not self.use_ondevice_nms:
            for H, W, s in [(80, 80, 8), (40, 40, 16), (20, 20, 32)]:
                gy, gx = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
                self._grids[(H, W)] = ((gx.flatten() + 0.5) * s,
                                       (gy.flatten() + 0.5) * s)

        mode = "on-device-NMS" if self.use_ondevice_nms else "raw-tensor-CPU-PP"
        print(f"[hailo] {os.path.basename(path)} in={self.input_w}x{self.input_h} "
              f"outputs={len(outs)} mode={mode}")

    def infer(self, image_mat: np.ndarray,
              timing: Optional[Dict] = None) -> List[Detection]:
        t = timing if timing is not None else {}
        src_h, src_w = image_mat.shape[:2]

        # Letterbox directly into the bound input buffer (NPU wants uint8 HWC).
        scale = min(self.input_w / src_w, self.input_h / src_h)
        new_w, new_h = int(round(src_w * scale)), int(round(src_h * scale))
        pad_left = (self.input_w - new_w) // 2
        pad_top = (self.input_h - new_h) // 2
        self._input_buf[:] = 114
        self._input_buf[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = \
            cv2.resize(image_mat, (new_w, new_h))
        t["pre_done"] = _now_ns()

        t["inf_start"] = _now_ns()
        self._configured.run([self._bindings], timeout=2000)
        t["inf_done"] = _now_ns()

        if self.use_ondevice_nms:
            # on-device path already rescales to source pixels.
            out_flat = list(self._output_buffers.values())[0].flatten()
            dd = self._pp_ondevice(
                out_flat, img_w=src_w, img_h=src_h, input_size=self.input_w,
                pad_left=pad_left, pad_top=pad_top, scale=scale, conf_thresh=self.conf)
        else:
            # raw-tensor path returns boxes in letterboxed model space -> rescale.
            dd = self._pp(self._output_buffers, src_h, src_w,
                          input_size=self.input_w, conf_thresh=self.conf,
                          iou_thresh=self.iou, grids=self._grids)
            for d in dd:
                d["x1"] = max(0, min(int((d["x1"] - pad_left) / scale), src_w - 1))
                d["y1"] = max(0, min(int((d["y1"] - pad_top) / scale), src_h - 1))
                d["x2"] = max(0, min(int((d["x2"] - pad_left) / scale), src_w - 1))
                d["y2"] = max(0, min(int((d["y2"] - pad_top) / scale), src_h - 1))
        t["nms_done"] = _now_ns()

        # dict -> canonical tuple (caller is layout/backend blind)
        return [(float(d["x1"]), float(d["y1"]), float(d["x2"]), float(d["y2"]),
                 int(d["class_id"]), float(d["confidence"])) for d in dd]

    def close(self) -> None:
        """Clean release. Leaking the VDevice/CIM wedges the Hailo SoC off PCIe."""
        try:
            if self._configured is not None:
                self._configured.shutdown()
        except Exception:
            pass
        try:
            if self._vdevice is not None:
                self._vdevice.release()
        except Exception:
            pass
        self._configured = None
        self._vdevice = None


# ═════════════════════════════════════════════════════════════════════════════
# Factory — the caller's single entry point
# ═════════════════════════════════════════════════════════════════════════════
_CPU_BACKENDS = {"cpu", "onnx", "ort", "onnxruntime"}
_NPU_BACKENDS = {"npu", "hailo", "hef", "hailort"}


def build_predictor(backend: str,
                    model_id: Optional[str] = None,
                    model_path: Optional[str] = None,
                    registry: Optional[ModelRegistry] = None,
                    conf: Optional[float] = None,
                    iou: Optional[float] = None,
                    threads: Optional[int] = None,
                    inter_threads: Optional[int] = None,
                    input_size: Optional[int] = None,
                    num_classes: Optional[int] = None,
                    **extra) -> BaseYoloPredictor:
    """Construct a predictor for `backend` ('cpu'|'npu').

    Resolve the model either by registry `model_id` or by an explicit
    `model_path`. All model-specific paths/params are swallowed here so callers
    stay oblivious to the backend.
    """
    backend = backend.lower()
    meta: Dict = {}
    if model_id:
        reg = registry or ModelRegistry()
        meta = reg.resolve(model_id)

    common = dict(
        input_size=input_size if input_size is not None else meta.get("input_size", 640),
        num_classes=num_classes if num_classes is not None else meta.get("classes", 80),
        conf=conf if conf is not None else meta.get("conf", 0.25),
        iou=iou if iou is not None else meta.get("iou", 0.45),
    )

    if backend in _CPU_BACKENDS:
        path = model_path or meta.get("onnx")
        if not path:
            raise ValueError("CPU backend needs --model_path or a registry id with an onnx artifact")
        if not os.path.exists(path):
            raise FileNotFoundError(f"onnx model not found: {path}")
        return ONNXYoloPredictor(
            path,
            intra_threads=threads if threads is not None else 2,
            inter_threads=inter_threads if inter_threads is not None else 1,
            onnx_layout=meta.get("onnx_layout"),
            **common)

    if backend in _NPU_BACKENDS:
        path = model_path or meta.get("hef")
        if not path:
            raise ValueError("NPU backend needs --model_path or a registry id with a hef artifact")
        if not os.path.exists(path):
            raise FileNotFoundError(f"hef model not found: {path}")
        return HailoNPUYoloPredictor(path, hef_postproc=meta.get("hef_postproc"), **common)

    raise ValueError(f"unknown backend '{backend}' (expected cpu|npu)")


if __name__ == "__main__":
    # Tiny smoke probe: `python3 yolo_runtime_engine.py cpu yolov8n /tmp/bus.jpg`
    import argparse
    ap = argparse.ArgumentParser(description="engine smoke probe")
    ap.add_argument("backend", choices=["cpu", "npu"])
    ap.add_argument("model_id")
    ap.add_argument("image", nargs="?", default="/tmp/bus.jpg")
    a = ap.parse_args()

    bgr = cv2.imread(a.image)
    if bgr is None:
        sys.exit(f"cannot read image: {a.image}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    pred = build_predictor(a.backend, model_id=a.model_id)
    tdict: Dict = {}
    dets = pred.infer(rgb, timing=tdict)
    print(f"{len(dets)} detections:")
    for x1, y1, x2, y2, cid, cf in dets[:10]:
        print(f"  {pred.class_name(cid):14s} {cf:.2f}  [{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]")
    if {"inf_start", "inf_done"} <= tdict.keys():
        print(f"inference: {(tdict['inf_done'] - tdict['inf_start']) / 1e6:.2f} ms")
    pred.close()
