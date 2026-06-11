#!/usr/bin/env python3
"""
hef_bench — Stand-alone, camera-free benchmark + integration check for HEF models.

Loads a single HEF the *exact* same way yolo_producer.py does (same Hailo setup,
same on-device-NMS auto-detection, same postprocess functions), runs it on a static
image, and reports:
  - whether it loads/configures/runs on the NPU            (functional gate)
  - whether our postprocess path accepts its outputs       (integration gate)
  - per-stage timing: preprocess / NPU inference / postproc (mean, median, p95)
  - end-to-end FPS
  - detections found (count + top classes)                 (sanity / quality)

It imports the real postprocess from src/yolo_producer/yolo_producer.py, so a PASS
here means the model is plug-and-play with the live producer — only the --hef path
(and, for raw-tensor models, nothing else) needs to change.

Usage:
    python3 benchmarks/hef_bench.py --hef models/hef/yolo26n.hef \
        --image /tmp/bus.jpg --iters 100 --warmup 10 \
        --csv benchmarks/results/hef/timing.csv \
        --annotate benchmarks/results/hef/yolo26n.jpg
"""

import argparse
import os
import sys
import time
import traceback

import cv2
import numpy as np

# ── Import the producer's real postprocess code (integration test) ──────────
_THIS = os.path.dirname(os.path.abspath(__file__))
_PRODUCER_DIR = os.path.join(_THIS, "..", "src", "yolo_producer")
sys.path.insert(0, _PRODUCER_DIR)
from yolo_producer import (  # noqa: E402
    postprocess, postprocess_ondevice_nms, draw_annotations, COCO_NAMES,
)

from hailo_platform import VDevice, FormatType  # noqa: E402


def pct(vals, p):
    if not vals:
        return 0.0
    s = sorted(vals)
    k = max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def main():
    ap = argparse.ArgumentParser(description="Camera-free HEF benchmark/integration check")
    ap.add_argument("--hef", required=True)
    ap.add_argument("--image", default="/tmp/bus.jpg",
                    help="Static test image (a real photo gives meaningful detections)")
    ap.add_argument("--iters", type=int, default=100)
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--csv", default=None, help="Append one summary row to this CSV")
    ap.add_argument("--annotate", default=None, help="Save annotated detection image here")
    args = ap.parse_args()

    name = os.path.splitext(os.path.basename(args.hef))[0]
    size_mb = os.path.getsize(args.hef) / 1e6

    # ── Load source image ───────────────────────────────────────────────────
    src_bgr = cv2.imread(args.image)
    if src_bgr is None:
        print(f"[error] cannot read image: {args.image}", file=sys.stderr)
        sys.exit(2)
    src_rgb = cv2.cvtColor(src_bgr, cv2.COLOR_BGR2RGB)  # producer feeds RGB
    src_h, src_w = src_rgb.shape[:2]

    print(f"══ {name}  ({size_mb:.1f} MB) ══════════════════════════════════")

    # ── Hailo setup — mirrors yolo_producer.main() exactly ──────────────────
    vdevice = None
    configured = None
    try:
        vdevice = VDevice()
        infer_model = vdevice.create_infer_model(args.hef)
        in_info = infer_model.input()
        input_h, input_w = in_info.shape[0], in_info.shape[1]

        for out in infer_model.outputs:
            out.set_format_type(FormatType.FLOAT32)
        configured = infer_model.configure()

        out_names = [o.name for o in infer_model.outputs]
        use_ondevice_nms = (len(infer_model.outputs) == 1 and
                            "nms_postprocess" in infer_model.outputs[0].name)
        mode = "on-device-NMS" if use_ondevice_nms else "raw-tensor-CPU-PP"

        bindings = configured.create_bindings()
        input_buf = np.empty((input_h, input_w, 3), dtype=np.uint8)
        bindings.input().set_buffer(input_buf)
        output_buffers = {}
        for ov in infer_model.outputs:
            buf = np.empty(ov.shape, dtype=np.float32)
            bindings.output(ov.name).set_buffer(buf)
            output_buffers[ov.name] = buf
    except Exception as e:
        print(f"  [FAIL] setup/configure: {e}")
        traceback.print_exc()
        _write_csv(args.csv, [name, f"{size_mb:.1f}", "SETUP_FAIL", "", "",
                              "", "", "", "", "", "", ""])
        _release(configured, vdevice)
        sys.exit(1)

    print(f"  input {input_w}x{input_h}  outputs={len(out_names)}  mode={mode}")

    # ── Precompute grids + letterbox (same as producer) ─────────────────────
    grids = {}
    if not use_ondevice_nms:
        for H, W, s in [(80, 80, 8), (40, 40, 16), (20, 20, 32)]:
            gy, gx = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
            grids[(H, W)] = ((gx.flatten() + 0.5) * s, (gy.flatten() + 0.5) * s)

    scale = min(input_w / src_w, input_h / src_h)
    new_w, new_h = int(src_w * scale), int(src_h * scale)
    pad_left = (input_w - new_w) // 2
    pad_top = (input_h - new_h) // 2
    max_x, max_y = src_w - 1, src_h - 1
    resized = cv2.resize(src_rgb, (new_w, new_h))

    # ── Timed loop ──────────────────────────────────────────────────────────
    t_pre, t_inf, t_pp = [], [], []
    last_dets = []
    total = args.warmup + args.iters
    try:
        for i in range(total):
            warm = i < args.warmup

            t0 = time.perf_counter()
            input_buf[:] = 114
            input_buf[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
            t1 = time.perf_counter()

            configured.run([bindings], timeout=2000)
            t2 = time.perf_counter()

            if use_ondevice_nms:
                out_flat = list(output_buffers.values())[0].flatten()
                dets = postprocess_ondevice_nms(
                    out_flat, img_w=src_w, img_h=src_h, input_size=input_w,
                    pad_left=pad_left, pad_top=pad_top, scale=scale,
                    conf_thresh=args.conf)
            else:
                dets = postprocess(output_buffers, src_h, src_w,
                                   input_size=input_w, conf_thresh=args.conf,
                                   iou_thresh=args.iou, grids=grids)
                for d in dets:  # rescale letterbox->source (producer does this)
                    d["x1"] = max(0, min(int((d["x1"] - pad_left) / scale), max_x))
                    d["y1"] = max(0, min(int((d["y1"] - pad_top) / scale), max_y))
                    d["x2"] = max(0, min(int((d["x2"] - pad_left) / scale), max_x))
                    d["y2"] = max(0, min(int((d["y2"] - pad_top) / scale), max_y))
            t3 = time.perf_counter()

            if not warm:
                t_pre.append((t1 - t0) * 1000)
                t_inf.append((t2 - t1) * 1000)
                t_pp.append((t3 - t2) * 1000)
            last_dets = dets
    except Exception as e:
        print(f"  [FAIL] inference/postprocess: {e}")
        traceback.print_exc()
        _write_csv(args.csv, [name, f"{size_mb:.1f}", "RUN_FAIL", mode,
                              str(len(out_names)), "", "", "", "", "", "", ""])
        _release(configured, vdevice)
        sys.exit(1)

    # ── Stats ───────────────────────────────────────────────────────────────
    pre_m = np.mean(t_pre); inf_m = np.mean(t_inf); pp_m = np.mean(t_pp)
    e2e = pre_m + inf_m + pp_m
    fps = 1000.0 / e2e if e2e > 0 else 0.0

    # detection summary
    by_cls = {}
    for d in last_dets:
        cn = COCO_NAMES[d["class_id"]] if d["class_id"] < len(COCO_NAMES) else str(d["class_id"])
        by_cls[cn] = by_cls.get(cn, 0) + 1
    det_summary = ", ".join(f"{k}×{v}" for k, v in
                            sorted(by_cls.items(), key=lambda x: -x[1])) or "(none)"

    print(f"  preprocess  {pre_m:6.2f} ms   (p50 {pct(t_pre,50):.2f}  p95 {pct(t_pre,95):.2f})")
    print(f"  NPU infer   {inf_m:6.2f} ms   (p50 {pct(t_inf,50):.2f}  p95 {pct(t_inf,95):.2f})")
    print(f"  postprocess {pp_m:6.2f} ms   (p50 {pct(t_pp,50):.2f}  p95 {pct(t_pp,95):.2f})")
    print(f"  END-TO-END  {e2e:6.2f} ms  →  {fps:5.1f} FPS")
    print(f"  detections  {len(last_dets)}: {det_summary}")
    print(f"  [PASS] loads, runs, postprocess OK")

    if args.annotate:
        os.makedirs(os.path.dirname(args.annotate), exist_ok=True)
        ann = draw_annotations(src_rgb, last_dets)
        cv2.imwrite(args.annotate, cv2.cvtColor(ann, cv2.COLOR_RGB2BGR))
        print(f"  annotated → {args.annotate}")

    _write_csv(args.csv, [
        name, f"{size_mb:.1f}", "PASS", mode, str(len(out_names)),
        f"{pre_m:.3f}", f"{inf_m:.3f}", f"{pp_m:.3f}", f"{e2e:.3f}",
        f"{fps:.2f}", str(len(last_dets)), det_summary])

    # Clean release — leaking the VDevice/CIM at exit can wedge the NPU SoC.
    _release(configured, vdevice)


def _release(configured, vdevice):
    """Cleanly release Hailo resources. Skipping this leaves the CIM server in a
    bad state ('Lost communication with the server ... VDevice released while CIM
    in use') and repeated leaks crash the HAILO10H SoC off the PCIe bus."""
    try:
        if configured is not None:
            configured.shutdown()
    except Exception:
        pass
    try:
        if vdevice is not None:
            vdevice.release()
    except Exception:
        pass


def _write_csv(path, row):
    if not path:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    new = not os.path.exists(path)
    import csv
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["model", "size_mb", "status", "pp_mode", "n_outputs",
                        "preprocess_ms", "inference_ms", "postprocess_ms",
                        "e2e_ms", "fps", "n_dets", "detections"])
        w.writerow(row)


if __name__ == "__main__":
    main()
