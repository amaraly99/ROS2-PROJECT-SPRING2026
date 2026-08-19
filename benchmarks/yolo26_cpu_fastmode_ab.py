#!/usr/bin/env python3
"""
yolo26_cpu_fastmode_ab.py — Brief A. Quantify what YOLO26's NMS-free head saves
on the ARM CPU, by A/B-ing the two possible export shapes through one identical
ONNX Runtime path.

WHY THIS EXISTS
---------------
Dr. Hassan asked that YOLO26 be run in its NMS-free ("fast") mode on the CPU.
Reading the repo (branch exp/detector-benchmarks) shows the CPU path ALREADY
does this:

  models/model_registry.json      -> yolo26n/s/m have  "onnx_layout": "yolo26_e2e"
  src/yolo_producer/yolo_runtime_engine.py
      _decide_layout()   -> returns "e2e" for [1,N,6]
      _postprocess_e2e() -> "NMS-free head: output already top-k [N,6] ...
                             Only confidence-threshold + rescale -- NMS already
                             done in-graph."

So fast mode is ON. What the paper does NOT yet have is a MEASUREMENT of what
that buys on the CPU. That is what this script produces.

THE TWO ARMS
------------
  A "e2e"      native End2End export -> single (1,300,6) output.
                Host does: confidence threshold + rescale.   [what we deploy]
  B "one2many" conventional export   -> 6 raw conv tensors, 8400 anchors.
                Host does: sigmoid over 80x8400 + decode + cv2.dnn.NMSBoxes.
                This is the conventional decode+NMS path YOLO26 was designed
                to delete. It is the counterfactual, NOT a bug to fix.

Both arms run through the SAME onnxruntime settings the deployed engine uses
(intra_op=2, inter_op=1, allow_spinning=0), so the delta is attributable to the
head/postprocess and not to runtime configuration.

WHAT IT NEEDS
-------------
Raspberry Pi only. No Hailo NPU, no camera, no ROS 2, no MATLAB, no supervisor.
Synthetic frames by default; pass --image for a real photo.

    pip install ultralytics onnxruntime onnx opencv-python

USAGE
-----
    # simplest - exports both graphs from yolo26n.pt, benchmarks both
    python3 yolo26_cpu_fastmode_ab.py --weights /path/to/yolo26n.pt

    # use the ACTUAL deployed onnx for arm A (preferred - matches the paper)
    python3 yolo26_cpu_fastmode_ab.py --weights models/yolo26n.pt \
        --deployed-onnx models/onnx/yolo26n/yolo26n.onnx

    # all three scales
    for m in n s m; do
      python3 yolo26_cpu_fastmode_ab.py --weights models/yolo26${m}.pt --tag yolo26${m}
    done

OUTPUT
------
    yolo26_fastmode_ab_<tag>_<host>_<stamp>.csv   <- send this back
"""

import argparse
import csv
import os
import platform
import socket
import sys
import time
from datetime import datetime

import numpy as np

IMGSZ = 640
CONF = 0.20        # deployed operating point (hil_servo_params.yaml min_confidence, registry conf)
IOU = 0.45


def log(m):
    print(m, flush=True)


# ─────────────────────────────────────────────────────────── exports ─────────

def export_e2e(weights, out_dir, tag):
    """Native End2End export: single (1,300,6). This is the deployed shape."""
    from ultralytics import YOLO

    m = YOLO(weights)
    p = m.export(format="onnx", imgsz=IMGSZ, simplify=False, dynamic=False, opset=17)
    dst = os.path.join(out_dir, f"{tag}_e2e.onnx")
    if os.path.abspath(p) != os.path.abspath(dst):
        if os.path.exists(dst):
            os.remove(dst)
        os.replace(p, dst)
    return dst


def export_one2many(weights, out_dir, tag):
    """Conventional export: 6 raw conv tensors (box+cls per level), 8400 anchors.

    NOTE: we deliberately do NOT call fuse(). On YOLO26 fuse() deletes the
    one2many head (cv2/cv3 -> None), leaving only the NMS-free branch, which
    would defeat the whole point of this arm.
    """
    import torch
    from ultralytics import YOLO

    pt = YOLO(weights).model.eval()
    head = pt.model[-1]
    if not hasattr(head, "cv2") or head.cv2 is None:
        raise RuntimeError("one2many head unavailable on this checkpoint")

    class Cut(torch.nn.Module):
        def __init__(self, pt):
            super().__init__()
            self.mods, self.save = pt.model, pt.save

        def forward(self, im):
            x, y = im, []
            for mod in self.mods[:-1]:
                if mod.f != -1:
                    x = y[mod.f] if isinstance(mod.f, int) else \
                        [x if j == -1 else y[j] for j in mod.f]
                x = mod(x)
                y.append(x if mod.i in self.save else None)
            h = self.mods[-1]
            feats = [y[j] if j != -1 else x for j in h.f]
            outs = []
            for i in range(h.nl):
                outs.append(h.cv2[i](feats[i]))
                outs.append(h.cv3[i](feats[i]))
            return outs

    dst = os.path.join(out_dir, f"{tag}_one2many.onnx")
    names = []
    for i in range(head.nl):
        names += [f"box{i}", f"cls{i}"]
    with torch.no_grad():
        torch.onnx.export(Cut(pt).eval(), torch.zeros(1, 3, IMGSZ, IMGSZ), dst,
                          opset_version=17, input_names=["images"],
                          output_names=names, dynamo=False)
    return dst


# ─────────────────────────────────────────────────── runtime + postprocess ───

def make_session(path, intra, inter):
    """Mirror yolo_runtime_engine.ONNXYoloPredictor session configuration."""
    import onnxruntime as ort

    so = ort.SessionOptions()
    so.intra_op_num_threads = intra
    so.inter_op_num_threads = inter
    so.add_session_config_entry("session.intra_op.allow_spinning", "0")
    return ort.InferenceSession(path, sess_options=so,
                                providers=["CPUExecutionProvider"])


def post_e2e(outs, conf=CONF):
    """Mirror _postprocess_e2e: threshold + (rescale). No NMS."""
    pred = outs[0]
    pred = pred[0] if pred.ndim == 3 else pred
    if pred.ndim != 2:
        return 0
    if pred.shape[0] == 6 and pred.shape[1] != 6:
        pred = pred.T
    if pred.shape[1] < 6 or pred.size == 0:
        return 0
    coords = pred[:, :4].astype(np.float32)
    confs = pred[:, 4].astype(np.float32)
    if coords.size and float(coords.max()) <= 1.5:
        coords = coords * float(IMGSZ)
    keep = confs >= conf
    coords = coords[keep]
    # rescale would happen here; it is O(n_kept<=300) and negligible
    return int(coords.shape[0])


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x, dtype=np.float32))


def post_one2many(outs, conf=CONF, iou=IOU, nc=80):
    """Mirror the conventional host path: sigmoid over the dense class map,
    decode boxes, then class-aware cv2.dnn.NMSBoxes -- i.e. exactly the work
    YOLO26's one-to-one head exists to delete."""
    import cv2

    boxes_l, scores_l = [], []
    for a in outs:
        a = np.squeeze(a)
        if a.ndim != 3:
            continue
        c, h, w = a.shape
        flat = a.reshape(c, h * w)
        if c == nc:
            scores_l.append(flat)
        else:
            boxes_l.append(flat)
    if not scores_l or not boxes_l:
        return 0

    scores = _sigmoid(np.concatenate(scores_l, axis=1))      # (80, A)
    raw_b = np.concatenate(boxes_l, axis=1)                  # (4*reg, A)

    cls_ids = scores.argmax(0)
    cls_conf = scores.max(0)
    keep = cls_conf >= conf
    if not np.any(keep):
        return 0

    # reg_max==1 on YOLO26 -> the 4 channels are already ltrb distances
    b = raw_b[:4, keep].T.astype(np.float32)
    cc = cls_conf[keep].astype(np.float32)
    ci = cls_ids[keep]

    # xywh-ish boxes for NMS purposes (absolute scale is irrelevant to the cost)
    rects = np.stack([b[:, 0], b[:, 1],
                      np.abs(b[:, 2]) + 1.0, np.abs(b[:, 3]) + 1.0], axis=1)

    total = 0
    for c in np.unique(ci):
        m = ci == c
        idx = cv2.dnn.NMSBoxes(rects[m].tolist(), cc[m].tolist(), conf, iou)
        total += len(idx) if idx is not None else 0
    return int(total)


# ─────────────────────────────────────────────────────────── benchmark ───────

def run(sess, arm, frames, warmup, intra, inter, image=None):
    inp = sess.get_inputs()[0].name
    outs_meta = [(o.name, o.shape) for o in sess.get_outputs()]
    n_vals = 0
    for _, s in outs_meta:
        v = 1
        for d in s:
            v *= d if isinstance(d, int) else 1
        n_vals += v

    if image is not None:
        import cv2

        img = cv2.imread(image)
        if img is None:
            raise FileNotFoundError(image)
        img = cv2.resize(img, (IMGSZ, IMGSZ))
        pool = [img]
    else:
        rng = np.random.default_rng(0)
        pool = [rng.integers(0, 256, (IMGSZ, IMGSZ, 3), dtype=np.uint8)
                for _ in range(8)]

    post = post_e2e if arm == "e2e" else post_one2many

    def one(frame):
        t0 = time.perf_counter()
        x = frame.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        x = np.ascontiguousarray(x)
        t1 = time.perf_counter()
        o = sess.run(None, {inp: x})
        t2 = time.perf_counter()
        n = post(o)
        t3 = time.perf_counter()
        return (t1 - t0) * 1e3, (t2 - t1) * 1e3, (t3 - t2) * 1e3, (t3 - t0) * 1e3, n

    for i in range(warmup):
        one(pool[i % len(pool)])

    pre, inf, po, tot, nd = [], [], [], [], []
    for i in range(frames):
        a, b, c, d, n = one(pool[i % len(pool)])
        pre.append(a); inf.append(b); po.append(c); tot.append(d); nd.append(n)

    r = dict(
        arm=arm,
        conf=CONF,
        outputs=str(outs_meta),
        n_output_values=n_vals,
        frames=frames,
        intra_op=intra,
        inter_op=inter,
        preprocess_ms=round(float(np.mean(pre)), 3),
        inference_ms=round(float(np.mean(inf)), 3),
        postprocess_ms=round(float(np.mean(po)), 3),
        postprocess_ms_p95=round(float(np.percentile(po, 95)), 3),
        total_ms=round(float(np.mean(tot)), 3),
        total_ms_std=round(float(np.std(tot)), 3),
        fps=round(1000.0 / float(np.mean(tot)), 2),
        mean_dets=round(float(np.mean(nd)), 2),
    )
    log(f"  [{arm:9s}] outputs={len(outs_meta)}  values/frame={n_vals:,}")
    log(f"  [{arm:9s}] pre={r['preprocess_ms']:.2f}  inf={r['inference_ms']:.2f}  "
        f"post={r['postprocess_ms']:.3f}  total={r['total_ms']:.2f} ms  "
        f"({r['fps']:.2f} fps)  dets={r['mean_dets']}")
    return r


def main():
    ap = argparse.ArgumentParser(description="YOLO26 CPU fast-mode A/B (Brief A)")
    ap.add_argument("--weights", required=True, help="path to yolo26N.pt")
    ap.add_argument("--deployed-onnx", default="",
                    help="use the ACTUAL deployed .onnx for the e2e arm "
                         "(preferred: models/onnx/yolo26n/yolo26n.onnx)")
    ap.add_argument("--tag", default="")
    ap.add_argument("--frames", type=int, default=50)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--threads", type=int, default=2, help="intra_op (engine default 2)")
    ap.add_argument("--inter-threads", type=int, default=1)
    ap.add_argument("--image", default="", help="optional real image instead of synthetic")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--skip-one2many", action="store_true")
    args = ap.parse_args()

    tag = args.tag or os.path.splitext(os.path.basename(args.weights))[0]
    os.makedirs(args.out_dir, exist_ok=True)

    log("=" * 72)
    log("  Brief A - YOLO26 CPU fast-mode A/B")
    log("=" * 72)
    log(f"  host       : {socket.gethostname()}")
    log(f"  platform   : {platform.platform()}  {platform.machine()}")
    log(f"  python     : {sys.version.split()[0]}")
    try:
        import ultralytics, onnxruntime as ort
        log(f"  ultralytics: {ultralytics.__version__}")
        log(f"  onnxruntime: {ort.__version__}")
    except Exception as e:
        log(f"  ERROR importing deps: {e}")
        return 2
    log(f"  threads    : intra={args.threads} inter={args.inter_threads}")
    log(f"  frames     : {args.frames} (warmup {args.warmup})")
    log(f"  source     : {args.image or 'synthetic random frames'}")

    rows = []

    # ---- arm A: e2e -------------------------------------------------------
    log("\n-- arm A: e2e (NMS-free, deployed) --")
    if args.deployed_onnx:
        e2e_path = args.deployed_onnx
        log(f"  using deployed onnx: {e2e_path}")
        if not os.path.exists(e2e_path):
            log("  !! not found; falling back to exporting from .pt")
            e2e_path = export_e2e(args.weights, args.out_dir, tag)
    else:
        e2e_path = export_e2e(args.weights, args.out_dir, tag)
        log(f"  exported: {e2e_path}")
    try:
        r = run(make_session(e2e_path, args.threads, args.inter_threads),
                "e2e", args.frames, args.warmup, args.threads,
                args.inter_threads, args.image or None)
        r["model"] = tag
        r["onnx"] = os.path.basename(e2e_path)
        rows.append(r)
    except Exception:
        import traceback
        traceback.print_exc()

    # ---- arm B: one2many --------------------------------------------------
    if not args.skip_one2many:
        log("\n-- arm B: one2many (conventional decode + NMS, counterfactual) --")
        try:
            o2m = export_one2many(args.weights, args.out_dir, tag)
            log(f"  exported: {o2m}")
            r = run(make_session(o2m, args.threads, args.inter_threads),
                    "one2many", args.frames, args.warmup, args.threads,
                    args.inter_threads, args.image or None)
            r["model"] = tag
            r["onnx"] = os.path.basename(o2m)
            rows.append(r)
        except Exception:
            import traceback
            traceback.print_exc()
            log("  !! one2many arm failed - send the traceback, the e2e arm is "
                "still usable on its own")

    if not rows:
        return 1

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(args.out_dir,
                       f"yolo26_fastmode_ab_{tag}_{socket.gethostname()}_{stamp}.csv")
    fields = ["model", "arm", "onnx", "outputs", "n_output_values", "frames",
              "conf", "intra_op", "inter_op", "preprocess_ms", "inference_ms",
              "postprocess_ms", "postprocess_ms_p95", "total_ms", "total_ms_std",
              "fps", "mean_dets"]
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    log("\n" + "=" * 72)
    log("  RESULT")
    log("=" * 72)
    by = {r["arm"]: r for r in rows}
    if "e2e" in by and "one2many" in by:
        a, b = by["e2e"], by["one2many"]
        log(f"  host postprocess : {a['postprocess_ms']:.3f} ms (e2e)  vs  "
            f"{b['postprocess_ms']:.3f} ms (one2many)")
        log(f"  output volume    : {a['n_output_values']:,} vs {b['n_output_values']:,} "
            f"values/frame")
        log(f"  total/frame      : {a['total_ms']:.2f} ms vs {b['total_ms']:.2f} ms")
        if b["total_ms"] > 0:
            log(f"  NMS-free head saves {b['total_ms'] - a['total_ms']:.2f} ms/frame "
                f"on this CPU")
    log(f"\n  CSV: {out}")
    log("  Send back: this CSV + the full console output above.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
