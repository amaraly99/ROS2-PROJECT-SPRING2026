#!/usr/bin/env python3
"""
coco_accuracy_bench.py — Brief C. Standalone detector ACCURACY (COCO mAP) and
LATENCY measured in the SAME pass, through the deployed inference path.

WHY BOTH IN ONE PASS
--------------------
The professor asked for accuracy AND latency, standalone AND in-pipeline. Measuring
them in one pass guarantees the two correspond to the same run, the same thresholds
and the same preprocessing — you cannot accidentally quote an mAP from one
configuration against a latency from another.

WHAT IT MEASURES
----------------
  mAP50-95, mAP50, mAP75      (COCO primary metrics, all 80 classes)
  AP for 'stop sign'          (COCO category 13 = YOLO class 11) -- the class the
                              mission actually depends on
  preprocess/inference/postprocess/total ms, and fps

PREFERRED: uses the repo's own engine
-------------------------------------
By default this imports `build_predictor` from src/yolo_producer/yolo_runtime_engine.py
(branch exp/detector-benchmarks) so the numbers describe the ACTUAL deployed pipeline
— same letterboxing, same layout auto-detection, same postprocess — for both the
Hailo NPU (HEF) and the ARM CPU (ONNX Runtime). Point --repo-root at your checkout.

Fallback `--standalone-onnx` runs a plain onnxruntime session instead. Use it only
for smoke-testing on a laptop; do not quote its numbers in the paper.

*** THE CONFIDENCE-THRESHOLD ISSUE — READ THIS ***
Published mAP is computed at a very low threshold (conf=0.001, max_det=300) so the
full precision/recall curve is available. The deployed pipeline runs at conf=0.25.
These give DIFFERENT mAP values and are not interchangeable:

  --conf 0.001  -> "standard" mAP, comparable to Ultralytics / published tables
  --conf 0.25   -> "operating point" mAP, what the robot actually achieves in flight

Run BOTH where the backend allows it. It does not always allow it: YOLOv8/v11 HEFs
compiled with on-device NMS have the confidence threshold BAKED IN at compile time
(access.tex Sec.IV-A), so they cannot be swept down to 0.001 and their standard mAP
is structurally unobtainable on the NPU. YOLO26's raw-tensor HEF CAN be swept,
because its threshold is applied on the host. That asymmetry is a genuine finding,
not a measurement error — it is the accuracy-side consequence of the same
on-device-NMS trade-off the paper already describes.

USAGE
-----
    # CPU, standard mAP, full val2017
    python3 coco_accuracy_bench.py --backend cpu --model yolo26n \
        --repo-root ~/ROS2-PROJECT-SPRING2026 \
        --coco-images /data/coco/val2017 \
        --coco-ann /data/coco/annotations/instances_val2017.json \
        --conf 0.001

    # same model at the deployed operating point
    ... --conf 0.25

    # NPU
    ... --backend npu --model yolo26n --conf 0.001

    # subset (recommended for the slow CPU-medium configs)
    ... --limit 1000

RUNTIME GUIDE (5000 images, from the paper's measured per-frame latencies)
    NPU  any scale   ~26-49 ms   ->  2-4 min      <- run the full set
    CPU  nano        ~180-220 ms ->  15-18 min
    CPU  small       ~470-590 ms ->  40-50 min
    CPU  medium      ~1.4-1.5 s  ->  2 h          <- use --limit 1000
Always report the --limit you used.

    pip install pycocotools opencv-python numpy
"""

import argparse
import csv
import json
import os
import platform
import socket
import sys
import time
from datetime import datetime

import numpy as np

# YOLO's 80 contiguous class indices -> COCO's sparse 91-category ids.
# Index 11 ('stop sign') maps to COCO category 13.
COCO80_TO_COCO91 = [
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 27, 28, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
    43, 44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61,
    62, 63, 64, 65, 67, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84,
    85, 86, 87, 88, 89, 90,
]
STOP_SIGN_YOLO_CLASS = 11
STOP_SIGN_COCO_ID = 13


def log(m):
    print(m, flush=True)


# ───────────────────────────────────────────────────────── predictors ────────

class RepoPredictor:
    """Wraps the repo's own build_predictor() so accuracy reflects the deployed path."""

    def __init__(self, repo_root, backend, model_id, conf, threads, inter_threads):
        engine_dir = os.path.join(repo_root, "src", "yolo_producer")
        if engine_dir not in sys.path:
            sys.path.insert(0, engine_dir)
        try:
            from yolo_runtime_engine import build_predictor  # noqa
        except Exception as e:
            raise RuntimeError(
                f"could not import yolo_runtime_engine from {engine_dir}: {e}\n"
                "Check --repo-root, and that you are on branch exp/detector-benchmarks."
            )
        kw = dict(backend=backend, model_id=model_id, conf=conf)
        # the engine's signature has varied; pass thread args only if accepted
        import inspect
        sig = inspect.signature(build_predictor)
        if "threads" in sig.parameters:
            kw["threads"] = threads
        if "inter_threads" in sig.parameters:
            kw["inter_threads"] = inter_threads
        log(f"  build_predictor({', '.join(f'{k}={v!r}' for k, v in kw.items())})")
        self.pred = build_predictor(**kw)

    def infer(self, rgb):
        timing = {}
        dets = self.pred.infer(rgb, timing=timing)
        return dets, timing

    def close(self):
        """Release the Hailo VDevice. NOT optional: the engine's own close()
        warns that leaking the VDevice/CIM wedges the Hailo SoC off PCIe, which
        would break every subsequent model in a sequential sweep."""
        try:
            self.pred.close()
        except Exception as e:
            log(f"  !! predictor close failed: {e}")


class StandaloneOnnxPredictor:
    """Laptop smoke-test only. Plain onnxruntime + minimal letterbox + e2e postprocess."""

    def __init__(self, onnx_path, conf, threads, inter_threads, imgsz=640):
        import onnxruntime as ort

        so = ort.SessionOptions()
        so.intra_op_num_threads = threads
        so.inter_op_num_threads = inter_threads
        self.sess = ort.InferenceSession(onnx_path, sess_options=so,
                                         providers=["CPUExecutionProvider"])
        self.iname = self.sess.get_inputs()[0].name
        self.conf = conf
        self.imgsz = imgsz
        log(f"  standalone onnx: {os.path.basename(onnx_path)} "
            f"outputs={[(o.name, o.shape) for o in self.sess.get_outputs()]}")

    def infer(self, rgb):
        import cv2

        t = {}
        t0 = time.perf_counter()
        h, w = rgb.shape[:2]
        scale = min(self.imgsz / w, self.imgsz / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(rgb, (nw, nh))
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, dtype=np.uint8)
        pl, pt = (self.imgsz - nw) // 2, (self.imgsz - nh) // 2
        canvas[pt:pt + nh, pl:pl + nw] = resized
        x = canvas.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        x = np.ascontiguousarray(x)
        t["pre_done"] = time.perf_counter()
        t["inf_start"] = t["pre_done"]
        outs = self.sess.run(None, {self.iname: x})
        t["inf_done"] = time.perf_counter()

        pred = outs[0]
        pred = pred[0] if pred.ndim == 3 else pred
        dets = []
        if pred.ndim == 2 and pred.shape[1] >= 6:
            coords = pred[:, :4].astype(np.float32)
            confs = pred[:, 4].astype(np.float32)
            cls = pred[:, 5].astype(np.int32)
            if coords.size and float(coords.max()) <= 1.5:
                coords = coords * float(self.imgsz)
            keep = confs >= self.conf
            coords, confs, cls = coords[keep], confs[keep], cls[keep]
            for i in range(len(confs)):
                x1 = (coords[i, 0] - pl) / scale
                y1 = (coords[i, 1] - pt) / scale
                x2 = (coords[i, 2] - pl) / scale
                y2 = (coords[i, 3] - pt) / scale
                dets.append((float(np.clip(x1, 0, w)), float(np.clip(y1, 0, h)),
                             float(np.clip(x2, 0, w)), float(np.clip(y2, 0, h)),
                             int(cls[i]), float(confs[i])))
        t["nms_done"] = time.perf_counter()
        t["_t0"] = t0
        return dets, t


# ────────────────────────────────────────────────────────── helpers ──────────

def unpack_det(d):
    """Engine may return namedtuples/objects or plain 6-tuples."""
    for attrs in (("x1", "y1", "x2", "y2", "class_id", "confidence"),
                  ("x1", "y1", "x2", "y2", "cls", "conf")):
        if all(hasattr(d, a) for a in attrs):
            return tuple(getattr(d, a) for a in attrs)
    if isinstance(d, dict):
        return (d["x1"], d["y1"], d["x2"], d["y2"],
                d.get("class_id", d.get("cls")), d.get("confidence", d.get("conf")))
    seq = list(d)
    return tuple(seq[:6])


def stage_ms(t):
    """Convert the engine's timing stamps into per-stage milliseconds.

    The engine stamps pre_done/inf_start/inf_done/nms_done with _now_ns(), i.e.
    CLOCK_MONOTONIC *nanoseconds* — not perf_counter seconds. `_t0` must therefore
    be supplied on the same clock (see the call site), and every delta converts
    with 1e-6. Using 1e3 here inflates every stage by 1e6.
    """
    t0 = t.get("_t0")
    pre, inf, post = float("nan"), float("nan"), float("nan")
    try:
        if t0 is not None and "pre_done" in t:
            pre = (t["pre_done"] - t0) * 1e-6
        if "inf_start" in t and "inf_done" in t:
            inf = (t["inf_done"] - t["inf_start"]) * 1e-6
        if "inf_done" in t and "nms_done" in t:
            post = (t["nms_done"] - t["inf_done"]) * 1e-6
    except Exception:
        pass
    return pre, inf, post


# ──────────────────────────────────────────────────────────── main ───────────

def main():
    ap = argparse.ArgumentParser(description="COCO accuracy + latency (Brief C)")
    ap.add_argument("--backend", choices=["cpu", "npu"], default="cpu")
    ap.add_argument("--model", required=True, help="registry id, e.g. yolo26n")
    ap.add_argument("--repo-root", default="",
                    help="checkout root (exp/detector-benchmarks) for the real engine")
    ap.add_argument("--standalone-onnx", default="",
                    help="laptop smoke-test only: path to a .onnx, bypasses the engine")
    ap.add_argument("--coco-images", required=True)
    ap.add_argument("--coco-ann", required=True)
    ap.add_argument("--conf", type=float, default=0.001,
                    help="0.001 = standard mAP; 0.20 = deployed operating point")
    ap.add_argument("--limit", type=int, default=0, help="0 = all images")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--inter-threads", type=int, default=1)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args()

    import cv2
    from pycocotools.coco import COCO
    from pycocotools.cocoeval import COCOeval

    log("=" * 74)
    log("  Brief C — COCO accuracy + latency")
    log("=" * 74)
    log(f"  host      : {socket.gethostname()}  {platform.machine()}")
    log(f"  backend   : {args.backend}   model: {args.model}")
    log(f"  conf      : {args.conf}  "
        f"({'standard mAP' if args.conf <= 0.01 else 'DEPLOYED OPERATING POINT'})")

    if args.standalone_onnx:
        log("  !! --standalone-onnx: smoke-test path, NOT for paper numbers")
        pred = StandaloneOnnxPredictor(args.standalone_onnx, args.conf,
                                       args.threads, args.inter_threads)
    else:
        if not args.repo_root:
            log("ERROR: pass --repo-root (or --standalone-onnx for a smoke test)")
            return 2
        pred = RepoPredictor(args.repo_root, args.backend, args.model,
                             args.conf, args.threads, args.inter_threads)

    # Release the backend on EVERY exit path (normal return, error, Ctrl-C).
    import atexit
    if hasattr(pred, "close"):
        atexit.register(pred.close)

    coco = COCO(args.coco_ann)
    img_ids = sorted(coco.getImgIds())
    if args.limit:
        img_ids = img_ids[:args.limit]
    log(f"  images    : {len(img_ids)}"
        f"{' (SUBSET — report this)' if args.limit else ' (full val2017)'}")

    # warmup
    if img_ids:
        w0 = coco.loadImgs(img_ids[0])[0]
        wp = os.path.join(args.coco_images, w0["file_name"])
        wi = cv2.imread(wp)
        if wi is not None:
            for _ in range(args.warmup):
                pred.infer(cv2.cvtColor(wi, cv2.COLOR_BGR2RGB))

    results = []
    pre_l, inf_l, post_l, tot_l = [], [], [], []
    missing = 0
    t_start = time.perf_counter()

    for n, iid in enumerate(img_ids, 1):
        info = coco.loadImgs(iid)[0]
        path = os.path.join(args.coco_images, info["file_name"])
        bgr = cv2.imread(path)
        if bgr is None:
            missing += 1
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        t0 = time.perf_counter()
        # engine stamps are CLOCK_MONOTONIC ns; _t0 must share that clock/unit.
        t0_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        dets, timing = pred.infer(rgb)
        t1 = time.perf_counter()
        timing.setdefault("_t0", t0_ns)

        p, i_, po = stage_ms(timing)
        pre_l.append(p); inf_l.append(i_); post_l.append(po)
        tot_l.append((t1 - t0) * 1e3)

        for d in dets:
            x1, y1, x2, y2, cls, cf = unpack_det(d)
            cls = int(cls)
            if not (0 <= cls < len(COCO80_TO_COCO91)):
                continue
            results.append({
                "image_id": int(iid),
                "category_id": COCO80_TO_COCO91[cls],
                "bbox": [round(float(x1), 2), round(float(y1), 2),
                         round(float(x2 - x1), 2), round(float(y2 - y1), 2)],
                "score": round(float(cf), 5),
            })

        if n % 250 == 0:
            el = time.perf_counter() - t_start
            log(f"    {n}/{len(img_ids)}  {n/el:.1f} img/s  "
                f"eta {(len(img_ids)-n)/max(n/el,1e-9)/60:.1f} min")

    wall = time.perf_counter() - t_start
    if missing:
        log(f"  !! {missing} images could not be read")

    def m(a):
        a = [v for v in a if v == v]
        return float(np.mean(a)) if a else float("nan")

    tag = f"{args.model}_{args.backend}_conf{args.conf}"
    os.makedirs(args.out_dir, exist_ok=True)
    det_json = os.path.join(args.out_dir, f"detections_{tag}.json")
    with open(det_json, "w") as f:
        json.dump(results, f)
    log(f"\n  {len(results)} detections -> {det_json}")

    if not results:
        log("  !! no detections at all — check the model/threshold before trusting this")
        return 1

    log("\n--- COCO evaluation (all 80 classes) ---")
    dt = coco.loadRes(det_json)
    ev = COCOeval(coco, dt, "bbox")
    ev.params.imgIds = img_ids
    ev.evaluate(); ev.accumulate(); ev.summarize()
    mAP, mAP50, mAP75 = ev.stats[0], ev.stats[1], ev.stats[2]

    log("\n--- COCO evaluation ('stop sign' only, the mission-critical class) ---")
    ev2 = COCOeval(coco, dt, "bbox")
    ev2.params.imgIds = img_ids
    ev2.params.catIds = [STOP_SIGN_COCO_ID]
    ev2.evaluate(); ev2.accumulate(); ev2.summarize()
    stop_mAP, stop_mAP50 = ev2.stats[0], ev2.stats[1]

    row = dict(
        model=args.model, backend=args.backend, conf=args.conf,
        images=len(img_ids), subset=bool(args.limit),
        intra_op=args.threads, inter_op=args.inter_threads,
        mAP50_95=round(mAP, 4), mAP50=round(mAP50, 4), mAP75=round(mAP75, 4),
        stopsign_AP50_95=round(stop_mAP, 4), stopsign_AP50=round(stop_mAP50, 4),
        preprocess_ms=round(m(pre_l), 3), inference_ms=round(m(inf_l), 3),
        postprocess_ms=round(m(post_l), 3), total_ms=round(m(tot_l), 3),
        fps=round(len(tot_l) / wall, 2) if wall > 0 else float("nan"),
        n_detections=len(results),
        engine="repo" if not args.standalone_onnx else "standalone-onnx",
    )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.out_dir,
                            f"coco_accuracy_{tag}_{socket.gethostname()}_{stamp}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        w.writeheader(); w.writerow(row)

    log("\n" + "=" * 74)
    log("  SUMMARY")
    log("=" * 74)
    log(f"  mAP50-95 {row['mAP50_95']:.4f}   mAP50 {row['mAP50']:.4f}   "
        f"mAP75 {row['mAP75']:.4f}")
    log(f"  stop sign: AP50-95 {row['stopsign_AP50_95']:.4f}   "
        f"AP50 {row['stopsign_AP50']:.4f}")
    log(f"  latency  : pre {row['preprocess_ms']:.2f}  inf {row['inference_ms']:.2f}  "
        f"post {row['postprocess_ms']:.3f}  total {row['total_ms']:.2f} ms "
        f"({row['fps']:.2f} fps)")
    log(f"\n  CSV: {csv_path}")
    log("  Send back the CSV + this console output. State --limit if you subset.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
