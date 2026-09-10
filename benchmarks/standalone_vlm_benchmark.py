#!/usr/bin/env python3
"""
standalone_vlm_benchmark.py — isolated OWL-ViT timing, same protocol as the YOLO rows.

benchmarks/standalone_yolo_benchmarker.py resolves models through the YOLO registry
and cannot drive the vision-language detector, so tab:detectors carried an
unsourced "~1700 ms / ~0.6 fps" for OWL-ViT -- a figure the manuscript also leans on
when it reports the in-loop cost as ~2.2x standalone. This measures it under the
same conditions as the YOLO CPU rows: one still image, warm-up frames discarded,
per-stage means, 2 torch threads.

Reuses OwlViTPredictor from src/yolo_producer/vlm_detector_producer.py, so the timing
covers the deployed code path rather than a reimplementation.

    python3 benchmarks/standalone_vlm_benchmark.py \\
        --frames 25 --warmup 3 --threads 2 \\
        --image benchmarks/assets/stop_sign_test.jpg \\
        --json benchmarks/paper_data/standalone/bench_owlvit_cpu_t2.json
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "yolo_producer"))
from vlm_detector_producer import OwlViTPredictor  # noqa: E402


def pct(v, p):
    s = sorted(v)
    return s[max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))]


def stats(v):
    return {"mean": float(np.mean(v)), "p50": pct(v, 50), "p95": pct(v, 95)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prompt", default="stop sign")
    ap.add_argument("--box-threshold", dest="box_threshold", type=float, default=0.20)
    ap.add_argument("--model", default="google/owlvit-base-patch32")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--frames", type=int, default=25)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--image", default="benchmarks/assets/stop_sign_test.jpg")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    bgr = cv2.imread(a.image)
    if bgr is None:
        raise SystemExit(f"cannot read {a.image}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    pred = OwlViTPredictor(a.prompt, a.box_threshold, a.model, a.threads)

    pre, inf, post, e2e = [], [], [], []
    n_dets = 0
    for i in range(a.warmup + a.frames):
        t = {}
        t0 = time.monotonic_ns()
        dets = pred.infer(rgb, t)
        t1 = time.monotonic_ns()
        if i < a.warmup:
            continue
        pre.append((t["pre_done"] - t0) / 1e6)
        inf.append((t["inf_done"] - t["inf_start"]) / 1e6)
        post.append((t["nms_done"] - t["inf_done"]) / 1e6)
        e2e.append((t1 - t0) / 1e6)
        n_dets = len(dets)

    out = {
        "config": {"model_id": "owlvit-base-patch32", "label": "OWL-ViT base-32",
                   "backend": "cpu", "prompt": a.prompt,
                   "box_threshold": a.box_threshold,
                   "intra_threads": a.threads, "inter_threads": 1,
                   "frames": a.frames, "warmup": a.warmup, "image": a.image,
                   "src_resolution": list(bgr.shape[:2])},
        "latency_ms": {"preprocess": stats(pre), "inference": stats(inf),
                       "postprocess": stats(post), "end_to_end": stats(e2e)},
        "fps": {"mean": 1000.0 / float(np.mean(e2e))},
        "n_detections_last_frame": n_dets,
    }
    print(f"OWL-ViT  threads={a.threads}  frames={a.frames}")
    for k in ("preprocess", "inference", "postprocess", "end_to_end"):
        s = out["latency_ms"][k]
        print(f"  {k:12s} mean {s['mean']:9.1f}  p50 {s['p50']:9.1f}  p95 {s['p95']:9.1f} ms")
    print(f"  fps          {out['fps']['mean']:.2f}     detections: {n_dets}")
    if a.json:
        os.makedirs(os.path.dirname(a.json), exist_ok=True)
        json.dump(out, open(a.json, "w"), indent=2)
        print(f"  json -> {a.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
