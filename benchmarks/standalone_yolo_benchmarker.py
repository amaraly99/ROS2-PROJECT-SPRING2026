#!/usr/bin/env python3
"""
standalone_yolo_benchmarker — isolated model performance, no ROS2 / no SHM.

Measures a single (model, backend, thread-config) cell of the experimental matrix
with zero pipeline overhead, so the numbers reflect the model+backend alone. Built
on the plug-and-play `yolo_runtime_engine`, so the exact same code path benchmarks
CPU (onnxruntime) and NPU (pyHailoRT).

Per the systems-paper protocol it skips warm-up frames, then over the measured
frames records (in seconds): preprocess, raw inference, postprocess/NMS, end-to-end,
plus throughput (FPS). Results dump to a structured JSON file and a clean stdout table.

Examples:
    # by family/variant (resolved through the registry)
    python3 benchmarks/standalone_yolo_benchmarker.py \
        --backend cpu --model_family v8 --variant n --threads 2 --frames 500

    # by explicit path
    python3 benchmarks/standalone_yolo_benchmarker.py \
        --backend npu --model_path models/hef/yolov8n.hef --frames 500

    # whole CPU sweep is just a shell loop over --model_family/--variant/--threads
"""

import argparse
import json
import os
import sys
import time
from typing import Dict, List

import cv2
import numpy as np

# Make the engine importable wherever this script is run from.
_THIS = os.path.dirname(os.path.abspath(__file__))
_ENGINE_DIR = os.path.join(_THIS, "..", "src", "yolo_producer")
sys.path.insert(0, _ENGINE_DIR)
from yolo_runtime_engine import build_predictor, ModelRegistry  # noqa: E402


def pct(vals: List[float], p: float) -> float:
    """Nearest-rank percentile (matches benchmarks/hef_bench.py)."""
    if not vals:
        return 0.0
    s = sorted(vals)
    k = max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def _stats(vals: List[float]) -> Dict[str, float]:
    """Per-stage summary in SECONDS (mean/p50/p95) as the paper protocol asks."""
    if not vals:
        return {"mean": 0.0, "p50": 0.0, "p95": 0.0}
    return {"mean": float(np.mean(vals)),
            "p50": float(pct(vals, 50)),
            "p95": float(pct(vals, 95))}


def main():
    ap = argparse.ArgumentParser(
        description="Isolated YOLO benchmark (CPU/onnxruntime vs NPU/pyHailoRT)")
    # model selection — registry id OR family+variant OR explicit path
    ap.add_argument("--backend", required=True, choices=["cpu", "npu"])
    ap.add_argument("--model_id", default=None, help="registry id e.g. yolov8n")
    ap.add_argument("--model_family", default=None, help="v8 | v11 | yolo26")
    ap.add_argument("--variant", default=None, help="n | s | m")
    ap.add_argument("--model_path", default=None,
                    help="explicit .onnx (cpu) / .hef (npu); bypasses the registry")
    # CPU thread control (the contention-avoidance knobs)
    ap.add_argument("--threads", type=int, default=2,
                    help="onnxruntime intra_op_num_threads (CPU only)")
    ap.add_argument("--inter_threads", type=int, default=1,
                    help="onnxruntime inter_op_num_threads (CPU only)")
    # workload
    ap.add_argument("--frames", type=int, default=500, help="measured frames")
    ap.add_argument("--warmup", type=int, default=10, help="warm-up frames to skip")
    ap.add_argument("--conf", type=float, default=None)
    ap.add_argument("--iou", type=float, default=None)
    ap.add_argument("--image", default="/tmp/bus.jpg",
                    help="static RGB photo (real image -> meaningful detections)")
    ap.add_argument("--synthetic", action="store_true",
                    help="use a random frame instead of --image (timing only)")
    ap.add_argument("--registry", default=None, help="override model_registry.json path")
    ap.add_argument("--json", default=None, help="output JSON path (auto-named if omitted)")
    args = ap.parse_args()

    reg = ModelRegistry(args.registry)

    # ── Resolve a logical model id (for labelling + registry lookup) ───────────
    model_id = args.model_id
    if model_id is None and args.model_family and args.variant:
        model_id = ModelRegistry.id_from(args.model_family, args.variant)
    if model_id is None and args.model_path is None:
        ap.error("specify --model_id, or --model_family + --variant, or --model_path")

    # ── Build the predictor (backend-agnostic) ─────────────────────────────────
    pred = build_predictor(
        args.backend,
        model_id=model_id,
        model_path=args.model_path,
        registry=reg,
        conf=args.conf, iou=args.iou,
        threads=args.threads, inter_threads=args.inter_threads,
    )

    label = model_id or os.path.splitext(os.path.basename(args.model_path))[0]

    # ── Source frame ───────────────────────────────────────────────────────────
    if args.synthetic:
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    else:
        bgr = cv2.imread(args.image)
        if bgr is None:
            print(f"[error] cannot read image: {args.image} "
                  f"(use --synthetic for timing-only)", file=sys.stderr)
            sys.exit(2)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    src_h, src_w = rgb.shape[:2]

    print(f"══ {label}  backend={args.backend}  "
          f"threads={args.threads if args.backend == 'cpu' else 'n/a'} ══")

    # ── Timed loop (warm-up excluded) ──────────────────────────────────────────
    s_pre, s_inf, s_pp, s_e2e = [], [], [], []
    last_dets = []
    total = args.warmup + args.frames
    timing: Dict[str, int] = {}
    for i in range(total):
        t_start = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        timing.clear()
        dets = pred.infer(rgb, timing=timing)

        if i >= args.warmup:
            # Engine fills absolute CLOCK_MONOTONIC ns stamps; derive per-stage spans.
            pre = (timing["pre_done"] - t_start) / 1e9
            inf = (timing["inf_done"] - timing["inf_start"]) / 1e9
            pp = (timing["nms_done"] - timing["inf_done"]) / 1e9
            e2e = (timing["nms_done"] - t_start) / 1e9
            s_pre.append(pre); s_inf.append(inf); s_pp.append(pp); s_e2e.append(e2e)
        last_dets = dets

    pred.close()

    # ── Aggregate ──────────────────────────────────────────────────────────────
    e2e_mean = float(np.mean(s_e2e)) if s_e2e else 0.0
    fps = 1.0 / e2e_mean if e2e_mean > 0 else 0.0

    by_cls: Dict[str, int] = {}
    for d in last_dets:
        cn = pred.class_name(int(d[4]))
        by_cls[cn] = by_cls.get(cn, 0) + 1

    result = {
        "config": {
            "model_id": model_id,
            "label": label,
            "backend": args.backend,
            "model_path": pred.model_path,
            "family": args.model_family,
            "variant": args.variant,
            "intra_threads": args.threads if args.backend == "cpu" else None,
            "inter_threads": args.inter_threads if args.backend == "cpu" else None,
            "input_size": pred.input_size,
            "conf": pred.conf, "iou": pred.iou,
            "frames": args.frames, "warmup": args.warmup,
            "image": None if args.synthetic else args.image,
            "src_resolution": [src_w, src_h],
        },
        "latency_s": {
            "preprocess": _stats(s_pre),
            "inference": _stats(s_inf),
            "postprocess": _stats(s_pp),
            "end_to_end": _stats(s_e2e),
        },
        "fps": {"mean": fps},
        "detections": {"count": len(last_dets), "by_class": by_cls},
        "host": {"cpu_count": os.cpu_count()},
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    # ── Output: JSON file ──────────────────────────────────────────────────────
    out_path = args.json
    if not out_path:
        tag = f"t{args.threads}" if args.backend == "cpu" else "npu"
        out_dir = os.path.join(_THIS, "results", "standalone")
        out_path = os.path.join(out_dir, f"bench_{label}_{args.backend}_{tag}.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)

    # ── Output: clean stdout table (ms for readability) ────────────────────────
    def ms(x):  # seconds -> ms
        return x * 1000.0
    L = result["latency_s"]
    det_summary = ", ".join(f"{k}×{v}" for k, v in
                            sorted(by_cls.items(), key=lambda x: -x[1])) or "(none)"
    print(f"  stage          mean      p50      p95   (ms)")
    print(f"  preprocess   {ms(L['preprocess']['mean']):7.2f}  "
          f"{ms(L['preprocess']['p50']):7.2f}  {ms(L['preprocess']['p95']):7.2f}")
    print(f"  inference    {ms(L['inference']['mean']):7.2f}  "
          f"{ms(L['inference']['p50']):7.2f}  {ms(L['inference']['p95']):7.2f}")
    print(f"  postprocess  {ms(L['postprocess']['mean']):7.2f}  "
          f"{ms(L['postprocess']['p50']):7.2f}  {ms(L['postprocess']['p95']):7.2f}")
    print(f"  END-TO-END   {ms(L['end_to_end']['mean']):7.2f}  "
          f"{ms(L['end_to_end']['p50']):7.2f}  {ms(L['end_to_end']['p95']):7.2f}"
          f"   →  {fps:5.1f} FPS")
    print(f"  detections   {len(last_dets)}: {det_summary}")
    print(f"  json → {out_path}")


if __name__ == "__main__":
    main()
