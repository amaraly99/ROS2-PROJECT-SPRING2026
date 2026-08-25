#!/usr/bin/env python3
"""
export_yolo26_raw_tensor.py — build a CPU ONNX export for YOLO26 that matches
the architecture the Hailo HEF actually runs, not YOLO26's usual NMS-free export.

WHY THIS EXISTS
---------------
The registry's yolo26*.onnx (onnx_layout: yolo26_e2e) is an NMS-free, end-to-end
graph: (1,300,6), already-selected detections, near-zero postprocess. The HEF
(hef_postproc: raw_tensor) is six raw conv tensors -- bbox+cls per scale,
8400 anchors, decoded and NMS'd on the host, EXACTLY the v8/v11 architecture.

So "YOLO26 CPU FP32 vs YOLO26 NPU INT8" was never a clean quantization
comparison: it also silently changed the architecture. This produces a CPU
ONNX with the SAME six-tensor raw layout as the HEF (verified: same tensor
count, same per-scale shapes, same 4-channel bbox regression -- YOLO26 has no
DFL, unlike v8/v11's 64-channel bins), so a CPU-vs-NPU run differs in
precision only, matching v8/v11's clean comparison.

Reuses benchmarks/yolo26_cpu_fastmode_ab.py:export_one2many() -- the same
export Brief A used for its e2e-vs-one2many latency A/B -- rather than writing
a second exporter that could drift from it.

USAGE
    python3 benchmarks/export_yolo26_raw_tensor.py [--out-dir models/onnx_raw_tensor]

Then feed the result to the accuracy bench with the matching layout hint:
    python3 benchmarks/coco_accuracy_bench.py --backend cpu --model yolo26n \\
        --model-path models/onnx_raw_tensor/yolo26n_one2many.onnx \\
        --onnx-layout raw_tensor ...
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from yolo26_cpu_fastmode_ab import export_one2many  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="models/onnx_raw_tensor")
    ap.add_argument("--scales", default="n,s,m")
    ap.add_argument("--weights-dir", default="models",
                    help="directory containing yolo26<scale>.pt")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    for scale in args.scales.split(","):
        weights = os.path.join(args.weights_dir, f"yolo26{scale}.pt")
        if not os.path.exists(weights):
            print(f"!! {weights} not found, skipping yolo26{scale}")
            continue
        dst = export_one2many(weights, args.out_dir, f"yolo26{scale}")
        print(f"yolo26{scale}: {dst}")


if __name__ == "__main__":
    main()
