#!/usr/bin/env python3
"""
quantization_score_shift.py — does INT8 quantization move confidence scores down?

The manuscript claims a mechanism for why the YOLO26n INT8 penalty more than
doubles between conf=0.001 and conf=0.20: that quantization perturbs confidence
scores downwards, so a permissive threshold still admits detections that the
operating threshold discards. That sentence was reasoned from the two mAP
numbers, not measured. This script measures it.

Method: the same model, the same images, the same detection head, evaluated on
CPU in FP32 and on the Hailo-10H in INT8, both at conf=0.001 so the full score
range is visible on both sides. Match each CPU detection to the NPU detection of
the same category on the same image by box overlap, then compare the score the
two backends assigned to the SAME object. Aggregate score histograms cannot do
this: they say nothing about which detection moved where.

The quantity the claim rests on is the net flow across the operating threshold:
how many matched objects sit above 0.20 on CPU but below it on the NPU, against
how many cross the other way.

    python3 benchmarks/quantization_score_shift.py [--repo-root .] [--iou 0.5]
"""

import argparse
import json
import os
import statistics
from collections import defaultdict

OPERATING_CONF = 0.20


def iou_xywh(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    iw, ih = ix2 - ix1, iy2 - iy1
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def index(dets):
    """image_id -> category_id -> [detection, ...], highest score first."""
    out = defaultdict(lambda: defaultdict(list))
    for d in dets:
        out[d["image_id"]][d["category_id"]].append(d)
    for img in out.values():
        for lst in img.values():
            lst.sort(key=lambda d: -d["score"])
    return out


def match(cpu_idx, npu_idx, iou_thr):
    """Greedy highest-score-first matching within (image, category)."""
    pairs = []
    cpu_only = 0
    cpu_only_above = 0
    for img_id, cats in cpu_idx.items():
        npu_cats = npu_idx.get(img_id, {})
        for cat, cpu_dets in cats.items():
            npu_dets = npu_cats.get(cat, [])
            taken = [False] * len(npu_dets)
            for c in cpu_dets:
                best, best_iou = -1, iou_thr
                for j, n in enumerate(npu_dets):
                    if taken[j]:
                        continue
                    v = iou_xywh(c["bbox"], n["bbox"])
                    if v >= best_iou:
                        best, best_iou = j, v
                if best >= 0:
                    taken[best] = True
                    pairs.append((c["score"], npu_dets[best]["score"]))
                else:
                    cpu_only += 1
                    if c["score"] >= OPERATING_CONF:
                        cpu_only_above += 1
    return pairs, cpu_only, cpu_only_above


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", default=".")
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    base = os.path.join(os.path.abspath(args.repo_root),
                        "benchmarks", "results", "accuracy", "coco", "v2_controlled")
    cpu_f = os.path.join(base, "detections_yolo26n_cpu_conf0.001_controlled_raw_tensor.json")
    npu_f = os.path.join(base, "detections_yolo26n_npu_conf0.001_controlled.json")
    for f in (cpu_f, npu_f):
        if not os.path.exists(f):
            raise SystemExit(f"missing input: {f}")

    cpu = json.load(open(cpu_f))
    npu = json.load(open(npu_f))
    print(f"CPU FP32 detections : {len(cpu):,}")
    print(f"NPU INT8 detections : {len(npu):,}")

    pairs, cpu_only, cpu_only_above = match(index(cpu), index(npu), args.iou)
    if not pairs:
        raise SystemExit("no matched pairs")

    deltas = [n - c for c, n in pairs]
    down = sum(1 for d in deltas if d < 0)

    # The claim's own test: matched objects crossing the operating threshold.
    lost = sum(1 for c, n in pairs if c >= OPERATING_CONF > n)
    gained = sum(1 for c, n in pairs if n >= OPERATING_CONF > c)
    above_cpu = sum(1 for c, _ in pairs if c >= OPERATING_CONF)

    # Counted straight off the two files, with no matching involved. If the
    # matched-pair accounting above is right, this difference should land near
    # the net crossing loss. Two independent routes to the same number.
    cpu_above_raw = sum(1 for d in cpu if d["score"] >= OPERATING_CONF)
    npu_above_raw = sum(1 for d in npu if d["score"] >= OPERATING_CONF)

    lines = [
        f"matched pairs (IoU >= {args.iou})      : {len(pairs):,}",
        f"CPU detections with no NPU match      : {cpu_only:,}",
        "",
        f"mean   score shift (NPU - CPU)        : {statistics.mean(deltas):+.5f}",
        f"median score shift (NPU - CPU)        : {statistics.median(deltas):+.5f}",
        f"fraction of matches scored LOWER on NPU: {down / len(pairs):.4f}",
        "",
        f"matched objects above {OPERATING_CONF} on CPU     : {above_cpu:,}",
        f"  ... pushed BELOW {OPERATING_CONF} by INT8        : {lost:,} "
        f"({lost / above_cpu:.4f} of them)" if above_cpu else "",
        f"  ... pulled ABOVE {OPERATING_CONF} by INT8        : {gained:,}",
        f"net loss across the operating threshold: {lost - gained:+,}",
        "",
        f"CPU detections above {OPERATING_CONF} with no NPU match at all: {cpu_only_above:,}",
        "",
        "Independent cross-check, counted without any matching:",
        f"  CPU detections above {OPERATING_CONF}                 : {cpu_above_raw:,}",
        f"  NPU detections above {OPERATING_CONF}                 : {npu_above_raw:,}",
        f"  difference                              : {cpu_above_raw - npu_above_raw:+,}",
    ]
    report = "\n".join(l for l in lines if l != "")
    print("\n" + report)

    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        with open(args.out, "w") as f:
            f.write("# Quantization score shift, YOLO26n, matched detections\n\n")
            f.write(f"Inputs (conf=0.001, controlled postprocess, matched raw-tensor head):\n")
            f.write(f"- CPU FP32: {os.path.basename(cpu_f)} ({len(cpu):,} detections)\n")
            f.write(f"- NPU INT8: {os.path.basename(npu_f)} ({len(npu):,} detections)\n\n")
            f.write("```\n" + report + "\n```\n")
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
