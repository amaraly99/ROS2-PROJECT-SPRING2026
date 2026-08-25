# Postprocess ablation — how much of the v1 YOLO26 deficit was measurement artifact

**Run:** 2026-08-24 · `benchmarks/run_postprocess_ablation.sh` · 13 cells
**Data:** `postprocess_ablation/ablation_all_cells.csv` (one row per cell)
**Conditions:** COCO val2017, 500 images, `conf=0.001`, ONNX Runtime 2 intra-op / 1 inter-op,
unpinned. `conf` is deliberately low: these artifacts act on the tail of the precision/recall
curve, which the deployed 0.20 threshold hides.

## Why

The v1 sweep reported YOLO26 losing to YOLOv8/v11 on the NPU in both accuracy and latency. That
result could not be trusted, because the four backend×family postprocess paths had never been
controlled against each other and every uncontrolled difference happened to land on YOLO26/NPU —
the one cell where YOLO26 lost. Before re-running 40 configs, this measures what each difference
was actually worth.

## Arms

| | |
|---|---|
| `yolo26n` / NPU | full 2×2×2: NMS IoU {0.45, 0.70} × max_det {64, 300} × coords {int, float} |
| `yolov8n` / NPU | max_det {64, 300} × coords {int, float}; IoU is not a free variable — its NMS runs on-device at a baked 0.70 |
| `yolo26n` / CPU | FP32 anchor at controlled postprocess |

## Results — main effects, yolo26n / NPU

| Factor | Δ mAP50-95 | Relative |
|---|---|---|
| NMS IoU 0.45 → 0.70 | **+0.0089** | **+2.2 %** |
| Box coords int → float | **+0.0076** | **+1.9 %** |
| max_det 64 → 300 | +0.0022 | +0.5 % |
| **v1 configuration → fully controlled** | **0.4080 → 0.4267** | **+4.6 %** |

Same correction applied to `yolov8n` / NPU is worth only **+1.2 %** (coords alone; its NMS IoU is
immovable and its cap never binds). **The harness was therefore biased by ≈3.4 % relative in
YOLOv8/v11's favour** — against a v1 gap at the deployed threshold of 4.0 % relative
(v8n 0.3183 vs yolo26n 0.3061). The correction very nearly erases the reported deficit.

## Findings

**1. The NMS IoU mismatch was the dominant artifact.** `yolo_runtime_engine.py:185` defaults
`iou=0.45` for host-side NMS, while the v8/v11 HEFs bake `IoU 0.70`. YOLO26's raw-tensor export
is post-processed on the host and so was the only configuration paying the stricter threshold.
0.70 is also what Ultralytics' published mAP uses, so the v1 YOLO26 numbers were not comparable
to the external reference either.

**2. The 64-detection cap was nearly irrelevant — a correction to our own earlier claim.**
`YOLO_MAX_DETS = 64` is applied by both NPU postprocess paths and by neither CPU path, which is a
genuine inconsistency, and YOLO26 averages ~35 det/img against v8/v11's ~7, so it binds almost
exclusively on YOLO26. It was initially flagged as one of three major biases. It is worth
**+0.5 %** for yolo26n and **exactly zero** for yolov8n (3174 detections either way). Detections
are confidence-sorted and COCO evaluates at maxDets=100, so the top 64 already carry almost all
the AP. Report it as an inconsistency worth fixing, not as an explanation of anything.

**3. Integer truncation hurts the mission-critical class most.** `int()` truncates rather than
rounds, biasing every box inward sub-pixel:

| | aggregate mAP50-95 | stop-sign AP50-95 |
|---|---|---|
| `yolo26n` int → float | +1.9 % | **+4.9 %** |
| `yolov8n` int → float | +1.2 % | **+2.9 %** |

The effect is 2–2.5× larger on stop signs than on aggregate mAP, because truncation costs a fixed
sub-pixel amount and therefore penalises small boxes proportionally more. Since the deployed
controller servos on the stop-sign box, this is the artifact with the most direct mission
relevance — and it is a property of the deployed code, not only of the benchmark.

**4. The INT8 quantization penalty was overstated by roughly 3×.** Same postprocess, same 500
images, `yolo26n`:

| | mAP50-95 | vs FP32 |
|---|---|---|
| CPU FP32 (controlled) | 0.4478 | — |
| NPU INT8 (controlled) | 0.4267 | **−4.7 %** |
| *v1's uncontrolled estimate* | | *−14.2 %* |

About two-thirds of what v1 attributed to quantization was postprocess artifact. This is the
number that was heading for the manuscript against the cited "HailoRT INT8 preserves 0.541
mAP50-95" (Suchy & Turcanik) claim.

**5. Latency is untouched.** Postprocess configuration does not change inference time:
`yolo26n` 27.18 ms vs `yolov8n` 15.30 ms on the NPU — YOLO26 remains **1.78× slower**. Whatever
the corrected accuracy ranking turns out to be, the speed finding stands on its own.

## Cross-check against the full corrected sweep

The ablation predicted the corrections from 500 images; the v2 sweep re-measured them on 5000.
They agree closely, which is the main evidence that the 500-image ablation is not subset noise:

| NPU, conf 0.001 | ablation predicted | v2 measured (5000 img) |
|---|---|---|
| YOLO26 n/s/m | +4.6 % | +4.6 % / +4.8 % / +4.8 % |
| YOLOv8, YOLOv11 (nano) | +1.2 % | +1.0 % / +1.2 % |

## Reproduce

```bash
bash benchmarks/run_postprocess_ablation.sh     # ~15 min, resumable
```

Each cell writes to its own directory because the bench's filename tag encodes the profile but
not `iou`/`max_det`/`coords`, so same-model cells would otherwise overwrite one another.
