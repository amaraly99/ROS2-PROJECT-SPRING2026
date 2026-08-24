# COCO accuracy sweep — v1, SUPERSEDED

**Run:** 2026-08-19 22:29 → 2026-08-20 01:27 · 30/30 configs, no failures
**Status:** **Do not quote these numbers.** Retained for provenance and as the baseline
arm of the postprocess-artifact ablation. Superseded by the v2 sweep.

`coco_v1_all_runs.csv` is all 30 rows concatenated; the per-config CSVs are the originals.

## What is still valid

**The censoring experiment.** All six YOLOv8/v11 NPU configs return *digit-identical* mAP at
`conf=0.001` and `conf=0.20`:

| model | mAP50-95 @0.001 | mAP50-95 @0.20 |
|---|---|---|
| yolov8n  | 0.3183 | 0.3183 |
| yolov8s  | 0.3955 | 0.3955 |
| yolov8m  | 0.4447 | 0.4447 |
| yolov11n | 0.3324 | 0.3324 |
| yolov11s | 0.4046 | 0.4046 |
| yolov11m | 0.4501 | 0.4501 |

The HEFs bake a 0.200 on-device score floor (`hailortcli parse-hef`: `Score threshold: 0.200`,
`IoU threshold: 0.70`), so the host can only filter *further*, never recover what on-device NMS
already discarded. YOLO26's raw-tensor export genuinely sweeps (n: 0.3653 → 0.3061). **The
equality is the measurement, not a bug.** These are censored lower bounds and are not comparable
to published mAP tables.

This finding is unaffected by the defect below — it is an equality between two runs through the
*same* path, so any shared bias cancels.

## Why the rest was re-run

The four backend×family postprocess paths were never controlled against each other:

| Path | Postprocess | NMS IoU | Box coords | Det cap |
|---|---|---|---|---|
| **YOLO26 / NPU** | `yolo_producer.postprocess()` | **0.45** (host) | `int()` truncated **twice** | **64** |
| v8,v11 / NPU | `yolo_producer.postprocess_ondevice_nms()` | **0.70** (baked in HEF) | `int()` once | **64** |
| YOLO26 / CPU | engine `_postprocess_e2e` | n/a — in-graph top-300 | float | uncapped |
| v8,v11 / CPU | engine `_postprocess_anchors` | **0.45** (host) | float | uncapped |

Note the split: the **NPU** paths call into `yolo_producer.py`; the **CPU** paths use the engine's
own implementations in `yolo_runtime_engine.py`. The two were never reconciled.

1. **NMS IoU 0.45 vs 0.70 — YOLO26/NPU only.** `yolo_runtime_engine.py:185` defaults `iou=0.45`;
   the v8/v11 HEFs bake 0.70 (`hailortcli parse-hef`). Stricter NMS suppresses true positives.
   Ultralytics' published mAP — our external reference — also uses 0.7. This is a direct,
   uncontrolled handicap on exactly the cell where v1 reported YOLO26 losing.

2. **`YOLO_MAX_DETS = 64`** (`yolo_producer.py:61`) is applied by **both** NPU postprocess
   functions (`:501` and `:571`) and by **neither** CPU path. In code it is therefore an
   NPU-vs-CPU asymmetry, not a family one — but *in effect* it binds almost exclusively on
   YOLO26/NPU, because on-device censoring has already stripped v8/v11 to ~7 det/img:

   | NPU @ conf 0.001 | det/img | 64-cap binds? |
   |---|---|---|
   | yolo26 n/s/m | 36.8 / 34.0 / 30.7 | yes, on a large fraction of images |
   | yolov8 n/s/m | 6.4 / 7.4 / 7.8 | effectively never |
   | yolov11 n/s/m | 6.3 / 7.2 / 7.4 | effectively never |

   COCO evaluates at maxDets=100, so the cap truncates the tail of the precision/recall curve for
   YOLO26/NPU and not for its competitors. For scale, the same yolo26n emits **143.4 det/img** on
   CPU at the same threshold.

3. **Integer box truncation on the NPU paths only** — `int()` truncates rather than rounds, a
   systematic sub-pixel inward bias that hits mAP75/mAP50-95 hardest; CPU paths keep floats. This
   applies to both families equally, so it does not distort the family *ranking*, but it does
   contaminate every NPU-vs-CPU comparison — the INT8-vs-FP32 quantization delta is therefore in
   part a postprocess artifact rather than a property of quantization.

A fourth, separate confound: **NPU ran 5000 images, CPU ran 1000**, so every cross-backend
comparison in v1 is also subset-mismatched.

Evidence this is harness rather than model: **yolo26n on CPU FP32 scores 0.4247, above
Ultralytics' ~0.40 reference.** The ONNX weights and preprocessing are sound; the deficit is
NPU-path-specific. Preprocessing *was* properly controlled — identical letterbox (pad=114,
INTER_LINEAR, centre-pad), 640×640 on all nine models, per-class NMS everywhere.

**These CSVs cannot tell you which NMS IoU or detection cap produced them** — v1 recorded neither.
That omission is why the defect went unnoticed, and v2 records `iou`, `max_det`, `coord_mode` and
`profile` as columns.

## v2 replacement

Two profiles, each applied **equally** to all nine models on both backends:

- **`controlled`** — IoU 0.70, max_det 300, float coords. Literature-comparable.
- **`deployed`** — IoU 0.45, max_det 64, int coords. What the robot actually runs.

The gap between them is itself a result: it is the paper's thesis — component benchmarks
mispredicting deployed behaviour — showing up in the postprocess configuration.
