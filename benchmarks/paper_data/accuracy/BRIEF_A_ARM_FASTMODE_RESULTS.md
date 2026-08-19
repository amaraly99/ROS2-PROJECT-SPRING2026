# Brief A — YOLO26 NMS-free ("fast mode") A/B on ARM

**Date:** 2026-08-19 · **Host:** pi (RPi5, BCM2712 Cortex-A76) · **Branch:** `exp/detector-accuracy`
**Runtime:** onnxruntime 1.24.4, `intra_op=2, inter_op=1, allow_spinning=0`, `taskset -c 1,2`
**Input:** `benchmarks/assets/stop_sign_test.jpg`, 640x640 · **conf:** 0.20 (deployed operating point)
**Repeats:** yolo26n n=4; yolo26s / yolo26m n=1 (variance shown to be negligible on n)

## Arms

| Arm | Export | Host work |
|---|---|---|
| `e2e` | native End2End, single `(1,300,6)` | confidence threshold + rescale only |
| `one2many` | conventional, 6 raw conv tensors, 8400 anchors | sigmoid over 80x8400 + decode + `cv2.dnn.NMSBoxes` |

Both arms share identical ONNX Runtime configuration, so the delta is attributable to the head
and its postprocess, not to runtime settings.

## Results

| Model | Arm | Inference (ms) | Postprocess (ms) | Total (ms) |
|---|---|---|---|---|
| YOLO26n | NMS-free (deployed) | 164.43 | **0.103** | 167.72 |
| YOLO26n | one-to-many + NMS | 159.35 | **8.133** | 170.57 |
| YOLO26s | NMS-free (deployed) | 482.49 | 0.103 | 485.80 |
| YOLO26s | one-to-many + NMS | 476.71 | 8.055 | 487.86 |
| YOLO26m | NMS-free (deployed) | 1412.66 | 0.106 | 1415.97 |
| YOLO26m | one-to-many + NMS | 1404.67 | 8.220 | 1415.96 |

| Model | Postprocess saved | Inference penalty | **Net saved** | Net % of frame |
|---|---|---|---|---|
| YOLO26n | 8.03 ms | 5.07 ms | **2.85 ms** | **1.67%** |
| YOLO26s | 7.95 ms | 5.78 ms | **2.06 ms** | **0.42%** |
| YOLO26m | 8.11 ms | 7.99 ms | **-0.01 ms** | **0.00%** |

Output volume: **1,800 vs 705,600** values/frame (392x reduction).

## Findings

**1. The NMS-free head does not delete the work — it relocates it into the graph.**
Postprocess collapses by ~8 ms (79x, 8.13 -> 0.103 ms), but inference *rises* by 5-8 ms because
the top-k selection now runs in-graph. Roughly 63-99% of the postprocess saving is clawed back.
This is a stable effect, not noise: across 4 repeats of yolo26n the two inference distributions
do not overlap (e2e 164.43+/-0.33, one2many 159.35+/-0.36).

**2. The net benefit shrinks to zero as the model grows.** The postprocess saving is *constant*
at ~8 ms — it is set by the anchor count (8400), which depends on input resolution, not model
size. The in-graph selection penalty *grows* with model size (5.07 -> 5.78 -> 7.99 ms). At
YOLO26m the two arms are identical within noise (-0.01 ms). **On ARM CPU the NMS-free head's
advertised benefit is real but marginal at nano scale and entirely absent at medium scale.**

**3. This sharpens, rather than contradicts, the existing claim.** The paper already argues NMS
was never the bottleneck. The ARM measurement gives the mechanism: even the full 8 ms
postprocess is only ~4.8% of a 168 ms nano frame, and over half of it returns as inference cost.
The remaining >95% is FLOPs-bound convolution that no head redesign touches.

**4. The x86 reference did not surface the inference penalty.** x86 reported postprocess
6.509 -> 0.062 ms and inference "~27-28 ms either way". On ARM the inference penalty is
unambiguous. Quoting the x86 figures as evidence of a net speedup would overstate the benefit.

## Cross-checks against the published standalone table

| Model | Published CPU (ms) | Measured here, e2e total (ms) |
|---|---|---|
| yolo26n | 179.8 | 167.72 |
| yolo26s | 472.6 | 485.80 |
| yolo26m | 1410.0 | 1415.97 |

s and m agree within 1-3%, validating the harness. **yolo26n is 7% faster here (167.7 vs 179.8)**
— most likely core pinning (`taskset -c 1,2` here; the 2026-06-25 batch's pinning is not
recorded in the CSV). Flagged rather than resolved; it does not affect the A/B, which is an
internal comparison under identical conditions.

## Reproduce

```bash
taskset -c 1,2 python3 benchmarks/yolo26_cpu_fastmode_ab.py \
  --weights models/onnx/yolo26n/yolo26n.pt \
  --deployed-onnx models/onnx/yolo26n/yolo26n.onnx \
  --tag yolo26n --frames 50 --warmup 5 \
  --image benchmarks/assets/stop_sign_test.jpg \
  --out-dir benchmarks/results/accuracy
```

Raw CSVs: `benchmarks/paper_data/accuracy/yolo26_fastmode_ab_*.csv`
LaTeX: `benchmarks/paper_data/accuracy/tables/table_fastmode_ab.tex`
