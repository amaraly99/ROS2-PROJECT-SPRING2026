# Quantization score shift — the mechanism behind the threshold-dependent penalty

Run 2026-09-16 on the UAE Pi. Offline analysis only: no NPU, no simulator, no
re-inference. Reads two detection dumps that already existed from the v2
controlled sweep.

## The question

`access.tex` reports that YOLO26n's INT8 penalty is 5.1 % at conf=0.001 and
11.2 % at the conf=0.20 operating point, and asserted a mechanism: that
quantization perturbs confidence scores downwards, so the permissive threshold
still admits detections that the operating threshold discards.

That sentence was reasoned from the two mAP numbers. Nothing measured it. This
directory measures it.

## Method

Same model, same images, same detection head, both backends at conf=0.001 so the
full score range is visible on each side:

| | file | detections |
|---|---|---|
| CPU FP32 | `detections_yolo26n_cpu_conf0.001_controlled_raw_tensor.json` | 596,200 |
| NPU INT8 | `detections_yolo26n_npu_conf0.001_controlled.json` | 453,077 |

Both live under `benchmarks/results/accuracy/coco/v2_controlled/`, which is
gitignored, so `benchmarks/quantization_score_shift.py` reads them from there and
only the summary is committed. The CPU run is the 0.4029 cell and the NPU run the
0.3822 cell of the threshold result.

Each CPU detection is matched to the NPU detection of the same category on the
same image by box overlap, greedily, highest score first. The comparison is then
between the scores two backends gave the **same object**. Aggregate score
histograms cannot answer this: they say nothing about which detection moved
where, and on these two files they actively mislead (the NPU has a *higher*
fraction of detections above 0.20, 4.99 % against 4.80 %, purely because it emits
fewer detections overall).

## Result

Quantization moves scores down, and the operating threshold is where that is paid.

```
matched pairs (IoU >= 0.5)              : 330,879
mean   score shift (NPU - CPU)          : -0.01580
median score shift (NPU - CPU)          : -0.00387
fraction of matches scored LOWER on NPU : 0.7870

matched objects above 0.20 on CPU       : 28,281
  ... pushed BELOW 0.20 by INT8         : 6,654  (23.5 % of them)
  ... pulled ABOVE 0.20 by INT8         : 889
CPU detections above 0.20 with no NPU match at all : 309
```

Nearly four in five matched detections score lower after quantization, and close
to a quarter of the objects the FP32 model was confident about fall below the
threshold the deployed pipeline actually uses.

## Two checks

**Robust to the matching parameter.** Sweeping the IoU threshold moves nothing
that matters:

| IoU | matched lower on NPU | pushed below 0.20 | net crossing loss |
|---|---|---|---|
| 0.3 | 78.8 % | 23.8 % | +5,844 |
| 0.5 | 78.7 % | 23.5 % | +5,765 |
| 0.7 | 77.7 % | 22.9 % | +5,556 |

**The accounting closes against a matching-free count.** Counting detections
above 0.20 straight off the two files gives 28,590 on CPU and 22,602 on the NPU, a
difference of 5,988. The matched route gives 6,654 pushed below, plus 309 that
vanished entirely, less 889 pulled above, so 6,074. The two agree to within 1.4 %,
and the CPU side reconciles exactly: 28,281 matched + 309 unmatched = 28,590.

## What this does NOT settle

- **One model.** YOLO26n only. YOLOv8 and YOLOv11 cannot be measured this way on
  this device: their HEFs bake a 0.20 on-device score floor, so no NPU detection
  below the operating threshold ever reaches the host and the crossing is
  structurally unobservable.
- **Not a claim about mAP arithmetic.** This measures where scores move. It does
  not decompose the 5.1 % versus 11.2 % mAP gap into contributions, and no such
  decomposition is claimed.
- **Matching is greedy, not optimal.** A Hungarian assignment would pair a few
  detections differently. The IoU sweep above bounds how much that could matter.
