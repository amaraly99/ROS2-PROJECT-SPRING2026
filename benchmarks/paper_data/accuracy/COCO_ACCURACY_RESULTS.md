# Detector accuracy on COCO — corrected sweep (v2)

**Run:** 2026-08-25 · 61 runs · `benchmarks/run_coco_sweep.sh {controlled,deployed}`
**Data:** `coco_v2_all_runs.csv` (every run, one row, postprocess config recorded per row)
**Supersedes:** `coco_v1_superseded/` — see that README for the defect that forced the re-run.
**Companion:** `POSTPROCESS_ABLATION_RESULTS.md` attributes the v1 error to specific causes.

Platform: Raspberry Pi 5 (BCM2712, 4x Cortex-A76) + Hailo-10H NPU. COCO val2017.
ONNX Runtime 2 intra-op / 1 inter-op, unpinned. Accuracy is deterministic (fixed weights,
fixed images, no sampling), so these are exact values, not means — no error bars apply.

---

## The headline result

**The same nine models, on the same hardware, rank in opposite orders depending on whether you
measure them as components or as deployed.**

**A — component benchmark** (CPU FP32, `conf=0.001`, standard mAP50-95):

| rank | model | mAP50-95 |
|---|---|---|
| 1 | **yolo26n** | **0.4247** |
| 2 | yolov11n | 0.4111 |
| 3 | yolov8n | 0.3889 |

**B — deployed operating point** (NPU INT8, `conf=0.20`, full val2017, postprocess held identical
across all nine):

| scale | 1st | 2nd | 3rd |
|---|---|---|---|
| n | yolov11n 0.3364 | yolov8n 0.3216 | **yolo26n 0.3124** |
| s | yolov11s 0.4111 | yolov8s 0.4015 | **yolo26s 0.3838** |
| m | yolov11m 0.4570 | yolov8m 0.4524 | **yolo26m 0.4258** |

YOLO26 is **first by the component benchmark and last of three at deployment, at every scale.**
This is the paper's thesis — component-level benchmarks systematically mispredicting system-level
behaviour — extended from throughput to accuracy, and it survives a postprocess-controlled
re-measurement designed specifically to destroy it if it were an artifact.

---

## Is YOLO26 "as advertised"? — No, on this hardware.

Four independent measurements, each controlled:

**1. It is less accurate at the operating point it actually runs at.** 2.9-8.9 % relative below
the best competitor at every scale (table B). The gap narrowed after correcting the v1 harness
defect — from -3.8 % to -2.9 % for nano — but did not close or reverse.

**2. It is 1.55-1.77x slower on the NPU.** Inference only; postprocess configuration cannot
change this:

| scale | YOLO26 | YOLOv8 | YOLOv11 |
|---|---|---|---|
| n | 27.16 ms (1.77x) | **15.34 ms** | 16.09 ms |
| s | 30.30 ms (1.63x) | **18.62 ms** | 19.53 ms |
| m | 38.63 ms (1.55x) | **25.00 ms** | 26.89 ms |

**3. It quantizes 7-8x worse to INT8.** Architecture-matched (see below), budget-matched,
identical postprocess — precision is the only variable:

| | CPU FP32 -> NPU INT8 |
|---|---|
| yolo26 n/s/m | **-10.2 % / -11.0 % / -10.3 %** |
| yolov8 n/s/m | -0.4 % / -1.2 % / -0.5 % |
| yolov11 n/s/m | -1.7 % / -0.5 % / -1.3 % |

The flatness across scales is what distinguishes this from a measurement artifact.

**4. Its headline NMS-free feature is a losing trade here — and is absent from the NPU entirely.**
Same weights, same postprocess, only the head differs:

| | end-to-end (NMS-free) | one2many (conventional) | cost of NMS-free |
|---|---|---|---|
| yolo26n | 0.3566 | 0.3762 | **-5.2 %** |
| yolo26s | 0.4292 | 0.4528 | **-5.2 %** |
| yolo26m | 0.4887 | 0.5061 | **-3.4 %** |

Brief A measured what that buys on ARM: +2.85 ms on nano (1.67 % of frame time), +2.06 ms on
small (0.42 %), and -0.01 ms on medium — nothing. **So the feature costs 3.4-5.2 % mAP to save at
most 1.7 % of a frame, converging to zero benefit by medium scale.**

On the NPU it is not present at all: the HEF is compiled from the conventional dense head
(`hef_postproc: raw_tensor`, six raw conv tensors), while the registry ONNX is the end-to-end
export. The advertised feature does not survive the Hailo toolchain.

---

## Methodological findings

These matter more than the rankings, and several are reusable beyond this project.

### The censoring experiment — confirmed on the full dataset

YOLOv8/v11 HEFs bake a `0.200` score floor and `IoU 0.70` into on-device NMS
(`hailortcli parse-hef`). The host can filter *further* but cannot recover what the device already
discarded. So their mAP at `conf=0.001` must equal their mAP at `conf=0.20` — and does, to four
decimals, for all six configs on the full 5000 images:

| model | @0.001 | @0.20 |
|---|---|---|
| yolov8 n/s/m | 0.3216 / 0.4015 / 0.4524 | 0.3216 / 0.4015 / 0.4524 |
| yolov11 n/s/m | 0.3364 / 0.4111 / 0.4570 | 0.3364 / 0.4111 / 0.4570 |

YOLO26's raw-tensor export genuinely sweeps (n: 0.3822 -> 0.3124), because its threshold is
applied on the host.

**The equality is the measurement, not a bug.** Consequences for reporting:

- YOLOv8/v11 NPU numbers are **censored lower bounds**. They are not comparable to published mAP
  tables, which are computed at `conf=0.001`.
- A standard-mAP comparison between the families is **structurally unobtainable on this NPU**. The
  only honest cross-family comparison is at the deployed operating point, which is table B.
- Passing `--conf 0.001` to a v8/v11 NPU config does not error. It silently returns a truncated
  number that looks plausible.

### Architecture matching, not just parameter matching

A CPU-vs-NPU comparison that fixes `conf`, `iou` and `max_det` can still be confounded if the two
backends run different graphs. For YOLO26 they do: end-to-end on CPU, dense raw-tensor on NPU.
Measuring the quantization penalty against the registry ONNX gave **-5.3 %/-6.2 %/-7.1 %**;
against an architecture-matched export it is **-10.2 %/-11.0 %/-10.3 %**. The head change was
masking roughly half the quantization loss.

`benchmarks/export_yolo26_raw_tensor.py` builds the matching export; the CPU engine gained a
`raw_tensor` layout that routes through the *same* `yolo_producer.postprocess` the NPU path uses,
so matched runs share the decode code and not merely its settings.

### Subset bias is real and one-directional

Nine models, `conf=0.20`, NPU, same postprocess — 1000-image subset vs full 5000:

| | subset bias |
|---|---|
| yolo26 n/s/m | +8.1 % / +5.0 % / +6.6 % |
| yolov8 n/s/m | +5.8 % / +7.1 % / +8.2 % |
| yolov11 n/s/m | +6.5 % / +8.9 % / +6.0 % |

`sorted(getImgIds())[:1000]` is deterministic and reproducible but **not a random sample**, and it
is consistently optimistic by 5-9 %. It does not distort *rankings* — the bias is common to all
nine — but any absolute mAP quoted from a 1000-image run is 5-9 % high. **Every CPU number in this
report is from that subset; every cross-backend comparison here is subset-matched.**

### Controlled vs deployed postprocess

The deployed postprocess (`IoU 0.45`, `max_det 64`, integer box coordinates) costs 1.0-2.4 %
relative against the controlled one (`IoU 0.70`, `max_det 300`, float coordinates), consistently
across all nine models. Small, but it is the difference between a literature-comparable number and
a real one, and reporting either without saying which is how v1 went wrong.

Note the deployed profile is *faithful* but *not* a controlled cross-family comparison: it applies
host NMS at `IoU 0.45` to YOLO26 while YOLOv8/v11 keep their baked `0.70`. That asymmetry is real
— the robot genuinely runs that way — so the deployed profile answers "what does the robot
achieve", and the controlled profile answers "which detector is better". Table B uses the
controlled profile for exactly this reason.

### External validation

`yolo26n`, CPU FP32, full val2017, standard mAP: **0.4022**, against Ultralytics' published ~0.40
for the same model. The harness agrees with a reference outside this project to within ~0.5 %.

---

## Stop-sign AP — the mission-critical class

The controller servos on this class, so its AP matters more than aggregate mAP.
NPU, `conf=0.20`, controlled, full 5000:

| scale | YOLO26 | YOLOv8 | YOLOv11 |
|---|---|---|---|
| n | 0.5251 | **0.5779** | 0.5403 |
| s | 0.5188 | **0.6411** | 0.6183 |
| m | 0.6190 | 0.6536 | **0.6579** |

YOLO26 is weakest on stop signs at every scale, and the gap (up to 23.6 % relative at small) is
wider than its aggregate-mAP gap. Separately, the ablation found integer box truncation costs
stop-sign AP 2-2.5x more than aggregate mAP, because a fixed sub-pixel bias penalises small boxes
proportionally more — that one is a property of the *deployed* code, not only the benchmark.

---

## Reproduce

```bash
bash benchmarks/run_coco_sweep.sh controlled   # 40 configs
bash benchmarks/run_coco_sweep.sh deployed     # 18 configs
python3 benchmarks/export_yolo26_raw_tensor.py # matched-head export
python3 benchmarks/aggregate_coco_v2.py
```

Both sweeps are resumable and skip completed configs. Runs are deliberately **unpinned** so CPU
latency stays comparable to the published standalone table, whose pinning is unrecorded.

## Caveats to state in the paper

1. YOLOv8/v11 NPU accuracies are censored lower bounds, not standard mAP.
2. All CPU runs use a 1000-image subset, which is optimistic by 5-9 %; cross-backend comparisons
   are subset-matched, absolute CPU values are not directly comparable to full-set numbers.
3. The 1000-image subset is deterministic (`sorted(getImgIds())[:1000]`), not random.
4. COCO is real-world photography; the robot sees Unreal Engine renders. The domain gap is
   unmeasured here — that is the recorded-bag replay experiment, still outstanding.
5. These are standalone numbers. They cannot capture co-scheduling contention, which requires
   live HIL.
