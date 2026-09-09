# Conversion-path control — does the DFC/Model Zoo release version explain YOLO26's penalty?

**Run:** 2026-09-09 · `coco_accuracy_bench.py --backend npu --model-path models/hef/original_june02_hefs/<m>.hef`
**Conditions:** COCO val2017, first 1000 images (`--limit 1000`, deterministic:
`sorted(getImgIds())[:1000]`, so the same subset as every other n=1000 cell),
`controlled` profile (NMS IoU 0.70, max_det 300, float coords), conf 0.20 and 0.001.

## Why

Table 5 reports an INT8 penalty of 10.2–11.0 % for YOLO26 against 0.4–1.7 % for
YOLOv8/YOLOv11. The two HEF groups were built by different Hailo toolchain
versions — the deployed YOLOv8/v11 HEFs carry `sdk-version: 5.1.0` in their
headers, the YOLO26 HEFs `sdk-version: 5.3.0` — so "the 5.3.0 path quantizes
badly" was a live explanation that did not require the architecture to be at
fault.

`models/hef/original_june02_hefs/` holds YOLOv8/v11 HEFs built at **DFC 5.3.0**,
four of the six sharing the **exact git-commit `0dbbb124…` as the YOLO26 HEFs**.
Measuring those isolates the toolchain version on models already known not to be
quantization-fragile — and needs no x86 DFC host, no recompilation, and no
simulator.

## Result — the toolchain version is ruled out

conf = 0.20, 1000 images, identical postprocess:

| Model | CPU FP32 | NPU, MZ v5.1.0 | Δ | NPU, Jun-02 DFC 5.3.0 | Δ |
|---|---|---|---|---|---|
| YOLOv8n | 0.3417 | 0.3402 | −0.4 % | **0.3399** | **−0.5 %** |
| YOLOv11n | 0.3647 | 0.3584 | −1.7 % | **0.3568** | **−2.2 %** |
| YOLO26n | 0.3762 | 0.3378 | — | — | **−10.2 %** |

Models that are not fragile stay at −0.5 % and −2.2 % through the same compiler
build that produced the YOLO26 HEFs. **The conversion path does not explain the
YOLO26 result**, and the manuscript should not offer it as the explanation.

## What is still conflated

Two candidates remain, and they cannot be separated with artifacts available for
this device:

1. **Architecture** — YOLO26's DFL-free regression head and NMS-free training.
2. **Export mode** — the YOLO26 HEFs emit raw feature-map tensors decoded on the
   host, while every YOLOv8/v11 HEF performs NMS on-device. There is no
   on-device-NMS YOLO26 build and no raw-tensor YOLOv8/v11 build for hailo10h, so
   no measurement here holds export mode fixed across families.

State this as a bounded limitation, not as an architecture finding.

## Side result — censoring confirmed on a second, independent HEF set

conf 0.001 reproduced conf 0.20 exactly for both models (0.3399 and 0.3568), so
the June-02 builds bake the same 0.20 on-device score floor as the Model Zoo
ones. This independently corroborates the censored-lower-bound caveat on
Table 4, which until now rested on a single HEF set.
