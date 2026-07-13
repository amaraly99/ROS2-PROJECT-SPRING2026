# yolo_producer

The pluggable detector stage. Reads camera frames from `/ovcam_frames`, runs a detector, and writes
detections (plus an optional annotated BGR image) to `/yolo_shm`.

**Runs natively on the host** — it talks to the Hailo NPU at `/dev/hailo0`.

## Backends

| Producer | Detector |
|---|---|
| `yolo_shm_producer.py` | YOLO on the Hailo-10H NPU (HEF) or on the CPU (ONNX Runtime). This is the one the stack uses. |
| `opencv_detector_producer.py` | OpenCV MOG2 background subtraction |
| `vlm_detector_producer.py` | OWL-ViT open-vocabulary detection |
| `yolo_cpu_producer.py` | CPU-only YOLO, for standalone benchmarking |

All four implement `detector_producer_base.py`, so they are interchangeable from the launcher's point
of view. `yolo_runtime_engine.py` holds the model-agnostic inference/postprocess logic and resolves
models through [`models/model_registry.json`](../../models/model_registry.json).

## Run

```bash
pip3 install hailort-*.whl --break-system-packages     # from the Hailo Developer Zone
bash models/fetch_models.sh                            # models are not committed

taskset -c 1 python3 src/yolo_producer/yolo_shm_producer.py --backend npu --conf 0.20
```

| Flag | Default | Effect |
|---|---|---|
| `--backend` | `npu` | `npu` (Hailo HEF) or `cpu` (ONNX Runtime) |
| `--conf` | `0.20` | Confidence floor — see below before changing this |
| `--iou` | `0.45` | NMS IoU threshold |
| `--threads` | | ONNX Runtime intra-op threads (CPU backend) |
| `--no-image` | | Skip writing the annotated frame (saves bandwidth) |

Normally you don't invoke this directly — `run_stack.sh` and `run_stack_hil.sh --model/--backend/--detector`
do it for you.

## Two things that will bite you

**Do not raise `--conf` above 0.20 when comparing backends.** The YOLOv8/v11 HEFs bake a 0.20 NMS
score threshold into the on-device op and it cannot be raised at runtime. A higher CPU-side threshold
would discard detections that the NPU path is still handed, making the comparison meaningless.

**The Hailo NMS-BY-CLASS output is variable-length per class.** Each class's detection block is
preceded by its own count. Parsing it with a fixed stride appears to work — it yields plausible
`person` detections — while silently dropping every other class. This looked like a miscalibrated
HEF for weeks; it was a parser bug.

## Timing

Seven timestamps per frame are written into the SHM header's `_reserved[8]` (stages S1–S7: acquire,
pickup, preprocess, inference, NMS, SHM write). `benchmarks/latency_stage_report.py` joins them with
the servo's own stamps to produce the end-to-end breakdown. See
[`benchmarks/LATENCY_STAGES.md`](../../benchmarks/LATENCY_STAGES.md).
