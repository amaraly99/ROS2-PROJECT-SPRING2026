# Standalone detector benchmarks (Table 2, CPU rows)

One JSON per (model, backend, thread-config) cell, produced by
`benchmarks/standalone_yolo_benchmarker.py` on the Raspberry Pi 5 (Cortex-A76, 4 cores).

Protocol, identical for every row:

    python3 benchmarks/standalone_yolo_benchmarker.py \
        --backend cpu --model_id <id> --threads 2 --inter_threads 1 \
        --frames <N> --warmup <W> --image benchmarks/assets/bus.jpg \
        --json benchmarks/paper_data/standalone/bench_<id>_cpu_t2.json

All rows use ONNX Runtime with `intra_op=2`, `inter_op=1` and thread spinning disabled
(see `src/yolo_producer/yolo_runtime_engine.py`), the registry ONNX export
(`models/model_registry.json`), and a 640x640 network input letterboxed from
`benchmarks/assets/bus.jpg` (810x1080).

## Provenance

`bench_yolo26n_cpu_t2.json` and `bench_yolo26s_cpu_t2.json` were re-measured on
2026-09-09. The figures previously carried in Table 2 for these two models
(179.8 / 183.9 ms and 472.6 / 476.7 ms) came from a different harness
(`benchmarks/yolo_cpu_bench.sh` -> `src/yolo_producer/yolo_cpu_producer.py`), which loads
through Ultralytics `YOLO()` without configuring ONNX Runtime `SessionOptions` — so its
thread count was the ORT default, not 2 — and used a separately exported ONNX
(`models/yolo26n.onnx`) rather than the registry artifact. Those rows were therefore not
comparable to the other seven. They have been replaced.

The remaining seven JSONs are the original measurement artifacts, unchanged.

## Harness control

Before re-measuring, `yolov8n` was re-run under the current harness and image to confirm
it reproduces its archived row: inference 214.8 ms vs 216.5 ms archived, end-to-end
223.9 ms vs 226.0 ms (within 1%). The seven unchanged JSONs are therefore comparable to
the two new ones.

## Known inconsistency in the archived rows

`bench_yolov8n_cpu_t2.json` was measured at `conf 0.25`; every other row used `conf 0.20`.
This affects the postprocess stage only (~2.6 ms of a 224 ms end-to-end) and does not
change the ranking, but the Table 2 caption should not claim a uniform threshold.
