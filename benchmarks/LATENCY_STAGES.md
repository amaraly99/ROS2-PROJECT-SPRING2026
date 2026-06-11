# Pipeline Latency Stages — Definitions and Measurement Anchors

**Scope:** Pi-side decomposition of the HIL vision pipeline, from MATLAB camera
publish to `/cmd_vel` publish. The `/cmd_vel → MATLAB` return hop is *not*
instrumented (cross-host clock); the paper reports it under a symmetric-link
assumption based on S1.

All Pi-side anchors are `CLOCK_MONOTONIC` ns unless stated otherwise. The
camera frame's SHM-write stamp `t_ns` is the **join key** across every data
source: it is written by `sim_camera_bridge`, carried by the detector as
`ts_capture`, and published by `yolo_bridge` as the detection `header.stamp`.

## Stage table

| Stage | Definition (from → to) | Anchors | Source file | Clock notes |
|-------|------------------------|---------|-------------|-------------|
| S1 | MATLAB publish → Pi DDS receive | `header.stamp` (MATLAB) → callback entry | sim_cam stamp log (`t_sim_stamp_ns`, `t_recv_mono_ns`) | cross-clock: report **rate + delivery jitter**, not absolute latency |
| S2 | DDS receive → camera SHM write | callback entry → `t_ns` | sim_cam stamp log (`t_recv_mono_ns`, `t_shm_write_mono_ns`) | single clock; includes rgb→NV12 conversion |
| S3 | SHM write → detector picks frame up | `t_ns` → `ts_yolo_shm_read` | producer telemetry CSV | includes scheduling wait; dominated by frame-rate vs detector-rate mismatch |
| S4 | preprocess | `ts_yolo_shm_read` → `ts_yolo_preprocess_done` | telemetry CSV | resize/normalise (YOLO), processor (VLM), none (OpenCV) |
| S5 | inference | `ts_inference_start` → `ts_inference_done` | telemetry CSV | NPU HEF / onnxruntime / MOG2+contours / OWL-ViT forward |
| S6 | postprocess | `ts_inference_done` → `ts_nms_done` | telemetry CSV | decode + NMS (YOLO), thresholds (others) |
| S7 | detection SHM write | `ts_nms_done` → `ts_yolo_shm_write` | telemetry CSV | seqlock write + sem_post |
| S8 | det SHM → probe receives `/yolo/detections` | derived: `cam_to_det_ms` − (S3+S4+S5+S6+S7) | probe CSV + telemetry CSV joined on stamp | yolo_bridge wakeup + msg build + DDS hop |
| S9 | detection receive → `/cmd_vel` receive | `t_det_recv_ns` → `t_cmd_recv_ns` (= `approx_e2e_ms`) | probe CSV | controller compute + publish + DDS hop |
| **E2E** | camera SHM write → `/cmd_vel` | `cam_to_det_ms` + `approx_e2e_ms` | probe CSV | full Pi-side closed-loop path |

`cam_to_det_ms` (probe): the probe converts the detection's monotonic
`header.stamp` to wall clock using its own `(REALTIME − MONOTONIC)` offset —
valid because probe and producers share a kernel — and subtracts from its wall
receive time. It therefore equals S3+S4+S5+S6+S7+S8 per detection, measured
independently of the telemetry CSV (the two sources cross-check each other).

## Data sources per run

| File | Producer | Enable with |
|------|----------|-------------|
| `/tmp/sim_cam_stamps.csv` | sim_camera_bridge | `run_stack_hil.sh --stamp-log` (launch arg `cam_stamp_log:=`) |
| `/tmp/yolo_telemetry.csv` | detector producer (all types) | always on in HIL (env `TELEMETRY_CSV` overrides path) |
| probe CSV + `_camera.csv` + `_stats.json` | `benchmarks/e2e_latency_probe.py` | run during measurement window |

## Producing the stage table

```bash
python3 benchmarks/latency_stage_report.py \
    --telemetry /tmp/yolo_telemetry.csv \
    --probe     benchmarks/results/e2e/e2e_latency_<label>.csv \
    --bridge-log /tmp/sim_cam_stamps.csv \
    --out       benchmarks/results/hil/<label>_stages.csv
```

`benchmarks/run_hil_experiments.sh` collects all three files per run and calls
the report automatically.

## Known caveats

- **S1 is cross-clock.** MATLAB's `header.stamp` epoch differs from the Pi's.
  We report the delivery-rate and frame-interval jitter (`Δrecv − Δstamp`),
  which are offset-free. Absolute S1 latency would need PTP/NTP sync between
  the laptop and the Pi.
- **S3 absorbs the rate mismatch.** A detector slower than the camera shows a
  large S3 (it processes the latest frame, skipping older ones). This is by
  design: S3 is "frame age at detector pickup", a real component of E2E age.
- **S9 includes one DDS hop back to the probe**, the same way S8 does forward.
  Controller-internal compute is ~0.15 ms (visp callback instrumentation), so
  S9 is dominated by message transport and controller publish cadence.
- Detections and `/cmd_vel` are paired consecutively (the original probe
  semantics, unchanged) — at equal rates this is 1:1.
