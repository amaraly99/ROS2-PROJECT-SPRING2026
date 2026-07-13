# Benchmarks

Everything used to measure the stack. For running the stack itself, see the
[root README](../README.md).

The paper's dataset and the scripts that regenerate its tables live in
[`paper_data/`](paper_data/) — start there if you want to check our numbers rather than collect new
ones.

## Reproducing the detector sweep

Requires the HIL rig: MATLAB/Simulink on a sim host driving a simulated drone, the Pi closing the
loop. Bring up the sim host first ([`matlab/README.md`](matlab/README.md)), then from the Pi:

```bash
./benchmarks/sweep_detectors.sh                    # all 20 configs x 3 runs (several hours)
./benchmarks/run_hil_experiments.sh --label smoke --runs 1 --duration 30   # one config
```

Each run resets the simulation (drone back to its starting pose), brings the stack up with the
requested detector, collects for `--duration` seconds, and tears down.

### The config matrix

20 configurations: **9 models** (YOLOv8 / YOLOv11 / YOLO26, each n/s/m) × **2 backends** (Hailo NPU
via HEF, CPU via ONNX Runtime), plus **OpenCV MOG2** and **OWL-ViT**. Models resolve through
[`models/model_registry.json`](../models/model_registry.json).

Two things are easy to get wrong here:

- **Confidence must be 0.20 for every config.** The v8/v11 HEFs bake a 0.20 NMS score threshold into
  the on-device op and cannot be raised at runtime, so any higher CPU-side threshold makes the
  backend comparison unfair.
- **CPU detectors need two cores** (`--detector-core 0,1`). ONNX Runtime uses two intra-op threads;
  pinning it to one core roughly halves throughput and changes the conclusion.

## Output

Per run, `{label}_run{N}`:

| File | Contents |
|---|---|
| `_telemetry.csv` | per-frame producer stamps S1–S7 (acquire → preprocess → infer → NMS → SHM write) |
| `{prefix}.csv` | e2e probe: detection publish (S7–S8), control (S9), `/cmd_vel` arrivals |
| `_cam_stamps.csv` | MATLAB sim clock paired with the Pi's monotonic clock |
| `_visp.csv` | per-frame servo state and centroid error |
| `_mission.json` | derived mission outcome (reach time, approach duration, re-acquisitions) |
| `_stages.csv`, `_stats.json`, `{label}_summary.json` | per-run and per-config aggregates |

Stage definitions are in [`LATENCY_STAGES.md`](LATENCY_STAGES.md).

## Post-processing

```bash
python3 benchmarks/mission_from_visp_csv.py --sweep <results-dir>   # missions from the ViSP CSV
python3 benchmarks/plot_detector_sweep.py            <results-dir>   # detector_comparison.csv + figures
python3 benchmarks/make_paper_tables.py --results-dir <d> --out-dir <o>   # the LaTeX tables
```

**Mission time is in simulation time, never wall-clock.** MATLAB's sim rate drifts 0.78–0.95× of
real time between runs, which stretched identical missions from 43.7 s to 54.8 s of wall-clock and
manufactured a ~10 s "detector effect" that does not exist. Use `time_to_reached_sim_s`;
`time_to_reached_s_wall` exists for transparency only and must not be compared across configs.

## Other harnesses

| Script | Measures |
|---|---|
| `e2e_latency_probe.py` | end-to-end latency, camera → `/cmd_vel` |
| `ipc_latency_bench.py` | POSIX shm seqlock vs DDS for 921 KB frames |
| `standalone_yolo_benchmarker.py`, `hef_bench.py` | detector throughput, no ROS in the loop |
| `contention_bench.sh`, `core_pinning_bench.sh` | CPU contention and core-affinity effects |
| `yolo_cpu_bench.sh`, `yolo_resolution_bench.sh`, `yolo_nms_comparison.sh` | CPU backends, input resolution, NMS implementations |
| `run_benchmark.sh` + `aggregate_slam_results.py` | OV²SLAM accuracy/latency on EuRoC — see [docs/EUROC.md](../docs/EUROC.md) |
