# slam_eval — per-SLAM HIL evaluation

Evaluate one HIL SLAM benchmark run: align the online SLAM trajectory to ground
truth, compute **ATE (APE RMSE) with [evo](https://github.com/MichaelGrupp/evo)**
(the same tool the offline ORB-SLAM2 / OV2SLAM studies used), read the timing/CPU
sidecar CSVs, and draw the plots in the visual language of `SLAM_benchmark/images/`.

## Run it

```bash
# via the shim (unchanged usage):
python3 benchmarks/eval_slam_hil.py bags/run_ov2slam_oracle_fast_<stamp>

# or directly:
benchmarks/.evo_venv/bin/python -m slam_eval bags/run_<config>_<stamp> [--slam <type>]
```

The SLAM backend is read from `--slam` or the run's `meta.txt` (`slam_type=`).
For OV2SLAM, fast vs accurate is inferred from the config name (`config=` in
`meta.txt`, e.g. `ov2slam_oracle_fast`).

## One-time setup (the evo venv)

evo can't be installed system-wide on the Pi (PEP 668), so it lives in a venv:

```bash
python3 -m venv benchmarks/.evo_venv
benchmarks/.evo_venv/bin/pip install evo rosbags matplotlib numpy
```

## Outputs (into the run dir)

`slam_metrics.csv`, `gt_traj.tum`, `slam_traj.tum`, and
`fig_slam_{trajectory,rmse,frontend_bar,frontend_timing,cpu_total,cpu_per_thread}.png`.

## Layout

| file | role |
|------|------|
| `base.py` | `SlamEvaluator` — all shared logic (bag→evo ATE→plots→CSV). **Read its header.** |
| `orbslam2.py` | `Orbslam2Evaluator` — 6 lines of ORB-SLAM2 specifics |
| `ov2slam.py` | `Ov2slamEvaluator` — OV2SLAM specifics + fast/accurate mode |
| `__main__.py` | dispatch by `slam_type`; `SLAM_REGISTRY` is where you register a new backend |

## Add a new SLAM backend

1. New file (e.g. `rtabmap.py`), subclass `SlamEvaluator`, set the `OVERRIDE`
   class attributes (label, colour, timing CSV name + category).
2. Register it in `__main__.py`'s `SLAM_REGISTRY`.

You do not need to touch any bag/ROS/evo/plot code — that's all in `base.py`.
