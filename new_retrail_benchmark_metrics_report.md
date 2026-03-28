# OV2SLAM `new_retrail_benchmark` Metrics Report

This document summarizes the latest run stored under:

- [new_retrail_benchmark](/home/amaraly/ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark/new_retrail_benchmark)

## Summary Table

| Sequence | APE RMSE (m) | RPE RMSE (m) | FE mean (ms) | FE approx (Hz) | Trajectory | Matched pairs |
|---|---:|---:|---:|---:|---|---|
| MH_01_easy | 0.039595 | 0.032183 | 21.2251 | 47.11 | `ov2slam_kfs_traj.txt` | 392 / 399 (98.25%) |
| MH_02_easy | 0.041421 | 0.027524 | 21.9419 | 45.57 | `ov2slam_kfs_traj.txt` | 411 / 418 (98.33%) |
| MH_03_medium | 0.050564 | 0.033797 | 25.0877 | 39.86 | `ov2slam_kfs_traj.txt` | 519 / 523 (99.24%) |
| MH_04_difficult | 0.067078 | 0.028454 | 18.4407 | 54.23 | `ov2slam_kfs_traj.txt` | 338 / 341 (99.12%) |
| MH_05_difficult | 0.060975 | 0.029302 | 19.4147 | 51.51 | `ov2slam_kfs_traj.txt` | 356 / 360 (98.89%) |
| V1_01_easy | 0.056415 | 0.012799 | 22.8416 | 43.78 | `ov2slam_kfs_traj.txt` | 409 / 410 (99.76%) |
| V1_02_medium | 0.066436 | 0.022159 | 28.6814 | 34.87 | `ov2slam_kfs_traj.txt` | 569 / 570 (99.82%) |
| V1_03_difficult | 0.085580 | 0.031842 | 23.1675 | 43.16 | `ov2slam_kfs_traj.txt` | 776 / 781 (99.36%) |
| V2_01_easy | 0.075830 | 0.031940 | 20.4247 | 48.96 | `ov2slam_kfs_traj.txt` | 366 / 369 (99.19%) |
| V2_02_medium | 0.074830 | 0.022986 | 24.9365 | 40.10 | `ov2slam_kfs_traj.txt` | 773 / 776 (99.61%) |

## Evaluation Process

### 1. Ground truth used

For each sequence, the benchmark runner validates the EuRoC ground-truth file and writes a normalized TUM-format copy as `gt.tum`.

The pose format used is:

```text
timestamp tx ty tz qx qy qz qw
```

This is the same format expected by `evo_ape tum`.

### 2. Estimated trajectory used

This run used the `paper` trajectory policy, so the benchmark selected:

- `ov2slam_kfs_traj.txt`

for every sequence in this folder.

That means the evaluation here is being done on the timestamped keyframe trajectory exported by OV2SLAM, not on:

- `ov2slam_traj.txt`
- `ov2slam_fullba_kfs_traj.txt`
- `ov2slam_full_traj_wlc*.txt`

The last family is excluded because in this codebase those files are not written with real timestamps in column 1, so they are not valid direct `evo_ape tum` inputs against EuRoC GT.

### 3. Timestamp association

For each sequence, `evo` first synchronizes the GT and estimated trajectories by timestamp.

The benchmark uses:

```text
--t_max_diff 0.1
```

So an estimated pose is eligible to match a GT pose only if the timestamp difference is at most `0.1 s`.

The `Matched pairs` column in the summary table means:

- `found / max`

where:

- `found` = number of timestamp associations `evo` actually used
- `max` = the maximum possible number of matches on the estimated trajectory side
- `%` = `found / max * 100`

Example:

- `392 / 399 (98.25%)`

means `evo` successfully associated 392 estimated poses out of 399 possible estimated poses for that sequence.

### 4. Alignment used before APE

For stereo runs, this benchmark uses:

```text
evo_ape tum GT EST --verbose --align --t_max_diff 0.1
```

Important details:

- `--align` applies SE(3) Umeyama alignment
- `--correct_scale` is not used for stereo
- so the scale is fixed at `1.0` for these runs

This means the estimated trajectory is rigidly aligned to GT by rotation and translation only, with no scale correction.

### 5. APE value in the table

`APE RMSE (m)` in the table is taken directly from each sequence's saved `*_evo.txt`.

It is therefore the same number reported by `evo_ape` after:

1. timestamp association
2. SE(3) Umeyama alignment
3. APE translation-part computation

### 6. RPE value in the table

`RPE RMSE (m)` in the table is translational RPE recomputed from the saved GT and estimated trajectories using the same high-level logic as the benchmark plot generation:

1. load `gt.tum`
2. load the trajectory that was actually used by `evo`
3. associate poses with the same `t_max_diff = 0.1`
4. apply the same SE(3) Umeyama alignment to the estimated positions
5. compute frame-to-frame translation deltas on the matched, aligned trajectories
6. compute translational RPE per step as the norm of the delta difference
7. report RMSE over those translational RPE values

So, in plain terms:

- APE asks: “how far is the aligned estimate from GT at each matched pose?”
- RPE asks: “how different is the local step-to-step motion compared with GT?”

### 7. Front-end speed field

`FE mean (ms)` comes from:

- `0.Full-Front_End`

inside each sequence's `ov2slam_timings.csv`.

`FE approx (Hz)` is simply:

```text
1000 / FE mean (ms)
```

This is an approximate front-end capacity number, not a claim that the dataset itself was played at that frequency.

### 8. Why the table mixes APE/RPE and FE timing

The table is intentionally combining:

- accuracy metrics: APE and RPE
- front-end timing: `0.Full-Front_End`

so you can compare, per sequence, whether higher difficulty is showing up more in:

- global trajectory drift
- local relative motion quality
- or front-end processing cost

## Per-Sequence Plots

Each sequence below includes:

- the APE XY plot generated by the benchmark
- the translational RPE plot generated by the benchmark

### MH_01_easy

![MH_01_easy APE](results/ov2slam_benchmark/new_retrail_benchmark/MH_01_easy/MH_01_easy_ape_xy.png)

![MH_01_easy RPE](results/ov2slam_benchmark/new_retrail_benchmark/MH_01_easy/MH_01_easy_rpe_trans.png)

### MH_02_easy

![MH_02_easy APE](results/ov2slam_benchmark/new_retrail_benchmark/MH_02_easy/MH_02_easy_ape_xy.png)

![MH_02_easy RPE](results/ov2slam_benchmark/new_retrail_benchmark/MH_02_easy/MH_02_easy_rpe_trans.png)

### MH_03_medium

![MH_03_medium APE](results/ov2slam_benchmark/new_retrail_benchmark/MH_03_medium/MH_03_medium_ape_xy.png)

![MH_03_medium RPE](results/ov2slam_benchmark/new_retrail_benchmark/MH_03_medium/MH_03_medium_rpe_trans.png)

### MH_04_difficult

![MH_04_difficult APE](results/ov2slam_benchmark/new_retrail_benchmark/MH_04_difficult/MH_04_difficult_ape_xy.png)

![MH_04_difficult RPE](results/ov2slam_benchmark/new_retrail_benchmark/MH_04_difficult/MH_04_difficult_rpe_trans.png)

### MH_05_difficult

![MH_05_difficult APE](results/ov2slam_benchmark/new_retrail_benchmark/MH_05_difficult/MH_05_difficult_ape_xy.png)

![MH_05_difficult RPE](results/ov2slam_benchmark/new_retrail_benchmark/MH_05_difficult/MH_05_difficult_rpe_trans.png)

### V1_01_easy

![V1_01_easy APE](results/ov2slam_benchmark/new_retrail_benchmark/V1_01_easy/V1_01_easy_ape_xy.png)

![V1_01_easy RPE](results/ov2slam_benchmark/new_retrail_benchmark/V1_01_easy/V1_01_easy_rpe_trans.png)

### V1_02_medium

![V1_02_medium APE](results/ov2slam_benchmark/new_retrail_benchmark/V1_02_medium/V1_02_medium_ape_xy.png)

![V1_02_medium RPE](results/ov2slam_benchmark/new_retrail_benchmark/V1_02_medium/V1_02_medium_rpe_trans.png)

### V1_03_difficult

![V1_03_difficult APE](results/ov2slam_benchmark/new_retrail_benchmark/V1_03_difficult/V1_03_difficult_ape_xy.png)

![V1_03_difficult RPE](results/ov2slam_benchmark/new_retrail_benchmark/V1_03_difficult/V1_03_difficult_rpe_trans.png)

### V2_01_easy

![V2_01_easy APE](results/ov2slam_benchmark/new_retrail_benchmark/V2_01_easy/V2_01_easy_ape_xy.png)

![V2_01_easy RPE](results/ov2slam_benchmark/new_retrail_benchmark/V2_01_easy/V2_01_easy_rpe_trans.png)

### V2_02_medium

![V2_02_medium APE](results/ov2slam_benchmark/new_retrail_benchmark/V2_02_medium/V2_02_medium_ape_xy.png)

![V2_02_medium RPE](results/ov2slam_benchmark/new_retrail_benchmark/V2_02_medium/V2_02_medium_rpe_trans.png)
