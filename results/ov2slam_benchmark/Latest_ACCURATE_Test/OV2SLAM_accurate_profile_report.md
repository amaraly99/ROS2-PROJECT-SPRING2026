# OV2SLAM Accurate Profiling Report

_Generated from `Latest_ACCURATE_Test` on 2026-03-29 17:08:09 +04._

## Executive Summary

This report profiles **OV2SLAM accurate stereo mode** on the **10 EuRoC sequences** present in `Latest_ACCURATE_Test`. The runs were executed at **1.0x playback**, with **trajectory policy `paper`**, using the parameter file `/workspace/src/ov2slam_ros/parameters_files/accurate/euroc/euroc_stereo.yaml`, and OV2SLAM pinned to CPUs **0,1,2,3**. The report is built directly from each sequence folder rather than from the top-level summary table, because the per-sequence EVO, timing, CPU, and plotting artifacts are the authoritative sources.

| Sequence | APE RMSE (m) | Matched timestamps | FE mean (ms) | FE std (ms) | FE Hz | Process CPU mean | Process CPU max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MH_01_easy | 0.040172 | 410/419 (97.9%) | 17.0830 | 8.0722 | 58.5 | 153.391% | 367.277% |
| MH_02_easy | 0.049323 | 412/419 (98.3%) | 17.0884 | 8.8434 | 58.5 | 149.588% | 370.828% |
| MH_03_medium | 0.046219 | 508/511 (99.4%) | 19.5224 | 12.2610 | 51.2 | 160.917% | 373.251% |
| MH_04_difficult | 0.069219 | 344/347 (99.1%) | 15.5783 | 7.9175 | 64.2 | 147.008% | 355.124% |
| MH_05_difficult | 0.066874 | 364/368 (98.9%) | 15.8146 | 8.5767 | 63.2 | 147.490% | 395.051% |
| V1_01_easy | 0.055497 | 410/411 (99.8%) | 19.7325 | 11.5328 | 50.7 | 157.147% | 377.398% |
| V1_02_medium | 0.088209 | 557/558 (99.8%) | 21.6984 | 12.1757 | 46.1 | 164.895% | 380.852% |
| V1_03_difficult | 0.117647 | 774/779 (99.4%) | 21.2245 | 12.8669 | 47.1 | 161.980% | 374.751% |
| V2_01_easy | 0.071501 | 355/358 (99.2%) | 15.0330 | 8.0701 | 66.5 | 141.735% | 359.467% |
| V2_02_medium | 0.053746 | 778/780 (99.7%) | 20.4879 | 11.8118 | 48.8 | 163.912% | 397.887% |

Meeting-ready observations:
- APE RMSE spans **0.040 m** to **0.118 m**, with `MH_01_easy` currently the easiest sequence and `V1_03_difficult` the hardest.
- Full front-end time spans **15.03 ms** to **21.70 ms**, or roughly **46.1 Hz** to **66.5 Hz** of estimated front-end capacity.
- Across the sequence set, the dominant logical thread roles are `ov2_est` 63.4%, `ov2_lc` 42.4%, `ov2_main` 14.3%, `ov2_map` 12.9%.
- Treat these as **preliminary single-run results**: they are strong enough for an executive baseline, but not yet a repeated-run statistical benchmark.

![](accurate_ape_rmse_by_sequence.png)

![](accurate_fe_mean_std_by_sequence.png)

## Evaluation and Profiling Method

- **Ground truth source:** each sequence uses the normalized `gt.tum` file saved in its result folder; the benchmark metadata identifies the source dataset as EuRoC and confirms the GT normalization step.
- **Evaluated estimate:** `ov2slam_kfs_traj.txt` is the trajectory evaluated for this accurate stereo report, matching the benchmark’s current paper-style stereo choice.
- **Alignment rule:** EVO compares trajectories with `evo_ape tum --align` and **does not apply `--correct_scale`**, because these are stereo runs with metric scale.
- **Matched timestamps:** the `found / max (%)` values in this report come directly from the `Synchronizing trajectories...` section in each `*_evo.txt` file.
- **APE and RPE visuals:** the report reuses the saved `*_ape_xy.png` and `*_rpe_trans.png` files already generated during benchmarking.
- **Front-end and backend timing:** timing values come from `ov2slam_timings.csv`, specifically the `mean_ms` and `std_ms` columns for the selected timers.
- **Process CPU:** process-level CPU statistics are computed from the sampled `process_cpu_percent` values in `ov2slam_process_cpu.csv`.
- **Thread-role CPU:** thread-role statistics come from `ov2slam_named_thread_roles_aggregated.csv`, where samples are grouped by logical role such as `ov2_est`, `ov2_lc`, `ov2_main`, and `ov2_map`.
- **CPU averaging note:** CPU means and standard deviations are averages over the monitor samples across time, not instantaneous values copied from a single frame.
- **Thread-role aggregation note:** the role values are logical-role aggregates over all matching TIDs, so repeated `ov2_main` worker threads are merged before summary.

## Cross-Sequence Results

| Sequence | APE mean (m) | APE RMSE (m) | APE std (m) | Matched pairs | FE mean±std (ms) | Process CPU mean±std (max) | Top 3 thread roles |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| MH_01_easy | 0.034381 | 0.040172 | 0.020779 | 410/419 (97.9%) | 17.083 ± 8.072 | 153.39% ± 87.50% (max 367.28%) | `ov2_est` 67.8%, `ov2_lc` 34.7%, `ov2_main` 14.2% |
| MH_02_easy | 0.042588 | 0.049323 | 0.024880 | 412/419 (98.3%) | 17.088 ± 8.843 | 149.59% ± 80.65% (max 370.83%) | `ov2_est` 71.5%, `ov2_lc` 35.0%, `ov2_main` 13.2% |
| MH_03_medium | 0.042523 | 0.046219 | 0.018111 | 508/511 (99.4%) | 19.522 ± 12.261 | 160.92% ± 90.38% (max 373.25%) | `ov2_est` 63.3%, `ov2_lc` 51.0%, `ov2_map` 14.2% |
| MH_04_difficult | 0.064916 | 0.069219 | 0.024024 | 344/347 (99.1%) | 15.578 ± 7.918 | 147.01% ± 87.76% (max 355.12%) | `ov2_est` 62.5%, `ov2_lc` 30.6%, `ov2_main` 14.0% |
| MH_05_difficult | 0.060201 | 0.066874 | 0.029121 | 364/368 (98.9%) | 15.815 ± 8.577 | 147.49% ± 87.15% (max 395.05%) | `ov2_est` 66.2%, `ov2_lc` 30.6%, `ov2_main` 13.8% |
| V1_01_easy | 0.051132 | 0.055497 | 0.021575 | 410/411 (99.8%) | 19.733 ± 11.533 | 157.15% ± 87.79% (max 377.40%) | `ov2_est` 70.9%, `ov2_lc` 44.5%, `ov2_map` 14.0% |
| V1_02_medium | 0.084095 | 0.088209 | 0.026625 | 557/558 (99.8%) | 21.698 ± 12.176 | 164.90% ± 104.28% (max 380.85%) | `ov2_est` 62.5%, `ov2_lc` 52.9%, `ov2_map` 16.9% |
| V1_03_difficult | 0.097796 | 0.117647 | 0.065398 | 774/779 (99.4%) | 21.224 ± 12.867 | 161.98% ± 98.38% (max 374.75%) | `ov2_lc` 60.5%, `ov2_est` 47.4%, `ov2_main` 16.2% |
| V2_01_easy | 0.065134 | 0.071501 | 0.029494 | 355/358 (99.2%) | 15.033 ± 8.070 | 141.73% ± 82.81% (max 359.47%) | `ov2_est` 64.3%, `ov2_lc` 28.8%, `ov2_main` 13.5% |
| V2_02_medium | 0.049206 | 0.053746 | 0.021618 | 778/780 (99.7%) | 20.488 ± 11.812 | 163.91% ± 95.36% (max 397.89%) | `ov2_est` 57.5%, `ov2_lc` 55.5%, `ov2_map` 18.5% |

Interpretation starter: accurate OV2SLAM stays in a relatively tight accuracy band on most EuRoC sequences, while the front-end remains below the 22 ms mark across this dataset. The main variation appears in backend pressure and sequence difficulty rather than in any obvious front-end collapse.

## Profiling Breakdown

### Timing Breakdown

| Sequence | Full FE (ms) | FE Track (ms) | FE KF create (ms) | Mapper KF proc (ms) | Local BA (ms) | LC ProcessKF (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| MH_01_easy | 17.083 ± 8.072 | 15.028 | 16.872 | 47.601 | 292.162 | 82.734 |
| MH_02_easy | 17.088 ± 8.843 | 14.630 | 17.054 | 51.340 | 280.729 | 83.636 |
| MH_03_medium | 19.522 ± 12.261 | 15.628 | 20.002 | 57.868 | 211.845 | 64.739 |
| MH_04_difficult | 15.578 ± 7.918 | 12.727 | 16.122 | 31.519 | 195.776 | 64.356 |
| MH_05_difficult | 15.815 ± 8.577 | 13.087 | 16.426 | 33.355 | 223.581 | 66.218 |
| V1_01_easy | 19.733 ± 11.533 | 16.799 | 20.179 | 71.542 | 271.416 | 75.109 |
| V1_02_medium | 21.698 ± 12.176 | 14.990 | 20.409 | 43.554 | 140.398 | 73.415 |
| V1_03_difficult | 21.224 ± 12.867 | 13.773 | 20.282 | 35.529 | 86.274 | 66.747 |
| V2_01_easy | 15.033 ± 8.070 | 12.450 | 16.287 | 34.761 | 212.102 | 68.419 |
| V2_02_medium | 20.488 ± 11.812 | 14.194 | 18.857 | 42.090 | 107.253 | 74.974 |

Discussion starters:
- The accurate front-end stays between **15.03 ms** and **21.70 ms**, so the front-end is not the main source of cross-sequence instability here.
- Local BA and loop-closure processing are much heavier than front-end tracking, which is consistent with the CPU plots showing strong backend activity.
- If you need a meeting takeaway, the phrasing can be: “timing pressure is dominated by optimization stages rather than image-to-image tracking.”

### CPU and Thread-Role Utilization

| Sequence | Process CPU mean±std (max) | Top role 1 | Top role 2 | Top role 3 |
| --- | --- | --- | --- | --- |
| MH_01_easy | 153.39% ± 87.50% (max 367.28%) | `ov2_est` 67.8% ± 41.3% | `ov2_lc` 34.7% ± 42.7% | `ov2_main` 14.2% ± 18.7% |
| MH_02_easy | 149.59% ± 80.65% (max 370.83%) | `ov2_est` 71.5% ± 39.7% | `ov2_lc` 35.0% ± 42.5% | `ov2_main` 13.2% ± 22.8% |
| MH_03_medium | 160.92% ± 90.38% (max 373.25%) | `ov2_est` 63.3% ± 40.9% | `ov2_lc` 51.0% ± 45.9% | `ov2_map` 14.2% ± 21.5% |
| MH_04_difficult | 147.01% ± 87.76% (max 355.12%) | `ov2_est` 62.5% ± 41.1% | `ov2_lc` 30.6% ± 40.1% | `ov2_main` 14.0% ± 18.3% |
| MH_05_difficult | 147.49% ± 87.15% (max 395.05%) | `ov2_est` 66.2% ± 40.3% | `ov2_lc` 30.6% ± 40.0% | `ov2_main` 13.8% ± 18.3% |
| V1_01_easy | 157.15% ± 87.79% (max 377.40%) | `ov2_est` 70.9% ± 38.3% | `ov2_lc` 44.5% ± 44.9% | `ov2_map` 14.0% ± 23.0% |
| V1_02_medium | 164.90% ± 104.28% (max 380.85%) | `ov2_est` 62.5% ± 39.4% | `ov2_lc` 52.9% ± 44.1% | `ov2_map` 16.9% ± 20.2% |
| V1_03_difficult | 161.98% ± 98.38% (max 374.75%) | `ov2_lc` 60.5% ± 41.3% | `ov2_est` 47.4% ± 37.0% | `ov2_main` 16.2% ± 21.6% |
| V2_01_easy | 141.73% ± 82.81% (max 359.47%) | `ov2_est` 64.3% ± 41.4% | `ov2_lc` 28.8% ± 40.1% | `ov2_main` 13.5% ± 19.5% |
| V2_02_medium | 163.91% ± 95.36% (max 397.89%) | `ov2_est` 57.5% ± 37.4% | `ov2_lc` 55.5% ± 42.6% | `ov2_map` 18.5% ± 20.4% |

| Logical thread role | Mean of per-sequence means | Std across sequences | Present in sequences |
| --- | ---: | ---: | ---: |
| `ov2_est` | 63.386% | 6.659% | 10 |
| `ov2_lc` | 42.399% | 11.259% | 10 |
| `ov2_main` | 14.343% | 0.982% | 10 |
| `ov2_map` | 12.944% | 3.184% | 10 |
| `ov2slam_node_main` | 1.118% | 0.080% | 10 |
| `ov2_feed` | 1.085% | 0.073% | 10 |
| `ov2_merge` | 0.000% | 0.000% | 10 |
| `ov2_vizfr` | 0.000% | 0.000% | 7 |
| `ov2_vizkf` | 0.000% | 0.000% | 1 |

Discussion starters:
- The dominant logical roles on average are `ov2_est` 63.4%, `ov2_lc` 42.4%, `ov2_main` 14.3%, `ov2_map` 12.9%.
- The estimator and loop-closure workers consistently dominate sustained CPU consumption, while feeder and main-node bookkeeping remain lightweight.
- A concise meeting phrasing is: “the optimization workers, not the ROS wrapper, carry most of the runtime load in accurate mode.”

## Per-Sequence Appendix

### MH_01_easy

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.034381 / 0.040172 / 0.020779 m |
| Matched timestamps | 410/419 (97.9%) |
| FE mean / std / Hz | 17.0830 ms / 8.0722 ms / 58.5 Hz |
| Process CPU mean / std / max | 153.391% / 87.498% / 367.277% |
| Top thread roles | `ov2_est` 67.8%, `ov2_lc` 34.7%, `ov2_main` 14.2% |

![](MH_01_easy/MH_01_easy_ape_xy.png)

![](MH_01_easy/MH_01_easy_rpe_trans.png)

![](MH_01_easy/ov2slam_cpu_usage.png)

![](MH_01_easy/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_01_easy/ov2slam.log), [timings](MH_01_easy/ov2slam_timings.csv), [thread roles](MH_01_easy/ov2slam_named_thread_roles_aggregated.csv)

### MH_02_easy

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.042588 / 0.049323 / 0.024880 m |
| Matched timestamps | 412/419 (98.3%) |
| FE mean / std / Hz | 17.0884 ms / 8.8434 ms / 58.5 Hz |
| Process CPU mean / std / max | 149.588% / 80.653% / 370.828% |
| Top thread roles | `ov2_est` 71.5%, `ov2_lc` 35.0%, `ov2_main` 13.2% |

![](MH_02_easy/MH_02_easy_ape_xy.png)

![](MH_02_easy/MH_02_easy_rpe_trans.png)

![](MH_02_easy/ov2slam_cpu_usage.png)

![](MH_02_easy/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_02_easy/ov2slam.log), [timings](MH_02_easy/ov2slam_timings.csv), [thread roles](MH_02_easy/ov2slam_named_thread_roles_aggregated.csv)

### MH_03_medium

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.042523 / 0.046219 / 0.018111 m |
| Matched timestamps | 508/511 (99.4%) |
| FE mean / std / Hz | 19.5224 ms / 12.2610 ms / 51.2 Hz |
| Process CPU mean / std / max | 160.917% / 90.375% / 373.251% |
| Top thread roles | `ov2_est` 63.3%, `ov2_lc` 51.0%, `ov2_map` 14.2% |

![](MH_03_medium/MH_03_medium_ape_xy.png)

![](MH_03_medium/MH_03_medium_rpe_trans.png)

![](MH_03_medium/ov2slam_cpu_usage.png)

![](MH_03_medium/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_03_medium/ov2slam.log), [timings](MH_03_medium/ov2slam_timings.csv), [thread roles](MH_03_medium/ov2slam_named_thread_roles_aggregated.csv)

### MH_04_difficult

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.064916 / 0.069219 / 0.024024 m |
| Matched timestamps | 344/347 (99.1%) |
| FE mean / std / Hz | 15.5783 ms / 7.9175 ms / 64.2 Hz |
| Process CPU mean / std / max | 147.008% / 87.761% / 355.124% |
| Top thread roles | `ov2_est` 62.5%, `ov2_lc` 30.6%, `ov2_main` 14.0% |

![](MH_04_difficult/MH_04_difficult_ape_xy.png)

![](MH_04_difficult/MH_04_difficult_rpe_trans.png)

![](MH_04_difficult/ov2slam_cpu_usage.png)

![](MH_04_difficult/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_04_difficult/ov2slam.log), [timings](MH_04_difficult/ov2slam_timings.csv), [thread roles](MH_04_difficult/ov2slam_named_thread_roles_aggregated.csv)

### MH_05_difficult

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.060201 / 0.066874 / 0.029121 m |
| Matched timestamps | 364/368 (98.9%) |
| FE mean / std / Hz | 15.8146 ms / 8.5767 ms / 63.2 Hz |
| Process CPU mean / std / max | 147.490% / 87.150% / 395.051% |
| Top thread roles | `ov2_est` 66.2%, `ov2_lc` 30.6%, `ov2_main` 13.8% |

![](MH_05_difficult/MH_05_difficult_ape_xy.png)

![](MH_05_difficult/MH_05_difficult_rpe_trans.png)

![](MH_05_difficult/ov2slam_cpu_usage.png)

![](MH_05_difficult/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_05_difficult/ov2slam.log), [timings](MH_05_difficult/ov2slam_timings.csv), [thread roles](MH_05_difficult/ov2slam_named_thread_roles_aggregated.csv)

### V1_01_easy

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.051132 / 0.055497 / 0.021575 m |
| Matched timestamps | 410/411 (99.8%) |
| FE mean / std / Hz | 19.7325 ms / 11.5328 ms / 50.7 Hz |
| Process CPU mean / std / max | 157.147% / 87.790% / 377.398% |
| Top thread roles | `ov2_est` 70.9%, `ov2_lc` 44.5%, `ov2_map` 14.0% |

![](V1_01_easy/V1_01_easy_ape_xy.png)

![](V1_01_easy/V1_01_easy_rpe_trans.png)

![](V1_01_easy/ov2slam_cpu_usage.png)

![](V1_01_easy/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V1_01_easy/ov2slam.log), [timings](V1_01_easy/ov2slam_timings.csv), [thread roles](V1_01_easy/ov2slam_named_thread_roles_aggregated.csv)

### V1_02_medium

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.084095 / 0.088209 / 0.026625 m |
| Matched timestamps | 557/558 (99.8%) |
| FE mean / std / Hz | 21.6984 ms / 12.1757 ms / 46.1 Hz |
| Process CPU mean / std / max | 164.895% / 104.276% / 380.852% |
| Top thread roles | `ov2_est` 62.5%, `ov2_lc` 52.9%, `ov2_map` 16.9% |

![](V1_02_medium/V1_02_medium_ape_xy.png)

![](V1_02_medium/V1_02_medium_rpe_trans.png)

![](V1_02_medium/ov2slam_cpu_usage.png)

![](V1_02_medium/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V1_02_medium/ov2slam.log), [timings](V1_02_medium/ov2slam_timings.csv), [thread roles](V1_02_medium/ov2slam_named_thread_roles_aggregated.csv)

### V1_03_difficult

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.097796 / 0.117647 / 0.065398 m |
| Matched timestamps | 774/779 (99.4%) |
| FE mean / std / Hz | 21.2245 ms / 12.8669 ms / 47.1 Hz |
| Process CPU mean / std / max | 161.980% / 98.379% / 374.751% |
| Top thread roles | `ov2_lc` 60.5%, `ov2_est` 47.4%, `ov2_main` 16.2% |

![](V1_03_difficult/V1_03_difficult_ape_xy.png)

![](V1_03_difficult/V1_03_difficult_rpe_trans.png)

![](V1_03_difficult/ov2slam_cpu_usage.png)

![](V1_03_difficult/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V1_03_difficult/ov2slam.log), [timings](V1_03_difficult/ov2slam_timings.csv), [thread roles](V1_03_difficult/ov2slam_named_thread_roles_aggregated.csv)

### V2_01_easy

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.065134 / 0.071501 / 0.029494 m |
| Matched timestamps | 355/358 (99.2%) |
| FE mean / std / Hz | 15.0330 ms / 8.0701 ms / 66.5 Hz |
| Process CPU mean / std / max | 141.735% / 82.811% / 359.467% |
| Top thread roles | `ov2_est` 64.3%, `ov2_lc` 28.8%, `ov2_main` 13.5% |

![](V2_01_easy/V2_01_easy_ape_xy.png)

![](V2_01_easy/V2_01_easy_rpe_trans.png)

![](V2_01_easy/ov2slam_cpu_usage.png)

![](V2_01_easy/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V2_01_easy/ov2slam.log), [timings](V2_01_easy/ov2slam_timings.csv), [thread roles](V2_01_easy/ov2slam_named_thread_roles_aggregated.csv)

### V2_02_medium

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.049206 / 0.053746 / 0.021618 m |
| Matched timestamps | 778/780 (99.7%) |
| FE mean / std / Hz | 20.4879 ms / 11.8118 ms / 48.8 Hz |
| Process CPU mean / std / max | 163.912% / 95.362% / 397.887% |
| Top thread roles | `ov2_est` 57.5%, `ov2_lc` 55.5%, `ov2_map` 18.5% |

![](V2_02_medium/V2_02_medium_ape_xy.png)

![](V2_02_medium/V2_02_medium_rpe_trans.png)

![](V2_02_medium/ov2slam_cpu_usage.png)

![](V2_02_medium/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V2_02_medium/ov2slam.log), [timings](V2_02_medium/ov2slam_timings.csv), [thread roles](V2_02_medium/ov2slam_named_thread_roles_aggregated.csv)

