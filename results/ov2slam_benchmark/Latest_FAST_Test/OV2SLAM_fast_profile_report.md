# OV2SLAM Fast Profiling Report

_Generated from `Latest_FAST_Test` on 2026-03-30 00:30:14 +04._

## Executive Summary

This report profiles **OV2SLAM fast stereo mode** on the **10 EuRoC sequences** present in `Latest_FAST_Test`. The runs were executed at **1.0x playback**, with **trajectory policy `paper`**, using the parameter file `/workspace/src/ov2slam_ros/parameters_files/fast/euroc/euroc_stereo.yaml`, and OV2SLAM pinned to CPUs **0,1,2,3**. This is a **default-speed fast profile**, not the paper’s accelerated 100–400 Hz replay experiment.

| Sequence | APE RMSE (m) | Matched timestamps | FE mean (ms) | FE std (ms) | FE Hz | Process CPU mean | Process CPU max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MH_01_easy | 0.054081 | 405/417 (97.1%) | 4.8948 | 1.4569 | 204.3 | 42.770% | 169.658% |
| MH_02_easy | 0.052482 | 405/414 (97.8%) | 5.0246 | 1.5248 | 199.0 | 43.611% | 170.493% |
| MH_03_medium | 0.078613 | 489/493 (99.2%) | 5.0880 | 1.7494 | 196.5 | 48.421% | 170.378% |
| MH_04_difficult | 0.152413 | 334/337 (99.1%) | 4.9897 | 1.8740 | 200.4 | 42.714% | 162.747% |
| MH_05_difficult | 0.141954 | 343/347 (98.8%) | 5.0355 | 1.9369 | 198.6 | 42.279% | 169.694% |
| V1_01_easy | 0.094320 | 453/454 (99.8%) | 5.1141 | 1.7984 | 195.5 | 46.122% | 171.640% |
| V1_02_medium | 0.094627 | 556/559 (99.5%) | 5.6978 | 2.2194 | 175.5 | 46.791% | 153.828% |
| V1_03_difficult | 0.669767 | 775/777 (99.7%) | 6.1049 | 3.2407 | 163.8 | 46.573% | 168.693% |
| V2_01_easy | 0.086232 | 332/335 (99.1%) | 4.9082 | 1.9155 | 203.7 | 42.010% | 169.702% |
| V2_02_medium | 0.203507 | 737/740 (99.6%) | 5.7925 | 2.5753 | 172.6 | 51.881% | 154.180% |

Meeting-ready observations:
- APE RMSE spans **0.052 m** to **0.670 m**, with `MH_02_easy` currently the easiest sequence and `V1_03_difficult` the hardest.
- Full front-end time spans **4.89 ms** to **6.10 ms**, or roughly **163.8 Hz** to **204.3 Hz** of estimated front-end capacity.
- Across the sequence set, the dominant logical thread roles are `ov2_est` 14.4%, `ov2_main` 6.2%, `ov2_map` 4.5%, `ov2slam_node_main` 0.8%.
- Process/thread CPU telemetry is available for part of this run set and is summarized below.
- Treat these as **preliminary single-run results**: they are useful for a fast-mode baseline, but not a repeated-run statistical benchmark.

![](fast_ape_rmse_by_sequence.png)

![](fast_fe_mean_std_by_sequence.png)

## Evaluation and Profiling Method

- **Ground truth source:** if `gt.tum` was missing from the result folder, the report used the EuRoC dataset file `datasets/euroc/<seq>/<seq>.txt` directly.
- **Evaluated estimate:** `ov2slam_kfs_traj.txt` is the trajectory evaluated for this fast stereo report.
- **Alignment rule:** EVO compares trajectories with `evo_ape tum --align` and does not use scale correction because these are stereo runs.
- **Matched timestamps:** the `found / max (%)` values come from regenerated `*_evo.txt` files whenever the original text output was not present in the result folder.
- **APE and RPE visuals:** the report reuses saved PNGs when available and regenerates them from the saved trajectories when they are missing.
- **Front-end and backend timing:** timing values are taken from the final `Time Logs Summary` block in `ov2slam.log`; `ov2slam_timings.csv` is only used as a fallback if the log summary is unavailable.
- **Process CPU:** process-level CPU statistics come from `ov2slam_process_cpu.csv` when present.
- **Thread-role CPU:** thread-role statistics come from `ov2slam_named_thread_roles_aggregated.csv` when present.
- **CPU averaging note:** CPU means and standard deviations are averages over the monitor samples across time.
- **Thread-role aggregation note:** role values are logical-role aggregates over all matching TIDs.

## Cross-Sequence Results

| Sequence | APE mean (m) | APE RMSE (m) | APE std (m) | Matched pairs | FE mean±std (ms) | Process CPU mean±std (max) | Top 3 thread roles |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| MH_01_easy | 0.048019 | 0.054081 | 0.024879 | 405/417 (97.1%) | 4.895 ± 1.457 | 42.77% ± 34.79% (max 169.66%) | `ov2_est` 15.1%, `ov2_main` 5.4%, `ov2_map` 3.4% |
| MH_02_easy | 0.041618 | 0.052482 | 0.031973 | 405/414 (97.8%) | 5.025 ± 1.525 | 43.61% ± 36.19% (max 170.49%) | `ov2_est` 15.6%, `ov2_main` 5.6%, `ov2_map` 3.7% |
| MH_03_medium | 0.068722 | 0.078613 | 0.038174 | 489/493 (99.2%) | 5.088 ± 1.749 | 48.42% ± 37.67% (max 170.38%) | `ov2_est` 18.5%, `ov2_main` 5.8%, `ov2_map` 4.5% |
| MH_04_difficult | 0.143254 | 0.152413 | 0.052038 | 334/337 (99.1%) | 4.990 ± 1.874 | 42.71% ± 33.75% (max 162.75%) | `ov2_est` 13.9%, `ov2_main` 5.8%, `ov2_map` 3.8% |
| MH_05_difficult | 0.126961 | 0.141954 | 0.063496 | 343/347 (98.8%) | 5.036 ± 1.937 | 42.28% ± 34.36% (max 169.69%) | `ov2_est` 14.1%, `ov2_main` 5.7%, `ov2_map` 3.4% |
| V1_01_easy | 0.088706 | 0.094320 | 0.032054 | 453/454 (99.8%) | 5.114 ± 1.798 | 46.12% ± 36.21% (max 171.64%) | `ov2_est` 16.2%, `ov2_main` 5.9%, `ov2_map` 4.4% |
| V1_02_medium | 0.087643 | 0.094627 | 0.035680 | 556/559 (99.5%) | 5.698 ± 2.219 | 46.79% ± 36.01% (max 153.83%) | `ov2_est` 14.8%, `ov2_main` 6.7%, `ov2_map` 5.7% |
| V1_03_difficult | 0.621162 | 0.669767 | 0.250491 | 775/777 (99.7%) | 6.105 ± 3.241 | 46.57% ± 32.01% (max 168.69%) | `ov2_est` 9.1%, `ov2_main` 7.9%, `ov2_map` 4.6% |
| V2_01_easy | 0.083026 | 0.086232 | 0.023293 | 332/335 (99.1%) | 4.908 ± 1.915 | 42.01% ± 33.28% (max 169.70%) | `ov2_est` 13.0%, `ov2_main` 5.8%, `ov2_map` 3.6% |
| V2_02_medium | 0.170448 | 0.203507 | 0.111188 | 737/740 (99.6%) | 5.793 ± 2.575 | 51.88% ± 37.23% (max 154.18%) | `ov2_est` 14.1%, `ov2_map` 7.8%, `ov2_main` 7.4% |

Interpretation starter: fast OV2SLAM clearly reduces front-end latency, but the trade-off in this default-speed run is visible in several of the more difficult sequences where accuracy deteriorates.

## Profiling Breakdown

### Timing Breakdown

| Sequence | Full FE (ms) | FE Track (ms) | FE KF create (ms) | Mapper KF proc (ms) | Local BA (ms) | LC ProcessKF (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| MH_01_easy | 4.895 ± 1.457 | 4.383 | 3.763 | 8.310 | 41.283 | disabled |
| MH_02_easy | 5.025 ± 1.525 | 4.427 | 3.876 | 8.459 | 38.392 | disabled |
| MH_03_medium | 5.088 ± 1.749 | 4.313 | 3.800 | 8.497 | 33.928 | disabled |
| MH_04_difficult | 4.990 ± 1.874 | 4.170 | 4.220 | 6.961 | 28.558 | disabled |
| MH_05_difficult | 5.036 ± 1.937 | 4.273 | 4.229 | 6.370 | 31.544 | disabled |
| V1_01_easy | 5.114 ± 1.798 | 4.309 | 4.086 | 8.346 | 32.296 | disabled |
| V1_02_medium | 5.698 ± 2.219 | 4.156 | 4.315 | 7.464 | 17.691 | disabled |
| V1_03_difficult | 6.105 ± 3.241 | 3.886 | 5.953 | 4.934 | 7.846 | disabled |
| V2_01_easy | 4.908 ± 1.915 | 4.093 | 4.733 | 7.099 | 28.772 | disabled |
| V2_02_medium | 5.793 ± 2.575 | 4.038 | 5.329 | 10.646 | 16.232 | disabled |

Discussion starters:
- The fast front-end stays between **4.89 ms** and **6.10 ms**, giving a noticeably higher front-end headroom than the accurate configuration.
- Backend costs are also reduced, but difficult sequences still show substantial optimization work and this remains the main cost center even in fast mode.
- A concise meeting phrasing is: “fast mode cuts front-end latency aggressively, but accuracy becomes more sequence-sensitive.”

### CPU and Thread-Role Utilization

| Sequence | Process CPU mean±std (max) | Top role 1 | Top role 2 | Top role 3 |
| --- | --- | --- | --- | --- |
| MH_01_easy | 42.77% ± 34.79% (max 169.66%) | `ov2_est` 15.1% ± 25.9% | `ov2_main` 5.4% ± 8.9% | `ov2_map` 3.4% ± 8.0% |
| MH_02_easy | 43.61% ± 36.19% (max 170.49%) | `ov2_est` 15.6% ± 26.2% | `ov2_main` 5.6% ± 9.1% | `ov2_map` 3.7% ± 8.0% |
| MH_03_medium | 48.42% ± 37.67% (max 170.38%) | `ov2_est` 18.5% ± 26.9% | `ov2_main` 5.8% ± 9.1% | `ov2_map` 4.5% ± 9.0% |
| MH_04_difficult | 42.71% ± 33.75% (max 162.75%) | `ov2_est` 13.9% ± 21.9% | `ov2_main` 5.8% ± 9.1% | `ov2_map` 3.8% ± 8.0% |
| MH_05_difficult | 42.28% ± 34.36% (max 169.69%) | `ov2_est` 14.1% ± 22.9% | `ov2_main` 5.7% ± 9.0% | `ov2_map` 3.4% ± 7.5% |
| V1_01_easy | 46.12% ± 36.21% (max 171.64%) | `ov2_est` 16.2% ± 24.7% | `ov2_main` 5.9% ± 9.1% | `ov2_map` 4.4% ± 8.8% |
| V1_02_medium | 46.79% ± 36.01% (max 153.83%) | `ov2_est` 14.8% ± 19.4% | `ov2_main` 6.7% ± 9.7% | `ov2_map` 5.7% ± 9.6% |
| V1_03_difficult | 46.57% ± 32.01% (max 168.69%) | `ov2_est` 9.1% ± 13.2% | `ov2_main` 7.9% ± 10.4% | `ov2_map` 4.6% ± 8.4% |
| V2_01_easy | 42.01% ± 33.28% (max 169.70%) | `ov2_est` 13.0% ± 21.3% | `ov2_main` 5.8% ± 9.2% | `ov2_map` 3.6% ± 8.0% |
| V2_02_medium | 51.88% ± 37.23% (max 154.18%) | `ov2_est` 14.1% ± 17.9% | `ov2_map` 7.8% ± 11.8% | `ov2_main` 7.4% ± 10.1% |

| Logical thread role | Mean of per-sequence means | Std across sequences | Present in sequences |
| --- | ---: | ---: | ---: |
| `ov2_est` | 14.445% | 2.293% | 10 |
| `ov2_main` | 6.192% | 0.786% | 10 |
| `ov2_map` | 4.509% | 1.274% | 10 |
| `ov2slam_node_main` | 0.801% | 0.041% | 10 |
| `ov2_feed` | 0.626% | 0.021% | 10 |
| `ov2_merge` | 0.000% | 0.000% | 3 |
| `ov2_vizfr` | 0.000% | 0.000% | 7 |
| `ov2_vizkf` | 0.000% | 0.000% | 2 |

Discussion starters:
- The dominant logical roles on average are `ov2_est` 14.4%, `ov2_main` 6.2%, `ov2_map` 4.5%, `ov2slam_node_main` 0.8%.
- Even in fast mode, the estimator and optimization-related roles remain the main sustained CPU consumers.
- A concise meeting phrasing is: “fast mode reduces latency more than it reduces optimization dominance.”

## Per-Sequence Appendix

### MH_01_easy

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.048019 / 0.054081 / 0.024879 m |
| Matched timestamps | 405/417 (97.1%) |
| FE mean / std / Hz | 4.8948 ms / 1.4569 ms / 204.3 Hz |
| Process CPU mean / std / max | 42.77% ± 34.79% (max 169.66%) |
| Top thread roles | `ov2_est` 15.1%, `ov2_main` 5.4%, `ov2_map` 3.4% |

![](MH_01_easy/MH_01_easy_ape_xy.png)

![](MH_01_easy/MH_01_easy_rpe_trans.png)

![](MH_01_easy/ov2slam_cpu_usage.png)

![](MH_01_easy/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_01_easy/ov2slam.log), [evo text](MH_01_easy/MH_01_easy_evo.txt), [timings csv](MH_01_easy/ov2slam_timings.csv), [thread roles](MH_01_easy/ov2slam_named_thread_roles_aggregated.csv)

### MH_02_easy

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.041618 / 0.052482 / 0.031973 m |
| Matched timestamps | 405/414 (97.8%) |
| FE mean / std / Hz | 5.0246 ms / 1.5248 ms / 199.0 Hz |
| Process CPU mean / std / max | 43.61% ± 36.19% (max 170.49%) |
| Top thread roles | `ov2_est` 15.6%, `ov2_main` 5.6%, `ov2_map` 3.7% |

![](MH_02_easy/MH_02_easy_ape_xy.png)

![](MH_02_easy/MH_02_easy_rpe_trans.png)

![](MH_02_easy/ov2slam_cpu_usage.png)

![](MH_02_easy/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_02_easy/ov2slam.log), [evo text](MH_02_easy/MH_02_easy_evo.txt), [timings csv](MH_02_easy/ov2slam_timings.csv), [thread roles](MH_02_easy/ov2slam_named_thread_roles_aggregated.csv)

### MH_03_medium

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.068722 / 0.078613 / 0.038174 m |
| Matched timestamps | 489/493 (99.2%) |
| FE mean / std / Hz | 5.0880 ms / 1.7494 ms / 196.5 Hz |
| Process CPU mean / std / max | 48.42% ± 37.67% (max 170.38%) |
| Top thread roles | `ov2_est` 18.5%, `ov2_main` 5.8%, `ov2_map` 4.5% |

![](MH_03_medium/MH_03_medium_ape_xy.png)

![](MH_03_medium/MH_03_medium_rpe_trans.png)

![](MH_03_medium/ov2slam_cpu_usage.png)

![](MH_03_medium/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_03_medium/ov2slam.log), [evo text](MH_03_medium/MH_03_medium_evo.txt), [timings csv](MH_03_medium/ov2slam_timings.csv), [thread roles](MH_03_medium/ov2slam_named_thread_roles_aggregated.csv)

### MH_04_difficult

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.143254 / 0.152413 / 0.052038 m |
| Matched timestamps | 334/337 (99.1%) |
| FE mean / std / Hz | 4.9897 ms / 1.8740 ms / 200.4 Hz |
| Process CPU mean / std / max | 42.71% ± 33.75% (max 162.75%) |
| Top thread roles | `ov2_est` 13.9%, `ov2_main` 5.8%, `ov2_map` 3.8% |

![](MH_04_difficult/MH_04_difficult_ape_xy.png)

![](MH_04_difficult/MH_04_difficult_rpe_trans.png)

![](MH_04_difficult/ov2slam_cpu_usage.png)

![](MH_04_difficult/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_04_difficult/ov2slam.log), [evo text](MH_04_difficult/MH_04_difficult_evo.txt), [timings csv](MH_04_difficult/ov2slam_timings.csv), [thread roles](MH_04_difficult/ov2slam_named_thread_roles_aggregated.csv)

### MH_05_difficult

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.126961 / 0.141954 / 0.063496 m |
| Matched timestamps | 343/347 (98.8%) |
| FE mean / std / Hz | 5.0355 ms / 1.9369 ms / 198.6 Hz |
| Process CPU mean / std / max | 42.28% ± 34.36% (max 169.69%) |
| Top thread roles | `ov2_est` 14.1%, `ov2_main` 5.7%, `ov2_map` 3.4% |

![](MH_05_difficult/MH_05_difficult_ape_xy.png)

![](MH_05_difficult/MH_05_difficult_rpe_trans.png)

![](MH_05_difficult/ov2slam_cpu_usage.png)

![](MH_05_difficult/ov2slam_named_thread_roles_aggregated.png)

Links: [log](MH_05_difficult/ov2slam.log), [evo text](MH_05_difficult/MH_05_difficult_evo.txt), [timings csv](MH_05_difficult/ov2slam_timings.csv), [thread roles](MH_05_difficult/ov2slam_named_thread_roles_aggregated.csv)

### V1_01_easy

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.088706 / 0.094320 / 0.032054 m |
| Matched timestamps | 453/454 (99.8%) |
| FE mean / std / Hz | 5.1141 ms / 1.7984 ms / 195.5 Hz |
| Process CPU mean / std / max | 46.12% ± 36.21% (max 171.64%) |
| Top thread roles | `ov2_est` 16.2%, `ov2_main` 5.9%, `ov2_map` 4.4% |

![](V1_01_easy/V1_01_easy_ape_xy.png)

![](V1_01_easy/V1_01_easy_rpe_trans.png)

![](V1_01_easy/ov2slam_cpu_usage.png)

![](V1_01_easy/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V1_01_easy/ov2slam.log), [evo text](V1_01_easy/V1_01_easy_evo.txt), [timings csv](V1_01_easy/ov2slam_timings.csv), [thread roles](V1_01_easy/ov2slam_named_thread_roles_aggregated.csv)

### V1_02_medium

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.087643 / 0.094627 / 0.035680 m |
| Matched timestamps | 556/559 (99.5%) |
| FE mean / std / Hz | 5.6978 ms / 2.2194 ms / 175.5 Hz |
| Process CPU mean / std / max | 46.79% ± 36.01% (max 153.83%) |
| Top thread roles | `ov2_est` 14.8%, `ov2_main` 6.7%, `ov2_map` 5.7% |

![](V1_02_medium/V1_02_medium_ape_xy.png)

![](V1_02_medium/V1_02_medium_rpe_trans.png)

![](V1_02_medium/ov2slam_cpu_usage.png)

![](V1_02_medium/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V1_02_medium/ov2slam.log), [evo text](V1_02_medium/V1_02_medium_evo.txt), [timings csv](V1_02_medium/ov2slam_timings.csv), [thread roles](V1_02_medium/ov2slam_named_thread_roles_aggregated.csv)

### V1_03_difficult

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.621162 / 0.669767 / 0.250491 m |
| Matched timestamps | 775/777 (99.7%) |
| FE mean / std / Hz | 6.1049 ms / 3.2407 ms / 163.8 Hz |
| Process CPU mean / std / max | 46.57% ± 32.01% (max 168.69%) |
| Top thread roles | `ov2_est` 9.1%, `ov2_main` 7.9%, `ov2_map` 4.6% |

![](V1_03_difficult/V1_03_difficult_ape_xy.png)

![](V1_03_difficult/V1_03_difficult_rpe_trans.png)

![](V1_03_difficult/ov2slam_cpu_usage.png)

![](V1_03_difficult/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V1_03_difficult/ov2slam.log), [evo text](V1_03_difficult/V1_03_difficult_evo.txt), [timings csv](V1_03_difficult/ov2slam_timings.csv), [thread roles](V1_03_difficult/ov2slam_named_thread_roles_aggregated.csv)

### V2_01_easy

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.083026 / 0.086232 / 0.023293 m |
| Matched timestamps | 332/335 (99.1%) |
| FE mean / std / Hz | 4.9082 ms / 1.9155 ms / 203.7 Hz |
| Process CPU mean / std / max | 42.01% ± 33.28% (max 169.70%) |
| Top thread roles | `ov2_est` 13.0%, `ov2_main` 5.8%, `ov2_map` 3.6% |

![](V2_01_easy/V2_01_easy_ape_xy.png)

![](V2_01_easy/V2_01_easy_rpe_trans.png)

![](V2_01_easy/ov2slam_cpu_usage.png)

![](V2_01_easy/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V2_01_easy/ov2slam.log), [evo text](V2_01_easy/V2_01_easy_evo.txt), [timings csv](V2_01_easy/ov2slam_timings.csv), [thread roles](V2_01_easy/ov2slam_named_thread_roles_aggregated.csv)

### V2_02_medium

| Metric | Value |
| --- | --- |
| APE mean / RMSE / std | 0.170448 / 0.203507 / 0.111188 m |
| Matched timestamps | 737/740 (99.6%) |
| FE mean / std / Hz | 5.7925 ms / 2.5753 ms / 172.6 Hz |
| Process CPU mean / std / max | 51.88% ± 37.23% (max 154.18%) |
| Top thread roles | `ov2_est` 14.1%, `ov2_map` 7.8%, `ov2_main` 7.4% |

![](V2_02_medium/V2_02_medium_ape_xy.png)

![](V2_02_medium/V2_02_medium_rpe_trans.png)

![](V2_02_medium/ov2slam_cpu_usage.png)

![](V2_02_medium/ov2slam_named_thread_roles_aggregated.png)

Links: [log](V2_02_medium/ov2slam.log), [evo text](V2_02_medium/V2_02_medium_evo.txt), [timings csv](V2_02_medium/ov2slam_timings.csv), [thread roles](V2_02_medium/ov2slam_named_thread_roles_aggregated.csv)

