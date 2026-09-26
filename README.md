# HIL SLAM Benchmarks: Stereo and Mono

Raspberry Pi 5 + MATLAB/Simulink/Unreal HIL. Scene `parking_lot_far`, 2.0 m/s,
~33.7 m scored path, Wi-Fi, SLAM unpinned (CPU is a share of 400%).
APE RMSE is Sim(3)-aligned, mean +/- SD over 10 trials.

## Stereo

**Accuracy: APE RMSE [m] by baseline**

| Backend | 0.36 m | 0.42 m | 0.54 m | 0.61 m |
|---|---:|---:|---:|---:|
| **OV2SLAM-accurate** | **0.100 +/- 0.035** | **0.096 +/- 0.016** | **0.096 +/- 0.008** | **0.092 +/- 0.015** |
| OV2SLAM-fast | 0.155 +/- 0.038 | 0.152 +/- 0.026 | 0.142 +/- 0.032 | 0.140 +/- 0.033 |
| ORB-SLAM2 | 0.217 +/- 0.022 | 0.190 +/- 0.032 | 0.255 +/- 0.054 | 0.206 +/- 0.039 |
| ORB-SLAM3 | 0.207 +/- 0.043 | 0.233 +/- 0.030 | 0.243 +/- 0.033 | 0.197 +/- 0.030 |
| RTAB-Map | 0.220 +/- 0.041 | 0.218 +/- 0.041 | 0.193 +/- 0.035 | 0.174 +/- 0.029 |

**Latency and rate** (mean over the 40 trials at 0.36 to 0.61 m)

| Backend | Frame time [ms] | p95 [ms] | Capacity [Hz] | Input [Hz] | CPU mean [%] | CPU max [%] |
|---|---:|---:|---:|---:|---:|---:|
| OV2SLAM-fast | 3.3 | 6.7 | 302 | 10.9 | 38 | 58 |
| OV2SLAM-accurate | 8.1 | 13.6 | 123 | 10.9 | 56 | 94 |
| ORB-SLAM3 | 30.2 | 39.0 | 33 | 11.0 | 67 | 131 |
| ORB-SLAM2 | 42.1 | 53.0 | 24 | 11.0 | 85 | 144 |
| RTAB-Map | 81.9 | 100.6 | 12 | 7.8 | 116 | 272 |

## Mono

**Accuracy**

| Backend | Valid | APE RMSE [m] | Median [m] | Scale |
|---|---:|---:|---:|---:|
| **OV2SLAM-accurate** | 10/10 | **0.106 +/- 0.027** | **0.095** | 2.6 |
| ORB-SLAM2 | 5/10 | 0.194 +/- 0.122 | 0.152 | 39.9 |
| ORB-SLAM3 | 10/10 | 0.375 +/- 0.474 | 0.212 | 46.6 |
| OV2SLAM-fast | 0/10 | n/a | n/a | n/a |

**Latency and rate**

| Backend | Frame time [ms] | p95 [ms] | Capacity [Hz] | Input [Hz] | CPU mean [%] | CPU max [%] |
|---|---:|---:|---:|---:|---:|---:|
| OV2SLAM-accurate | 6.4 | 11.9 | 157 | 16.0 | 48 | 74 |
| ORB-SLAM2 | 25.4 | 39.5 | 39 | 16.1 | 91 | 156 |
| ORB-SLAM3 | 28.5 | 68.0 | 35 | 15.9 | 86 | 139 |
| OV2SLAM-fast | n/a | n/a | n/a | n/a | n/a | n/a |

OV2SLAM-fast mono never initialised on this flight (0/10).
