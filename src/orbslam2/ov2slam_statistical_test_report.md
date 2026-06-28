# OV²SLAM Statistical Test Report

**Batch:** `20260604_203944`  
**Generated:** 2026-06-05 01:40 UTC+4  
**Mode:** stereo · accurate  
**Sequences:** MH_01_easy, MH_02_easy  
**Runs per sequence:** 3  

---

## 2 × 3 Trajectory Grid

Each cell: trajectory XY overlay · **APE RMSE** · GT path covered % · pose count.

| Sequence | Run 1 | Run 2 | Run 3 |
| --- | --- | --- | --- |
| **MH_01_easy** | <div align="center"><img src="../ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_statistical/20260604_203944/MH_01_easy/run_01/traj_xy.png" width="220" alt="run_01"/><br/><b>0.0371 m</b><br/><span>GT path: 97.5%</span><br/><small>415 poses</small></div> | <div align="center"><img src="../ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_statistical/20260604_203944/MH_01_easy/run_02/traj_xy.png" width="220" alt="run_02"/><br/><b>0.0434 m</b><br/><span>GT path: 97.8%</span><br/><small>418 poses</small></div> | <div align="center"><img src="../ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_statistical/20260604_203944/MH_01_easy/run_03/traj_xy.png" width="220" alt="run_03"/><br/><b>0.0362 m</b><br/><span>GT path: 97.8%</span><br/><small>420 poses</small></div> |
| **MH_02_easy** | <div align="center"><img src="../ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_statistical/20260604_203944/MH_02_easy/run_01/traj_xy.png" width="220" alt="run_01"/><br/><b>0.0350 m</b><br/><span>GT path: 97.5%</span><br/><small>420 poses</small></div> | <div align="center"><img src="../ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_statistical/20260604_203944/MH_02_easy/run_02/traj_xy.png" width="220" alt="run_02"/><br/><b>0.0383 m</b><br/><span>GT path: 97.4%</span><br/><small>413 poses</small></div> | <div align="center"><img src="../ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_statistical/20260604_203944/MH_02_easy/run_03/traj_xy.png" width="220" alt="run_03"/><br/><b>0.0424 m</b><br/><span>GT path: 97.6%</span><br/><small>407 poses</small></div> |

---

## Statistics

| Sequence | Run 1 | Run 2 | Run 3 | Mean | Median | Std | GT cov mean | GT cov median | GT cov std |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| MH_01_easy | 0.0371 | 0.0434 | 0.0362 | 0.0389 | 0.0371 | 0.0032 | 97.7% | 97.8% | 0.1% |
| MH_02_easy | 0.0350 | 0.0383 | 0.0424 | 0.0386 | 0.0383 | 0.0031 | 97.5% | 97.5% | 0.1% |

---

## Per-Run Detail

| Sequence | Run | RMSE (m) | GT path % | Poses | CPU mean % | FE mean (ms) | Status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| MH_01_easy | run_01 | 0.0371 | 97.5% | 415 | 152.4 | 20.0 | ✓ |
| MH_01_easy | run_02 | 0.0434 | 97.8% | 418 | 160.7 | 20.6 | ✓ |
| MH_01_easy | run_03 | 0.0362 | 97.8% | 420 | 160.3 | 19.2 | ✓ |
| MH_02_easy | run_01 | 0.0350 | 97.5% | 420 | 154.4 | 19.4 | ✓ |
| MH_02_easy | run_02 | 0.0383 | 97.4% | 413 | 155.3 | 19.2 | ✓ |
| MH_02_easy | run_03 | 0.0424 | 97.6% | 407 | 148.9 | 18.6 | ✓ |

---

## Verdict

**All 6/6 runs succeeded.** Zero failures, zero early terminations.

| Metric | Value |
| --- | --- |
| Overall mean RMSE | 0.0387 m |
| Overall median RMSE | 0.0377 m |
| Overall RMSE std | 0.0031 m |
| Mean GT path coverage | 97.6% |
| Min GT path coverage | 97.4% |
| SIGINT fallback (expected) | 6/6 runs — normal OV²SLAM behavior |

### ✅ Ready for full 10-sequence × 10-run deployment

**Evidence:**
- All runs complete with `success=true`
- GT path coverage **97.4–97.8%** — no premature tracking deaths
- RMSE std ≤ **0.0032 m** per sequence — highly repeatable
- RMSE matches paper: MH_01 paper=0.040 m, measured median=0.0371 m ✓
- All artifacts present across all 6 runs: trajectory.tum, traj_xy.png, ape_traj.png, CPU CSVs, thread role plots, timings
- `named_thread_roles_aggregated.png` generating correctly

**Before launching full 10×10 flight:**
- Verify `experiment_configuration.yaml` has all 10 sequences and `n_runs: 10`
- Run `ulimit -c 0` in the container to suppress core dumps
- Estimated wall time: ~2–4 days continuous