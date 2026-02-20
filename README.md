# OV²SLAM — Embedded Build & Precision Evaluation Guide

This document describes the validated embedded build and execution workflow used to achieve **≈ 0.06 m RMSE** on the **EuRoC MH_05_difficult (Stereo)** sequence.

It consolidates:

* Lightweight dependency-minimal build (no SuiteSparse)
* Ceres + Eigen sparse configuration requirements
* Deterministic execution protocol
* Explicit Global Bundle Adjustment trigger
* Reproducible evaluation procedure

---

# Target Configuration

* **Dataset:** EuRoC MH_05_difficult (Stereo)
* **Metric:** Absolute Pose Error (APE)
* **Expected RMSE:** ≈ 0.06 m
* **ROS Version:** ROS 2 Jazzy
* **Platform:** Embedded (SuiteSparse disabled)
* **Evaluation Tool:** evo_ape

---

# Part 1 — System Preparation & Build

## 1. Remove SuiteSparse

```bash
sudo apt-get purge -y 'libsuitesparse*'
sudo apt-get autoremove -y
```

---

## 2. Resolve ROS Dependencies

```bash
cd ~/src
source /opt/ros/jazzy/setup.bash
sudo apt update && sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 3. Build Thirdparty Libraries

```bash
cd src/ov2slam_ros/
./build_thirdparty.sh
```

### Important — Ceres Configuration

During Ceres build, ensure the output includes:

```
CERES_USE_EIGEN_SPARSE
```

If not enabled:

```bash
sudo apt install libeigen3-dev
```

Failure to enable Eigen sparse may cause premature termination with:

```
Function tolerance reached
```

---

## 4. Build Workspace

```bash
colcon build \
  --packages-select ov2slam \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

---

# Part 2 — Execution Protocol (Precision Run)

**Objective:** Run dataset at 1× speed, ensure simulated time synchronization, and explicitly trigger end-of-sequence optimization.

Use separate terminals (tmux recommended).

---

## Terminal 1 — OV²SLAM Node

```bash
source install/setup.bash
ros2 run ov2slam ov2slam_node \
  ./src/ov2slam_ros/parameters_files/accurate/euroc/euroc_stereo.yaml \
  --ros-args -p use_sim_time:=true
```

---

## Terminal 2 — Dataset Playback

```bash
ros2 bag play \
  ./datasets/MH_05_difficult/MH_05_difficult.db3 \
  --clock -r 1.0
```

---

## Terminal 3 — Visualization (Optional)

```bash
source install/setup.bash
ros2 run foxglove_bridge foxglove_bridge --port 8765
```

---

# Manual End-of-Sequence Trigger (Clock Kick)

When bag playback finishes, `/clock` stops publishing.
OV²SLAM will not finalize automatically.

Force a simulated time advance:

```bash
ros2 topic pub --once /clock rosgraph_msgs/msg/Clock \
  "{clock: {sec: 1450000000, nanosec: 0}}"
```

This triggers:

* Global Bundle Adjustment
* Trajectory export

---

# Part 3 — Verification & Evaluation

## 1. Confirm in Logs

Wait for:

* `[GlobalBundleAdjustment] Starting...`
* `Kfs Trajectory file written!`
* `Full Trajectory w. LC file written!`

Expected delay: 30–60 seconds.

---

## 2. Output File

```
ov2slam_fullba_kfs_traj.txt
```

Located in workspace root.

---

## 3. Evaluate Accuracy

```bash
evo_ape tum \
  ./datasets/MH_05_difficult/gt.tum \
  ov2slam_fullba_kfs_traj.txt \
  --align --correct_scale \
  --plot --plot_mode xy \
  --t_max_diff 0.1
```

---

# Expected Result

```
RMSE ≈ 0.06 m
```

Observed stable range: 0.06 – 0.08 m.

---

# Determinism Note

Deviating from:

* 1× playback rate
* Proper simulated time
* Explicit clock trigger

may prevent Global Bundle Adjustment from
