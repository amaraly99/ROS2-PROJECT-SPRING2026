# OV²SLAM — Embedded Build & Precision Evaluation Guide

This guide documents the **embedded, dependency-minimal build** and the **validated execution workflow** used to achieve **≈ 0.06 m RMSE** on the **EuRoC MH_05_difficult (Stereo)** sequence.

It consolidates:
- The lightweight C++ build configuration (no SuiteSparse)
- Critical runtime optimizations to avoid CPU bottlenecks
- The validated YAML parameter set
- The execution protocol required to reliably trigger **Global Bundle Adjustment** and trajectory export

---

## Target Configuration

- **Dataset:** EuRoC MH_05_difficult (Stereo)
- **Metric:** Absolute Pose Error (APE), RMSE ≈ **0.06 m**
- **ROS:** ROS 2 Jazzy
- **Platform:** Embedded (SuiteSparse disabled)
- **Evaluation:** `evo_ape`

---

## Part 1 — System Preparation & Build

**Goal:** Produce a clean, lightweight OV²SLAM build without SuiteSparse or heavy sparse backends.

### 1. Remove SuiteSparse (System Level)

```bash
sudo apt-get purge -y 'libsuitesparse*'
sudo apt-get autoremove -y
````

---

### 2. Fix Ownership and Clean Third-Party Artifacts

```bash
cd ./ov2slam_ros/ov2slam
sudo chown -R "$USER:$USER" Thirdparty
rm -rf Thirdparty/*/build Thirdparty/*/install
```

---

### 3. Build Ceres Solver (SuiteSparse Disabled)

```bash
cd ./ov2slam_ros/ov2slam/Thirdparty/ceres-solver
rm -rf build install
mkdir -p build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DSUITESPARSE=OFF \
  -DCXSPARSE=OFF \
  -DEIGENSPARSE=OFF \
  -DGLOG=OFF \
  -DMINIGLOG=ON \
  -DBUILD_TESTING=OFF

make -j"$(nproc)"
```

---

### 4. Resolve ROS Dependencies

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
sudo apt update && sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

### 5. Build the Workspace

```bash
colcon build \
  --packages-select ov2slam \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

---

## Part 2 — Code & Configuration (Validated Setup)

**Goal:** Minimize CPU overhead and ensure the Global Bundle Adjustment phase executes reliably.

---

### 1. Visualization CPU Optimization (“Silent Mode”)

**File:**
`src/ov2slam_ros/src/ov2slam.cpp`

**Function:**
`visualizeFinalKFsTraj`

**Rationale:**
Publishing the full keyframe history causes unnecessary CPU load on embedded systems and can prevent the final optimization phase from running. Replace the loop with a constant-time operation.

```cpp
void SlamManager::visualizeFinalKFsTraj()
{
    // Exit early if no subscribers are listening
    if (prosviz_->pub_final_kfs_traj_->get_subscription_count() == 0)
        return;

    // Publish only the latest keyframe
    int last_kf_id = pcurframe_->kfid_;
    auto pkf = pmap_->getKeyframe(last_kf_id);
    if (pkf != nullptr) {
        prosviz_->pubFinalKFsTraj(pkf->getTwc(), pkf->img_time_);
    }
}
```

---

### 2. Validated YAML Configuration

**File:**
`src/ov2slam_ros/parameters_files/accurate/euroc/euroc_stereo.yaml`

```yaml
# Camera intrinsics/extrinsics omitted for brevity
# Use standard EuRoC calibration values

# Disable real-time frame skipping
force_realtime: 0

# Feature density (empirically validated)
# ~600 features per frame
nmaxdist: 25

# FAST + descriptor quality thresholds
nfast_th: 10
dmaxquality: 0.0001

# Enable Global Bundle Adjustment
do_full_ba: 1
```

---

## Part 3 — Execution Protocol (Precision Run)

**Goal:** Run the dataset at 1× speed, ensure synchronization, and explicitly trigger end-of-sequence optimization.

---

### Terminal 1 — OV²SLAM Node

```bash
source install/setup.bash
ros2 run ov2slam ov2slam_node \
  ./src/ov2slam_ros/parameters_files/accurate/euroc/euroc_stereo.yaml \
  --ros-args -p use_sim_time:=true
```

---

### Terminal 2 — Dataset Playback

```bash
ros2 bag play \
  ./datasets/MH_05_difficult/MH_05_difficult.db3 \
  --clock -r 1.0
```

---

### Terminal 3 — Visualization Bridge (Optional)

```bash
source install/setup.bash
ros2 run foxglove_bridge foxglove_bridge --port 8765
```

---

### Manual End-of-Sequence Trigger (“Clock Kick”)

When bag playback finishes, `/clock` stops publishing.
OV²SLAM will **not** automatically finalize or save trajectories in this state.

Manually advance the simulated clock to trigger the internal timeout logic:

```bash
ros2 topic pub --once /clock rosgraph_msgs/msg/Clock \
  "{clock: {sec: 1450000000, nanosec: 0}}"
```

This forces:

* Global Bundle Adjustment
* Trajectory export

---

## Part 4 — Verification & Results

### 1. Monitor Logs

In **Terminal 1**, wait for:

* `[GlobalBundleAdjustment] Starting...`
* `Kfs Trajectory file written!`
* `Full Trajectory w. LC file written!`

(≈ 30–60 seconds)

---

### 2. Retrieve Output

**File:**

```text
ov2slam_fullba_kfs_traj.txt
```

Located in the workspace root.

---

### 3. Evaluate Accuracy

```bash
evo_ape tum \
  ./datasets/MH_05_difficult/gt.tum \
  ov2slam_fullba_kfs_traj.txt \
  --align --correct_scale \
  --plot --plot_mode xy \
  --t_max_diff 0.1
```

**Expected Result:**
**RMSE ≈ 0.06 m**

---

## Runtime Environment Note

To avoid missing shared library issues, export the following:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/src/ov2slam_ros/Thirdparty/ibow_lcd/build
```

---

## Summary

This workflow prioritizes:

* Deterministic execution
* Minimal dependencies
* Embedded-friendly performance
* Reproducible precision results

Deviating from these steps (especially visualization behavior or clock handling) may prevent Global Bundle Adjustment from running and invalidate the final trajectory.

