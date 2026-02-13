This guide consolidates your embedded build environment with the "Golden Standard" execution workflow we developed to achieve **0.06m RMSE**. It includes the critical C++ optimization, the specific YAML tuning, and the "Clock Kick" technique required to force the Global Bundle Adjustment save.

```markdown
# OV²SLAM: Embedded Build & Precision Evaluation Guide
> **Target:** 0.06m RMSE on `MH_05_difficult` (Stereo)  
> **Platform:** ROS 2 Jazzy (Embedded / No SuiteSparse)

---

## Part 1: System Prep & Build (Embedded Baseline)

> **Goal:** Clean build without heavy dependencies like SuiteSparse, ensuring a lightweight runtime.

### 1) Remove SuiteSparse Completely (System Level)
```bash
sudo apt-get purge -y 'libsuitesparse*'
sudo apt-get autoremove -y

```

### 2) Fix Ownership and Clean Thirdparty Artifacts

```bash
cd ./ov2slam_ros/ov2slam
sudo chown -R "$USER:$USER" Thirdparty
rm -rf Thirdparty/*/build Thirdparty/*/install

```

### 3) Build Ceres Solver (SuiteSparse Disabled)

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

### 4) ROS Dependency Resolution

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
sudo apt update && sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

```

### 5) Build Workspace

```bash
colcon build --packages-select ov2slam --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

```

---

## Part 2: Code & Configuration (The "Golden" Setup)

> **Goal:** Eliminate CPU overhead to allow 1x speed playback and enable "Final Boss" optimization.

### 1) The C++ "Silent Mode" Fix

**File:** `src/ov2slam_ros/src/ov2slam.cpp`

**Function:** `visualizeFinalKFsTraj`

**Action:** Replace the loop with this O(1) operation to prevent CPU choking.

```cpp
void SlamManager::visualizeFinalKFsTraj()
{
    // If nobody is listening, do nothing
    if( prosviz_->pub_final_kfs_traj_->get_subscription_count() == 0 ) return;

    // PUBLISH ONLY THE LATEST KEYFRAME (No History Looping)
    int last_kf_id = pcurframe_->kfid_;
    auto pkf = pmap_->getKeyframe(last_kf_id);
    if( pkf != nullptr ) {
        prosviz_->pubFinalKFsTraj(pkf->getTwc(), pkf->img_time_);
    }
}

```

### 2) The "Golden" YAML Config

**File:** `src/ov2slam_ros/parameters_files/accurate/euroc/euroc_stereo.yaml`

**Key Settings:**

```yaml
# ... (Camera Intrinsics/Extrinsics per standard EuRoC) ...

# 1. Disable Realtime Skipping (Force accurate processing)
force_realtime: 0

# 2. Feature Density "Goldilocks Zone"
# 15 = Crash, 35 = Sparse. 25 is perfect (~600 features).
nmaxdist: 25

# 3. Quality Threshold (Balanced)
nfast_th: 10
dmaxquality: 0.0001

# 4. THE MAGIC SWITCH (Enables Global Bundle Adjustment)
do_full_ba: 1

```

---

## Part 3: Execution Protocol (0.06m Run)

> **Goal:** Run at 1x speed, visualize remotely, and force the final save using the "Clock Kick".

### Terminal 1: The SLAM Node

Runs the node with simulated time to synchronize with the dataset.

```bash
source install/setup.bash
ros2 run ov2slam ov2slam_node ./src/ov2slam_ros/parameters_files/accurate/euroc/euroc_stereo.yaml --ros-args -p use_sim_time:=true

```

### Terminal 3: Visualization Bridge (Optional but Recommended)

Runs Foxglove Bridge for remote visualization on port 8765.

```bash
source install/setup.bash
ros2 run foxglove_bridge foxglove_bridge --port 8765

```

### Terminal 2: Dataset Playback & The "Clock Kick"

**Step A: Play the Dataset**
Plays the difficult sequence at real-time speed.

```bash
ros2 bag play ./datasets/MH_05_difficult/MH_05_difficult.db3 --clock -r 1.0

```

**Step B: The "Clock Kick" (Run IMMEDIATELY after playback finishes)**
When the bag finishes, the `/clock` topic stops, freezing the SLAM node. It will never save the trajectory on its own. You must manually send a future timestamp to trigger the "End of Sequence" logic.

```bash
# Force the clock forward to trigger timeout and Global Bundle Adjustment
ros2 topic pub --once /clock rosgraph_msgs/msg/Clock "{clock: {sec: 1450000000, nanosec: 0}}"

```

---

## Part 4: Verification & Retrieval

1. **Wait for Logs:** After the "Clock Kick", watch Terminal 1.
* You will see: `[GlobalBundleAdjustment] Starting...`
* Wait (approx. 30-60s).
* Look for: `Kfs Trajectory file written!` and `Full Trajectory w. LC file written!`


2. **Terminate:** Once the files are written, `Ctrl+C` Terminal 1.
3. **The Prize:** Locate the target file in your workspace root.
* **Target File:** `ov2slam_fullba_kfs_traj.txt`


4. **Evaluate:**
```bash
evo_ape tum ./datasets/MH_05_difficult/gt.tum ov2slam_fullba_kfs_traj.txt --align --correct_scale --plot --plot_mode xy --t_max_diff 0.1

```


* **Expected Result:** RMSE ≈ **0.06m**

# IMPORTANT FOR OV2SLAM
export this environment variable to prevent any missing .so files
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/workspace ros2_ws/src/ov2slam_ros/Thirdparty/ibow_lcd/build
```