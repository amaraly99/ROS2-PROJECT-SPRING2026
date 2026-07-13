# EuRoC dataset replay

Used to evaluate OV²SLAM against ground truth, independently of the live camera.

## 1. Get a sequence

Download a ROS bag sequence (e.g. `MH_01_easy`) from the
[EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
into `datasets/` (git-ignored).

If you only have a ROS 1 bag, convert it:

```bash
ros2 bag convert -i MH_01_easy.bag -o MH_01_easy_bag2 -s rosbag_v2
```

## 2. Run SLAM against it

```bash
export LD_LIBRARY_PATH=/workspace/opencv/build/lib:$LD_LIBRARY_PATH
ros2 run ov2slam ov2slam_node /workspace/camera_calib/euroc_mono.yaml
```

In another terminal:

```bash
ros2 bag play /workspace/datasets/MH_01_easy_bag2 --clock
```

`benchmarks/euroc_publisher.py` replays a raw (non-bag) EuRoC sequence over ROS 2 if you prefer.

## 3. Evaluate

OV²SLAM writes trajectories to the working directory: `ov2slam_traj.txt` (TUM),
`ov2slam_kfs_traj.txt` (keyframes), `ov2slam_traj_kitti.txt` (KITTI).

```bash
pip install evo
evo_ape tum datasets/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
        ov2slam_traj.txt -va --plot
```

`benchmarks/run_benchmark.sh` automates a full multi-config, multi-run sweep and
`benchmarks/aggregate_slam_results.py` collapses the results into a summary table.

## Ground truth from the HIL simulator

For the Simulink rig rather than EuRoC, `scripts/bag_to_tum_gt.py` extracts `/sim/drone_pose` from a
recorded bag and writes it in TUM format, so the same `evo` tooling applies.
