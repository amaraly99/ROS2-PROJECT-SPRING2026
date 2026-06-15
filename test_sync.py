import sys
from evo.tools import file_interface
from evo.core import sync

ref = file_interface.read_tum_trajectory_file("/home/amaraly/ORBSLAM3_ROS2/results/orbslam_benchmark/20260608_101634/MH_01_easy/run_01/gt.tum")
est = file_interface.read_tum_trajectory_file("/home/amaraly/ORBSLAM3_ROS2/results/orbslam_benchmark/20260608_101634/MH_01_easy/run_01/trajectory.tum")

ref_sync, est_sync = sync.associate_trajectories(ref, est, max_diff=0.01)
print(f"max_diff=0.01: {est_sync.num_poses}")
ref_sync, est_sync = sync.associate_trajectories(ref, est, max_diff=0.05)
print(f"max_diff=0.05: {est_sync.num_poses}")
