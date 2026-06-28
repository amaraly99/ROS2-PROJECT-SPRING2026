# ORB-SLAM2 -- HIL Integration

ORB-SLAM2 monocular SLAM sidecar. Publishes /slam_pose (geometry_msgs/PoseStamped) each tracked frame.

## Build

Run from this directory (src/orbslam2/):

    ./start_container

What start_container does:
1. Starts orbslam2_fixed:latest Docker container, mounting PWD as /workspace
2. Rebuilds ORB-SLAM2 core with cmake (fast if build/ cache is present)
3. Copies .so libs to /usr/local/lib and runs ldconfig
4. Runs colcon build for the ROS2 wrapper -> installs into install_stereo/
5. Verifies shared-library links with ldd

After completion the container exits; install_stereo/ is the build artifact.

## Test with EuRoC bag

Terminal 1 -- run mono inside orbslam2_fixed container:

    sudo docker run --rm -it --net=host \
      -v /home/amaraly/ROS2-PROJECT-SPRING2026/src/orbslam2:/workspace \
      orbslam2_fixed:latest bash -lc \
      "source /opt/ros/jazzy/setup.bash && \
        source /root/ws/install/setup.bash && \
        source /workspace/install_stereo/setup.bash && \
        export LD_LIBRARY_PATH=/workspace/lib:/usr/local/lib:$LD_LIBRARY_PATH && \
        ros2 run orbslam mono \
          /workspace/Vocabulary/ORBvoc.txt \
          /workspace/ros2-ORB_SLAM2/src/monocular/EuRoC.yaml \
          --ros-args -r camera:=/cam0/image_raw"

Terminal 2 -- replay bag:

    ros2 bag play /home/amaraly/ROS2-PROJECT-SPRING2026/datasets/euroc/MH_01_easy.db3 --rate 1.0

Check topic:

    ros2 topic hz /slam_pose

## HIL stack integration

Config: config/hil/stack/orbslam2_eval.yaml

    ./run_stack_hil.sh --config orbslam2_eval

The sidecar runs mono subscribed to /camera (from sim_camera_bridge).
Topic /slam_pose publishes at ~30 Hz when ORB-SLAM2 is tracking.

## Camera settings

- EuRoC bags:    ros2-ORB_SLAM2/src/monocular/EuRoC.yaml
- HIL simulator: ros2-ORB_SLAM2/src/monocular/hil_sim.yaml
  640x480, fx=fy=554, cx=320, cy=240, no distortion (synthetic camera)

## Vocabulary

Vocabulary/ORBvoc.txt (318 MB) must exist.
start_container auto-extracts from the .tar.gz archive if only the archive is present.
