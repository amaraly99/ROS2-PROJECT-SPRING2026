#!/bin/bash
# Run this from an SSH terminal WHILE a trial is flying (sim + stack up, after
# the "wait for camera frames to flow" step passes). It grabs ~3 s of the sim
# camera, then renders a CLAHE-off vs CLAHE-on comparison of your actual scene.
cd ~/ROS2-PROJECT-SPRING2026
rm -rf tmp_clahe_bag
echo "recording 3s of /sim/camera/image_raw ..."
sudo docker exec ros2_perception_stack bash -lc \
  "source /opt/ros/jazzy/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
   cd /workspace; timeout 3 ros2 bag record -o tmp_clahe_bag /sim/camera/image_raw" 2>/dev/null
if [ ! -d tmp_clahe_bag ]; then
  echo "no frame captured -- is a trial actually flying? (camera must be live)"; exit 1
fi
python3 scripts/hil_matrix/clahe_demo.py tmp_clahe_bag clahe_scene_compare.png \
  && echo "DONE -> ~/ROS2-PROJECT-SPRING2026/clahe_scene_compare.png"
