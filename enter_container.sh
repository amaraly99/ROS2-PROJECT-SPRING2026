#!/bin/bash
# Enter the running ROS2 perception stack container
figlet ROS2 Perception Stack
sudo docker exec -it ros2_perception_stack /bin/bash -c \
  "export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\$LD_LIBRARY_PATH && \
   source /opt/ros/jazzy/setup.bash && \
   source /workspace/install/setup.bash && \
   exec bash"
