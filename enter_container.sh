#!/bin/bash
# Enter the running ROS2 perception stack container

sudo docker exec -it ros2_perception_stack /bin/bash -c "
  export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\$LD_LIBRARY_PATH
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  figlet 'ROS2-PERCEPTION-STACK'
  echo ''
  echo 'Stack ready. Before launching, export:'
  echo '   export MATLAB_HOST_IP=<matlab-pc-ip>'
  echo '   export PI_INTERFACE=eth0'
  echo ''
  exec bash"
