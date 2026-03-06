#!/bin/bash
# Start the ROS2 perception stack container
sudo docker rm -f ros2_perception_stack 2>/dev/null || true

sudo docker run -it --rm \
  --name ros2_perception_stack \
  --net=host \
  --ipc=host \
  --privileged \
  --device=/dev/hailo0:/dev/hailo0 \
  -v $(pwd):/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/bus/usb:/dev/bus/usb \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  ros2_perception_stack /bin/bash
