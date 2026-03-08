#!/bin/bash
# Start the ROS2 perception stack container (detached — no interactive tty).
# Use run_stack.sh to launch the full pipeline.
# Use enter_container.sh to open a shell inside.
sudo docker rm -f ros2_perception_stack 2>/dev/null || true

sudo docker run -d \
  --entrypoint "" \
  --name ros2_perception_stack \
  --net=host \
  --ipc=host \
  --privileged \
  --device=/dev/hailo0:/dev/hailo0 \
  -v "$(pwd)":/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/bus/usb:/dev/bus/usb \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  ros2_perception_stack sleep infinity

echo "Container started (detached). Use ./enter_container.sh to open a shell."
