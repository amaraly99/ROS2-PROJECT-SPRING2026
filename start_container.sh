#!/bin/bash
# Start the ROS2 perception stack container, prepare dependencies, and rebuild OV2SLAM.

set -euo pipefail

cd "$(dirname "$0")"
WS="$(pwd)"
CONTAINER_NAME="ros2_perception_stack"

log_step() {
  printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$1"
}

docker_exec_bash() {
  sudo docker exec "$CONTAINER_NAME" bash -lc "$1"
}

clear
log_step "Checking whether ${CONTAINER_NAME} is already running..."

if sudo docker inspect --format='{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null | grep -q 'true'; then
  echo "Container '$CONTAINER_NAME' is already running."
  read -rp "Kill the existing container and start a new one? [y/N]: " answer
  if [[ "$answer" != "y" && "$answer" != "Y" ]]; then
    echo "Aborted. Existing container left running."
    exit 0
  fi
fi

log_step "Removing any stale ${CONTAINER_NAME} container..."
sudo docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

log_step "Starting Docker container in detached mode..."
sudo docker run -d \
  --entrypoint "" \
  --name "$CONTAINER_NAME" \
  --net=host \
  --ipc=host \
  --privileged \
  --device=/dev/hailo0:/dev/hailo0 \
  -v "${WS}:/workspace" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/bus/usb:/dev/bus/usb \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  ros2_perception_stack sleep infinity >/dev/null

log_step "1) Installing evo inside the container..."
docker_exec_bash '
  set -e
  pip install evo --break-system-packages
'

log_step "2) Rebuilding OpenGV inside /workspace/src/ov2slam_ros/Thirdparty/opengv..."
docker_exec_bash '
  set -e
  cd /workspace/src/ov2slam_ros/Thirdparty/opengv
  mkdir -p build
  cd build
  cmake ..
  if command -v sudo >/dev/null 2>&1; then
    sudo make -j4 install
  else
    make -j4 install
  fi
'

log_step "3) Rebuilding OV2SLAM with colcon..."
docker_exec_bash '
  set -e
  source /opt/ros/jazzy/setup.bash
  cd /workspace
  colcon build --packages-select ov2slam
'

log_step "4) Sourcing ROS 2 and workspace setup..."
docker_exec_bash '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  env >/dev/null
'

clear
figlet "ROS2 PERCEPTION STACK"

log_step "ROS 2 version check..."
docker_exec_bash '
  set -e
  source /opt/ros/jazzy/setup.bash
  echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
  if dpkg-query -W "ros-${ROS_DISTRO}-ros2cli" >/dev/null 2>&1; then
    dpkg-query -W -f="ros2cli package version=${Version}\n" "ros-${ROS_DISTRO}-ros2cli"
  fi
  ros2 pkg list >/dev/null
'

log_step "OS version check..."
docker_exec_bash '
  set -e
  . /etc/os-release
  echo "$PRETTY_NAME"
'

log_step "Docker version health check..."
sudo docker --version
sudo docker inspect --format='Container {{.Name}} running={{.State.Running}} status={{.State.Status}}' "$CONTAINER_NAME"

echo "Container started (detached). Use ./enter_container.sh to open a shell."
