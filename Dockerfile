# Dockerfile
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# Base tooling + locales
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl ca-certificates gnupg lsb-release \
    build-essential cmake git pkg-config \
    python3-pip python3-setuptools python3-wheel \
    figlet \
    htop \
    tmux \
    libeigen3-dev \
 && sed -i 's/^# *\(en_US\.UTF-8 UTF-8\)/\1/' /etc/locale.gen \
 && locale-gen \
 && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS 2 repo key + source list
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates gnupg lsb-release \
 && mkdir -p /etc/apt/keyrings \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS2 Jazzy + libs (NOTE: SuiteSparse intentionally NOT installed)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-dev-tools \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs \
    ros-jazzy-rclpy \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-foxglove-bridge \
    libgoogle-glog-dev \
    libgflags-dev \
    liblapack-dev \
    libblas-dev \
 && rm -rf /var/lib/apt/lists/*

# Extra safety: if any layer pulled SuiteSparse, purge it once at build-time
RUN apt-get update \
 && apt-get purge -y 'libsuitesparse*' || true \
 && apt-get autoremove -y || true \
 && rm -rf /var/lib/apt/lists/*

# System-wide env
RUN cat >/etc/profile.d/ros2_jazzy.sh <<'EOF' \
 && chmod +x /etc/profile.d/ros2_jazzy.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi
EOF

# Checker script (runs on container start)
RUN cat >/usr/local/bin/ros2_check.sh <<'EOF' \
 && chmod +x /usr/local/bin/ros2_check.sh
#!/usr/bin/env bash
set -e

# Always source ROS
source /opt/ros/jazzy/setup.bash

# Show ros2 path early (your requested “which ros2”)
echo
echo "which ros2: $(command -v ros2 || echo 'NOT FOUND')"

pkgs=(
  ros-jazzy-ros-base
  ros-dev-tools
  ros-jazzy-rmw-cyclonedds-cpp
  ros-jazzy-tf2-geometry-msgs
  ros-jazzy-nav-msgs
  ros-jazzy-image-transport
  ros-jazzy-cv-bridge
  ros-jazzy-sensor-msgs
  ros-jazzy-rclpy
  ros-jazzy-pcl-conversions
  ros-jazzy-pcl-ros
  ros-jazzy-foxglove-bridge
  libgoogle-glog-dev
  libgflags-dev
  liblapack-dev
  libblas-dev
)

missing=()
for p in "${pkgs[@]}"; do
  dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
done

# ROS identifiers
ros_distro="${ROS_DISTRO:-unknown}"
ros2_cli_ver="$(ros2 --version 2>/dev/null | awk '{print $2}' || echo 'unknown')"
ros_pkg_ver="$(dpkg-query -W -f='${Version}' ros-jazzy-ros-base 2>/dev/null || echo 'unknown')"

# Confirm SuiteSparse is gone (nice explicit check)
suite_cnt="$(dpkg -l | awk '{print $2}' | grep -ci '^libsuitesparse' || true)"

echo
figlet "ROS2 CHECK"
echo

if ! command -v ros2 >/dev/null 2>&1; then
  echo "❌ ros2 command not found in PATH"
  exit 1
fi

if [ ${#missing[@]} -eq 0 ]; then
  echo "✅ ROS2 distro        : ${ros_distro}"
  echo "✅ ROS2 CLI version   : ${ros2_cli_ver}"
  echo "✅ ros-base pkg ver   : ${ros_pkg_ver}"
  echo "✅ all libraries installed"
  if [ "$suite_cnt" -eq 0 ]; then
    echo "✅ SuiteSparse        : not installed"
  else
    echo "⚠️  SuiteSparse        : still present (${suite_cnt} pkgs)"
  fi
else
  echo "✅ ROS2 distro        : ${ros_distro}"
  echo "✅ ROS2 CLI version   : ${ros2_cli_ver}"
  echo "✅ ros-base pkg ver   : ${ros_pkg_ver}"
  echo "❌ missing packages:"
  printf '   - %s\n' "${missing[@]}"
  exit 2
fi

echo
EOF

ENTRYPOINT ["/bin/bash", "-lc", "/usr/local/bin/ros2_check.sh; exec bash"]
WORKDIR /workspace
