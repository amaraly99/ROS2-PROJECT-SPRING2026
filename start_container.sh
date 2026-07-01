#!/bin/bash
set -euo pipefail

WS="$(cd "$(dirname "$0")" && pwd)"

# All stack packages except ov2slam (SLAM is optional, build separately if needed)
BUILD_PKGS="sim_camera_bridge ovcam_bridge yolo_bridge oracle_detector \
            hil_servo visp_servo h_vs_servo visp_pbvs_servo"

# ── 1. Kill any existing container ────────────────────────────────────────────
sudo docker rm -f ros2_perception_stack 2>/dev/null || true

# ── 2. Start container (detached) ─────────────────────────────────────────────
echo "[1/3] Starting container..."
sudo docker run -d \
  --entrypoint "" \
  --name ros2_perception_stack \
  --net=host --ipc=host --privileged \
  --device=/dev/hailo0:/dev/hailo0 \
  -v "${WS}:/workspace" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/bus/usb:/dev/bus/usb \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  ros2_perception_stack sleep infinity

# ── 3. Install system packages ─────────────────────────────────────────────────
echo "[2/3] Installing system packages (gettext-base, figlet, iproute2)..."
sudo docker exec ros2_perception_stack bash -c "
    apt-get update -qq && \
    apt-get install -y --no-install-recommends gettext-base figlet iproute2 2>&1 | tail -5
"

# ── 4. Build all stack packages (no ov2slam) ───────────────────────────────────
echo "[3/3] Building packages: ${BUILD_PKGS}..."
sudo docker exec ros2_perception_stack bash -lc "
    source /opt/ros/jazzy/setup.bash
    cd /workspace
    colcon build --symlink-install --base-paths src \
        --packages-up-to ${BUILD_PKGS} 2>&1
" || { echo "ERROR: colcon build failed — check output above."; exit 1; }

echo ""
echo "Done. Container is ready."
echo "Run ./enter_container.sh to open a shell."
