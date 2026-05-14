#!/bin/bash
# run_stack_hil.sh — start (or stop) the perception+servo stack in HIL mode
#
# HIL = Hardware-In-the-Loop. The OV5647 physical camera is replaced by a
# synthetic ROS image stream from MATLAB/Simulink/Unreal NYC scene published
# on /sim/camera/image_raw. Everything downstream (Hailo NPU YOLO, OV2SLAM,
# ViSP, /cmd_vel) runs unchanged on real RPi5 hardware.
#
# Architecture:
#   MATLAB (Windows) → /sim/camera/image_raw  (rgb8, 640x480, ~20 Hz)
#                          ↓
#   sim_camera_bridge (Docker, NEW) → writes /ovcam_frames POSIX SHM (NV12)
#                          ↓
#   ovcam_bridge + yolo_producer (host) + yolo_bridge → ov2slam → visp_servo
#                          ↓
#   /cmd_vel → MATLAB → drone dynamics → Unreal scene update
#
# Core layout (Pi 5, 4 cores) — same as run_stack.sh:
#   Core 0     sim_camera_bridge + ovcam_bridge + yolo_bridge + visp_servo
#   Core 1     yolo_producer  (Hailo NPU — DEDICATED)
#   Core 2,3   OV2SLAM
#
# Usage:
#   ./run_stack_hil.sh [--no-slam]
#   ./run_stack_hil.sh stop
#   ./run_stack_hil.sh hz [topic]      # rate check with HIL DDS profile loaded
#
# Env vars (override before invoking):
#   MATLAB_HOST_IP   — Windows host IP for unicast DDS discovery (REQUIRED)
#   DDS              — cyclonedds (default) | fastrtps
#   ROS_DOMAIN_ID    — defaults to 0

cd "$(dirname "$0")"
WS="$(pwd)"
OPT="${1:-}"

DDS="${DDS:-fastrtps}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

log()  { echo "[hil] $*"; }
die()  { echo "[hil] ERROR: $*" >&2; exit 1; }
sep()  { echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; }

# ── stop ──────────────────────────────────────────────────────────────────────
if [[ "$OPT" == "stop" ]]; then
    log "Stopping HIL stack..."
    sudo docker kill ros2_perception_stack 2>/dev/null || true
    sudo docker rm  ros2_perception_stack 2>/dev/null || true
    sudo pkill -f yolo_producer 2>/dev/null || true
    sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm \
               /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready 2>/dev/null || true
    log "All stopped."
    exit 0
fi

# ── hz <topic> — quick rate check with the right DDS env loaded ──────────────
if [[ "$OPT" == "hz" ]]; then
    TOPIC="${2:-/sim/camera/image_raw}"
    RESOLVED="/tmp/fastrtps_hil.resolved.xml"
    [[ -f "$RESOLVED" ]] || die "$RESOLVED not found — start the stack first"
    DOCKER_TTY_FLAGS=""
    [[ -t 0 && -t 1 ]] && DOCKER_TTY_FLAGS="-it"
    sudo docker exec $DOCKER_TTY_FLAGS ros2_perception_stack bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export FASTRTPS_DEFAULT_PROFILES_FILE=${RESOLVED}
        export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
        ros2 topic hz ${TOPIC}
    "
    exit 0
fi

# ── pre-flight ────────────────────────────────────────────────────────────────
command -v taskset >/dev/null    || die "'taskset' not found — install util-linux"
command -v envsubst >/dev/null   || die "'envsubst' not found — install gettext-base"
[[ -e /dev/hailo0 ]]             || die "/dev/hailo0 not found — Hailo HAT must be powered (yolo_producer needs it)"
MATLAB_HOST_IP="${MATLAB_HOST_IP:-192.168.56.1}"
# Detect Pi's local IP on the same L2 segment as MATLAB (needed for interface whitelist)
PI_LOCAL_IP=$(ip route get "${MATLAB_HOST_IP}" | grep -oP 'src \K[0-9.]+' | head -1)
[[ -z "$PI_LOCAL_IP" ]] && die "Cannot determine local IP toward ${MATLAB_HOST_IP} — is eth0 up?"

# Resolve DDS profile path + RMW
case "$DDS" in
    cyclonedds)
        RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
        DDS_TEMPLATE="${WS}/config/hil/cyclonedds_hil.xml"
        DDS_RESOLVED="/tmp/cyclonedds_hil.resolved.xml"
        DDS_ENV_VAR="CYCLONEDDS_URI=file://${DDS_RESOLVED}"
        ;;
    fastrtps)
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
        DDS_TEMPLATE="${WS}/config/hil/fastrtps_hil.xml"
        DDS_RESOLVED="/tmp/fastrtps_hil.resolved.xml"
        DDS_ENV_VAR="FASTRTPS_DEFAULT_PROFILES_FILE=${DDS_RESOLVED}"
        ;;
    *)
        die "Unknown DDS=${DDS}; expected 'cyclonedds' or 'fastrtps'"
        ;;
esac
[[ -f "$DDS_TEMPLATE" ]] || die "DDS template not found: $DDS_TEMPLATE"

# Kernel UDP buffers + IP fragment reassembly: MATLAB publishes raw 640x480 rgb8
# frames (~921 KB) which fragment into ~660 UDP datagrams each at MTU 1500. The
# stock RPi5 settings (rmem_max=208 KB, ipfrag_high_thresh=4 MB) drop fragments
# under load and cap the receive rate near 13 Hz. Bumping to 16 MB / 32 MB lets
# the full 30+ Hz stream through. Idempotent: only applies if not already set.
NEEDED_RMEM=16777216
NEEDED_FRAG_HIGH=33554432
CUR_RMEM=$(sysctl -n net.core.rmem_max)
CUR_FRAG=$(sysctl -n net.ipv4.ipfrag_high_thresh)
if [[ "$CUR_RMEM" -lt "$NEEDED_RMEM" || "$CUR_FRAG" -lt "$NEEDED_FRAG_HIGH" ]]; then
    log "Tuning kernel for MATLAB→Pi frame stream (sudo sysctl)..."
    sudo sysctl -q \
        net.core.rmem_max=16777216 \
        net.core.rmem_default=16777216 \
        net.core.wmem_max=16777216 \
        net.core.wmem_default=16777216 \
        net.ipv4.ipfrag_high_thresh=33554432 \
        net.ipv4.ipfrag_low_thresh=25165824 \
        net.ipv4.ipfrag_time=3 \
        || die "sysctl tuning failed — DDS frame rate will be capped"
fi

# Substitute MATLAB_HOST_IP into the DDS profile (avoids hardcoded IPs in the repo)
# Use sudo tee as fallback in case the file was previously created by root
MATLAB_HOST_IP="$MATLAB_HOST_IP" PI_LOCAL_IP="$PI_LOCAL_IP" envsubst < "$DDS_TEMPLATE" > "$DDS_RESOLVED" \
    || MATLAB_HOST_IP="$MATLAB_HOST_IP" PI_LOCAL_IP="$PI_LOCAL_IP" envsubst < "$DDS_TEMPLATE" | sudo tee "$DDS_RESOLVED" > /dev/null
log "DDS=${DDS}  RMW=${RMW_IMPLEMENTATION}  profile=${DDS_RESOLVED}"
log "MATLAB_HOST_IP=${MATLAB_HOST_IP}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# ── clean stale state ─────────────────────────────────────────────────────────
log "Cleaning up any stale processes / shm..."
sudo docker kill ros2_perception_stack 2>/dev/null || true
sudo docker rm  ros2_perception_stack 2>/dev/null || true
sudo pkill -f yolo_producer 2>/dev/null || true
sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm \
           /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready 2>/dev/null || true
sleep 0.5

# ── 1. Docker container (detached) ────────────────────────────────────────────
sep
log "1/4  Starting Docker container (detached)..."
sudo docker run -d \
    --entrypoint "" \
    --name ros2_perception_stack \
    --net=host --ipc=host --privileged \
    --device=/dev/hailo0:/dev/hailo0 \
    -v "${WS}:/workspace" \
    -v /tmp:/tmp \
    -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
    ros2_perception_stack sleep infinity
sleep 1

# helper: launch the ROS2 node group inside the container
LAUNCH_ARGS=""
if [[ "$OPT" == "--no-slam" ]]; then
    log "OV2SLAM will be skipped (--no-slam)"
    LAUNCH_ARGS="slam:=false"
fi

# ── 2. ROS2 launch (sim_camera_bridge + bridges + slam + visp) ────────────────
log "2/4  ros2 launch hil_simulation (cores 0,2,3)"
sudo docker exec -d ros2_perception_stack bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    export ${DDS_ENV_VAR}
    export WORKSPACE_DIR=/workspace
    export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\${LD_LIBRARY_PATH:-}
    ros2 launch sim_camera_bridge hil_simulation.launch.py ${LAUNCH_ARGS} \
        > /tmp/hil_launch.log 2>&1
"

# Wait for sim_camera_bridge to create the SHM (it does this on construction)
log "Waiting for /dev/shm/ovcam_frames to appear..."
for i in $(seq 1 20); do
    [[ -e /dev/shm/ovcam_frames ]] && break
    sleep 0.5
done
[[ -e /dev/shm/ovcam_frames ]] \
    || die "sim_camera_bridge did not create /dev/shm/ovcam_frames — check /tmp/hil_launch.log"
log "SHM created."

# Fix permissions: sim_camera_bridge runs as root inside Docker and creates the
# SHM + semaphore as root with umask-derived 0644. yolo_producer runs on the
# host as a normal user and needs write access to sem_timedwait on /ovcam_ready.
sudo chmod 666 /dev/shm/ovcam_frames /dev/shm/sem.ovcam_ready 2>/dev/null || true
log "SHM permissions fixed (0666)."

# ── 3. yolo_producer — host, Core 1 ───────────────────────────────────────────
log "3/4  yolo_producer  →  core 1 (host, native)"
# Ensure log file is writable (may have been left root-owned by a prior sudo run)
sudo chmod 666 /tmp/yolo_producer.log 2>/dev/null || true
taskset -c 1 python3 "${WS}/src/yolo_producer/yolo_producer.py" \
    --hef "${WS}/models/yolo26n_10h.hef" --no-image \
    > /tmp/yolo_producer.log 2>&1 &
YOLO_PID=$!
for i in $(seq 1 20); do
    [[ -e /dev/shm/yolo_shm ]] && break
    sleep 0.5
done
[[ -e /dev/shm/yolo_shm ]] \
    || die "yolo_producer failed — check /tmp/yolo_producer.log"

# ── 4. summary ────────────────────────────────────────────────────────────────
sep
echo ""
echo "  HIL stack is running."
echo ""
echo "  MATLAB side must publish:"
echo "    /sim/camera/image_raw  sensor_msgs/Image  rgb8  640x480  ~20 Hz"
echo "    DDS=${DDS}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo ""
echo "  Logs:"
echo "    /tmp/hil_launch.log     (ros2 launch — sim_camera_bridge, bridges, slam, visp)"
echo "    /tmp/yolo_producer.log  (host, PID $YOLO_PID)"
echo ""
echo "  Stop:  ./run_stack_hil.sh stop"
echo "  Rate:  ./run_stack_hil.sh hz [topic]    (defaults to /sim/camera/image_raw)"
echo ""
sep
