#!/bin/bash
# run_stack.sh — Start (or stop) the full perception + visual servoing stack
#
# Core layout (Pi 5, 4 cores):
#   Core 0     ovcam_producer   (libcamera capture)
#   Core 1     yolo_producer    (Hailo NPU inference — DEDICATED)
#   Core 0     ovcam_bridge     yolo_bridge     visp_servo  (lightweight)
#   Core 2,3   OV2SLAM          (visual SLAM — 2 dedicated cores)
#
# Usage:
#   ./run_stack.sh            # start everything (SLAM included)
#   ./run_stack.sh --no-slam  # skip OV2SLAM (visp_servo uses bbox depth fallback)
#   ./run_stack.sh stop       # graceful shutdown

cd "$(dirname "$0")"
WS="$(pwd)"
OPT="${1:-}"

log()  { echo "[stack] $*"; }
die()  { echo "[stack] ERROR: $*" >&2; exit 1; }
sep()  { echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; }

# ── stop ──────────────────────────────────────────────────────────────────────
if [[ "$OPT" == "stop" ]]; then
    log "Stopping all stack processes..."
    sudo docker kill ros2_perception_stack 2>/dev/null || true
    sudo docker rm  ros2_perception_stack 2>/dev/null || true
    sudo pkill -f ovcam_producer  2>/dev/null || true
    sudo pkill -f yolo_producer   2>/dev/null || true
    sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm \
               /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready 2>/dev/null || true
    log "All stopped."
    exit 0
fi

# ── pre-flight ────────────────────────────────────────────────────────────────
command -v taskset >/dev/null     || die "'taskset' not found — install util-linux"
[[ -e /dev/hailo0 ]]              || die "/dev/hailo0 not found — is the Hailo HAT powered?"
[[ -f "${WS}/src/ovcam_producer/build/ovcam_producer" ]] \
                                  || die "ovcam_producer not built — run: cd src/ovcam_producer && mkdir build && cd build && cmake .. && make -j4"

# ── clean up any stale state ──────────────────────────────────────────────────
log "Cleaning up any stale processes / shm..."
sudo docker kill ros2_perception_stack 2>/dev/null || true
sudo docker rm  ros2_perception_stack 2>/dev/null || true
sudo pkill -f ovcam_producer 2>/dev/null || true
sudo pkill -f yolo_producer  2>/dev/null || true
sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm \
           /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready 2>/dev/null || true
sleep 0.5

# ── 1. Docker container (detached — no -it avoids the tty spin-loop CPU hog) ─
sep
log "1/6  Starting Docker container (detached)..."
sudo docker run -d \
    --entrypoint "" \
    --name ros2_perception_stack \
    --net=host --ipc=host --privileged \
    --device=/dev/hailo0:/dev/hailo0 \
    -v "${WS}:/workspace" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    -e DISPLAY=:0 -e QT_X11_NO_MITSHM=1 \
    ros2_perception_stack sleep infinity
sleep 1

# ── 2. Camera producer — Core 0 ───────────────────────────────────────────────
log "2/6  ovcam_producer  →  core 0"
taskset -c 0 "${WS}/src/ovcam_producer/build/ovcam_producer" \
    --width 640 --height 480 --fps 60 \
    > /tmp/ovcam_producer.log 2>&1 &
OVCAM_PID=$!
# Wait up to 5s for shm to appear
for i in $(seq 1 10); do
    [[ -e /dev/shm/ovcam_frames ]] && break
    sleep 0.5
done
[[ -e /dev/shm/ovcam_frames ]] \
    || die "ovcam_producer failed — check /tmp/ovcam_producer.log"

# ── 3. YOLO producer — Core 1 ─────────────────────────────────────────────────
log "3/6  yolo_producer   →  core 1"
taskset -c 1 python3 "${WS}/src/yolo_producer/yolo_producer.py" \
    --hef "${WS}/models/yolo26n_10h.hef" --no-image \
    > /tmp/yolo_producer.log 2>&1 &
YOLO_PID=$!
# Wait up to 10s for yolo shm (Hailo model load takes a few seconds)
for i in $(seq 1 20); do
    [[ -e /dev/shm/yolo_shm ]] && break
    sleep 0.5
done
[[ -e /dev/shm/yolo_shm ]] \
    || die "yolo_producer failed — check /tmp/yolo_producer.log"

# helper: run a node inside Docker pinned to given cores
docker_node() {
    local cores="$1"; local pkg="$2"; local node="$3"; local logfile="$4"
    sudo docker exec -d ros2_perception_stack bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        taskset -c ${cores} ros2 run ${pkg} ${node} > ${logfile} 2>&1
    "
}

# ── 4. ROS bridges — Core 0 ───────────────────────────────────────────────────
log "4/6  ovcam_bridge + yolo_bridge  →  core 0"
docker_node "0" ovcam_bridge ovcam_bridge_node /tmp/ovcam_bridge.log
docker_node "0" yolo_bridge  yolo_bridge_node  /tmp/yolo_bridge.log
sleep 2

# ── 5. OV2SLAM — Cores 2,3 ────────────────────────────────────────────────────
if [[ "$OPT" != "--no-slam" ]]; then
    log "5/6  OV2SLAM  →  cores 2,3"
    sudo docker exec -d ros2_perception_stack bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\${LD_LIBRARY_PATH:-}
        taskset -c 2,3 ros2 run ov2slam ov2slam_node \
            /workspace/camera_calib/ov5647_ov2slam.yaml \
            > /tmp/ov2slam.log 2>&1
    "
else
    log "5/6  OV2SLAM  →  skipped (--no-slam)"
fi

# ── 6. visp_servo — Core 0 ────────────────────────────────────────────────────
log "6/6  visp_servo  →  core 0"
docker_node "0" visp_servo visp_servo_node /tmp/visp_servo.log
sleep 1

# ── summary ───────────────────────────────────────────────────────────────────
sep
echo ""
echo "  Stack is running."
echo ""
echo "  Core layout:"
printf "    Core 0     ovcam_producer   PID %-6s  log: /tmp/ovcam_producer.log\n" "$OVCAM_PID"
printf "    Core 1     yolo_producer    PID %-6s  log: /tmp/yolo_producer.log\n"  "$YOLO_PID"
echo   "    Core 0     ovcam_bridge              log (container): /tmp/ovcam_bridge.log"
echo   "    Core 0     yolo_bridge               log (container): /tmp/yolo_bridge.log"
echo   "    Core 0     visp_servo                log (container): /tmp/visp_servo.log"
if [[ "$OPT" != "--no-slam" ]]; then
echo   "    Core 2,3   OV2SLAM                   log (container): /tmp/ov2slam.log"
fi
echo ""
echo "  Monitor per-core util:"
echo "    watch -n1 'ps -eo pid,psr,pcpu,comm --sort=-pcpu | head -18'"
echo ""
echo "  Shell into container:"
echo "    ./enter_container.sh"
echo ""
echo "  Stop everything:"
echo "    ./run_stack.sh stop"
echo ""
sep
