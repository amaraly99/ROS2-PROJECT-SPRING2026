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
#   ./run_stack_hil.sh [--no-slam] [--debug-image] [--model <id>] [--backend npu|cpu]
#                      [--detector yolo|opencv|vlm] [--conf <f>] [--stamp-log]
#   ./run_stack_hil.sh stop
#   ./run_stack_hil.sh hz [topic]      # rate check with HIL DDS profile loaded
#
#   --no-slam       skip OV2SLAM (cores 2-3 free, ovcam_bridge also skipped)
#   --debug-image   publish /visp/debug_image back to MATLAB (921 KB/frame extra)
#   --model <id>    detector model from models/model_registry.json
#                   (yolo26n|yolo26s|yolo26m|yolov8n|...|yolov11m; default yolo26n)
#   --backend <b>   npu (Hailo HEF) | cpu (onnxruntime)        default: npu
#   --detector <d>  yolo | opencv (MOG2 bg-subtraction) | vlm (OWL-ViT)
#                   default: yolo. opencv/vlm ignore --model/--backend.
#   --conf <f>      detection confidence threshold              default: 0.20
#   --stamp-log     sim_camera_bridge logs per-frame stamps to
#                   /tmp/sim_cam_stamps.csv (latency stages S1/S2)
#
# Env vars (override before invoking):
#   MATLAB_HOST_IP   — Windows host IP for unicast DDS discovery (REQUIRED)
#   DDS              — cyclonedds (default) | fastrtps
#   ROS_DOMAIN_ID    — defaults to 0
#   TELEMETRY_CSV    — producer 7-stamp telemetry path (default /tmp/yolo_telemetry.csv)
#   TARGET_CLASS     — override visp target class (default derived from --detector)

cd "$(dirname "$0")"
WS="$(pwd)"
OPT="${1:-}"

# Parse flags (scan all args so order doesn't matter; value flags consume the next arg)
SLAM_ON=true
DEBUG_IMAGE_ON=false
MODEL_ID="yolo26n"
BACKEND="npu"
DETECTOR_TYPE="yolo"
CONF_THRESH="0.20"
STAMP_LOG_ON=false
ARGS=("$@")
for i in "${!ARGS[@]}"; do
    arg="${ARGS[$i]}"
    next="${ARGS[$((i+1))]:-}"
    case "$arg" in
        --no-slam)     SLAM_ON=false ;;
        --debug-image) DEBUG_IMAGE_ON=true ;;
        --stamp-log)   STAMP_LOG_ON=true ;;
        --model)       MODEL_ID="$next" ;;
        --backend)     BACKEND="$next" ;;
        --detector)    DETECTOR_TYPE="$next" ;;
        --conf)        CONF_THRESH="$next" ;;
    esac
done
case "$BACKEND" in npu|cpu) ;; *) echo "[hil] ERROR: --backend must be npu|cpu (got '$BACKEND')" >&2; exit 1 ;; esac
case "$DETECTOR_TYPE" in yolo|opencv|vlm) ;; *) echo "[hil] ERROR: --detector must be yolo|opencv|vlm (got '$DETECTOR_TYPE')" >&2; exit 1 ;; esac
TELEMETRY_CSV="${TELEMETRY_CSV:-/tmp/yolo_telemetry.csv}"

# Detector type decides the class-id→name map in yolo_bridge and the class
# visp_servo locks onto. Without a matching target_class the control loop is
# silently open (visp filters detections by class_name — visp_servo_node.cpp).
case "$DETECTOR_TYPE" in
    yolo)   DEFAULT_TARGET="stop sign";     CLASS_NAME_MAP="" ;;
    opencv) DEFAULT_TARGET="object";        CLASS_NAME_MAP="0:object" ;;
    vlm)    DEFAULT_TARGET="vlm_detection"; CLASS_NAME_MAP="100:vlm_detection" ;;
esac
TARGET_CLASS="${TARGET_CLASS:-$DEFAULT_TARGET}"

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
    sudo pkill -f yolo_shm_producer 2>/dev/null || true
    sudo pkill -f detector_producer 2>/dev/null || true
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
    # --window 30: report a ROLLING rate over the last ~30 messages (~2s at 15 Hz)
    # instead of ros2 topic hz's default cumulative average over ALL messages since
    # the subscriber started (default window 10000). A late-joining subscriber waits
    # one DDS discovery/announcement cycle (~20s on this profile) before the first
    # frame arrives; that single ~20s inter-arrival gap stays in the default window
    # forever and drags the printed "average rate" far below the true steady-state
    # rate (e.g. shows ~6.8 Hz while frames actually arrive at ~15 Hz). A short
    # rolling window flushes that startup gap after a couple seconds so the number
    # reflects what is really flowing right now.
    echo "[hil] ros2 topic hz --window 30 (rolling — ignore the first few lines while it warms up)"
    sudo docker exec $DOCKER_TTY_FLAGS ros2_perception_stack bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export FASTRTPS_DEFAULT_PROFILES_FILE=${RESOLVED}
        export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
        ros2 topic hz --window 30 ${TOPIC}
    "
    exit 0
fi

# ── pre-flight ────────────────────────────────────────────────────────────────
command -v taskset >/dev/null    || die "'taskset' not found — install util-linux"
command -v envsubst >/dev/null   || die "'envsubst' not found — install gettext-base"
if [[ "$DETECTOR_TYPE" == "yolo" && "$BACKEND" == "npu" ]]; then
    [[ -e /dev/hailo0 ]]         || die "/dev/hailo0 not found — Hailo HAT must be powered (NPU backend needs it)"
fi
if [[ "$DETECTOR_TYPE" == "yolo" ]]; then
    python3 - "$WS" "$MODEL_ID" <<'PYEOF' || die "model '$MODEL_ID' not in models/model_registry.json"
import json, sys
reg = json.load(open(f"{sys.argv[1]}/models/model_registry.json"))
sys.exit(0 if sys.argv[2] in reg["models"] else 1)
PYEOF
fi
MATLAB_HOST_IP="${MATLAB_HOST_IP:-192.168.56.1}"
if [[ "${MATLAB_HOST_IP}" == "192.168.56.1" ]]; then
    log "WARNING: MATLAB_HOST_IP is the default (192.168.56.1) — set MATLAB_HOST_IP env var if your MATLAB machine uses a different address"
fi
# Detect Pi's local IP and outbound interface toward MATLAB (needed for DDS interface whitelist)
PI_LOCAL_IP=$(ip route get "${MATLAB_HOST_IP}" | grep -oP 'src \K[0-9.]+' | head -1)
[[ -z "$PI_LOCAL_IP" ]] && die "Cannot determine local IP toward ${MATLAB_HOST_IP} — is eth0 up?"
PI_INTERFACE=$(ip route get "${MATLAB_HOST_IP}" | grep -oP 'dev \K\S+' | head -1)
[[ -z "$PI_INTERFACE" ]] && die "Cannot determine network interface toward ${MATLAB_HOST_IP}"

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

# Substitute env vars into the DDS profile (avoids hardcoded IPs/interfaces in the repo).
# PI_ADDR aliases PI_LOCAL_IP so fastrtps_matlab.xml template works if run manually.
# Use sudo tee as fallback in case the file was previously created by root.
MATLAB_HOST_IP="$MATLAB_HOST_IP" PI_LOCAL_IP="$PI_LOCAL_IP" PI_ADDR="$PI_LOCAL_IP" PI_INTERFACE="$PI_INTERFACE" \
    envsubst < "$DDS_TEMPLATE" > "$DDS_RESOLVED" \
    || MATLAB_HOST_IP="$MATLAB_HOST_IP" PI_LOCAL_IP="$PI_LOCAL_IP" PI_ADDR="$PI_LOCAL_IP" PI_INTERFACE="$PI_INTERFACE" \
       envsubst < "$DDS_TEMPLATE" | sudo tee "$DDS_RESOLVED" > /dev/null
log "DDS=${DDS}  RMW=${RMW_IMPLEMENTATION}  profile=${DDS_RESOLVED}"
log "MATLAB_HOST_IP=${MATLAB_HOST_IP}  PI_LOCAL_IP=${PI_LOCAL_IP}  PI_INTERFACE=${PI_INTERFACE}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# ── clean stale state ─────────────────────────────────────────────────────────
log "Cleaning up any stale processes / shm..."
sudo docker kill ros2_perception_stack 2>/dev/null || true
sudo docker rm  ros2_perception_stack 2>/dev/null || true
sudo pkill -f yolo_producer 2>/dev/null || true
sudo pkill -f yolo_shm_producer 2>/dev/null || true
sudo pkill -f detector_producer 2>/dev/null || true
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

# helper: build launch args from parsed flags.
# OV2SLAM is always started separately (below) so we always pass slam:=false to the
# launch file. This lets the drone begin its search pattern before SLAM initializes,
# which prevents the premature auto-exit that fires when parallax is near-zero at
# startup (buffered frames make cam_delay tiny → 100×cam_delay threshold fires fast).
LAUNCH_ARGS=" slam:=false"
LAUNCH_ARGS+=" target_class:='${TARGET_CLASS}'"
[[ -n "$CLASS_NAME_MAP" ]] && LAUNCH_ARGS+=" class_name_map:='${CLASS_NAME_MAP}'"
[[ "$STAMP_LOG_ON" == "true" ]] && LAUNCH_ARGS+=" cam_stamp_log:=/tmp/sim_cam_stamps.csv" \
    && log "Camera stamp log enabled → /tmp/sim_cam_stamps.csv"
[[ "$SLAM_ON"        == "false" ]] && log "OV2SLAM will be skipped (--no-slam)"
[[ "$DEBUG_IMAGE_ON" == "true"  ]] && LAUNCH_ARGS+=" debug_image:=true" && log "Debug image enabled (--debug-image)"
# ovcam_bridge is needed when SLAM will be started later, or debug is on
if [[ "$SLAM_ON" == "false" && "$DEBUG_IMAGE_ON" == "false" ]]; then
    LAUNCH_ARGS+=" ovcam:=false"
    log "ovcam_bridge will be skipped (slam off + debug off)"
fi

# ── 2. ROS2 launch (sim_camera_bridge + bridges + visp — NO slam yet) ─────────
log "2/4  ros2 launch hil_simulation (cores 0,2,3 — slam deferred)"
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

# ── 3. detector producer — host, Core 1 ──────────────────────────────────────
log "3/4  detector=${DETECTOR_TYPE} model=${MODEL_ID} backend=${BACKEND} conf=${CONF_THRESH}  →  core 1 (host, native)"
# Ensure log file is writable (may have been left root-owned by a prior sudo run)
sudo chmod 666 /tmp/yolo_producer.log 2>/dev/null || true
# conf default 0.20: publish detections down to 0.20 so visp (min_confidence=0.20)
#   can act on low-confidence stop signs (0.25 silently clipped the 0.20-0.25 band).
# python3 -u: unbuffered stdout so the per-stage timing/fps log is readable live
#   (block-buffered output otherwise leaves /tmp/yolo_producer.log empty for minutes).
case "$DETECTOR_TYPE" in
    yolo)
        PRODUCER_CMD=(python3 -u "${WS}/src/yolo_producer/yolo_shm_producer.py"
                      --model_id "$MODEL_ID" --backend "$BACKEND"
                      --conf "$CONF_THRESH" --no-image
                      --telemetry_csv "$TELEMETRY_CSV")
        ;;
    opencv)
        PRODUCER_CMD=(python3 -u "${WS}/src/yolo_producer/opencv_detector_producer.py"
                      --conf "$CONF_THRESH" --telemetry_csv "$TELEMETRY_CSV")
        ;;
    vlm)
        PRODUCER_CMD=(python3 -u "${WS}/src/yolo_producer/vlm_detector_producer.py"
                      --prompt "stop sign" --box-threshold "$CONF_THRESH"
                      --telemetry_csv "$TELEMETRY_CSV")
        ;;
esac
taskset -c 1 "${PRODUCER_CMD[@]}" > /tmp/yolo_producer.log 2>&1 &
YOLO_PID=$!
for i in $(seq 1 20); do
    [[ -e /dev/shm/yolo_shm ]] && break
    sleep 0.5
done
[[ -e /dev/shm/yolo_shm ]] \
    || die "yolo_producer failed — check /tmp/yolo_producer.log"

# ── 4. OV2SLAM — deferred start + watchdog, cores 2,3 ────────────────────────
if [[ "$SLAM_ON" == "true" ]]; then
    log "4/4  OV2SLAM deferred: waiting 15 s for drone to begin motion before init..."
    sleep 15
    log "Starting OV2SLAM watchdog  →  cores 2,3 (Docker)"
    # Watchdog loop: restarts ov2slam_node whenever it exits (crash or auto-exit).
    # Runs in the background so run_stack_hil.sh returns immediately.
    # Stops when the Docker container is gone (stack was stopped).
    # Pre-create log file as root (via docker) so the host ahmed user can append.
    sudo docker exec ros2_perception_stack bash -c "
        touch /tmp/ov2slam_hil.log && chmod 666 /tmp/ov2slam_hil.log
    " 2>/dev/null || true
    (
        RMW="${RMW_IMPLEMENTATION}"
        DDS_VAR="${DDS_ENV_VAR}"
        ROS_ID="${ROS_DOMAIN_ID}"
        while sudo docker inspect ros2_perception_stack --format '{{.State.Running}}' 2>/dev/null | grep -q true; do
            # Redirect inside the container (root) to avoid host permission issues.
            sudo docker exec ros2_perception_stack bash -lc "
                source /opt/ros/jazzy/setup.bash
                source /workspace/install/setup.bash
                export RMW_IMPLEMENTATION=${RMW}
                export ROS_DOMAIN_ID=${ROS_ID}
                export ${DDS_VAR}
                export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\${LD_LIBRARY_PATH:-}
                taskset -c 2,3 ros2 run ov2slam ov2slam_node \
                    /workspace/camera_calib/hil_sim_ov2slam.yaml \
                    >> /tmp/ov2slam_hil.log 2>&1
            "
            sudo docker inspect ros2_perception_stack --format '{{.State.Running}}' 2>/dev/null | grep -q true \
                && echo "[hil-watchdog] ov2slam exited — restarting in 3 s..." >> /tmp/ov2slam_hil.log \
                && sleep 3
        done
        echo "[hil-watchdog] container gone — watchdog exiting" >> /tmp/ov2slam_hil.log
    ) &
    WATCHDOG_PID=$!
    log "OV2SLAM watchdog PID $WATCHDOG_PID — log: /tmp/ov2slam_hil.log"
else
    log "4/4  OV2SLAM skipped (--no-slam)"
    WATCHDOG_PID=""
fi

# ── 5. summary ────────────────────────────────────────────────────────────────
sep
echo ""
echo "  HIL stack is running."
echo ""
echo "  MATLAB side must publish:"
echo "    /sim/camera/image_raw  sensor_msgs/Image  rgb8  640x480  ~20 Hz"
echo "    DDS=${DDS}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo ""
echo "  Logs:"
echo "    /tmp/hil_launch.log     (ros2 launch — sim_camera_bridge, bridges, visp)"
echo "    /tmp/yolo_producer.log  (host, PID $YOLO_PID)"
if [[ "$SLAM_ON" == "true" ]]; then
echo "    /tmp/ov2slam_hil.log    (OV2SLAM watchdog PID $WATCHDOG_PID — auto-restarts on exit)"
fi
echo ""
echo "  Stop:  ./run_stack_hil.sh stop"
echo "  Rate:  ./run_stack_hil.sh hz [topic]    (defaults to /sim/camera/image_raw)"
echo ""
sep
