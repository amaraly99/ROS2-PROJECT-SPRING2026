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
#   ./run_stack_hil.sh --config <name>        # load config/hil/stack/<name>.yaml (NESTED)
#   ./run_stack_hil.sh --config <name> --mode benchmark    # override the config's mode
#   ./run_stack_hil.sh [--no-slam] [--debug-image]    # raw flags (no config)
#   ./run_stack_hil.sh build [pkg]            # colcon build inside Docker, then exit
#   ./run_stack_hil.sh stop
#   ./run_stack_hil.sh hz [topic]             # rate check with HIL DDS profile loaded
#
#   --config <name>   read config/hil/stack/<name>.yaml via scripts/parse_stack.py
#   --mode <m>        override config mode: benchmark | scout
#   --build           colcon build all HIL stack packages first, then launch
#   --no-slam         override: skip the SLAM sidecar even if config enables it
#   --debug-image     override: publish /visp/debug_image (921 KB/frame extra)
#
# Config is the NESTED schema (config/hil/stack/full_ov2slam.yaml is the reference):
#   mode (benchmark|scout), network{matlab_host_ip,interface,dds,ros_domain_id},
#   detector{type,cpu,host_cpu,topics,yolo}, controller{type,cpu,topics},
#   slam{enabled,type,image,container_name,cpu,startup_delay_sec,restart,command,
#        topics,remap}.  parse_stack.py flattens it to the env vars consumed below.
#
#   scout      engage on boot (live flight / demo). No recording.
#   benchmark  FSM waits for a clean sim reset (t=0); records bags/run_<cfg>_<stamp>/
#              for offline RMSE (controller approach and/or SLAM pose vs GT).
#
# Available configs:
#   default        YOLO + proportional, no SLAM   (safe general-purpose default)
#   ibvs           YOLO + IBVS,         no SLAM   (fastest settle ~20 s)
#   h_vs           YOLO + h_vs,         no SLAM   (smoothest commands)
#   oracle         oracle detector + IBVS, no SLAM (no Hailo HAT needed)
#   full_ov2slam   YOLO + proportional + OV2SLAM   (closest to real flight)
#   ov2slam_ibvs   YOLO + ViSP IBVS + OV2SLAM
#   ov2slam_oracle oracle + proportional + OV2SLAM, benchmark mode (SLAM RMSE capture)
#
# Env vars (used as fallback when no --config; can still override after config):
#   MATLAB_HOST_IP   — Windows host IP for unicast DDS discovery
#   DDS              — cyclonedds | fastrtps  (default: fastrtps)
#   ROS_DOMAIN_ID    — defaults to 0
#   CONTROLLER       — ibvs | proportional | h_vs  (default: proportional)
#   DETECTOR         — yolo | oracle               (default: yolo)
#
# Examples:
#   ./run_stack_hil.sh --config default
#   ./run_stack_hil.sh --config oracle
#   ./run_stack_hil.sh --config full --no-slam      # full config but skip SLAM today
#   MATLAB_HOST_IP=192.168.1.50 ./run_stack_hil.sh --config ibvs   # override one field

cd "$(dirname "$0")"
WS="$(pwd)"
OPT="${1:-}"

log()  { echo "[hil] $*"; }
die()  { echo "[hil] ERROR: $*" >&2; exit 1; }
sep()  { echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; }

# Leaf packages of the HIL stack. --packages-up-to walks these and pulls their
# in-workspace deps automatically (yolo_msgs, servo_core, ceres-solver, Sophus).
# NOTE: never bare `colcon build` here — the workspace also contains the vendored
# OpenCV source tree (/workspace/opencv) which is NOT a colcon package; building
# it standalone dies with "Unknown CMake command ocv_define_module". We restrict
# discovery to src/ (--base-paths src) and select only these leaves.
STACK_PKGS="sim_camera_bridge ovcam_bridge yolo_bridge oracle_detector hil_servo visp_servo h_vs_servo ov2slam visp_pbvs_servo"

# ── load_config: parse NESTED config/hil/stack/<name>.yaml via parse_stack.py ─
# The Python parser flattens the nested schema to shell-safe KEY=value lines
# (MODE, MATLAB_HOST_IP, DDS, CONTROLLER[_CPU], DETECTOR[_CPU/_HOST_CPU],
#  SLAM_ENABLED/_TYPE/_IMAGE/_CONTAINER/_CPU/_DELAY/_RESTART/_COMMAND/_REMAPS, …).
# Every value is shlex-quoted so spaces in SLAM_COMMAND/SLAM_REMAPS survive eval.
load_config() {
    local name="$1"
    local cfg="${WS}/config/hil/stack/${name}.yaml"
    if [[ ! -f "$cfg" ]]; then
        local available
        available=$(ls "${WS}/config/hil/stack/"*.yaml 2>/dev/null \
                    | xargs -n1 basename | sed 's/\.yaml$//' | tr '\n' ' ')
        die "Config '${name}' not found at ${cfg}\n  Available: ${available}"
    fi
    log "Config: ${cfg}"
    eval "$(python3 "${WS}/scripts/parse_stack.py" "$cfg" --emit-env)" \
        || die "parse_stack.py failed on ${cfg} (is pyyaml installed? is the YAML valid?)"
}

# ── Parse flags (all args; order doesn't matter) ──────────────────────────────
SLAM_ENABLED=true
DEBUG_IMAGE_ON=false
CONFIG_NAME=""
_NO_SLAM_FLAG=false
_DEBUG_FLAG=false
_MODE_OVERRIDE=""

_BUILD_FIRST=false

_i=1
while [[ $_i -le $# ]]; do
    case "${!_i}" in
        --no-slam)     _NO_SLAM_FLAG=true ;;
        --debug-image) _DEBUG_FLAG=true ;;
        --build)       _BUILD_FIRST=true ;;
        --mode)
            _i=$((_i + 1))
            [[ $_i -le $# ]] || die "--mode requires an argument (benchmark|scout)"
            _MODE_OVERRIDE="${!_i}" ;;
        --config)
            _i=$((_i + 1))
            [[ $_i -le $# ]] || die "--config requires a name argument"
            CONFIG_NAME="${!_i}" ;;
    esac
    _i=$((_i + 1))
done

# Load config file (sets MODE, MATLAB_HOST_IP, DDS, CONTROLLER, DETECTOR,
# SLAM_ENABLED, and all the *_CPU / SLAM_* vars consumed below).
[[ -n "$CONFIG_NAME" ]] && load_config "$CONFIG_NAME"

# CLI flags override config values
[[ "$_NO_SLAM_FLAG"    == "true" ]] && SLAM_ENABLED=false
[[ "$_DEBUG_FLAG"      == "true" ]] && DEBUG_IMAGE_ON=true
[[ -n "$_MODE_OVERRIDE"          ]] && MODE="$_MODE_OVERRIDE"

# Env-var defaults for anything not set by config or caller
MODE="${MODE:-scout}"                      # benchmark | scout
DDS="${DDS:-fastrtps}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
CONTROLLER="${CONTROLLER:-proportional}"   # ibvs | proportional | h_vs | pbvs
DETECTOR="${DETECTOR:-yolo}"               # yolo | oracle

case "$MODE" in
    benchmark|scout) ;;
    *) die "Unknown MODE='${MODE}'. Valid: benchmark | scout" ;;
esac

# ── build [pkg] — colcon build inside a throwaway container ──────────────────
# Artifacts land in /workspace/install/ (host filesystem via volume mount) and
# persist across container restarts — so you only need to do this after code changes.
#   ./run_stack_hil.sh build                      # build all HIL stack packages
#   ./run_stack_hil.sh build sim_camera_bridge    # build one package
if [[ "$OPT" == "build" ]]; then
    PKG="${2:-}"
    if [[ -n "$PKG" ]]; then
        BUILD_SEL="--packages-select ${PKG}"
        log "Building '${PKG}' in Docker..."
    else
        BUILD_SEL="--packages-up-to ${STACK_PKGS}"
        log "Building HIL stack packages in Docker (${STACK_PKGS})..."
    fi
    sudo docker run --rm \
        --entrypoint "" \
        -v "${WS}:/workspace" \
        ros2_perception_stack bash -lc "
            source /opt/ros/jazzy/setup.bash
            cd /workspace
            colcon build --symlink-install --base-paths src ${BUILD_SEL} 2>&1
        " || die "colcon build failed — fix errors above"
    log "Build done. install/ updated on host disk — run the stack normally now."
    exit 0
fi

# ── stop ──────────────────────────────────────────────────────────────────────
if [[ "$OPT" == "stop" ]]; then
    log "Stopping HIL stack..."
    # Send SIGINT to ros2 bag record FIRST so it can write the MCAP footer cleanly.
    # docker kill sends SIGKILL which truncates the file and leaves it unreadable.
    sudo docker exec ros2_perception_stack \
        bash -c "pkill -INT -f 'ros2 bag record' || true" 2>/dev/null || true
    sleep 2   # ros2 bag record typically flushes footer in <1 s
    sudo docker stop --time 5 ros2_perception_stack 2>/dev/null || true
    sudo docker rm  ros2_perception_stack 2>/dev/null || true
    # SLAM sidecars use --restart, so a plain kill is not enough: force-remove any
    # container whose name starts with 'slam_' (the schema mandates that prefix).
    _slam_cids=$(sudo docker ps -aq --filter 'name=^slam_' 2>/dev/null)
    [[ -n "$_slam_cids" ]] && sudo docker rm -f $_slam_cids 2>/dev/null || true
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

# ── validate env vars ─────────────────────────────────────────────────────────
case "$CONTROLLER" in
    ibvs|proportional|h_vs|pbvs) ;;
    *) die "Unknown CONTROLLER='${CONTROLLER}'. Valid: ibvs | proportional | h_vs | pbvs" ;;
esac
case "$DETECTOR" in
    yolo|oracle) ;;
    *) die "Unknown DETECTOR='${DETECTOR}'. Valid: yolo | oracle" ;;
esac

# ── pre-flight ────────────────────────────────────────────────────────────────
command -v taskset >/dev/null    || die "'taskset' not found — install util-linux"
command -v envsubst >/dev/null   || die "'envsubst' not found — install gettext-base"
[[ "$DETECTOR" == "yolo" ]] && { [[ -e /dev/hailo0 ]] || die "/dev/hailo0 not found — Hailo HAT must be powered (yolo detector needs it)"; }
export MATLAB_HOST_IP="${MATLAB_HOST_IP:-192.168.56.1}"
if [[ "${MATLAB_HOST_IP}" == "192.168.56.1" ]]; then
    log "WARNING: MATLAB_HOST_IP is the default (192.168.56.1) — set it in your config yaml or via env var"
fi
# Detect Pi's local IP + outbound interface (needed for DDS interface whitelist).
# If 'interface' was set in the config, use that directly and read the IP from it.
# Otherwise fall back to auto-detection via routing table.
if [[ -n "${PI_INTERFACE:-}" ]]; then
    log "Interface pinned by config: ${PI_INTERFACE}"
    PI_LOCAL_IP=$(ip -4 addr show "$PI_INTERFACE" 2>/dev/null | grep -oP 'inet \K[0-9.]+' | head -1)
    [[ -z "$PI_LOCAL_IP" ]] && die "Interface '${PI_INTERFACE}' has no IPv4 address — check 'ip addr show ${PI_INTERFACE}'"
else
    PI_LOCAL_IP=$(ip route get "${MATLAB_HOST_IP}" | grep -oP 'src \K[0-9.]+' | head -1)
    [[ -z "$PI_LOCAL_IP" ]] && die "Cannot determine local IP toward ${MATLAB_HOST_IP} — is the network up?"
    PI_INTERFACE=$(ip route get "${MATLAB_HOST_IP}" | grep -oP 'dev \K\S+' | head -1)
    [[ -z "$PI_INTERFACE" ]] && die "Cannot determine network interface toward ${MATLAB_HOST_IP}"
fi

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
log "MODE=${MODE}  CONTROLLER=${CONTROLLER}  DETECTOR=${DETECTOR}  SLAM=${SLAM_ENABLED} (${SLAM_TYPE:-none})"

# ── optional build step ───────────────────────────────────────────────────────
if [[ "$_BUILD_FIRST" == "true" ]]; then
    sep
    log "Building HIL stack packages in Docker before launch..."
    log "  packages: ${STACK_PKGS}"
    sudo docker run --rm \
        --entrypoint "" \
        -v "${WS}:/workspace" \
        ros2_perception_stack bash -lc "
            source /opt/ros/jazzy/setup.bash
            cd /workspace
            colcon build --symlink-install --base-paths src --packages-up-to ${STACK_PKGS} 2>&1
        " || die "colcon build failed — fix errors above before launching"
    log "Build complete."
    sep
fi

# ── clean stale state ─────────────────────────────────────────────────────────
log "Cleaning up any stale processes / shm..."
# Graceful bag finalization before killing stale container.
sudo docker exec ros2_perception_stack \
    bash -c "pkill -INT -f 'ros2 bag record' || true" 2>/dev/null || true
sleep 1
sudo docker stop --time 3 ros2_perception_stack 2>/dev/null || true
sudo docker rm  ros2_perception_stack 2>/dev/null || true
# Force-remove stale SLAM sidecars (they use --restart; a kill alone respawns them).
_slam_cids=$(sudo docker ps -aq --filter 'name=^slam_' 2>/dev/null)
[[ -n "$_slam_cids" ]] && sudo docker rm -f $_slam_cids 2>/dev/null || true
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

# helper: build launch args from parsed flags.
# OV2SLAM is always started separately (below) so we always pass slam:=false to the
# launch file. This lets the drone begin its search pattern before SLAM initializes,
# which prevents the premature auto-exit that fires when parallax is near-zero at
# startup (buffered frames make cam_delay tiny → 100×cam_delay threshold fires fast).
LAUNCH_ARGS=" slam:=false"
[[ "$SLAM_ENABLED"   == "false" ]] && log "SLAM will be skipped (--no-slam / slam.enabled=false)"
[[ "$DEBUG_IMAGE_ON" == "true"  ]] && LAUNCH_ARGS+=" debug_image:=true" && log "Debug image enabled (--debug-image)"
# ovcam_bridge is needed when SLAM will be started later, or debug is on
if [[ "$SLAM_ENABLED" == "false" && "$DEBUG_IMAGE_ON" == "false" ]]; then
    LAUNCH_ARGS+=" ovcam:=false"
    log "ovcam_bridge will be skipped (slam off + debug off)"
fi
# benchmark mode → FSM waits for a clean sim reset (t=0); scout → engage on boot.
BENCH_FLAG=false
[[ "$MODE" == "benchmark" ]] && BENCH_FLAG=true
# Controller / detector selection + per-module CPU pinning (applied as Node taskset).
LAUNCH_ARGS+=" controller:=${CONTROLLER} detector:=${DETECTOR} benchmark_mode:=${BENCH_FLAG}"
LAUNCH_ARGS+=" controller_cpu:=${CONTROLLER_CPU:-} detector_cpu:=${DETECTOR_CPU:-}"
LAUNCH_ARGS+=" use_slam_depth:=${CONTROLLER_USE_SLAM_DEPTH:-false}"
LAUNCH_ARGS+=" use_slam_pose:=${CONTROLLER_USE_SLAM_POSE:-false}"
if [[ "$DETECTOR" == "oracle" ]]; then
    log "Detector: oracle (yolo_bridge skipped, oracle_detector_node starts in Docker)"
else
    log "Detector: yolo (Hailo NPU via yolo_producer + yolo_bridge)"
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

# ── 3. Detector — yolo_producer (host) OR oracle (Docker) ─────────────────────
YOLO_PID=""
if [[ "$DETECTOR" == "yolo" ]]; then
    DETECTOR_HOST_CPU="${DETECTOR_HOST_CPU:-1}"
    log "3/4  yolo_producer  →  core ${DETECTOR_HOST_CPU} (host, native)"
    # Ensure log file is writable (may have been left root-owned by a prior sudo run)
    sudo chmod 666 /tmp/yolo_producer.log 2>/dev/null || true
    # --conf 0.20: publish detections down to 0.20 so the controller (min_confidence=0.20)
    #   can act on low-confidence stop signs.
    # python3 -u: unbuffered stdout so per-stage timing/fps log is readable live.
    taskset -c "${DETECTOR_HOST_CPU}" /usr/bin/python3 -u "${WS}/src/yolo_producer/yolo_producer.py" \
        --hef "${WS}/models/yolo26n_10h.hef" --no-image --conf 0.20 \
        > /tmp/yolo_producer.log 2>&1 &
    YOLO_PID=$!
    for i in $(seq 1 20); do
        [[ -e /dev/shm/yolo_shm ]] && break
        sleep 0.5
    done
    [[ -e /dev/shm/yolo_shm ]] \
        || die "yolo_producer failed — check /tmp/yolo_producer.log"
else
    log "3/4  Detector=oracle — yolo_producer skipped (oracle_detector_node starts via launch)"
fi

# ── 4. SLAM sidecar — separate docker-run container, config-driven ───────────
# Replaces the old bash watchdog: `docker run -d --restart` gives a named,
# auto-restarting sidecar. The startup delay runs INSIDE the container so this
# script returns immediately (the drone can start moving for parallax). All SLAM
# wiring (image/cpu/command/remaps) comes from the config via parse_stack.py.
if [[ "$SLAM_ENABLED" == "true" ]]; then
    : "${SLAM_CONTAINER:?slam.container_name missing from config}"
    : "${SLAM_IMAGE:?slam.image missing from config}"
    : "${SLAM_COMMAND:?slam.command missing from config}"
    log "4/4  SLAM sidecar: ${SLAM_TYPE} → '${SLAM_CONTAINER}' on cores ${SLAM_CPU:-all} (delay ${SLAM_DELAY:-0}s)"
    # docker run -d prints the container ID (or an error) to the terminal; the SLAM
    # node's own stdout/stderr is reachable via `docker logs ${SLAM_CONTAINER}`.
    sudo docker run -d \
        --entrypoint "" \
        --name "$SLAM_CONTAINER" \
        --restart "${SLAM_RESTART:-no}" \
        --net=host --ipc=host --privileged \
        -v "${WS}:/workspace" -v /tmp:/tmp \
        "$SLAM_IMAGE" bash -lc "
            sleep ${SLAM_DELAY:-0}
            source /opt/ros/jazzy/setup.bash
            source /workspace/install/setup.bash
            [[ -n ${SLAM_SETUP_OVERLAY:-} ]] && source ${SLAM_SETUP_OVERLAY}
            export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
            export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
            export ${DDS_ENV_VAR}
            export LD_LIBRARY_PATH=${SLAM_LD_PREFIX:+${SLAM_LD_PREFIX}:}/workspace/opencv/build/lib:\${LD_LIBRARY_PATH:-}
            exec ${SLAM_CPU:+taskset -c ${SLAM_CPU}} ${SLAM_COMMAND} --ros-args ${SLAM_REMAPS}
        " || die "SLAM sidecar failed to start (is image '${SLAM_IMAGE}' present? name clash on '${SLAM_CONTAINER}'?)"
    log "SLAM sidecar started — log: docker logs ${SLAM_CONTAINER}"
else
    log "4/4  SLAM skipped (slam.enabled=false / --no-slam)"
fi

# ── 4b. Benchmark recording — only in benchmark mode ─────────────────────────
# Records the metric topics to a timestamped bag for offline RMSE analysis
# (controller approach and/or SLAM-pose-vs-GT). Runs inside the main container so
# the bag lands on the host via the /workspace volume mount.
BAG_HOST=""
if [[ "$MODE" == "benchmark" ]]; then
    STAMP=$(date +%Y%m%d_%H%M%S)
    RUNREL="bags/run_${CONFIG_NAME:-nocfg}_${STAMP}"
    BAG_HOST="${WS}/${RUNREL}"
    mkdir -p "$BAG_HOST"
    GIT_SHA=$(git -C "$WS" rev-parse --short HEAD 2>/dev/null || echo nogit)
    {
        echo "config=${CONFIG_NAME:-nocfg}"
        echo "mode=benchmark"
        echo "controller=${CONTROLLER}"
        echo "detector=${DETECTOR}"
        echo "slam_enabled=${SLAM_ENABLED}"
        echo "slam_type=${SLAM_TYPE:-none}"
        echo "git_sha=${GIT_SHA}"
        echo "stamp=${STAMP}"
        echo "matlab_host_ip=${MATLAB_HOST_IP}"
        echo "dds=${DDS} domain=${ROS_DOMAIN_ID}"
    } > "${BAG_HOST}/meta.txt"

    # Base metric topics (controller RMSE); add SLAM pose + TF when SLAM is on.
    BAG_TOPICS="/cmd_vel /sim/drone_pose /sim/target_pose /sim/heartbeat /bench/state /yolo/detections"
    [[ "$SLAM_ENABLED" == "true" ]] && BAG_TOPICS+=" ${SLAM_POSE_OUT:-/slam/pose} /tf /tf_static"

    log "Benchmark mode: recording → ${RUNREL}/bag"
    log "  topics: ${BAG_TOPICS}"
    sudo docker exec -d ros2_perception_stack bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/install/setup.bash
        export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
        export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
        export ${DDS_ENV_VAR}
        ros2 bag record -o /workspace/${RUNREL}/bag \
            --qos-profile-overrides-path /workspace/config/hil/bench_bag_qos.yaml \
            ${BAG_TOPICS} > /workspace/${RUNREL}/bag_record.log 2>&1
    "
fi

# ── 5. summary ────────────────────────────────────────────────────────────────
sep
echo ""
echo "  HIL stack is running."
echo ""
echo "  Mode : ${MODE}   Controller : ${CONTROLLER}   Detector : ${DETECTOR}   SLAM : ${SLAM_ENABLED} (${SLAM_TYPE:-none})"
echo ""
echo "  MATLAB side must publish:"
echo "    /sim/camera/image_raw       sensor_msgs/Image  rgb8  640x480  ~20 Hz"
echo "    /sim/drone_pose             std_msgs/Float64MultiArray  [x,y,z,pitch,yaw]"
echo "    /sim/target_pose            std_msgs/Float64MultiArray  [x,y,z,yaw]"
echo "    DDS=${DDS}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo ""
echo "  Logs:"
echo "    /tmp/hil_launch.log     (ros2 launch — camera bridge, detector, controller)"
if [[ "$DETECTOR" == "yolo" ]]; then
echo "    /tmp/yolo_producer.log  (host Hailo NPU, PID ${YOLO_PID})"
fi
if [[ "$SLAM_ENABLED" == "true" ]]; then
echo "    docker logs ${SLAM_CONTAINER}   (SLAM sidecar — auto-restarts via --restart=${SLAM_RESTART:-no})"
fi
if [[ "$MODE" == "benchmark" ]]; then
echo ""
echo "  Recording (benchmark mode):"
echo "    ${RUNREL}/bag            (RMSE input — meta.txt + bag_record.log alongside)"
echo "    → restart the Simulink sim NOW for a clean t=0, then let the run play out."
fi
echo ""
echo "  Stop:  ./run_stack_hil.sh stop"
echo "  Rate:  ./run_stack_hil.sh hz [topic]    (defaults to /sim/camera/image_raw)"
echo ""
sep
