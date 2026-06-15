#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# controller_hil_bench.sh — ONE benchmark run (N=1) of ONE controller.
#
#   ./benchmarks/controller_hil_bench.sh <ibvs|proportional> [duration_sec]
#
# CycloneDDS ONLY (mandated). YOLO-AGNOSTIC: launches just the oracle
# detector + one controller node — no camera/YOLO/SLAM. Records a rosbag
# of the full interaction + a CPU trace of the controller, so all metrics
# are computed offline (fly once, analyse forever).
#
# Flow for a clean run:
#   1. MATLAB already up (init + Simulink running, CycloneDDS, domain 0).
#   2. Run this script — it starts recording immediately.
#   3. STOP + START the Simulink sim → controller resets to SEARCHING at
#      sim_t=0 (heartbeat jumps back). This is your clean t=0.
#   4. Let it approach + hold, then Ctrl-C here (or pass a duration).
#
# Env:
#   MATLAB_HOST_IP   REQUIRED — Windows host IP for unicast DDS discovery
#   ROS_DOMAIN_ID    default 0
#   CONTAINER        default ros2_perception_stack (reuses the built image)
# ─────────────────────────────────────────────────────────────────
set -u
cd "$(dirname "$0")/.."
WS="$(pwd)"

die() { echo "[bench] ERROR: $*" >&2; exit 1; }
log() { echo "[bench] $*"; }

CONTROLLER="${1:-}"
DURATION="${2:-0}"   # 0 = until Ctrl-C
[[ "$CONTROLLER" == "ibvs" || "$CONTROLLER" == "proportional" ]] \
    || die "usage: $0 <ibvs|proportional> [duration_sec]"

command -v envsubst >/dev/null || die "'envsubst' missing — apt install gettext-base"
command -v docker   >/dev/null || die "'docker' not found"
: "${MATLAB_HOST_IP:?set MATLAB_HOST_IP to your Windows host IP}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
CONTAINER="${CONTAINER:-ros2_perception_stack}"

# ── Render CycloneDDS profile (same approach as run_stack_hil.sh) ──
PI_INTERFACE=$(ip route get "$MATLAB_HOST_IP" 2>/dev/null | grep -oP 'dev \K\S+' | head -1)
[[ -z "$PI_INTERFACE" ]] && die "cannot find interface toward $MATLAB_HOST_IP — is the link up?"
DDS_RESOLVED=/tmp/cyclonedds_hil.resolved.xml
MATLAB_HOST_IP="$MATLAB_HOST_IP" PI_INTERFACE="$PI_INTERFACE" \
    envsubst < "$WS/config/hil/cyclonedds_hil.xml" > "$DDS_RESOLVED" \
    || die "envsubst of cyclone profile failed"
log "CycloneDDS profile → $DDS_RESOLVED (peer=$MATLAB_HOST_IP iface=$PI_INTERFACE)"

# ── Output dir (bags/ is gitignored) ──
STAMP=$(date +%Y%m%d_%H%M%S)
RUNREL="bags/ctrl_${CONTROLLER}_N1_${STAMP}"
mkdir -p "$WS/$RUNREL"
GIT_SHA=$(git rev-parse --short HEAD 2>/dev/null || echo nogit)
{
  echo "controller=$CONTROLLER"
  echo "git_sha=$GIT_SHA"
  echo "stamp=$STAMP"
  echo "matlab_host_ip=$MATLAB_HOST_IP"
  echo "dds=cyclonedds domain=$ROS_DOMAIN_ID"
} > "$WS/$RUNREL/meta.txt"
log "Run dir: $RUNREL  (git $GIT_SHA)"

# ── Ensure container is up (reuse the built image; minimal flags) ──
if ! sudo docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
    log "Starting container '$CONTAINER' (detached)..."
    sudo docker run -d --entrypoint "" --name "$CONTAINER" \
        --net=host --ipc=host \
        -v "$WS:/workspace" -v /tmp:/tmp \
        "$CONTAINER" sleep infinity >/dev/null \
        || die "could not start container '$CONTAINER' — is the image built?"
    sleep 1
fi

NODE_EXEC=$([[ "$CONTROLLER" == ibvs ]] && echo visp_servo_node || echo hil_servo_node)

log "Launching oracle + $CONTROLLER ... recording to $RUNREL/bag"
log ">>> When you see 'recording', STOP+START the Simulink sim for a clean t=0 <<<"

DOCKER_TTY=""; [[ -t 0 && -t 1 ]] && DOCKER_TTY="-it"

sudo docker exec $DOCKER_TTY "$CONTAINER" bash -lc "
set +u   # ROS2 setup.bash references AMENT_TRACE_SETUP_FILES without initialising it
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$DDS_RESOLVED
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export WORKSPACE_DIR=/workspace
export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\${LD_LIBRARY_PATH:-}

cleanup() { kill \$BAG_PID \$CPU_PID \$LAUNCH_PID 2>/dev/null; sleep 1; exit 0; }
trap cleanup INT TERM

ros2 launch /workspace/benchmarks/controller_bench.launch.py \
    controller:=$CONTROLLER workspace:=/workspace \
    > /workspace/$RUNREL/launch.log 2>&1 &
LAUNCH_PID=\$!

sleep 4
CTRL_PID=\$(pgrep -f $NODE_EXEC | head -1)
echo \"controller_pid=\$CTRL_PID exec=$NODE_EXEC\" >> /workspace/$RUNREL/meta.txt

# CPU sampler (portable — no sysstat dependency). 1 Hz: epoch,%cpu,%mem.
if [[ -n \"\$CTRL_PID\" ]]; then
  ( echo 'epoch,pcpu,pmem'
    while kill -0 \$CTRL_PID 2>/dev/null; do
      read C M < <(ps -o %cpu=,%mem= -p \$CTRL_PID)
      echo \"\$(date +%s.%N),\$C,\$M\"; sleep 1
    done ) > /workspace/$RUNREL/cpu.csv &
  CPU_PID=\$!
else
  echo '[bench] WARN: controller PID not found — CPU not sampled' ; CPU_PID=
fi

ros2 bag record -o /workspace/$RUNREL/bag \
    --qos-profile-overrides-path /workspace/config/hil/bench_bag_qos.yaml \
    /cmd_vel /sim/drone_pose /sim/target_pose /sim/pitch_angle \
    /sim/heartbeat /yolo/detections /bench/state \
    > /workspace/$RUNREL/bag_record.log 2>&1 &
BAG_PID=\$!

echo '[bench] recording — restart the Simulink sim NOW for clean t=0'
if [[ $DURATION -gt 0 ]]; then
  sleep $DURATION; cleanup
else
  echo '[bench] running until Ctrl-C ...'; wait \$LAUNCH_PID
fi
"

log "Done. Saved: $RUNREL"
log "Analyse:  python3 benchmarks/plot_controller_hil.py $RUNREL"
