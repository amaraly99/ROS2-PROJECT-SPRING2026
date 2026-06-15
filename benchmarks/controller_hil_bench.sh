#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# controller_hil_bench.sh — ONE benchmark run (N=1) of ONE controller.
#
#   ./benchmarks/controller_hil_bench.sh <ibvs|proportional> [duration_sec]
#
# Run this FROM INSIDE the container (enter_container.sh first).
# CycloneDDS ONLY. YOLO-AGNOSTIC: oracle detector replaces all perception.
#
# Flow:
#   1. MATLAB already up (init + Simulink running, CycloneDDS, domain 0).
#   2. Run this script — it starts recording immediately.
#   3. STOP + START the Simulink sim → controller resets to SEARCHING (clean t=0).
#   4. Let it approach + hold REACHED, then Ctrl-C.
#
# Required env:
#   MATLAB_HOST_IP   Windows host IP for unicast DDS discovery
#   ROS_DOMAIN_ID    default 0
# ─────────────────────────────────────────────────────────────────
set -u
cd "$(dirname "$0")/.."
WS="$(pwd)"

die() { echo "[bench] ERROR: $*" >&2; exit 1; }
log() { echo "[bench] $*"; }

CONTROLLER="${1:-}"
DURATION="${2:-0}"
[[ "$CONTROLLER" == "ibvs" || "$CONTROLLER" == "proportional" ]] \
    || die "usage: $0 <ibvs|proportional> [duration_sec]"

command -v envsubst >/dev/null || die "'envsubst' missing — apt install gettext-base"
: "${MATLAB_HOST_IP:?set MATLAB_HOST_IP to your Windows host IP}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── ROS2 environment (already inside container) ──
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash" || die "workspace not built — run colcon build first"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID
export LD_LIBRARY_PATH="${WS}/opencv/build/lib:${LD_LIBRARY_PATH:-}"

# ── Render CycloneDDS profile ──
PI_INTERFACE=$(ip route get "$MATLAB_HOST_IP" 2>/dev/null | grep -oP 'dev \K\S+' | head -1)
[[ -z "$PI_INTERFACE" ]] && die "cannot find interface toward $MATLAB_HOST_IP — is the link up?"
DDS_RESOLVED=/tmp/cyclonedds_hil.resolved.xml
MATLAB_HOST_IP="$MATLAB_HOST_IP" PI_INTERFACE="$PI_INTERFACE" \
    envsubst < "$WS/config/hil/cyclonedds_hil.xml" > "$DDS_RESOLVED" \
    || die "envsubst of cyclone profile failed"
export CYCLONEDDS_URI="file://$DDS_RESOLVED"
log "CycloneDDS profile → $DDS_RESOLVED (peer=$MATLAB_HOST_IP iface=$PI_INTERFACE)"

# ── Output dir ──
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

NODE_EXEC=$([[ "$CONTROLLER" == ibvs ]] && echo visp_servo_node || echo hil_servo_node)

cleanup() {
    log "Stopping..."
    kill "$BAG_PID" "$CPU_PID" "$LAUNCH_PID" 2>/dev/null
    sleep 1
    exit 0
}
trap cleanup INT TERM

log "Launching oracle + $CONTROLLER ... recording to $RUNREL/bag"
log ">>> When you see 'recording', STOP+START the Simulink sim for a clean t=0 <<<"

ros2 launch /workspace/benchmarks/controller_bench.launch.py \
    controller:="$CONTROLLER" workspace:=/workspace \
    > "$WS/$RUNREL/launch.log" 2>&1 &
LAUNCH_PID=$!

sleep 4

# Check the launch didn't immediately die
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log "Launch failed — printing launch.log:"
    cat "$WS/$RUNREL/launch.log"
    die "ros2 launch exited immediately (see above)"
fi

CTRL_PID=$(pgrep -f "$NODE_EXEC" | head -1)
echo "controller_pid=${CTRL_PID:-none} exec=$NODE_EXEC" >> "$WS/$RUNREL/meta.txt"

# CPU sampler — 1 Hz: epoch,%cpu,%mem
if [[ -n "${CTRL_PID:-}" ]]; then
    (
        echo 'epoch,pcpu,pmem'
        while kill -0 "$CTRL_PID" 2>/dev/null; do
            read -r C M < <(ps -o %cpu=,%mem= -p "$CTRL_PID")
            echo "$(date +%s.%N),$C,$M"
            sleep 1
        done
    ) > "$WS/$RUNREL/cpu.csv" &
    CPU_PID=$!
else
    log "WARN: controller PID not found — CPU not sampled"
    CPU_PID=
fi

ros2 bag record -o "$WS/$RUNREL/bag" \
    --qos-profile-overrides-path "$WS/config/hil/bench_bag_qos.yaml" \
    /cmd_vel /sim/drone_pose /sim/target_pose /sim/pitch_angle \
    /sim/heartbeat /yolo/detections /bench/state \
    > "$WS/$RUNREL/bag_record.log" 2>&1 &
BAG_PID=$!

log "recording — restart the Simulink sim NOW for clean t=0"

if [[ "$DURATION" -gt 0 ]]; then
    sleep "$DURATION"
    cleanup
else
    log "running until Ctrl-C ..."
    wait "$LAUNCH_PID"
    # If we get here the launch died on its own — print why
    log "Launch process exited unexpectedly:"
    cat "$WS/$RUNREL/launch.log"
    cleanup
fi