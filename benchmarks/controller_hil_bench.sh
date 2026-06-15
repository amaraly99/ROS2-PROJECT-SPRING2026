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
set -euo pipefail
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
if [[ ! -f "$WS/install/setup.bash" ]]; then
    die "workspace not built — run: colcon build --packages-select yolo_msgs servo_core hil_servo oracle_detector --symlink-install"
fi
source "$WS/install/setup.bash"
set -u

# ── Verify required packages are installed ──
for pkg in hil_servo oracle_detector servo_core; do
    ros2 pkg list | grep -q "^${pkg}$" \
        || die "ROS2 package '${pkg}' not found — did you source install/setup.bash and build?"
done
log "Packages verified: hil_servo oracle_detector servo_core"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID
export LD_LIBRARY_PATH="${WS}/opencv/build/lib:${LD_LIBRARY_PATH:-}"

# ── Resolve network interface (allow env override: export PI_INTERFACE=wlan0) ──
if [[ -z "${PI_INTERFACE:-}" ]]; then
    PI_INTERFACE=$(ip route get "$MATLAB_HOST_IP" 2>/dev/null | grep -oP 'dev \K\S+' | head -1)
    if [[ -z "${PI_INTERFACE:-}" ]]; then
        PI_INTERFACE=$(ip route show default 2>/dev/null | grep -oP 'dev \K\S+' | head -1)
    fi
fi
[[ -z "${PI_INTERFACE:-}" ]] && die "cannot detect network interface — set manually: export PI_INTERFACE=wlan0"
log "Network interface: $PI_INTERFACE"

DDS_RESOLVED=/tmp/cyclonedds_hil.resolved.xml
MATLAB_HOST_IP="$MATLAB_HOST_IP" PI_INTERFACE="$PI_INTERFACE" \
    envsubst < "$WS/config/hil/cyclonedds_hil.xml" > "$DDS_RESOLVED" \
    || die "envsubst of cyclone profile failed"
export CYCLONEDDS_URI="file://$DDS_RESOLVED"
log "CycloneDDS: peer=$MATLAB_HOST_IP iface=$PI_INTERFACE"

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

BAG_PID=; CPU_PID=; LAUNCH_PID=
cleanup() {
    log "Stopping..."
    [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null
    [[ -n "$BAG_PID"    ]] && kill "$BAG_PID"    2>/dev/null
    [[ -n "$CPU_PID"    ]] && kill "$CPU_PID"    2>/dev/null
    sleep 1
    exit 0
}
trap cleanup INT TERM

log "Launching oracle + $CONTROLLER (output visible below)..."

# Launch output goes to TERMINAL so errors are visible immediately.
ros2 launch /workspace/benchmarks/controller_bench.launch.py \
    controller:="$CONTROLLER" workspace:=/workspace \
    2>&1 | tee "$WS/$RUNREL/launch.log" &
LAUNCH_PID=$!

log "Waiting 6s for nodes to start..."
sleep 6

# Hard check — both expected nodes must be alive
MISSING=()
ros2 node list 2>/dev/null | grep -q oracle_detector || MISSING+=(oracle_detector)
ros2 node list 2>/dev/null | grep -q "$NODE_EXEC"   || MISSING+=("$NODE_EXEC")

if [[ ${#MISSING[@]} -gt 0 ]]; then
    log "ERROR: these nodes did not start: ${MISSING[*]}"
    log "Check the launch output above for the actual error."
    kill "$LAUNCH_PID" 2>/dev/null
    exit 1
fi
log "Nodes confirmed running: oracle_detector + $NODE_EXEC"

# CPU sampler — 1 Hz
CTRL_PID=$(pgrep -f "$NODE_EXEC" | head -1 || true)
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
    log "Launch process exited on its own — see launch.log above"
    cleanup
fi