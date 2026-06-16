#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# controller_hil_bench.sh — ONE run of ONE controller.
#
#   ./benchmarks/controller_hil_bench.sh <ibvs|proportional> \
#       [run_num] [duration_sec] [benchmark_mode]
#
# Run this FROM INSIDE the container (enter_container.sh first).
# CycloneDDS ONLY. YOLO-AGNOSTIC: oracle detector replaces all perception.
#
# benchmark_mode (4th arg, default 'true'):
#   true  → BENCHMARKING. FSM waits for a fresh sim reset before engaging, so
#           the recorded approach starts from a clean plant reset (t=0). You
#           MUST Stop+Run the Simulink sim once after 'recording' appears.
#   false → SCOUTING. FSM engages immediately on boot — the drone searches and
#           approaches with NO Stop→Run needed. Use for demos / live flights.
#
# Flow (benchmark_mode=true):
#   1. MATLAB already up (init + Simulink running, CycloneDDS, domain 0).
#   2. Run this script — it starts recording immediately.
#   3. STOP + START the Simulink sim → controller resets to SEARCHING (clean t=0).
#   4. Let it approach + hold REACHED, then Ctrl-C.
#
# Flow (benchmark_mode=false):
#   1. MATLAB already up.
#   2. Run this script — drone scouts and approaches on its own. No Stop+Run.
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
RUN_NUM="${2:-1}"
DURATION="${3:-0}"
BENCHMARK_MODE="${4:-true}"
[[ "$CONTROLLER" == "ibvs" || "$CONTROLLER" == "proportional" ]] \
    || die "usage: $0 <ibvs|proportional> [run_num] [duration_sec] [benchmark_mode]"
[[ "$RUN_NUM" =~ ^[0-9]+$ ]] || die "run_num must be a positive integer (got '$RUN_NUM')"
case "$BENCHMARK_MODE" in
    true|false) ;;
    *) die "benchmark_mode must be 'true' or 'false' (got '$BENCHMARK_MODE')" ;;
esac

command -v envsubst >/dev/null || die "'envsubst' missing — apt install gettext-base"
: "${MATLAB_HOST_IP:?set MATLAB_HOST_IP to your Windows host IP}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── ROS2 environment (already inside container) ──
# Disable strict mode while sourcing — ROS2 setup scripts have unbound
# variables and non-zero returns that would kill the script under set -u.
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$WS/install/setup.bash" ]]; then
    set -u
    die "workspace not built — run: colcon build --packages-select yolo_msgs servo_core hil_servo oracle_detector --symlink-install"
fi
source "$WS/install/setup.bash"
set -u

# ── Verify required executables exist in install tree ──
[[ -f "$WS/install/hil_servo/lib/hil_servo/hil_servo_node" ]] \
    || die "hil_servo_node not found in install/ — run: colcon build --packages-select servo_core hil_servo --symlink-install"
[[ -d "$WS/install/oracle_detector" ]] \
    || die "oracle_detector not found in install/ — run: colcon build --packages-select oracle_detector --symlink-install"
log "Packages verified: hil_servo oracle_detector"

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
RUNREL="bags/ctrl_${CONTROLLER}_N${RUN_NUM}_${STAMP}"
mkdir -p "$WS/$RUNREL"
GIT_SHA=$(git rev-parse --short HEAD 2>/dev/null || echo nogit)
{
  echo "controller=$CONTROLLER"
  echo "run_num=$RUN_NUM"
  echo "benchmark_mode=$BENCHMARK_MODE"
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

if [[ "$BENCHMARK_MODE" == "true" ]]; then
log "================================================================="
log "  MODE: BENCHMARK — FSM waits for a fresh sim reset (clean t=0)."
log "  If you see REACHED state right away — that is NORMAL (stale pose)."
log "  DO NOT Ctrl-C!  Wait for the 'recording' line, then Stop+Run"
log "  the MATLAB Simulink sim to reset to t=0.  The FSM will auto-"
log "  reset to SEARCHING on heartbeat drop and the clean run begins."
log "================================================================="
else
log "================================================================="
log "  MODE: SCOUT — FSM engages immediately on boot. NO Stop+Run."
log "  The drone will search and approach on its own once nodes are up."
log "================================================================="
fi
log "Killing any stale nodes from previous runs..."
pkill -f oracle_detector_node 2>/dev/null || true
pkill -f visp_servo_node      2>/dev/null || true
pkill -f hil_servo_node       2>/dev/null || true
sleep 0.5

log "Launching oracle + $CONTROLLER (output visible below)..."

# Launch output goes to TERMINAL so errors are visible immediately.
ros2 launch /workspace/benchmarks/controller_bench.launch.py \
    controller:="$CONTROLLER" workspace:=/workspace \
    benchmark_mode:="$BENCHMARK_MODE" \
    2>&1 | tee "$WS/$RUNREL/launch.log" &
LAUNCH_PID=$!

log "Waiting 6s for nodes to start (DO NOT abort — REACHED at startup is expected)..."
sleep 6

# Diagnostics: log what pgrep sees (goes to stderr so it's visible but not in the log)
log "Node health check (pgrep output):"
pgrep -fa oracle_detector_node 2>&1 | sed 's/^/  [pgrep] /' || log "  [pgrep] oracle_detector_node — NOT FOUND"
pgrep -fa "$NODE_EXEC"         2>&1 | sed 's/^/  [pgrep] /' || log "  [pgrep] $NODE_EXEC — NOT FOUND"
log "  [launch pid $LAUNCH_PID alive? $(kill -0 "$LAUNCH_PID" 2>/dev/null && echo YES || echo NO)]"

# Hard check — both expected node processes must be alive (pgrep avoids DDS discovery timing)
MISSING=()
pgrep -f oracle_detector_node >/dev/null 2>&1 || MISSING+=(oracle_detector)
pgrep -f "$NODE_EXEC"         >/dev/null 2>&1 || MISSING+=("$NODE_EXEC")

if [[ ${#MISSING[@]} -gt 0 ]]; then
    log "ERROR: these nodes did not start: ${MISSING[*]}"
    log "Check the launch output above for the actual error."
    kill "$LAUNCH_PID" 2>/dev/null
    exit 1
fi
log "Nodes confirmed running: oracle_detector + $NODE_EXEC"

# CPU sampler — 1 Hz, robust PID resolution + /proc instantaneous %CPU.
"$WS/benchmarks/cpu_sampler.sh" "$NODE_EXEC" "$WS/$RUNREL/cpu.csv" 1 &
CPU_PID=$!

ros2 bag record -o "$WS/$RUNREL/bag" \
    --qos-profile-overrides-path "$WS/config/hil/bench_bag_qos.yaml" \
    /cmd_vel /sim/drone_pose /sim/target_pose /sim/pitch_angle \
    /sim/heartbeat /yolo/detections /bench/state \
    > "$WS/$RUNREL/bag_record.log" 2>&1 &
BAG_PID=$!

if [[ "$BENCHMARK_MODE" == "true" ]]; then
    log "recording — restart the Simulink sim NOW for clean t=0"
else
    log "recording — SCOUT mode: drone is searching/approaching on its own"
fi

if [[ "$DURATION" -gt 0 ]]; then
    sleep "$DURATION"
    cleanup
else
    log "running until Ctrl-C ..."
    wait "$LAUNCH_PID"
    log "Launch process exited on its own — see launch.log above"
    cleanup
fi