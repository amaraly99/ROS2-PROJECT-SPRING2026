#!/bin/bash
# scripts/record_bag.sh — record a ROS2 bag from the running HIL stack
#
# Usage:
#   ./scripts/record_bag.sh          # lightweight topics only
#   ./scripts/record_bag.sh --full   # + all image streams and SLAM visualisation
#
# Run from the HOST while run_stack_hil.sh is active. ros2 bag record executes
# inside the Docker container (ROS2 is not installed on the host) against the
# workspace volume-mounted at /workspace, so bags appear on the host at bags/.
#
# Output:  bags/hil_YYYYMMDD_HHMMSS        (lightweight)
#          bags/hil_full_YYYYMMDD_HHMMSS   (full)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$SCRIPT_DIR")"
CONTAINER="ros2_perception_stack"

# ── argument parsing ──────────────────────────────────────────────────────────
FULL=false
for arg in "$@"; do
    case "$arg" in
        --full) FULL=true ;;
        *) echo "[record_bag] Unknown argument: $arg" >&2
           echo "Usage: $0 [--full]" >&2
           exit 1 ;;
    esac
done

# ── container check ───────────────────────────────────────────────────────────
if ! sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "[record_bag] ERROR: container '${CONTAINER}' is not running — start the HIL stack first:" >&2
    echo "             ./run_stack_hil.sh" >&2
    exit 1
fi

# ── DDS config (reuse profile resolved by run_stack_hil.sh into /tmp) ────────
DDS="${DDS:-fastrtps}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

case "$DDS" in
    fastrtps)
        RMW_IMPL=rmw_fastrtps_cpp
        DDS_RESOLVED="/tmp/fastrtps_hil.resolved.xml"
        DDS_ENV="FASTRTPS_DEFAULT_PROFILES_FILE=${DDS_RESOLVED}"
        ;;
    cyclonedds)
        RMW_IMPL=rmw_cyclonedds_cpp
        DDS_RESOLVED="/tmp/cyclonedds_hil.resolved.xml"
        DDS_ENV="CYCLONEDDS_URI=file://${DDS_RESOLVED}"
        ;;
    *)
        echo "[record_bag] ERROR: Unknown DDS=${DDS}; expected 'fastrtps' or 'cyclonedds'" >&2
        exit 1
        ;;
esac

if [[ ! -f "$DDS_RESOLVED" ]]; then
    echo "[record_bag] WARNING: ${DDS_RESOLVED} not found — DDS peer discovery may be incomplete" >&2
fi

# ── topic lists ───────────────────────────────────────────────────────────────
# /tf and /tf_static included in lightweight: OV2SLAM broadcasts world→camera
# here; without them rviz replay has no camera pose in world frame.
LIGHTWEIGHT_TOPICS=(
    /cmd_vel
    /yolo/detections
    /vo_pose
    /rosout
    /tf
    /tf_static
)

# Full mode adds every image stream + SLAM visualisation.
# /ovcam/image_raw is the exact mono8 stream OV2SLAM subscribes to — teammate
# can replay it directly into ov2slam_node without any hardware.
# /camera/image_raw is included as requested; absent in HIL so bag record will
# warn once and skip it silently — no harm.
FULL_EXTRA_TOPICS=(
    /sim/camera/image_raw
    /camera/image_raw
    /ovcam/image_raw
    /yolo/annotated
    /visp/debug_image
    /image_track
    /point_cloud
    /vo_traj
    /sim/drone_pose
    /sim/target_pose
    /sim/pitch_angle
)

# ── output path ───────────────────────────────────────────────────────────────
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
if $FULL; then
    MODE="FULL"
    BAG_SUBDIR="hil_full_${TIMESTAMP}"
    TOPICS=("${LIGHTWEIGHT_TOPICS[@]}" "${FULL_EXTRA_TOPICS[@]}")
else
    MODE="LIGHTWEIGHT"
    BAG_SUBDIR="hil_${TIMESTAMP}"
    TOPICS=("${LIGHTWEIGHT_TOPICS[@]}")
fi

# Paths: host-side for display, container-side for ros2 bag record
BAG_HOST="${WS}/bags/${BAG_SUBDIR}"
BAG_CONTAINER="/workspace/bags/${BAG_SUBDIR}"
QOS_CONTAINER="/workspace/config/hil/bag_qos_overrides.yaml"

# ── pre-flight summary ────────────────────────────────────────────────────────
echo ""
echo "[record_bag] Mode:    ${MODE}"
echo "[record_bag] Output:  ${BAG_HOST}"
echo "[record_bag] DDS:     ${DDS}  RMW=${RMW_IMPL}  domain=${ROS_DOMAIN_ID}"
echo "[record_bag] Topics:"
for t in "${TOPICS[@]}"; do
    printf "[record_bag]   %s\n" "$t"
done
echo ""
echo "[record_bag] Recording inside container '${CONTAINER}' — press Ctrl+C to stop."
echo ""

# ── record (inside Docker; /workspace and /tmp are already volume-mounted) ───
# -it so that Ctrl+C sends SIGINT to ros2 bag record and it finalises the bag.
DOCKER_FLAGS="-i"
[[ -t 0 && -t 1 ]] && DOCKER_FLAGS="-it"

sudo docker exec $DOCKER_FLAGS "$CONTAINER" bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    export RMW_IMPLEMENTATION=${RMW_IMPL}
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    export ${DDS_ENV}
    ros2 bag record \\
        --output ${BAG_CONTAINER} \\
        --qos-profile-overrides-path ${QOS_CONTAINER} \\
        ${TOPICS[*]}
"
