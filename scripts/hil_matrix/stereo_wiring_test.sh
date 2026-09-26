#!/usr/bin/env bash
# stereo_wiring_test.sh <stack_config_name> [ros_domain_id] -- 2026-09-08
# Launches a stereo SLAM sidecar EXACTLY as run_stack_hil.sh would (image / command /
# setup_overlay / ld_prefix / cpu / remaps all taken from the real stack config through
# scripts/parse_stack.py), feeds it the synthetic stereo pair (synthetic_stereo_pub.py,
# same topics/QoS/stamps as the ovcam bridges), and reports whether the backend-agnostic
# contract is met: /slam/tracking_state reaches 2, /slam/pose publishes, timing CSV grows.
# Isolated ROS_DOMAIN_ID so it cannot touch a real stack. Cleans up its own containers.
set -u
CFG=$1; DOM=${2:-71}
WS=$HOME/ROS2-slam-hil; cd "$WS" || exit 1
eval "$(python3 scripts/parse_stack.py config/hil/stack/$CFG.yaml --emit-env)"
OUT=/tmp/wiring_$CFG; rm -rf "$OUT"; mkdir -p "$OUT"
echo "################ $CFG  (image=$SLAM_IMAGE cpu=${SLAM_CPU:-all} domain=$DOM) ################"
sudo -n docker rm -f wt_pub wt_slam >/dev/null 2>&1
sudo -n docker run -d --name wt_pub --net=host --ipc=host --entrypoint "" \
  -v "$WS:/workspace" -v /tmp:/tmp ros2_perception_stack bash -lc \
  "source /opt/ros/jazzy/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=$DOM; exec python3 /workspace/scripts/hil_matrix/synthetic_stereo_pub.py --hz 15 --depth 6 > $OUT/pub.log 2>&1" >/dev/null
SETUP_CMD=":"; [ -n "${SLAM_SETUP_OVERLAY:-}" ] && SETUP_CMD="source ${SLAM_SETUP_OVERLAY}"
sudo -n docker run -d --name wt_slam --net=host --ipc=host --privileged --entrypoint "" \
  -v "$WS:/workspace" -v /tmp:/tmp "$SLAM_IMAGE" bash -lc "
    sleep ${SLAM_DELAY:-0}
    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    ${SETUP_CMD}
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=$DOM
    export ORB_BENCH_TIMING_CSV=$OUT/timing_events.csv; export OV2_BENCH_TIMING_CSV=$OUT/ov2slam_timing_events.csv
    export LD_LIBRARY_PATH=${SLAM_LD_PREFIX:+${SLAM_LD_PREFIX}:}/workspace/opencv/build/lib:\${LD_LIBRARY_PATH:-}
    exec ${SLAM_CPU:+taskset -c ${SLAM_CPU}} ${SLAM_COMMAND} --ros-args ${SLAM_REMAPS} >$OUT/sidecar.log 2>&1
  " >/dev/null || { echo "SIDECAR FAILED TO START"; exit 2; }
sleep 30
probe(){ sudo -n docker exec wt_pub bash -lc "source /opt/ros/jazzy/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=$DOM; $1" 2>/dev/null; }
echo "-- containers: $(sudo -n docker ps --format '{{.Names}}:{{.Status}}' | grep -E 'wt_' | tr '\n' ' ')"
echo "-- publisher: $(tail -1 $OUT/pub.log)"
echo -n "-- tracking_state samples over ~45s: "
for i in 1 2 3 4 5 6; do v=$(probe "timeout 7 ros2 topic echo /slam/tracking_state --once --field data" | head -1); echo -n "${v:-none} "; sleep 1; done; echo
echo "-- /slam/pose: $(probe 'timeout 12 ros2 topic hz /slam/pose --window 30' | grep -m1 -E 'average|not appear')"
probe 'timeout 8 ros2 topic echo /slam/pose --once' | sed -n '1,4p;8,11p' | sed 's/^/   /'
for f in timing_events.csv ov2slam_timing_events.csv; do [ -f "$OUT/$f" ] && echo "-- $f: $(wc -l < $OUT/$f) lines; last: $(tail -1 $OUT/$f | cut -c1-90)"; done
echo "-- sidecar log (errors / state lines):"
grep -aiE "error|exception|fail|lost|initializ|new map|tracking|handshake|state=|ready" "$OUT/sidecar.log" | grep -av "Did not receive" | tail -8 | cut -c1-170 | sed 's/^/   /'
sudo -n docker rm -f wt_pub wt_slam >/dev/null 2>&1
echo "################ end $CFG ################"; echo
