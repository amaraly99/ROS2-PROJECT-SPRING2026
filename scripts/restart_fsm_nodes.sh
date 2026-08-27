#!/bin/bash
# Kill the detector + controller (FSM) stack nodes INSIDE ros2_perception_stack,
# leaving the camera bridges (sim_camera_bridge_node / ovcam_bridge_node) and the
# separate slam_* container untouched. Called by run_stack_hil.sh --resume-fsm so
# a retry RESTARTS the FSM instead of stacking duplicates.
#
# Uses exact-name matching (pkill -x, matches the kernel 'comm' field) for the C++
# nodes and a comm-checked kill for the python detector -- NOT 'pkill -f <pattern>',
# which self-matches the wrapper shell (its own cmdline contains the pattern) and
# kills itself mid-run. That self-match was the bug that let duplicate FSMs pile up.
set +e
for n in hil_servo_node visp_servo_node h_vs_servo_node visp_pbvs_node yolo_bridge_node; do
    pkill -9 -x "$n" 2>/dev/null
done
for pid in $(pgrep -f oracle_detector_node 2>/dev/null); do
    if [ "$(cat /proc/$pid/comm 2>/dev/null)" = "python3" ]; then
        kill -9 "$pid" 2>/dev/null
    fi
done
exit 0
