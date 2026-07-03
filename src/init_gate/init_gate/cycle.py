# cycle.py -- the step SEQUENCE + the run loop. Pure policy, no ROS wiring.
#
# center -> left -> center -> right -> center -> up -> center, looped, until
# readiness.ready() or TIMEOUT_SEC. Readiness is checked only at "center"
# checkpoints -- a deliberate, bounded-latency simplification (at most one
# leg's duration before reacting).
#
# On timeout: zero-hold, log SLAM_INIT_FAILED, return False. Do NOT try to
# tear anything down from in here -- run_stack_hil.sh's `stop` subcommand is
# host-only (sudo docker stop/rm), unreachable from inside this container, and
# asking to kill the very container this node runs in is not a safe pattern.
# Leave the bag/sidecar running for inspection; the operator runs
# `./run_stack_hil.sh stop` manually.

import time

from init_gate.params import LEG_SPEED, LEG_DURATION_SEC, TIMEOUT_SEC


def run_cycle(motion, pose, readiness, log_state):
    # Clock starts BEFORE the pose wait, not after -- otherwise a simulator
    # that never starts publishing /sim/drone_pose hangs this node forever
    # with no timeout and no error, defeating the whole point of TIMEOUT_SEC.
    t_start = time.monotonic()
    while not pose.has_pose():
        if time.monotonic() - t_start >= TIMEOUT_SEC:
            motion.zero_hold()
            log_state('SLAM_INIT_FAILED')
            return False
        time.sleep(0.1)

    legs = [
        (0.0, +LEG_SPEED, 0.0, 'y'),   # left,  return axis 'y'
        (0.0, -LEG_SPEED, 0.0, 'y'),   # right, return axis 'y'
        (0.0, 0.0, +LEG_SPEED, 'z'),   # up,    return axis 'z'
    ]
    while time.monotonic() - t_start < TIMEOUT_SEC:
        if readiness.ready():
            log_state('SLAM_READY')
            return True
        for vx, vy, vz, axis in legs:
            motion.run_leg(vx, vy, vz, LEG_DURATION_SEC)
            if readiness.ready():
                break
            motion.return_to_center(pose, axis)
            if readiness.ready():
                break
        if readiness.ready():
            log_state('SLAM_READY')
            return True

    motion.zero_hold()
    log_state('SLAM_INIT_FAILED')
    return False
