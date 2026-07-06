# cycle.py -- the warmup sequence. Pure policy, no ROS wiring.
#
# Open-loop mirror: each leg goes OUT for a fixed time, then its exact MIRROR
# (same body-frame command, sign flipped, same duration) brings it back. Looped
# until SLAM reports ready or TIMEOUT_SEC.
#
#   left 1s  -> right 1s   (go back)
#   right 1s -> left 1s    (go back)
#   up 1s    -> down 1s    (go back)
#
# Why mirror the COMMAND instead of computing a return from /sim/drone_pose:
# /cmd_vel is BODY frame, /sim/drone_pose is WORLD frame, and the drone starts
# at yaw=pi. The old pose-based return computed the correction in world frame
# but sent it as a body command -> at yaw=pi the sign inverted into positive
# feedback -> the drone strafed one direction forever. Mirroring the command
# itself cannot invert: whatever "left 1s" did, "right 1s" at the same speed
# undoes it, at any yaw. No pose, no frame math, no runaway.
#
# Readiness is only checked at "center" (after each mirror-back), so handoff to
# the normal stack always happens from ~the start position, never mid-leg.
#
# On timeout: zero-hold, log SLAM_INIT_FAILED, return False. Do NOT tear
# anything down from here (run_stack_hil.sh's `stop` is host-only). Leave the
# bag/sidecar running; the operator runs `./run_stack_hil.sh stop`.

import time

from init_gate.params import LEG_SPEED, LEG_DURATION_SEC, TIMEOUT_SEC


def run_cycle(motion, readiness, log_state):
    t_start = time.monotonic()
    legs = [
        #('left',  0.0, +LEG_SPEED, 0.0),
        #('right', 0.0, -LEG_SPEED, 0.0),
        ('up',    0.0, 0.0, +LEG_SPEED),
    ]
    while time.monotonic() - t_start < TIMEOUT_SEC:
        if readiness.ready():
            log_state('SLAM_READY')
            return True
        for name, vx, vy, vz in legs:
            motion.log(f'warmup: {name} (out + back)')
            motion.run_leg(vx, vy, vz, LEG_DURATION_SEC)         # go out
            motion.run_leg(-vx, -vy, -vz, LEG_DURATION_SEC)      # go back (mirror)
            if readiness.ready():
                log_state('SLAM_READY')
                return True

    motion.zero_hold()
    log_state('SLAM_INIT_FAILED')
    return False
