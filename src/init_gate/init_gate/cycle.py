# cycle.py -- the generic warmup driver. Pure policy, no ROS wiring, no SLAM
# specifics. The per-SLAM motion (LEGS) + scalar params come from a profile
# (init_gate/profiles/<module>.py) chosen by config, passed in by init_gate_node.
#
# Open-loop mirror: each leg goes OUT at leg_speed for leg_duration_sec, then its
# exact MIRROR (same body-frame command, sign flipped, same duration) brings it
# back. Looped until SLAM reports ready or timeout_sec.
#
# Why mirror the COMMAND instead of a pose-based return: /cmd_vel is BODY frame,
# /sim/drone_pose is WORLD frame, and the drone can start at yaw=pi. A pose-based
# return computed in world frame but sent as a body command inverts sign at yaw=pi
# -> positive feedback -> the drone strafes away forever. Mirroring the command
# itself cannot invert: whatever "out" did, the sign-flipped "back" undoes, at any
# yaw. No pose, no frame math, no runaway.
#
# Readiness is checked at "center" (after each mirror-back), so handoff to the
# normal stack always happens from ~the start position, never mid-leg.
#
# On timeout: zero-hold, log SLAM_INIT_FAILED, return False. Do NOT tear anything
# down here (run_stack_hil.sh stop is host-only) -- leave the bag/sidecar up.

import time


def run_cycle(motion, readiness, log_state, legs, leg_speed,
              leg_duration_sec, timeout_sec):
    t_start = time.monotonic()
    while time.monotonic() - t_start < timeout_sec:
        if readiness.ready():
            log_state("SLAM_READY")
            return True
        for name, ux, uy, uz in legs:
            vx, vy, vz = ux * leg_speed, uy * leg_speed, uz * leg_speed
            motion.log(f"warmup: {name} (out + back)")
            motion.run_leg(vx, vy, vz, leg_duration_sec)          # go out
            motion.run_leg(-vx, -vy, -vz, leg_duration_sec)       # go back (mirror)
            if readiness.ready():
                log_state("SLAM_READY")
                return True

    motion.zero_hold()
    log_state("SLAM_INIT_FAILED")
    return False
