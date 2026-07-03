# motion.py -- the only two movement primitives the gate uses.
#
# Both are either elapsed-time (run_leg) or one-shot (return_to_center) --
# neither ever re-checks a tolerance in a loop. That's deliberate: a repeated
# tight-tolerance chase against a plant with real lag is what caused the
# undamped yaw-oscillation bug this gate exists to avoid (see servo_fsm_node's
# SearchStep::YAW_* steps and docs/fixlog/011-slam-init-gate.md).

import time
import rclpy
from geometry_msgs.msg import Twist

from init_gate.params import LEG_SPEED, SAFETY_MIN_ALTITUDE


class MotionCommander:
    def __init__(self, node, pub_cmd):
        self.node = node
        self.pub = pub_cmd

    def _send(self, vx=0.0, vy=0.0, vz=0.0):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
        self.pub.publish(cmd)

    def run_leg(self, vx, vy, vz, duration):
        """Elapsed-time only -- no tolerance check, cannot get stuck."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            self._send(vx, vy, vz)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._send(0.0, 0.0, 0.0)

    def return_to_center(self, pose, axis):
        """ONE-SHOT correction anchored to the absolute recorded start pose --
        computed once, executed once, never re-checked."""
        delta = pose.delta_from_start(axis)
        if axis == 'z' and pose.z + delta < SAFETY_MIN_ALTITUDE:
            delta = SAFETY_MIN_ALTITUDE - pose.z   # cheap, optional floor insurance
        speed = LEG_SPEED if delta >= 0 else -LEG_SPEED
        duration = abs(delta) / LEG_SPEED
        vy, vz = (speed, 0.0) if axis == 'y' else (0.0, speed)
        self.run_leg(0.0, vy, vz, duration)

    def zero_hold(self):
        self._send(0.0, 0.0, 0.0)
