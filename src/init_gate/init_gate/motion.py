# motion.py -- one movement primitive, plus /cmd_vel publishing.
#
# run_leg commands a body-frame velocity for a fixed time then stops. That's
# it -- every "return" is just run_leg with the sign flipped (see cycle.py).
# No pose feedback, no tolerance loop, so nothing here can oscillate or run
# away.

import time
import rclpy
from geometry_msgs.msg import Twist


class MotionCommander:
    def __init__(self, node, pub_cmd):
        self.node = node
        self.pub = pub_cmd

    def _send(self, vx=0.0, vy=0.0, vz=0.0):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
        self.pub.publish(cmd)

    def run_leg(self, vx, vy, vz, duration):
        """Command a body-frame velocity for `duration` seconds, then stop.
        Spins the node each tick so subscription callbacks (SLAM readiness)
        keep firing while we move."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            self._send(vx, vy, vz)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._send(0.0, 0.0, 0.0)

    def zero_hold(self):
        self._send(0.0, 0.0, 0.0)

    def log(self, msg):
        self.node.get_logger().info(f'[init_gate] {msg}')
