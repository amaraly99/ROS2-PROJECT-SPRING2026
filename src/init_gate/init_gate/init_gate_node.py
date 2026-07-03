# init_gate_node.py -- thin ROS2 wiring only. All policy lives in cycle.py;
# all sensing lives in pose_tracker.py/slam_readiness.py; all actuation lives
# in motion.py. This file just connects them.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from init_gate.params import READY_DEBOUNCE
from init_gate.pose_tracker import PoseTracker
from init_gate.slam_readiness import SlamReadiness
from init_gate.motion import MotionCommander
from init_gate.cycle import run_cycle


class InitGateNode(Node):
    def __init__(self):
        super().__init__('init_gate_node')
        pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_bench = self.create_publisher(String, '/bench/state', 10)

        self.pose = PoseTracker(self)
        self.readiness = SlamReadiness(self, READY_DEBOUNCE)
        self.motion = MotionCommander(self, pub_cmd)

    def log_state(self, s):
        msg = String()
        msg.data = s
        self.pub_bench.publish(msg)
        self.get_logger().info(f'[init_gate] {s}')

    def run(self):
        return run_cycle(self.motion, self.pose, self.readiness, self.log_state)


def main():
    rclpy.init()
    node = InitGateNode()
    ok = node.run()
    rclpy.shutdown()
    exit(0 if ok else 1)


if __name__ == '__main__':
    main()
