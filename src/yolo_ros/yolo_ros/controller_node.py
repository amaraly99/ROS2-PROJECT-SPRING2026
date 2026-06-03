#!/usr/bin/env python3
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from yolo_msgs.msg import DetectionArray


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.cb,
            qos
        )

        # Moving average smoothing for center_x
        self.x_hist = deque(maxlen=5)

        # Thresholds for lateral centering
        self.left_thresh  = 0.4
        self.right_thresh = 0.6

        self.get_logger().info('Controller started, listening on /yolo/detections')

    def cb(self, msg: DetectionArray):
        if not msg.detections:
            return

        # Pick highest-confidence detection
        best = max(msg.detections, key=lambda d: d.confidence)

        x = float(best.center_x)
        self.x_hist.append(x)
        x_smooth = sum(self.x_hist) / len(self.x_hist)

        if x_smooth < self.left_thresh:
            self.get_logger().info(
                f'Target Left  (x={x_smooth:.2f}, class={best.class_name}) -> Turn Left')
        elif x_smooth > self.right_thresh:
            self.get_logger().info(
                f'Target Right (x={x_smooth:.2f}, class={best.class_name}) -> Turn Right')
        else:
            self.get_logger().info(
                f'Target Center(x={x_smooth:.2f}, class={best.class_name}) -> Move Forward')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
