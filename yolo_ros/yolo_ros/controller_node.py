#!/usr/bin/env python3
from collections import deque
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import Detection


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.sub = self.create_subscription(
            Detection,
            '/yolo/detection',
            self.cb,
            10
        )

        # Moving average smoothing for center_x
        self.x_hist = deque(maxlen=5)

        # Servo thresholds
        self.left_thresh = 0.4
        self.right_thresh = 0.6

        self.get_logger().info("🕹️ Controller started. Listening on /yolo/detection")

    def cb(self, msg: Detection):
        x = float(msg.center_x)
        self.x_hist.append(x)
        x_smooth = sum(self.x_hist) / len(self.x_hist)

        if x_smooth < self.left_thresh:
            self.get_logger().info(f"⬅️ Target Left (x={x_smooth:.2f}) -> Turn Left")
        elif x_smooth > self.right_thresh:
            self.get_logger().info(f"➡️ Target Right (x={x_smooth:.2f}) -> Turn Right")
        else:
            self.get_logger().info(f"⬆️ Target Center (x={x_smooth:.2f}) -> Move Forward")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
