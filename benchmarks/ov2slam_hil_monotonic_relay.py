#!/usr/bin/env python3

from __future__ import annotations

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class MonotonicImageRelay(Node):
    def __init__(self) -> None:
        super().__init__("ov2slam_hil_monotonic_relay")
        self.declare_parameter("input_topic", "/ovcam/image_raw")
        self.declare_parameter("output_topic", "/ovcam/image_raw_monotonic")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(Image, self.output_topic, qos)
        self.subscription = self.create_subscription(Image, self.input_topic, self._handle_image, qos)
        self.last_sec = -1
        self.last_nsec = -1
        self.adjusted_frames = 0
        self.get_logger().info(f"Relaying {self.input_topic} -> {self.output_topic} with monotonic timestamps")

    def _handle_image(self, msg: Image) -> None:
        out = copy.copy(msg)
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)
        if self.last_sec >= 0 and (sec < self.last_sec or (sec == self.last_sec and nsec <= self.last_nsec)):
            sec = self.last_sec
            nsec = self.last_nsec + 1
            if nsec >= 1_000_000_000:
                sec += 1
                nsec -= 1_000_000_000
            out.header.stamp.sec = sec
            out.header.stamp.nanosec = nsec
            self.adjusted_frames += 1
            if self.adjusted_frames <= 5 or self.adjusted_frames % 100 == 0:
                self.get_logger().warn(
                    f"Adjusted non-monotonic frame stamp to {sec}.{nsec:09d} (count={self.adjusted_frames})"
                )
        self.last_sec = sec
        self.last_nsec = nsec
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = MonotonicImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
