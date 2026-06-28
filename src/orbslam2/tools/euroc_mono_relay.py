#!/usr/bin/env python3

import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


class EuRoCMonoRelay(Node):
    def __init__(self) -> None:
        super().__init__("euroc_mono_relay")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._publisher = self.create_publisher(Image, "/camera", qos)
        self.create_subscription(Image, "/cam0/image_raw", self._handle_image, qos)

        self.get_logger().info("EuRoC mono relay ready: /cam0/image_raw -> /camera")

    def _handle_image(self, msg: Image) -> None:
        try:
            self._publisher.publish(msg)
        except Exception:
            if rclpy.ok():
                raise


def main() -> int:
    rclpy.init(args=None)
    node = EuRoCMonoRelay()
    shutdown_requested = False

    def _stop(signum, frame) -> None:  # type: ignore[unused-argument]
        nonlocal shutdown_requested
        if shutdown_requested:
            return
        shutdown_requested = True
        node.get_logger().info(f"Received signal {signum}, shutting down mono relay")
        try:
            try_shutdown = getattr(rclpy, "try_shutdown", None)
            if callable(try_shutdown):
                try_shutdown()
            elif rclpy.ok():
                rclpy.shutdown()
        except RuntimeError:
            pass

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if "Context must be initialized before it can be shutdown" not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            try_shutdown = getattr(rclpy, "try_shutdown", None)
            if callable(try_shutdown):
                try_shutdown()
            elif rclpy.ok():
                rclpy.shutdown()
        except RuntimeError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
