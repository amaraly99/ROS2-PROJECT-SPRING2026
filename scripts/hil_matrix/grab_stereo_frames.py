#!/usr/bin/env python3
"""Grab a few real, timestamp-matched left/right frame pairs off the live
/ovcam/* topics during a real trial and save them to disk, then exit.
Read-only: does not touch the trial, the stack, or any recorded bag.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "/tmp/stereo_frames"
N_PAIRS = 5


class Grabber(Node):
    def __init__(self):
        super().__init__("stereo_frame_grabber")
        import os
        os.makedirs(OUT_DIR, exist_ok=True)
        self.left = {}
        self.right = {}
        self.saved = 0
        self.create_subscription(Image, "/ovcam/image_raw", self._left_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/ovcam/right/image_raw", self._right_cb, qos_profile_sensor_data)
        self.get_logger().info(f"grabbing {N_PAIRS} pairs -> {OUT_DIR}")

    def _stamp_ns(self, msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _left_cb(self, msg):
        self.left[self._stamp_ns(msg)] = msg
        self._try_pair()

    def _right_cb(self, msg):
        self.right[self._stamp_ns(msg)] = msg
        self._try_pair()

    def _try_pair(self):
        if self.saved >= N_PAIRS:
            return
        common = set(self.left) & set(self.right)
        for ts in sorted(common):
            lmsg = self.left.pop(ts)
            rmsg = self.right.pop(ts)
            np.save(f"{OUT_DIR}/{self.saved:02d}_left.npy", self._to_arr(lmsg))
            np.save(f"{OUT_DIR}/{self.saved:02d}_right.npy", self._to_arr(rmsg))
            self.get_logger().info(f"saved pair {self.saved} @ stamp {ts} (exact stamp match)")
            self.saved += 1
            if self.saved >= N_PAIRS:
                self.get_logger().info("done")
                rclpy.shutdown()
                return

    def _to_arr(self, msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if msg.encoding == "mono8":
            return arr.reshape(msg.height, msg.width)
        return arr.reshape(msg.height, msg.width, 3)


def main():
    rclpy.init()
    node = Grabber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
