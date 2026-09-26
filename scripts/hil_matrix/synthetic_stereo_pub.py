#!/usr/bin/env python3
"""Synthetic stereo pair publisher for WIRING tests of a stereo SLAM sidecar (2026-09-08).

Publishes what the HIL camera bridges publish -- /ovcam/image_raw + /ovcam/right/image_raw,
mono8 640x480, reliable QoS depth 2, frame_ids camera / camera_right, IDENTICAL stamp per
pair -- without MATLAB. Scene: a random-dot texture on a fronto-parallel plane at depth Z,
so the right image is the left shifted by the true disparity d = fx*b/Z (fx=554, b=0.11).
The camera translates sideways (the texture window slides), giving consistent stereo +
motion so a backend can initialise, track, and publish non-zero poses.

This proves topics/QoS/stamps/calib parsing/publishing (the contract), not accuracy --
accuracy is only judged on the live HIL trial. Run inside a container that has rclpy:
    ROS_DOMAIN_ID=<isolated> python3 synthetic_stereo_pub.py [--hz 15] [--depth 6.0]
"""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

FX, B = 554.0, 0.11
W, H = 640, 480


class SyntheticStereo(Node):
    def __init__(self, hz, depth, speed_px):
        super().__init__("synthetic_stereo_pub")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.pl = self.create_publisher(Image, "/ovcam/image_raw", qos)
        self.pr = self.create_publisher(Image, "/ovcam/right/image_raw", qos)
        rng = np.random.default_rng(7)
        self.tex_w = 4000
        # Blurred random dots: enough corners for ORB/GFTT, no aliasing on the shift.
        tex = rng.integers(0, 256, size=(H, self.tex_w), dtype=np.uint8).astype(np.float32)
        k = np.array([1, 4, 6, 4, 1], dtype=np.float32) / 16.0
        for _ in range(2):
            tex = np.apply_along_axis(lambda r: np.convolve(r, k, mode="same"), 1, tex)
            tex = np.apply_along_axis(lambda c: np.convolve(c, k, mode="same"), 0, tex)
        tex = (tex - tex.min()) / (tex.max() - tex.min()) * 255.0
        self.tex = tex.astype(np.uint8)
        self.disp = int(round(FX * B / depth))
        self.x = 100.0
        self.dir = 1.0
        self.speed = speed_px
        self.n = 0
        self.timer = self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(f"synthetic stereo: {hz} Hz, depth {depth} m -> disparity {self.disp} px, lateral {speed_px} px/frame")

    def img(self, x0, frame_id, stamp):
        m = Image()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.height, m.width, m.encoding, m.is_bigendian, m.step = H, W, "mono8", 0, W
        m.data = self.tex[:, x0:x0 + W].tobytes()
        return m

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        xl = int(self.x)
        # A world point at texture column c is at u_l = c - xl in the left image. The right
        # camera sits +x of the left, so the same point lands FURTHER LEFT in the right
        # image: u_r = u_l - disparity. Hence the right window starts disp columns further
        # RIGHT in the texture (u_r = c - (xl + disp)). Getting this sign wrong yields
        # negative disparity = points behind the camera, and every backend rejects them.
        xr = xl + self.disp
        if xl <= 0 or xr + W >= self.tex_w:
            self.dir *= -1.0
        self.pl.publish(self.img(xl, "camera", stamp))
        self.pr.publish(self.img(xr, "camera_right", stamp))
        self.x += self.dir * self.speed
        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info(f"published {self.n} pairs")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--depth", type=float, default=6.0)
    ap.add_argument("--speed-px", type=float, default=1.5)
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = SyntheticStereo(a.hz, a.depth, a.speed_px)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
