#!/usr/bin/env python3

import numpy as np
import rclpy
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from sensor_msgs.msg import Image


def image_to_numpy(msg: Image):
    if msg.encoding in ("mono8", "8UC1"):
        channels = 1
        dtype = np.uint8
    elif msg.encoding in ("rgb8", "bgr8", "8UC3"):
        channels = 3
        dtype = np.uint8
    else:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    itemsize = np.dtype(dtype).itemsize
    row_elems = msg.step // itemsize
    flat = np.frombuffer(msg.data, dtype=dtype)
    if channels == 1:
        img = flat.reshape((msg.height, row_elems))[:, : msg.width]
    else:
        img = flat.reshape((msg.height, row_elems // channels, channels))[:, : msg.width, :]
    return img.copy()


def numpy_to_image(arr: np.ndarray, template: Image):
    out = Image()
    out.header = template.header
    out.height = template.height
    out.width = template.width
    out.encoding = template.encoding
    out.is_bigendian = template.is_bigendian
    out.step = int(arr.strides[0])
    out.data = arr.tobytes()
    return out


def robust_range(gray: np.ndarray):
    lo = float(np.percentile(gray, 5.0))
    hi = float(np.percentile(gray, 95.0))
    if hi <= lo:
        hi = lo + 1.0
    return lo, hi


def affine_match(image: np.ndarray, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float):
    scale = (dst_hi - dst_lo) / max(src_hi - src_lo, 1.0)
    shift = dst_lo - src_lo * scale
    out = image.astype(np.float32) * scale + shift
    return np.clip(out, 0, 255).astype(np.uint8)


class StereoExposureComp(Node):
    def __init__(self):
        super().__init__("stereo_exposure_comp")

        self.left_pub = self.create_publisher(Image, "/stereo_preproc/left/image_raw", 10)
        self.right_pub = self.create_publisher(Image, "/stereo_preproc/right/image_raw", 10)

        self.left_sub = Subscriber(self, Image, "/cam0/image_raw")
        self.right_sub = Subscriber(self, Image, "/cam1/image_raw")
        self.sync = TimeSynchronizer([self.left_sub, self.right_sub], queue_size=30)
        self.sync.registerCallback(self.callback)

        self.pairs = 0
        self.get_logger().info("Stereo exposure compensation node started.")
        self.get_logger().info("Subscribing to /cam0/image_raw and /cam1/image_raw")
        self.get_logger().info("Publishing to /stereo_preproc/left/image_raw and /stereo_preproc/right/image_raw")

    def callback(self, left_msg: Image, right_msg: Image):
        left = image_to_numpy(left_msg)
        right = image_to_numpy(right_msg)

        if left.ndim == 3:
            left_gray = left.mean(axis=2).astype(np.uint8)
            right_gray = right.mean(axis=2).astype(np.uint8)
        else:
            left_gray = left
            right_gray = right

        left_lo, left_hi = robust_range(left_gray)
        right_lo, right_hi = robust_range(right_gray)
        target_lo = 0.5 * (left_lo + right_lo)
        target_hi = 0.5 * (left_hi + right_hi)

        left_comp = affine_match(left, left_lo, left_hi, target_lo, target_hi)
        right_comp = affine_match(right, right_lo, right_hi, target_lo, target_hi)

        self.left_pub.publish(numpy_to_image(left_comp, left_msg))
        self.right_pub.publish(numpy_to_image(right_comp, right_msg))

        self.pairs += 1
        if self.pairs == 1 or self.pairs % 200 == 0:
            self.get_logger().info(
                f"Processed stereo pairs: {self.pairs} "
                f"(L[{left_lo:.1f},{left_hi:.1f}] R[{right_lo:.1f},{right_hi:.1f}] "
                f"-> target[{target_lo:.1f},{target_hi:.1f}])"
            )


def main():
    rclpy.init()
    node = StereoExposureComp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
