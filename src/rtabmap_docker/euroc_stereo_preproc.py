#!/usr/bin/env python3
from copy import deepcopy

import cv2
import message_filters
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


WIDTH = 752
HEIGHT = 480
SIZE = (WIDTH, HEIGHT)

# EuRoC MAV intrinsics: pinhole + radtan/plumb_bob
K0 = np.array([
    [458.654, 0.0, 367.215],
    [0.0, 457.296, 248.375],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

D0 = np.array([
    -0.28340811,
    0.07395907,
    0.00019359,
    0.0000176187114,
    0.0,
], dtype=np.float64)

K1 = np.array([
    [457.587, 0.0, 379.999],
    [0.0, 456.134, 255.238],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

D1 = np.array([
    -0.28368365,
    0.07451284,
    -0.00010473,
    -0.00003555907,
    0.0,
], dtype=np.float64)

# EuRoC T_BS matrices: transform from sensor/camera to body/IMU.
T_BS_CAM0 = np.array([
    [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
    [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
    [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
    [0.0, 0.0, 0.0, 1.0],
], dtype=np.float64)

T_BS_CAM1 = np.array([
    [0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556],
    [0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024],
    [-0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038],
    [0.0, 0.0, 0.0, 1.0],
], dtype=np.float64)


def image_to_numpy(msg: Image) -> np.ndarray:
    if msg.encoding not in ("mono8", "8UC1"):
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    dtype = np.uint8
    itemsize = np.dtype(dtype).itemsize
    row_elems = msg.step // itemsize
    flat = np.frombuffer(msg.data, dtype=dtype)
    return flat.reshape((msg.height, row_elems))[:, : msg.width].copy()


def numpy_to_image(arr: np.ndarray, template: Image) -> Image:
    out = Image()
    out.header = template.header
    out.height = arr.shape[0]
    out.width = arr.shape[1]
    out.encoding = template.encoding
    out.is_bigendian = template.is_bigendian
    out.step = int(arr.strides[0])
    out.data = arr.tobytes()
    return out


def make_camera_info(header, frame_id: str, k: np.ndarray, d: np.ndarray, r_rect: np.ndarray, p_rect: np.ndarray) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp = header.stamp
    msg.header.frame_id = frame_id
    msg.width = WIDTH
    msg.height = HEIGHT
    msg.distortion_model = "plumb_bob"
    msg.d = d.tolist()
    msg.k = k.reshape(-1).tolist()
    msg.r = r_rect.reshape(-1).tolist()
    msg.p = p_rect.reshape(-1).tolist()
    return msg


class EurocStereoPreprocessor(Node):
    def __init__(self) -> None:
        super().__init__("euroc_stereo_preprocessor")

        t_cam1_cam0 = np.linalg.inv(T_BS_CAM1) @ T_BS_CAM0
        r = t_cam1_cam0[:3, :3]
        t = t_cam1_cam0[:3, 3]

        r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
            K0,
            D0,
            K1,
            D1,
            SIZE,
            r,
            t,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
        )

        self.left_info_template = (K0, D0, r1, p1)
        self.right_info_template = (K1, D1, r2, p2)

        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            K0, D0, r1, p1[:, :3], SIZE, cv2.CV_32FC1
        )
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            K1, D1, r2, p2[:, :3], SIZE, cv2.CV_32FC1
        )

        self.pairs = 0
        self.last_output_stamp_ns = -1
        self.sync_slop_sec = 0.025

        # Keep a deeper reliable queue so stereo_odometry/rtabmap can absorb
        # occasional processing spikes without losing exact stamp alignment
        # between images and camera_info.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.left_rect_pub = self.create_publisher(Image, "/cam0/image_rect", output_qos)
        self.right_rect_pub = self.create_publisher(Image, "/cam1/image_rect", output_qos)
        self.left_info_pub = self.create_publisher(CameraInfo, "/cam0/camera_info", output_qos)
        self.right_info_pub = self.create_publisher(CameraInfo, "/cam1/camera_info", output_qos)

        self.left_sub = message_filters.Subscriber(
            self,
            Image,
            "/cam0/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.right_sub = message_filters.Subscriber(
            self,
            Image,
            "/cam1/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=120,
            slop=self.sync_slop_sec,
        )
        self.sync.registerCallback(self.process_pair)

        self.get_logger().info("EuRoC stereo preprocessor started.")
        self.get_logger().info(
            "Subscribing to /cam0/image_raw and /cam1/image_raw with one-to-one approximate sync."
        )
        self.get_logger().info("Publishing rectified stereo to /cam0/image_rect and /cam1/image_rect.")
        self.get_logger().info(
            f"Rectified outputs use a shared canonical stamp for exact downstream sync (slop={self.sync_slop_sec:.3f}s)."
        )

    @staticmethod
    def stamp_to_ns(msg: Image) -> int:
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    @staticmethod
    def ns_to_time(stamp_ns: int) -> Time:
        stamp = Time()
        stamp.sec = stamp_ns // 1_000_000_000
        stamp.nanosec = stamp_ns % 1_000_000_000
        return stamp

    def canonical_stamp_ns(self, left_msg: Image, right_msg: Image) -> int:
        stamp_ns = max(self.stamp_to_ns(left_msg), self.stamp_to_ns(right_msg))
        if stamp_ns <= self.last_output_stamp_ns:
            # RTAB-Map compares timestamps as floating-point seconds, so use a
            # full microsecond step when we need to force monotonicity.
            stamp_ns = self.last_output_stamp_ns + 1_000
        self.last_output_stamp_ns = stamp_ns
        return stamp_ns

    def with_canonical_header(self, msg: Image, stamp_ns: int, frame_id: str) -> Image:
        out = deepcopy(msg)
        out.header.stamp = self.ns_to_time(stamp_ns)
        out.header.frame_id = frame_id
        return out

    def process_pair(self, left_msg: Image, right_msg: Image) -> None:
        stamp_ns = self.canonical_stamp_ns(left_msg, right_msg)
        left_in = self.with_canonical_header(left_msg, stamp_ns, left_msg.header.frame_id or "cam0")
        right_in = self.with_canonical_header(right_msg, stamp_ns, right_msg.header.frame_id or "cam1")
        left = image_to_numpy(left_msg)
        right = image_to_numpy(right_msg)

        left_rect = cv2.remap(left, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, self.right_map1, self.right_map2, cv2.INTER_LINEAR)

        left_info = make_camera_info(
            left_in.header,
            left_in.header.frame_id,
            *self.left_info_template,
        )
        right_info = make_camera_info(
            right_in.header,
            right_in.header.frame_id,
            *self.right_info_template,
        )

        # Publish matching camera_info and rectified images exactly once per
        # stereo timestamp to avoid downstream duplicate synchronizer callbacks.
        self.left_info_pub.publish(left_info)
        self.right_info_pub.publish(right_info)
        self.left_rect_pub.publish(numpy_to_image(left_rect, left_in))
        self.right_rect_pub.publish(numpy_to_image(right_rect, right_in))

        self.pairs += 1
        if self.pairs == 1 or self.pairs % 200 == 0:
            stamp = stamp_ns * 1e-9
            self.get_logger().info(
                f"Published rectified stereo pairs: {self.pairs} (stamp={stamp:.9f}s)"
            )


def main() -> None:
    rclpy.init()
    node = EurocStereoPreprocessor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
