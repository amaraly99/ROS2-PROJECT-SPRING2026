#!/usr/bin/env python3

import signal
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


IMAGE_WIDTH = 752
IMAGE_HEIGHT = 480


def _camera_info_msg(frame_id: str, k_rect: np.ndarray, p_rect: np.ndarray) -> CameraInfo:
    msg = CameraInfo()
    msg.width = IMAGE_WIDTH
    msg.height = IMAGE_HEIGHT
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = k_rect.reshape(-1).tolist()
    msg.r = np.eye(3, dtype=np.float64).reshape(-1).tolist()
    msg.p = p_rect.reshape(-1).tolist()
    msg.header.frame_id = frame_id
    return msg


class EuRoCRectifier(Node):
    def __init__(self) -> None:
        super().__init__("euroc_rectifier")

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._bridge = CvBridge()

        self._left_pub = self.create_publisher(Image, "/camera/left", output_qos)
        self._right_pub = self.create_publisher(Image, "/camera/right", output_qos)
        self._left_info_pub = self.create_publisher(CameraInfo, "/camera/left/camera_info", output_qos)
        self._right_info_pub = self.create_publisher(CameraInfo, "/camera/right/camera_info", output_qos)

        self._left_maps = self._build_maps(
            k=np.array([[458.654, 0.0, 367.215], [0.0, 457.296, 248.375], [0.0, 0.0, 1.0]], dtype=np.float64),
            d=np.array([-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0], dtype=np.float64),
            r=np.array(
                [
                    [0.999966347530033, -0.001422739138722922, 0.008079580483432283],
                    [0.001365741834644127, 0.9999741760894847, 0.007055629199258132],
                    [-0.008089410156878961, -0.007044357138835809, 0.9999424675829176],
                ],
                dtype=np.float64,
            ),
            p=np.array(
                [
                    [435.2046959714599, 0.0, 367.4517211914062, 0.0],
                    [0.0, 435.2046959714599, 252.2008514404297, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                ],
                dtype=np.float64,
            ),
        )

        self._right_maps = self._build_maps(
            k=np.array([[457.587, 0.0, 379.999], [0.0, 456.134, 255.238], [0.0, 0.0, 1.0]], dtype=np.float64),
            d=np.array([-0.28368365, 0.07451284, -0.00010473, -3.555907e-05, 0.0], dtype=np.float64),
            r=np.array(
                [
                    [0.9999633526194376, -0.003625811871560086, 0.007755443660172947],
                    [0.003680398547259526, 0.9999684752771629, -0.007035845251224894],
                    [-0.007729688520722713, 0.007064130529506649, 0.999945173484644],
                ],
                dtype=np.float64,
            ),
            p=np.array(
                [
                    [435.2046959714599, 0.0, 367.4517211914062, -47.90639384423901],
                    [0.0, 435.2046959714599, 252.2008514404297, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                ],
                dtype=np.float64,
            ),
        )

        self._left_info = _camera_info_msg(
            "cam0_rect",
            self._left_maps["p"][:, :3],
            self._left_maps["p"],
        )
        self._right_info = _camera_info_msg(
            "cam1_rect",
            self._right_maps["p"][:, :3],
            self._right_maps["p"],
        )

        self.create_subscription(Image, "/cam0/image_raw", self._handle_left, input_qos)
        self.create_subscription(Image, "/cam1/image_raw", self._handle_right, input_qos)

        self.get_logger().info("EuRoC rectifier ready: /cam0/image_raw -> /camera/left, /cam1/image_raw -> /camera/right")

    @staticmethod
    def _build_maps(k: np.ndarray, d: np.ndarray, r: np.ndarray, p: np.ndarray) -> dict[str, np.ndarray]:
        map1, map2 = cv2.initUndistortRectifyMap(
            cameraMatrix=k,
            distCoeffs=d,
            R=r,
            newCameraMatrix=p[:, :3],
            size=(IMAGE_WIDTH, IMAGE_HEIGHT),
            m1type=cv2.CV_32FC1,
        )
        return {"map1": map1, "map2": map2, "p": p}

    def _rectify(self, msg: Image, maps: dict[str, np.ndarray], frame_id: str) -> Image:
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        rectified = cv2.remap(cv_image, maps["map1"], maps["map2"], interpolation=cv2.INTER_LINEAR)
        out = self._bridge.cv2_to_imgmsg(rectified, encoding="mono8")
        out.header = msg.header
        out.header.frame_id = frame_id
        return out

    def _handle_left(self, msg: Image) -> None:
        out = self._rectify(msg, self._left_maps, "cam0_rect")
        try:
            self._left_pub.publish(out)
        except Exception:
            if rclpy.ok():
                raise
            return

        info = CameraInfo()
        info.header = out.header
        info.width = self._left_info.width
        info.height = self._left_info.height
        info.distortion_model = self._left_info.distortion_model
        info.d = list(self._left_info.d)
        info.k = list(self._left_info.k)
        info.r = list(self._left_info.r)
        info.p = list(self._left_info.p)
        try:
            self._left_info_pub.publish(info)
        except Exception:
            if rclpy.ok():
                raise

    def _handle_right(self, msg: Image) -> None:
        out = self._rectify(msg, self._right_maps, "cam1_rect")
        try:
            self._right_pub.publish(out)
        except Exception:
            if rclpy.ok():
                raise
            return

        info = CameraInfo()
        info.header = out.header
        info.width = self._right_info.width
        info.height = self._right_info.height
        info.distortion_model = self._right_info.distortion_model
        info.d = list(self._right_info.d)
        info.k = list(self._right_info.k)
        info.r = list(self._right_info.r)
        info.p = list(self._right_info.p)
        try:
            self._right_info_pub.publish(info)
        except Exception:
            if rclpy.ok():
                raise


def main() -> int:
    rclpy.init(args=None)
    node = EuRoCRectifier()
    shutdown_requested = False

    def _stop(signum, frame) -> None:  # type: ignore[unused-argument]
        nonlocal shutdown_requested
        if shutdown_requested:
            return
        shutdown_requested = True
        node.get_logger().info(f"Received signal {signum}, shutting down rectifier")
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
