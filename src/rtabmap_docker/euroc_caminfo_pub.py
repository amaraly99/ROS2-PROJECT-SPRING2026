#!/usr/bin/env python3

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo


WIDTH = 752
HEIGHT = 480
SIZE = (WIDTH, HEIGHT)

# EuRoC MAV intrinsics: pinhole + radtan/plumb_bob
K0 = np.array([
    [458.654, 0.0, 367.215],
    [0.0, 457.296, 248.375],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

D0 = np.array([
    -0.28340811,
     0.07395907,
     0.00019359,
     0.0000176187114,
     0.0
], dtype=np.float64)

K1 = np.array([
    [457.587, 0.0, 379.999],
    [0.0, 456.134, 255.238],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

D1 = np.array([
    -0.28368365,
     0.07451284,
    -0.00010473,
    -0.00003555907,
     0.0
], dtype=np.float64)

# EuRoC T_BS matrices: transform from sensor/camera to body/IMU.
T_BS_CAM0 = np.array([
    [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
    [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
    [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float64)

T_BS_CAM1 = np.array([
    [0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556],
    [0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024],
    [-0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float64)

# Relative transform from cam0 to cam1.
T_CAM1_CAM0 = np.linalg.inv(T_BS_CAM1) @ T_BS_CAM0
R = T_CAM1_CAM0[:3, :3]
T = T_CAM1_CAM0[:3, 3]

# Compute rectification matrices and projection matrices.
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
    K0, D0,
    K1, D1,
    SIZE,
    R, T,
    flags=cv2.CALIB_ZERO_DISPARITY,
    alpha=0
)


def make_camera_info(header, frame_id, K, D, R_rect, P_rect):
    msg = CameraInfo()

    # Critical: use the image timestamp so image_proc can synchronize.
    msg.header.stamp = header.stamp
    msg.header.frame_id = frame_id

    msg.width = WIDTH
    msg.height = HEIGHT

    msg.distortion_model = "plumb_bob"
    msg.d = D.tolist()

    # Original camera matrix.
    msg.k = K.reshape(-1).tolist()

    # Rectification matrix.
    msg.r = R_rect.reshape(-1).tolist()

    # Projection matrix after rectification.
    msg.p = P_rect.reshape(-1).tolist()

    return msg


class EurocCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("euroc_camera_info_publisher")

        # IMPORTANT:
        # Use default reliable QoS for CameraInfo.
        # image_proc rectify_node often expects reliable camera_info.
        self.pub0 = self.create_publisher(CameraInfo, "/cam0/camera_info", 10)
        self.pub1 = self.create_publisher(CameraInfo, "/cam1/camera_info", 10)

        # Images from rosbag are sensor data, so subscribe with sensor QoS.
        self.sub0 = self.create_subscription(
            Image,
            "/cam0/image_raw",
            self.cam0_callback,
            qos_profile_sensor_data
        )

        self.sub1 = self.create_subscription(
            Image,
            "/cam1/image_raw",
            self.cam1_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("EuRoC CameraInfo publisher started.")
        self.get_logger().info("Publishing /cam0/camera_info and /cam1/camera_info.")
        self.get_logger().info("CameraInfo timestamps are copied from incoming image headers.")

    def cam0_callback(self, img_msg):
        cam_info = make_camera_info(
            img_msg.header,
            "cam0",
            K0,
            D0,
            R1,
            P1
        )
        self.pub0.publish(cam_info)

    def cam1_callback(self, img_msg):
        cam_info = make_camera_info(
            img_msg.header,
            "cam1",
            K1,
            D1,
            R2,
            P2
        )
        self.pub1.publish(cam_info)


def main():
    rclpy.init()
    node = EurocCameraInfoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
