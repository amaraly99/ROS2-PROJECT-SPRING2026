#!/usr/bin/env python3

"""
Python node for the StereoMode cpp node.

Requirements
* Dataset must be configured in EuRoC MAV format
* The selected sequence must contain mav0/cam0 and mav0/cam1 folders
"""

import os
import time
from pathlib import Path

import cv2
import natsort
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String


class StereoDriver(Node):
    def __init__(self, node_name="stereo_py_node"):
        super().__init__(node_name)

        self.declare_parameter("settings_name", "EuRoC")
        self.declare_parameter("image_seq", "NULL")

        self.settings_name = str(self.get_parameter("settings_name").value)
        self.image_seq = str(self.get_parameter("image_seq").value)

        self.home_dir = str(Path(__file__).resolve().parent.parent)
        self.parent_dir = "TEST_DATASET"
        self.image_sequence_dir = f"{self.home_dir}/{self.parent_dir}/{self.image_seq}"

        self.br = CvBridge()
        (
            self.left_images_dir,
            self.right_images_dir,
            self.image_names,
        ) = self.get_image_dataset_asl(self.image_sequence_dir, "mav0")

        self.pub_exp_config_name = "/stereo_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/stereo_py_driver/exp_settings_ack"
        self.pub_left_img_name = "/stereo_py_driver/left_img_msg"
        self.pub_right_img_name = "/stereo_py_driver/right_img_msg"
        self.pub_timestep_name = "/stereo_py_driver/timestep_msg"
        self.pub_shutdown_name = "/stereo_py_driver/shutdown"
        self.send_config = True

        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.publish_left_img_ = self.create_publisher(Image, self.pub_left_img_name, 1)
        self.publish_right_img_ = self.create_publisher(Image, self.pub_right_img_name, 1)
        self.publish_timestep_ = self.create_publisher(Float64, self.pub_timestep_name, 1)
        self.publish_shutdown_ = self.create_publisher(Bool, self.pub_shutdown_name, 1)

        self.subscribe_exp_ack_ = self.create_subscription(
            String, self.sub_exp_ack_name, self.ack_callback, 10
        )

        self.exp_config_msg = self.settings_name
        self.frame_id = 0

        print("-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print(f"self.image_sequence_dir: {self.image_sequence_dir}\n")
        print(f"StereoDriver initialized with {len(self.image_names)} stereo pairs")

    def get_image_dataset_asl(self, exp_dir, agent_name="mav0"):
        left_cam_dir = f"{exp_dir}/{agent_name}/cam0/data"
        right_cam_dir = f"{exp_dir}/{agent_name}/cam1/data"

        left_images = set(os.listdir(left_cam_dir))
        right_images = set(os.listdir(right_cam_dir))
        common_images = natsort.natsorted(left_images.intersection(right_images), reverse=False)

        return left_cam_dir + "/", right_cam_dir + "/", common_images

    def ack_callback(self, msg):
        print(f"Got ack: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False

    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def publish_shutdown_signal(self):
        msg = Bool()
        msg.data = True

        for _ in range(3):
            self.publish_shutdown_.publish(msg)
            time.sleep(0.05)

    def publish_pair(self, image_name):
        stamp_ns = int(image_name.split(".")[0])
        timestep_seconds = stamp_ns / 1e9

        left_path = self.left_images_dir + image_name
        right_path = self.right_images_dir + image_name

        left_msg = self.br.cv2_to_imgmsg(cv2.imread(left_path), encoding="passthrough")
        right_msg = self.br.cv2_to_imgmsg(cv2.imread(right_path), encoding="passthrough")

        for msg in (left_msg, right_msg):
            msg.header.stamp.sec = stamp_ns // 1_000_000_000
            msg.header.stamp.nanosec = stamp_ns % 1_000_000_000

        timestep_msg = Float64()
        timestep_msg.data = timestep_seconds

        try:
            self.publish_timestep_.publish(timestep_msg)
            self.publish_left_img_.publish(left_msg)
            self.publish_right_img_.publish(right_msg)
        except CvBridgeError as exc:
            print(exc)

        self.frame_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = StereoDriver("stereo_py_node")
    rate = node.create_rate(20)

    while node.send_config:
        node.handshake_with_cpp_node()
        rclpy.spin_once(node)

    print("Handshake complete")

    for image_name in node.image_names:
        try:
            rclpy.spin_once(node)
            node.publish_pair(image_name)
            rate.sleep()
        except KeyboardInterrupt:
            break

    node.publish_shutdown_signal()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
