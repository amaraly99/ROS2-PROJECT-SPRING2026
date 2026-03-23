#!/usr/bin/env python3
"""
euroc_publisher.py — Publish EuRoC MAV dataset images to ROS2 topics.

Reads images from the ASL dataset format and publishes them as
sensor_msgs/Image on /cam0/image_raw at the original timestamps.

Usage:
    ros2 run --prefix 'python3' euroc_publisher.py /path/to/MH_01_easy [--rate 1.0]

    Or directly:
    python3 euroc_publisher.py /path/to/MH_01_easy --rate 1.0
"""

import argparse
import csv
import os
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class EurocPublisher(Node):
    def __init__(self, dataset_path: str, rate: float = 1.0):
        super().__init__('euroc_publisher')

        self.pub_left_ = self.create_publisher(Image, '/cam0/image_raw', 10)
        self.bridge_ = CvBridge()
        self.rate_ = rate

        # Load image list from EuRoC ASL format.
        # Supports standard mav0/ wrapper and flat layout (cam0/ at root).
        if os.path.isdir(os.path.join(dataset_path, 'mav0', 'cam0')):
            cam0_dir = os.path.join(dataset_path, 'mav0', 'cam0')
        else:
            cam0_dir = os.path.join(dataset_path, 'cam0')
        csv_path = os.path.join(cam0_dir, 'data.csv')
        img_dir = os.path.join(cam0_dir, 'data')

        if not os.path.isfile(csv_path):
            self.get_logger().fatal(f'data.csv not found: {csv_path}')
            sys.exit(1)

        self.images_ = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                ts_ns = int(row[0])
                img_file = os.path.join(img_dir, row[1].strip())
                if os.path.isfile(img_file):
                    self.images_.append((ts_ns, img_file))

        self.get_logger().info(
            f'Loaded {len(self.images_)} images from {dataset_path} (rate={rate}x)')

    def run(self):
        if not self.images_:
            self.get_logger().error('No images loaded')
            return

        t0_dataset = self.images_[0][0]  # first timestamp (ns)
        t0_wall = time.monotonic()
        published = 0

        for ts_ns, img_path in self.images_:
            if not rclpy.ok():
                break

            # Compute when this frame should be published
            dt_dataset = (ts_ns - t0_dataset) * 1e-9  # seconds
            target_wall = t0_wall + dt_dataset / self.rate_

            # Wait until it's time
            now = time.monotonic()
            if target_wall > now:
                time.sleep(target_wall - now)

            # Read and publish
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue

            msg = self.bridge_.cv2_to_imgmsg(img, encoding='mono8')
            msg.header.stamp.sec = int(ts_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(ts_ns % 1_000_000_000)
            msg.header.frame_id = 'cam0'

            self.pub_left_.publish(msg)
            published += 1

            if published % 200 == 0:
                elapsed = time.monotonic() - t0_wall
                self.get_logger().info(
                    f'Published {published}/{len(self.images_)} '
                    f'({elapsed:.1f}s elapsed)')

        elapsed = time.monotonic() - t0_wall
        fps = published / elapsed if elapsed > 0 else 0
        self.get_logger().info(
            f'Done: {published} images in {elapsed:.1f}s ({fps:.1f} fps)')


def main():
    parser = argparse.ArgumentParser(description='EuRoC dataset publisher')
    parser.add_argument('dataset_path', help='Path to EuRoC sequence (e.g., MH_01_easy/)')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='Playback rate multiplier (default: 1.0)')
    args = parser.parse_args()

    rclpy.init()
    node = EurocPublisher(args.dataset_path, args.rate)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
