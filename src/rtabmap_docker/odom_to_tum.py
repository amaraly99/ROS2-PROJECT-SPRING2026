#!/usr/bin/env python3

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class OdomToTum(Node):
    def __init__(self, topic, output_path, discovery_timeout, msg_timeout, qos_mode, fsync_each_line):
        super().__init__("odom_to_tum")

        self.topic = topic
        self.output_path = output_path
        self.count = 0
        self.last_msg_time = None
        self.fsync_each_line = fsync_each_line

        self.get_logger().info(f"Checking if topic exists: {topic}")

        deadline = time.time() + discovery_timeout
        found_types = []

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            topics = dict(self.get_topic_names_and_types())
            if topic in topics:
                found_types = topics[topic]
                break

        if not found_types:
            self.get_logger().error(f"Topic does not exist: {topic}")
            self.print_odom_like_topics()
            raise RuntimeError(f"Required topic missing: {topic}")

        if "nav_msgs/msg/Odometry" not in found_types:
            self.get_logger().error(f"Topic exists but has wrong type: {topic}")
            self.get_logger().error(f"Found types: {found_types}")
            raise RuntimeError(f"Topic {topic} is not nav_msgs/msg/Odometry")

        if qos_mode == "best_effort":
            reliability = ReliabilityPolicy.BEST_EFFORT
        elif qos_mode == "reliable":
            reliability = ReliabilityPolicy.RELIABLE
        else:
            raise RuntimeError(f"Unknown QoS mode: {qos_mode}")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Line-buffered file, plus explicit flush/fsync in callback.
        self.file = open(output_path, "w", buffering=1)

        self.sub = self.create_subscription(
            Odometry,
            topic,
            self.callback,
            qos
        )

        self.get_logger().info(f"Recording odometry topic: {topic}")
        self.get_logger().info(f"QoS reliability: {qos_mode}")
        self.get_logger().info(f"Writing TUM trajectory to: {output_path}")
        self.get_logger().info(f"Flush every line: True")
        self.get_logger().info(f"fsync every line: {fsync_each_line}")

        # Die if topic exists but no compatible messages arrive.
        self.deadline_timer = self.create_timer(
            msg_timeout,
            self.check_first_message_timeout
        )

    def print_odom_like_topics(self):
        self.get_logger().error("Available odom/pose/path-like topics:")
        for name, types in self.get_topic_names_and_types():
            lowered = name.lower()
            if "odom" in lowered or "odo" in lowered or "pose" in lowered or "path" in lowered:
                self.get_logger().error(f"  {name}: {types}")

    def check_first_message_timeout(self):
        if self.count == 0:
            self.get_logger().error(f"No Odometry messages received from {self.topic}.")
            self.get_logger().error("This is likely QoS mismatch or RTAB-Map is not publishing yet.")
            self.get_logger().error("Try --qos best_effort or verify with:")
            self.get_logger().error(f"  ros2 topic info {self.topic} --verbose")
            raise RuntimeError(f"No compatible messages received from {self.topic}")

        # Once first message arrived, stop using this watchdog.
        self.deadline_timer.cancel()

    def callback(self, msg):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.file.write(
            f"{t:.9f} "
            f"{p.x:.9f} {p.y:.9f} {p.z:.9f} "
            f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
        )

        self.file.flush()

        if self.fsync_each_line:
            os.fsync(self.file.fileno())

        self.count += 1
        self.get_logger().info(f"Recorded pose #{self.count}")

    def destroy_node(self):
        if hasattr(self, "file") and not self.file.closed:
            self.file.flush()
            if self.fsync_each_line:
                os.fsync(self.file.fileno())
            self.file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/rtabmap/odo")
    parser.add_argument("--out", default="/workspace/rtabmap_trajectory.tum")
    parser.add_argument("--discovery-timeout", type=float, default=5.0)
    parser.add_argument("--msg-timeout", type=float, default=5.0)
    parser.add_argument("--qos", choices=["best_effort", "reliable"], default="best_effort")
    parser.add_argument("--no-fsync", action="store_true")
    args = parser.parse_args()

    rclpy.init()

    node = None
    try:
        node = OdomToTum(
            topic=args.topic,
            output_path=args.out,
            discovery_timeout=args.discovery_timeout,
            msg_timeout=args.msg_timeout,
            qos_mode=args.qos,
            fsync_each_line=not args.no_fsync,
        )
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"[FATAL] {e}", file=sys.stderr)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    if node is not None:
        node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
