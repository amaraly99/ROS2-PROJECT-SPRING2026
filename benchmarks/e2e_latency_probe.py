#!/usr/bin/env python3
"""
e2e_latency_probe.py — End-to-end latency measurement probe

Subscribes to /yolo/detections and /cmd_vel.
Approximates detection-to-actuation latency by pairing consecutive messages.

Usage (inside Docker, after sourcing):
    python3 /workspace/benchmarks/e2e_latency_probe.py

Output: CSV file  benchmarks/results/e2e_latency_<config>.csv
        with columns: t_det_stamp_ns, t_det_recv_ns, t_cmd_recv_ns, approx_e2e_ms

Note on clock domains:
  - /yolo/detections header.stamp = camera capture time (CLOCK_MONOTONIC from ovcam_producer)
  - t_det_recv / t_cmd_recv = ROS2 wall clock at message receipt (same node, same clock)
  - approx_e2e_ms = t_cmd_recv - t_det_recv (queue-to-queue, excludes YOLO inference)
  - For full camera→cmd_vel, add YOLO inference time (~25ms NPU / varies on CPU)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sys
import os
import csv
import time
import argparse

from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import Twist


class E2ELatencyProbe(Node):
    def __init__(self, output_path: str):
        super().__init__("e2e_latency_probe")

        self.output_path = output_path
        self.rows: list = []

        self._last_det_stamp_ns: int = 0
        self._last_det_recv_ns: int = 0

        qos_be = QoSProfile(depth=10,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.RELIABLE)

        self.sub_dets = self.create_subscription(
            DetectionArray, "/yolo/detections",
            self._on_det, qos_be)

        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel",
            self._on_cmd, qos_rel)

        self.get_logger().info(
            f"e2e_latency_probe started — logging to {output_path}")

    def _on_det(self, msg: DetectionArray):
        t_recv = self.get_clock().now().nanoseconds
        self._last_det_stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._last_det_recv_ns = t_recv

    def _on_cmd(self, msg: Twist):
        if self._last_det_recv_ns == 0:
            return
        t_recv = self.get_clock().now().nanoseconds
        approx_e2e_ms = (t_recv - self._last_det_recv_ns) / 1e6
        self.rows.append({
            "t_det_stamp_ns": self._last_det_stamp_ns,
            "t_det_recv_ns":  self._last_det_recv_ns,
            "t_cmd_recv_ns":  t_recv,
            "approx_e2e_ms":  round(approx_e2e_ms, 3),
        })
        if len(self.rows) % 50 == 0:
            self.get_logger().info(
                f"[{len(self.rows)} samples] "
                f"last approx E2E: {approx_e2e_ms:.1f} ms  "
                f"(det→cmd_vel queue latency)")

    def save(self):
        if not self.rows:
            self.get_logger().warn("No samples collected — nothing to save.")
            return
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.rows[0].keys())
            writer.writeheader()
            writer.writerows(self.rows)

        # Print summary
        e2e_vals = [r["approx_e2e_ms"] for r in self.rows]
        e2e_vals.sort()
        n = len(e2e_vals)
        p50 = e2e_vals[n // 2]
        p95 = e2e_vals[int(n * 0.95)]
        mean = sum(e2e_vals) / n
        self.get_logger().info(
            f"Saved {n} samples → {self.output_path}\n"
            f"  Approx det→cmd_vel (queue latency):  "
            f"mean={mean:.1f}ms  P50={p50:.1f}ms  P95={p95:.1f}ms\n"
            f"  Note: add YOLO inference time for full camera→cmd_vel E2E")


def main():
    parser = argparse.ArgumentParser(description="E2E latency probe")
    parser.add_argument("--config", default="default",
                        help="Configuration label for output filename (e.g. 'yolo_npu', 'yolo_cpu_n')")
    parser.add_argument("--duration", type=int, default=60,
                        help="Measurement duration in seconds (default: 60)")
    args = parser.parse_args()

    output_path = os.path.join(
        os.path.dirname(__file__), "results", "e2e",
        f"e2e_latency_{args.config}.csv")

    rclpy.init()
    node = E2ELatencyProbe(output_path)

    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
