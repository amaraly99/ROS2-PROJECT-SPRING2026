#!/usr/bin/env python3
"""
e2e_latency_probe.py — End-to-end latency measurement probe

Subscribes to /yolo/detections and /cmd_vel (always), plus /ovcam/image_raw and
/sim/camera/image_raw (unless --no-camera) to capture camera delivery rates and
the camera→detection pipeline latency.

Usage (inside Docker, after sourcing):
    python3 /workspace/benchmarks/e2e_latency_probe.py
    python3 /workspace/benchmarks/e2e_latency_probe.py --config yolo26n_npu \
        --duration 60 --output /tmp/probe.csv

Output: CSV file  benchmarks/results/e2e/e2e_latency_<config>.csv (or --output)
        Columns 1-4 are UNCHANGED from the original probe (index-parsing safe):
          t_det_stamp_ns, t_det_recv_ns, t_cmd_recv_ns, approx_e2e_ms
        New columns are APPENDED:
          t_cam_stamp_ns   matched /ovcam/image_raw header.stamp (0 = no match)
          t_cam_recv_ns    wall clock when that camera frame arrived (0 = no match)
          cam_to_det_ms    camera SHM write → detection received at this probe
        Side outputs (camera enabled): <output>_camera.csv (per-frame arrivals)
        and <output>_stats.json (machine-readable summary for automation).

Note on clock domains:
  - /yolo/detections header.stamp = the camera frame's SHM-write time in
    CLOCK_MONOTONIC ns (stamped by sim_camera_bridge/ovcam_producer and carried
    through the detector unchanged). /ovcam/image_raw header.stamp is the same
    value for the same frame.
  - This probe runs on the same kernel, so it converts that monotonic stamp to
    wall clock using its own (REALTIME - MONOTONIC) offset, sampled per callback.
    cam_to_det_ms therefore covers: detector SHM wait + preprocess + inference +
    postprocess + SHM write + yolo_bridge publish + DDS hop to this probe.
  - /sim/camera/image_raw header.stamp is MATLAB's clock (different host):
    used for arrival RATE/jitter only, never subtracted against Pi stamps.
  - approx_e2e_ms (unchanged): det receipt → next /cmd_vel receipt, same node.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sys
import os
import csv
import json
import time
import argparse
from collections import deque

from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# A detection should match a camera frame with the identical stamp; allow a
# small tolerance for safety (stamps are copied verbatim, so this is generous).
CAM_MATCH_TOL_NS = 1_000_000        # 1 ms
CAM_MATCH_WARN_NS = 200_000_000     # warn if nearest preceding frame > 200 ms


def _mono_to_wall_offset_ns() -> int:
    """Current (CLOCK_REALTIME - CLOCK_MONOTONIC) in ns."""
    return time.time_ns() - time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def _pctl(sorted_vals, q):
    if not sorted_vals:
        return 0.0
    return sorted_vals[min(len(sorted_vals) - 1, int(len(sorted_vals) * q))]


def _rate_stats(recv_ns: list) -> dict:
    """Mean/std of arrival rate from a list of receive times (ns)."""
    if len(recv_ns) < 3:
        return {"hz_mean": 0.0, "hz_std": 0.0, "n": len(recv_ns)}
    span_s = (recv_ns[-1] - recv_ns[0]) / 1e9
    hz_mean = (len(recv_ns) - 1) / span_s if span_s > 0 else 0.0
    inst = [1e9 / (b - a) for a, b in zip(recv_ns, recv_ns[1:]) if b > a]
    m = sum(inst) / len(inst)
    var = sum((x - m) ** 2 for x in inst) / len(inst)
    return {"hz_mean": round(hz_mean, 3), "hz_std": round(var ** 0.5, 3),
            "n": len(recv_ns)}


class E2ELatencyProbe(Node):
    def __init__(self, output_path: str, camera: bool = True):
        super().__init__("e2e_latency_probe")

        self.output_path = output_path
        self.camera_enabled = camera
        self.rows: list = []

        self._last_det_stamp_ns: int = 0
        self._last_det_recv_ns: int = 0
        self._last_cam_to_det_ms: float = 0.0
        self._last_cam_match = (0, 0)            # (stamp_ns, recv_ns)

        # per-topic arrival logs (wall-clock recv ns) for rate stats
        self._det_recv: list = []
        self._cmd_recv: list = []
        self._ovcam_recv: list = []
        self._simcam_recv: list = []
        self._cam_to_det: list = []

        # recent /ovcam/image_raw frames for stamp matching
        self._cam_frames: deque = deque(maxlen=256)
        self._cam_csv_rows: list = []

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

        if camera:
            # /ovcam/image_raw: SHM→ROS2 bridge output (Pi monotonic stamps).
            # Only flows when ovcam_bridge runs (slam or debug_image on).
            self.sub_ovcam = self.create_subscription(
                Image, "/ovcam/image_raw",
                self._on_ovcam, qos_be)
            # /sim/camera/image_raw: MATLAB→Pi delivery (rate only, MATLAB clock).
            self.sub_simcam = self.create_subscription(
                Image, "/sim/camera/image_raw",
                self._on_simcam, qos_be)

        self.get_logger().info(
            f"e2e_latency_probe started — logging to {output_path} "
            f"(camera={'on' if camera else 'off'})")

    def _on_det(self, msg: DetectionArray):
        t_recv = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._last_det_stamp_ns = stamp_ns
        self._last_det_recv_ns = t_recv
        self._det_recv.append(t_recv)

        # camera SHM write (monotonic) → detection received here (wall),
        # via this host's monotonic↔wall offset.
        cam_to_det_ms = (t_recv - (stamp_ns + _mono_to_wall_offset_ns())) / 1e6
        self._last_cam_to_det_ms = cam_to_det_ms
        self._cam_to_det.append(cam_to_det_ms)

        # cross-check: match the /ovcam/image_raw frame with the same stamp
        self._last_cam_match = (0, 0)
        if self.camera_enabled and self._cam_frames:
            best = None
            for c_stamp, c_recv in reversed(self._cam_frames):
                if c_stamp <= stamp_ns + CAM_MATCH_TOL_NS:
                    best = (c_stamp, c_recv)
                    break
            if best is not None:
                gap = stamp_ns - best[0]
                if abs(gap) <= CAM_MATCH_TOL_NS:
                    self._last_cam_match = best
                elif gap > CAM_MATCH_WARN_NS:
                    self.get_logger().warning(
                        f"camera/detection stamp gap {gap/1e6:.0f} ms "
                        f"(> {CAM_MATCH_WARN_NS/1e6:.0f} ms) — frames dropping?",
                        throttle_duration_sec=5.0)

    def _on_cmd(self, msg: Twist):
        if self._last_det_recv_ns == 0:
            return
        t_recv = self.get_clock().now().nanoseconds
        self._cmd_recv.append(t_recv)
        approx_e2e_ms = (t_recv - self._last_det_recv_ns) / 1e6
        cam_stamp, cam_recv = self._last_cam_match
        self.rows.append({
            "t_det_stamp_ns": self._last_det_stamp_ns,
            "t_det_recv_ns":  self._last_det_recv_ns,
            "t_cmd_recv_ns":  t_recv,
            "approx_e2e_ms":  round(approx_e2e_ms, 3),
            "t_cam_stamp_ns": cam_stamp,
            "t_cam_recv_ns":  cam_recv,
            "cam_to_det_ms":  round(self._last_cam_to_det_ms, 3),
        })
        if len(self.rows) % 50 == 0:
            self.get_logger().info(
                f"[{len(self.rows)} samples] "
                f"last approx E2E: {approx_e2e_ms:.1f} ms (det→cmd_vel)  "
                f"cam→det: {self._last_cam_to_det_ms:.1f} ms")

    def _on_ovcam(self, msg: Image):
        t_recv = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._ovcam_recv.append(t_recv)
        self._cam_frames.append((stamp_ns, t_recv))
        self._cam_csv_rows.append(("ovcam", stamp_ns, t_recv))

    def _on_simcam(self, msg: Image):
        t_recv = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self._simcam_recv.append(t_recv)
        self._cam_csv_rows.append(("simcam", stamp_ns, t_recv))

    # ── output ───────────────────────────────────────────────────────────────
    def _base(self) -> str:
        return os.path.splitext(self.output_path)[0]

    def save(self):
        stats = self._compute_stats()

        if not self.rows:
            self.get_logger().warn("No det+cmd samples collected.")
        else:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            with open(self.output_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.rows[0].keys())
                writer.writeheader()
                writer.writerows(self.rows)

        if self.camera_enabled and self._cam_csv_rows:
            cam_path = self._base() + "_camera.csv"
            with open(cam_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["topic", "t_stamp_ns", "t_recv_ns"])
                w.writerows(self._cam_csv_rows)

        stats_path = self._base() + "_stats.json"
        os.makedirs(os.path.dirname(stats_path), exist_ok=True)
        with open(stats_path, "w") as f:
            json.dump(stats, f, indent=2)

        self.get_logger().info(
            f"Saved {len(self.rows)} samples → {self.output_path}\n"
            f"  det_hz={stats['det']['hz_mean']}  cmd_hz={stats['cmd_vel']['hz_mean']}  "
            f"ovcam_hz={stats['ovcam']['hz_mean']}  simcam_hz={stats['sim_camera']['hz_mean']}\n"
            f"  det→cmd_vel (queue):  mean={stats['e2e_det_to_cmd_ms']['mean']}ms  "
            f"P50={stats['e2e_det_to_cmd_ms']['p50']}ms  P95={stats['e2e_det_to_cmd_ms']['p95']}ms\n"
            f"  cam→det (pipeline):   mean={stats['cam_to_det_ms']['mean']}ms  "
            f"P50={stats['cam_to_det_ms']['p50']}ms  P95={stats['cam_to_det_ms']['p95']}ms\n"
            f"  stats JSON → {stats_path}")

    def _compute_stats(self) -> dict:
        e2e = sorted(r["approx_e2e_ms"] for r in self.rows)
        c2d = sorted(self._cam_to_det)

        def dist(vals):
            if not vals:
                return {"mean": 0.0, "p50": 0.0, "p95": 0.0, "n": 0}
            return {"mean": round(sum(vals) / len(vals), 3),
                    "p50": round(_pctl(vals, 0.50), 3),
                    "p95": round(_pctl(vals, 0.95), 3),
                    "n": len(vals)}

        return {
            "det": _rate_stats(self._det_recv),
            "cmd_vel": _rate_stats(self._cmd_recv),
            "ovcam": _rate_stats(self._ovcam_recv),
            "sim_camera": _rate_stats(self._simcam_recv),
            "e2e_det_to_cmd_ms": dist(e2e),
            "cam_to_det_ms": dist(c2d),
        }


def main():
    parser = argparse.ArgumentParser(description="E2E latency probe")
    parser.add_argument("--config", default="default",
                        help="Configuration label for output filename (e.g. 'yolo_npu', 'yolo_cpu_n')")
    parser.add_argument("--duration", type=int, default=60,
                        help="Measurement duration in seconds (default: 60)")
    parser.add_argument("--output", default=None,
                        help="Explicit output CSV path (overrides --config naming)")
    parser.add_argument("--no-camera", dest="no_camera", action="store_true",
                        help="Skip camera-topic subscriptions (original behavior)")
    args = parser.parse_args()

    output_path = args.output or os.path.join(
        os.path.dirname(__file__), "results", "e2e",
        f"e2e_latency_{args.config}.csv")

    rclpy.init()
    node = E2ELatencyProbe(output_path, camera=not args.no_camera)

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
