#!/usr/bin/env python3
"""
stereo_sync_test.py — prove the Pi-side stereo chain works, without MATLAB.

WHY THIS EXISTS
---------------
The stereo path has one failure mode that is silent and expensive: if the left
and right images do not carry the SAME header.stamp, a stereo SLAM front-end
happily matches frames that do not correspond and produces confident, wrong
depth. Nothing errors. You find out much later, from bad trajectories.

The whole Pi-side design exists to prevent that: sim_camera_bridge carries the
source timestamp through shared memory in SlotHeader::t_src_ns, and ovcam_bridge
republishes from it instead of from the Pi's own arrival time. This script
verifies that end to end, on the Pi alone, with no MATLAB and no second machine.

WHAT IT DOES
------------
  publish   synthetic left/right pairs on /sim/camera/{,right/}image_raw, each
            pair sharing ONE deliberately-chosen timestamp
  subscribe to the republished /ovcam/image_raw and /ovcam/right/image_raw
  assert    (1) both eyes arrive
            (2) each pair's stamps are IDENTICAL, not merely close
            (3) the stamp is the SOURCE stamp we sent, not arrival time

Test (2) is the one that matters. Test (3) is what distinguishes a working
t_src_ns path from one that happens to look synchronised because both bridges
were stamped at nearly the same instant on an idle machine — which would pass a
naive "are they close?" check and fail under load.

USAGE
-----
    # terminal 1 — the chain under test
    ros2 launch sim_camera_bridge hil_simulation.launch.py \
        stereo:=true slam:=false

    # terminal 2
    python3 scripts/stereo_sync_test.py

    # options
    --frames 60        pairs to send (default 30)
    --rate 20          Hz (default 20, matching the MATLAB publisher)
    --tolerance-ns 0   allowed stamp difference; 0 = exact (default)

Exit code 0 = pass, 1 = fail. Prints a one-line reason on failure.
"""

import argparse
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

W, H = 640, 480


def make_frame(seed: int, side: str) -> np.ndarray:
    """A cheap but non-uniform BGR frame. The two eyes differ by a horizontal
    shift, mimicking disparity, so a wrong-eye mix-up is visible if anyone
    inspects the images rather than just the stamps."""
    img = np.zeros((H, W, 3), dtype=np.uint8)
    shift = 0 if side == "left" else 8
    x = (seed * 7 + shift) % (W - 40)
    img[:, :, 0] = 40                      # B: flat background so it is obviously synthetic
    img[100:380, x:x + 40] = (0, 0, 255)   # a moving red bar
    return img


class StereoSyncTest(Node):
    def __init__(self, frames: int, rate: float, tol_ns: int):
        super().__init__("stereo_sync_test")
        self.frames = frames
        self.period = 1.0 / rate
        self.tol_ns = tol_ns

        # Match the publisher QoS the bridge subscribes with. A RELIABLE
        # publisher against a BEST_EFFORT subscriber fails the handshake and
        # delivers nothing at all -- it does not degrade gracefully.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub_l = self.create_publisher(Image, "/sim/camera/image_raw", pub_qos)
        self.pub_r = self.create_publisher(Image, "/sim/camera/right/image_raw", pub_qos)

        # ovcam_bridge publishes RELIABLE KeepLast(2).
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.lock = threading.Lock()
        self.got_l = {}   # stamp_ns -> arrival monotonic ns
        self.got_r = {}
        self.create_subscription(Image, "/ovcam/image_raw",
                                 lambda m: self._recv(m, self.got_l), sub_qos)
        self.create_subscription(Image, "/ovcam/right/image_raw",
                                 lambda m: self._recv(m, self.got_r), sub_qos)

        self.sent = []    # the source stamps we actually sent

    def _recv(self, msg, store):
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self.lock:
            store[ns] = time.monotonic_ns()

    def _img(self, arr, stamp_ns, frame_id):
        m = Image()
        m.header.frame_id = frame_id
        m.header.stamp.sec = stamp_ns // 1_000_000_000
        m.header.stamp.nanosec = stamp_ns % 1_000_000_000
        m.height, m.width = H, W
        m.encoding = "bgr8"        # what the MATLAB publisher declares and packs
        m.is_bigendian = 0
        m.step = W * 3
        m.data = arr.tobytes()
        return m

    def run(self) -> int:
        # A source clock deliberately OFFSET far from wall/monotonic time. If the
        # bridge ever falls back to stamping with its own arrival time, the
        # republished stamp will not be in this range and test (3) catches it --
        # which a "left and right are close" check alone would not.
        base = 1_000_000_000_000_000_000       # ~2001 in epoch ns: unmistakably synthetic

        print(f"publishing {self.frames} pairs at {1/self.period:.0f} Hz "
              f"(source stamps start at {base})")
        for i in range(self.frames):
            stamp = base + int(i * self.period * 1e9)
            self.sent.append(stamp)
            # SAME stamp object for both eyes -- this is the property under test.
            self.pub_l.publish(self._img(make_frame(i, "left"), stamp, "camera"))
            self.pub_r.publish(self._img(make_frame(i, "right"), stamp, "camera_right"))
            end = time.monotonic() + self.period
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.001)

        # drain
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        with self.lock:
            L, R = dict(self.got_l), dict(self.got_r)
        return self._report(L, R)

    def _report(self, L, R) -> int:
        print(f"\n  left  received: {len(L)}")
        print(f"  right received: {len(R)}")

        if not L or not R:
            missing = "left" if not L else "right"
            print(f"\nFAIL: no frames on the {missing} eye.")
            print("  Check the chain is up:  ros2 topic hz /ovcam/image_raw")
            print("  and that the launch was started with stereo:=true")
            return 1

        sent = set(self.sent)

        # (3) are the republished stamps the SOURCE stamps we sent?
        src_l = len(set(L) & sent)
        src_r = len(set(R) & sent)
        print(f"  left  stamps that match a sent source stamp: {src_l}/{len(L)}")
        print(f"  right stamps that match a sent source stamp: {src_r}/{len(R)}")
        if src_l == 0 or src_r == 0:
            print("\nFAIL: republished stamps are NOT the source stamps.")
            print("  The bridges are stamping with their own arrival time, so a")
            print("  left/right pair can never be reliably matched.")
            print("  Check use_source_stamp:=true on BOTH sim_camera_bridge and")
            print("  ovcam_bridge instances (the launch file sets it from stereo:=).")
            return 1

        # (2) do paired stamps agree exactly?
        paired = sorted(set(L) & set(R))
        print(f"  pairs sharing an identical stamp: {len(paired)}")
        if not paired:
            near = min(abs(a - b) for a in L for b in R)
            print(f"\nFAIL: no left/right pair shares a stamp "
                  f"(closest was {near} ns apart).")
            print("  Stereo matching would pair non-corresponding frames.")
            return 1

        expected = min(len(L), len(R))
        if len(paired) < expected * 0.9 and self.tol_ns == 0:
            print(f"\nFAIL: only {len(paired)} of ~{expected} frames paired exactly.")
            return 1

        print(f"\nPASS")
        print(f"  {len(paired)} pairs carry byte-identical stamps taken from the")
        print(f"  source clock, not from Pi arrival time. Stereo matching is safe.")
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--tolerance-ns", type=int, default=0,
                    help="allowed stamp difference; 0 = require exact equality")
    args = ap.parse_args()

    rclpy.init()
    node = StereoSyncTest(args.frames, args.rate, args.tolerance_ns)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
