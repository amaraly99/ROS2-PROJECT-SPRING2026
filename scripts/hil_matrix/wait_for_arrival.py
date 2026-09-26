#!/usr/bin/env python3
"""Block until the RECORDED /sim/drone_pose stream shows the drone has reached
the mission's standoff distance from the target, for real, not until MATLAB's
own internal sim_pose says so.

Replaces a fixed drain-time sleep that assumed a specific camera delivery
rate (~13.8 Hz). That assumption breaks at a different rate (e.g. a wider
stereo baseline slowing frame delivery), truncating the bag mid-flight even
though the mission itself completed normally. This polls the same lagging,
delivered channel that actually gets recorded, so "arrived" means the bag
has genuinely caught up, not that the simulation internally finished.

Exit 0 : arrived (N consecutive samples within tolerance of STANDOFF).
Exit 1 : timed out before arriving -- caller should fail the trial, the same
         way an `invalid` or `maxtime_hit` flag already does. A silently
         truncated bag scored as "ok" is the exact failure mode this exists
         to prevent, not something to fall back past.

Usage: python3 wait_for_arrival.py --max-wait 60
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

STANDOFF_M = 3.0        # matches slam_traj_probe.m's own STANDOFF constant
TOLERANCE_M = 0.3       # a band, not an exact match -- real position data has noise
NEEDED_CONSECUTIVE = 10  # same shape as aggregate_matrix.py's own window-end rule
                         # ("3 samples past GT speed < 0.10 m/s"), just a stricter count


class ArrivalWatcher(Node):
    def __init__(self, max_wait):
        super().__init__('wait_for_arrival')
        self.target = None
        self.consecutive = 0
        self.arrived = False
        self.deadline = time.monotonic() + max_wait
        self.create_subscription(Float64MultiArray, '/sim/target_pose',
                                  self.on_target, 10)
        self.create_subscription(Float64MultiArray, '/sim/drone_pose',
                                  self.on_drone, 10)

    def on_target(self, msg):
        d = msg.data
        if len(d) >= 3:
            self.target = (d[0], d[1], d[2])

    def on_drone(self, msg):
        if self.target is None or self.arrived:
            return
        d = msg.data
        if len(d) < 3:
            return
        dist = math.dist((d[0], d[1], d[2]), self.target)
        if abs(dist - STANDOFF_M) <= TOLERANCE_M:
            self.consecutive += 1
        else:
            self.consecutive = 0  # any real excursion resets the count
        if self.consecutive >= NEEDED_CONSECUTIVE:
            self.arrived = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-wait', type=float, required=True,
                     help='hard ceiling in seconds; exit 1 if reached first')
    args = ap.parse_args()

    rclpy.init()
    node = ArrivalWatcher(args.max_wait)
    try:
        while rclpy.ok() and not node.arrived:
            if time.monotonic() > node.deadline:
                print(f"TIMEOUT -- never held within {TOLERANCE_M}m of "
                      f"{STANDOFF_M}m standoff for {NEEDED_CONSECUTIVE} "
                      f"consecutive samples (target={node.target})",
                      file=sys.stderr)
                sys.exit(1)
            rclpy.spin_once(node, timeout_sec=0.5)
        print("ARRIVED")
        sys.exit(0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
