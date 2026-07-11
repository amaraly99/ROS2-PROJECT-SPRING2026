#!/usr/bin/env python3
"""
mission_monitor.py — capture the controller's mission-state timeline for
detector-axis benchmarking.

Subscribes to /visp/state (std_msgs/String, latched) which visp_servo_node
publishes on every transition between SEARCHING / APPROACHING / REACQUIRE /
REACHED. Records timestamps and, on exit, writes a JSON with the behavioural
comparison metrics that distinguish detectors at the system level:

  reached                  did the drone ever reach the target (bool)
  time_to_reached_s        monitor-start -> first REACHED (null if never)
  approach_duration_s      first APPROACHING -> first REACHED (warmup-independent)
  time_to_first_approach_s monitor-start -> first APPROACHING
  n_reacquire              entries into REACQUIRE (tracking instability)
  n_lost                   returns to SEARCHING after the first APPROACHING
  time_in_state_s          dwell time per state
  final_state, transitions

The monitor is started right after the per-run drone reset, so t0 ~= mission
start. It rewrites the JSON atomically on every transition (so the file is
always current even if hard-killed) and also flushes on SIGTERM/SIGINT.

Usage (inside the stack container, ROS sourced + DDS env set):
    python3 mission_monitor.py --output /tmp/mission.json [--duration 240]
"""
import argparse
import json
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

STATES = ("SEARCHING", "APPROACHING", "REACQUIRE", "REACHED")


class MissionMonitor(Node):
    def __init__(self, output_path: str):
        super().__init__("mission_monitor")
        self.output_path = output_path
        self.t0 = time.monotonic()
        self.events = []          # (t_rel_s, state)
        self.initial_state = None  # first (latched) sample — NOT a transition
        self.last_state = None
        self.last_t = self.t0
        self.dwell = {s: 0.0 for s in STATES}

        # Match the latched publisher: reliable + transient_local.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub = self.create_subscription(String, "/visp/state", self._on_state, qos)
        self._write()  # create the file immediately

    def _on_state(self, msg: String):
        now = time.monotonic()
        state = msg.data.strip()
        # accrue dwell time to the state we were in
        if self.last_state in self.dwell:
            self.dwell[self.last_state] += now - self.last_t
        self.last_t = now
        if state == self.last_state:
            return  # latched re-delivery of same state
        if self.last_state is None:
            # First sample is the latched state from before the drone reset
            # (the stack starts before the sim restart, so a stale REACHED /
            # APPROACHING from the previous run can be replayed here). Record
            # it as the initial state but NOT as a mission event — otherwise
            # time_to_reached collapses to ~0 and approach_duration goes
            # negative on every run after the first.
            self.initial_state = state
            self.last_state = state
            self._write()
            return
        self.events.append((round(now - self.t0, 3), state))
        self.last_state = state
        self._write()

    def _metrics(self) -> dict:
        # close out dwell for the current state
        dwell = dict(self.dwell)
        if self.last_state in dwell:
            dwell[self.last_state] += time.monotonic() - self.last_t

        def first_t(name):
            for t, s in self.events:
                if s == name:
                    return t
            return None

        t_reach = first_t("REACHED")
        t_app = first_t("APPROACHING")
        n_reacq = sum(1 for _, s in self.events if s == "REACQUIRE")
        # "lost" = a SEARCHING that occurs after the first APPROACHING
        n_lost = 0
        seen_app = False
        for _, s in self.events:
            if s == "APPROACHING":
                seen_app = True
            elif s == "SEARCHING" and seen_app:
                n_lost += 1
        return {
            "reached": t_reach is not None,
            "time_to_reached_s": t_reach,
            "approach_duration_s": (round(t_reach - t_app, 3)
                                    if (t_reach is not None and t_app is not None) else None),
            "time_to_first_approach_s": t_app,
            "n_reacquire": n_reacq,
            "n_lost": n_lost,
            "time_in_state_s": {s: round(v, 3) for s, v in dwell.items()},
            "initial_state": self.initial_state,
            "final_state": self.last_state,
            "n_transitions": len(self.events),
            "transitions": self.events,
        }

    def _write(self):
        tmp = self.output_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._metrics(), f, indent=2)
        os.replace(tmp, self.output_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    ap.add_argument("--duration", type=float, default=300.0,
                    help="safety self-terminate (s)")
    args = ap.parse_args()

    rclpy.init()
    node = MissionMonitor(args.output)

    stop = {"flag": False}

    def _sig(*_):
        stop["flag"] = True
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and not stop["flag"] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node._write()
        node.destroy_node()
        rclpy.shutdown()
        print(f"[mission_monitor] wrote {args.output}: {node._metrics()['final_state']} "
              f"reached={node._metrics()['reached']}", file=sys.stderr)


if __name__ == "__main__":
    main()
