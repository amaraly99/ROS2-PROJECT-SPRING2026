"""bench_geometry.py — the camera + stop-distance constants of the controller benchmark.

One place, imported by plot_controller_hil.py (scorer) and ctrl_runner.py (live view).
Values must match config/hil/bench_fsm.yaml and config/hil/bench_oracle.yaml.

Why the drone stops at STANDOFF metres (pinhole camera):
    a target of height KNOWN_H at distance Z looks FY * KNOWN_H / Z pixels tall.
    The controller stops when that height is TARGET_BBOX_RATIO of the image height:
        FY * KNOWN_H / Z = TARGET_BBOX_RATIO * IMG_H
        Z = FY * KNOWN_H / (TARGET_BBOX_RATIO * IMG_H) = 554 * 1.5 / (0.55 * 480) = 3.15 m
"""
import math

FY = 554.0                # px, focal length (bench_fsm.yaml cam_fy, SIM_PARAMS.md)
KNOWN_H = 1.5             # m, target height the oracle and the FSM assume (known_target_height)
IMG_H = 480               # px, image height
TARGET_BBOX_RATIO = 0.55  # box height / image height at which the FSM stops (target_bbox_ratio)
STANDOFF = FY * KNOWN_H / (TARGET_BBOX_RATIO * IMG_H)   # ≈ 3.15 m

# Shared world positions (SIM_PARAMS.md). One tuple each, so no module keeps its own copy.
SIGN = (35.1, 2.92, 3.08)                       # m, frozen stop sign (x, y, z)
START = (15.12, 3.84, 7.5, 2 * math.pi - 0.0460)   # m, m, m, rad: shared start (x, y, z, yaw)
SLAM_STANDOFF = 3.0       # m, where matlab/slam_traj_probe.m stops (its line 121); controllers stop at STANDOFF


def bearing_deg(x, y):
    """Bearing from (x, y) to the sign, degrees in [0, 360) (yaw convention of the MATLAB status)."""
    sx, sy, _ = SIGN
    return math.degrees(math.atan2(sy - y, sx - x)) % 360.0


def angle_diff(a, b):
    """Absolute difference of two angles in degrees, wrapped into [0, 180]."""
    return abs((a - b + 180.0) % 360.0 - 180.0)
