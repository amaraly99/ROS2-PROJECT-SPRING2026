"""bench_geometry.py — the camera + stop-distance constants of the controller benchmark.

One place, imported by plot_controller_hil.py (scorer) and ctrl_runner.py (live view).
Values must match config/hil/bench_fsm.yaml and config/hil/bench_oracle.yaml.

Why the drone stops at STANDOFF metres (pinhole camera):
    a target of height KNOWN_H at distance Z looks FY * KNOWN_H / Z pixels tall.
    The controller stops when that height is TARGET_BBOX_RATIO of the image height:
        FY * KNOWN_H / Z = TARGET_BBOX_RATIO * IMG_H
        Z = FY * KNOWN_H / (TARGET_BBOX_RATIO * IMG_H) = 554 * 1.5 / (0.55 * 480) = 3.15 m
"""
FY = 554.0                # px, focal length (bench_fsm.yaml cam_fy, SIM_PARAMS.md)
KNOWN_H = 1.5             # m, target height the oracle and the FSM assume (known_target_height)
IMG_H = 480               # px, image height
TARGET_BBOX_RATIO = 0.55  # box height / image height at which the FSM stops (target_bbox_ratio)
STANDOFF = FY * KNOWN_H / (TARGET_BBOX_RATIO * IMG_H)   # ≈ 3.15 m
