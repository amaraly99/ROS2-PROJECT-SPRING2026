#!/usr/bin/env python3
"""Sync every baseline-encoding file on the Pi to one stereo baseline value.

Five files encode the stereo baseline, in four different formats (HANDOFF Part G):
  1. camera_calib/hil_sim_ov2slam_stereo.yaml       body_T_cam1 translation (x)
  2. camera_calib/hil_sim_ov2slam_stereo_fast.yaml  body_T_cam1 translation (x)
  3. src/orbslam2/.../stereo/hil_sim.yaml           Camera.bf = fx * baseline
  4. src/orbslam3/.../Stereo/HIL_SIM.yaml           Stereo.b  = baseline
  5. src/rtabmap_docker/hil_stereo_bridge.py        BASELINE_M = baseline

Physical baseline (the .slx camera mount offset) and these five MUST agree. A
mismatch is silent: SLAM computes depth from the calibrated baseline, so a wrong
value scales every depth without any error being raised. Same failure class as
FIX-027 (fx 1200 vs 554).

Usage:  python3 scripts/hil_matrix/sync_baseline.py <baseline_m>   (from the repo root)
Verifies every write by reading the file back; exits non-zero on any mismatch.
"""
import re
import sys
from pathlib import Path

FX = 554.0
REPO = Path(__file__).resolve().parents[2]   # <repo>/scripts/hil_matrix/sync_baseline.py

OV2_STEREO      = REPO / "camera_calib/hil_sim_ov2slam_stereo.yaml"
OV2_STEREO_FAST = REPO / "camera_calib/hil_sim_ov2slam_stereo_fast.yaml"
ORB2_STEREO     = REPO / "src/orbslam2/ros2-ORB_SLAM2/src/stereo/hil_sim.yaml"
ORB3_STEREO     = REPO / "src/orbslam3/orb_slam3/config/Stereo/HIL_SIM.yaml"
RTAB_BRIDGE     = REPO / "src/rtabmap_docker/hil_stereo_bridge.py"


def patch(path, pattern, replacement, verify_pattern, expected):
    text = path.read_text()
    new_text, n = re.subn(pattern, replacement, text, count=1)
    if n != 1:
        sys.exit(f"FAIL {path.name}: pattern matched {n} times, expected exactly 1")
    path.write_text(new_text)

    # read back and verify
    back = path.read_text()
    m = re.search(verify_pattern, back)
    if not m:
        sys.exit(f"FAIL {path.name}: verify pattern not found after write")
    got = float(m.group(1))
    if abs(got - expected) > 1e-6:
        sys.exit(f"FAIL {path.name}: wrote {expected}, read back {got}")
    print(f"  OK  {path.name:34s} -> {got}")


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: sync_baseline.py <baseline_m>")
    b = float(sys.argv[1])
    bf = FX * b

    print(f"syncing all five files to baseline {b} m (Camera.bf = {FX} * {b} = {bf:.2f})")

    # 1 + 2: body_T_cam1 translation, the 4th element of the first matrix row
    for p in (OV2_STEREO, OV2_STEREO_FAST):
        patch(p,
              r"(body_T_cam1[\s\S]{0,200}?data: \[1\.0, 0\.0, 0\.0, )[-0-9.]+",
              rf"\g<1>{b}",
              r"body_T_cam1[\s\S]{0,200}?data: \[1\.0, 0\.0, 0\.0, ([-0-9.]+)",
              b)

    # 3: ORB-SLAM2 uses bf = fx * baseline, not the baseline itself
    patch(ORB2_STEREO,
          r"(?m)^(Camera\.bf: )[-0-9.]+",
          rf"\g<1>{bf:.2f}",
          r"(?m)^Camera\.bf: ([-0-9.]+)",
          round(bf, 2))

    # 4: ORB-SLAM3 stores the baseline directly
    patch(ORB3_STEREO,
          r"(?m)^(Stereo\.b: )[-0-9.]+",
          rf"\g<1>{b}",
          r"(?m)^Stereo\.b: ([-0-9.]+)",
          b)

    # 5: RTAB-Map python constant
    patch(RTAB_BRIDGE,
          r"(?m)^(BASELINE_M = )[-0-9.]+",
          rf"\g<1>{b}",
          r"(?m)^BASELINE_M = ([-0-9.]+)",
          b)

    print(f"all five verified at baseline {b} m")


if __name__ == "__main__":
    main()
