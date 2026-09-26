#!/usr/bin/env bash
# Revert of the 2026-09-08 stereo-integration work (OV2SLAM-fast / ORB-SLAM3 / ORB-SLAM2 /
# RTAB-Map stereo). Companion to docs/REVERT_2026-09-08_stereo_integration.md.
#
# Policy: NOTHING is deleted. Every path this work CREATED is moved into
# trash/stereo_integration_2026-09-08_<timestamp>/ (same relative layout), and the single
# tracked file it MODIFIED is restored from git. Re-runnable; absent paths are reported.
#
# Windows side (not reachable from here) -- run by hand in C:\Users\homie\Desktop\ROS2-PROJECT-SPRING2026:
#   git checkout -- scripts/hil_matrix/run_matrix.py
#   move config\hil\matrix\only_ov2_stereo_oracle_fast.yaml      trash\
#   move config\hil\matrix\only_orbslam3_stereo_oracle.yaml      trash\
#   move config\hil\matrix\only_orbslam2_stereo_oracle.yaml      trash\
#   move config\hil\matrix\only_rtabmap_stereo_oracle.yaml       trash\
set -u
cd "$(dirname "$0")/.." || exit 1
TS=$(date +%Y%m%d_%H%M%S)
T="trash/stereo_integration_2026-09-08_${TS}"
mkdir -p "$T"

move_to_trash() {
    local p="$1"
    if [ -e "$p" ] || [ -L "$p" ]; then
        mkdir -p "$T/$(dirname "$p")"
        mv "$p" "$T/$p" && echo "moved   $p"
    else
        echo "absent  $p"
    fi
}

NEW_PATHS=(
    # OV2SLAM fast stereo
    camera_calib/hil_sim_ov2slam_stereo_fast.yaml
    config/hil/stack/ov2slam_stereo_oracle_fast.yaml
    # ORB-SLAM3 stereo
    src/orbslam3/Pangolin
    src/orbslam3/orb_slam3/Vocabulary
    src/orbslam3/build
    src/orbslam3/install
    src/orbslam3/log
    src/orbslam3/build_stereo_2026-09-08.log
    src/orbslam3/orb_slam3/config/Stereo/HIL_SIM.yaml
    config/hil/stack/orbslam3_stereo_oracle.yaml
    # ORB-SLAM2 stereo
    src/orbslam2/Vocabulary
    src/orbslam2/build
    src/orbslam2/lib
    src/orbslam2/build_stereo
    src/orbslam2/install_stereo
    src/orbslam2/log
    src/orbslam2/build_stereo_2026-09-08.log
    src/orbslam2/ros2-ORB_SLAM2/src/stereo/hil_sim.yaml
    config/hil/stack/orbslam2_stereo_oracle.yaml
    # RTAB-Map stereo
    src/rtabmap_docker/hil_stereo_bridge.py
    src/rtabmap_docker/hil_stereo.launch.py
    config/hil/stack/rtabmap_stereo_oracle.yaml
    # build + test tooling, this manifest
    src/orbslam2/build_stereo_2026-09-08.sh
    scripts/hil_matrix/synthetic_stereo_pub.py
    scripts/hil_matrix/stereo_wiring_test.sh
    docs/REVERT_2026-09-08_stereo_integration.md
    docs/fixlog/028-orb-stereo-wrappers-published-nothing.md
    HANDOFF_2026-09-08.md
    # run_matrix.py's calib backup files, only present if a matrix aborted mid-run
    camera_calib/hil_sim_ov2slam_stereo_fast.yaml.matrixbak
    src/orbslam3/orb_slam3/config/Stereo/HIL_SIM.yaml.matrixbak
    src/orbslam2/ros2-ORB_SLAM2/src/stereo/hil_sim.yaml.matrixbak
    src/rtabmap_docker/hil_stereo_bridge.py.matrixbak
)
for p in "${NEW_PATHS[@]}"; do move_to_trash "$p"; done

# Build side-effects inside TRACKED dirs (ORB-SLAM2 core build writes here). Listed
# from `git status` taken after the build; only moved if they are untracked leftovers.
for p in src/orbslam2/Thirdparty/DBoW2/build src/orbslam2/Thirdparty/DBoW2/lib \
         src/orbslam2/Thirdparty/g2o/build src/orbslam2/Thirdparty/g2o/lib; do
    if [ -e "$p" ] && [ -z "$(git ls-files "$p" | head -1)" ]; then move_to_trash "$p"; fi
done

# The TWO tracked files modified on the Pi side (both: stereo path never got the mono HIL
# pose/tracking-state publishers -- docs/fixlog/028-*).
git checkout -- src/orbslam2/ros2-ORB_SLAM2/src/stereo/stereo.cpp && echo "restored src/orbslam2/ros2-ORB_SLAM2/src/stereo/stereo.cpp"
git checkout -- src/orbslam3/src/common.cpp && echo "restored src/orbslam3/src/common.cpp"

echo
echo "Everything moved to: $T"
echo "Remaining differences vs the pre-work baseline (/tmp/git_baseline_2026-09-08.txt):"
git status --short | diff - /tmp/git_baseline_2026-09-08.txt && echo "(none -- tree matches baseline)" || true
