#!/usr/bin/env bash
# ORB-SLAM2 core + ROS2 wrapper build for THIS worktree (2026-09-08). Runs INSIDE the
# orbslam2_fixed image with src/orbslam2 mounted at /workspace -- the documented recipe of
# README_HIL.md / start_container, step for step, MINUS the `pip install evo` step (a network
# download the sidecar does not need). Artifacts land on the host: build/, lib/, build_stereo/,
# install_stereo/ (all new, listed in docs/REVERT_2026-09-08_stereo_integration.md).
set -uo pipefail
source_safe(){ if [ -f "$1" ]; then set +u; . "$1"; set -u; fi; }
source_safe /opt/ros/jazzy/setup.bash
source_safe /root/ws/install/setup.bash
cd /workspace
step(){ echo; echo "===== [$(date +%H:%M:%S)] $1 ====="; }

step "vocabulary"
ls -la Vocabulary/ORBvoc.txt || { echo "Vocabulary/ORBvoc.txt missing"; exit 2; }

# Thirdparty libs: this repo's build.sh/start_container run only the top-level cmake, so
# check whether that already covers DBoW2/g2o; if their .so are absent afterwards, build
# them the upstream way (Thirdparty/*/build).
step "core: cmake configure"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 -DCMAKE_CXX_STANDARD_LIBRARIES='-lboost_system' || exit 3
step "core: build (-j3)"
cmake --build build -j3 || exit 4
for tp in DBoW2 g2o; do
  if ! ls Thirdparty/$tp/lib/lib$tp.so >/dev/null 2>&1; then
    step "Thirdparty/$tp: not produced by the top-level build -> building it directly"
    cmake -S Thirdparty/$tp -B Thirdparty/$tp/build -DCMAKE_BUILD_TYPE=Release || exit 5
    cmake --build Thirdparty/$tp/build -j3 || exit 5
  fi
done
step "core: install (ORB_SLAM2Config.cmake + headers into this container's /usr/local)"
cmake --install build || exit 6
step "refresh runtime libs in container"
cp lib/libORB_SLAM2.so /usr/local/lib/ && cp lib/libDBoW2.so /usr/local/lib/ 2>/dev/null || cp Thirdparty/DBoW2/lib/libDBoW2.so /usr/local/lib/
cp Thirdparty/g2o/lib/libg2o.so /usr/local/lib/ || exit 7
ldconfig
ls -la lib/ Thirdparty/g2o/lib/ Thirdparty/DBoW2/lib/ 2>&1

step "wrapper: colcon build -> install_stereo/"
export LIBRARY_PATH=/workspace/lib:/usr/local/lib:${LIBRARY_PATH:-}
export LD_LIBRARY_PATH=/workspace/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}
colcon build --base-paths /workspace/ros2-ORB_SLAM2 --packages-select orbslam --build-base /workspace/build_stereo --install-base /workspace/install_stereo --event-handlers console_direct+ || exit 8

step "verify"
ls -la /workspace/install_stereo/orbslam/lib/orbslam/
ldd /workspace/install_stereo/orbslam/lib/orbslam/stereo | grep -E "libORB_SLAM2|libg2o|libDBoW2|opencv_core|not found"
echo BUILD_EXIT=0
