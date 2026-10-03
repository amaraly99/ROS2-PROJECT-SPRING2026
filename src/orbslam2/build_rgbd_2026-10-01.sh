# build_rgbd_2026-10-01.sh -- builds the ORB-SLAM2 wrapper (incl. rgbd_hil) into
# /workspace/install_rgbd. Run inside orbslam2_fixed with src/orbslam2 mounted at /workspace:
#   sudo docker run --rm -v ~/ROS2-slam-hil/src/orbslam2:/workspace --entrypoint bash orbslam2_fixed -c 'bash /workspace/build_rgbd_2026-10-01.sh'
# Does NOT rebuild the ORB-SLAM2 core (lib/ stays the file the stereo arm uses); it only
# re-installs the existing core build so find_package(ORB_SLAM2) works in this container.
# install_stereo/ and build_stereo/ are not touched.
set -uo pipefail
source_safe(){ if [ -f "$1" ]; then set +u; . "$1"; set -u; fi; }
source_safe /opt/ros/jazzy/setup.bash
source_safe /root/ws/install/setup.bash
cd /workspace
step(){ echo; echo "===== [$(date +%H:%M:%S)] $1 ====="; }
step 'core: install existing build (no compile)'
md5sum lib/libORB_SLAM2.so
cmake --install build >/dev/null || exit 6
cp lib/libORB_SLAM2.so /usr/local/lib/ && { cp lib/libDBoW2.so /usr/local/lib/ 2>/dev/null || cp Thirdparty/DBoW2/lib/libDBoW2.so /usr/local/lib/; }
cp Thirdparty/g2o/lib/libg2o.so /usr/local/lib/ || exit 7
ldconfig
step 'wrapper: colcon build -> install_rgbd/'
export LIBRARY_PATH=/workspace/lib:/usr/local/lib:${LIBRARY_PATH:-}
export LD_LIBRARY_PATH=/workspace/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}
colcon build --base-paths /workspace/ros2-ORB_SLAM2 --packages-select orbslam --build-base /workspace/build_rgbd --install-base /workspace/install_rgbd --event-handlers console_direct+ || exit 8
step 'verify'
md5sum lib/libORB_SLAM2.so
ls -la /workspace/install_rgbd/orbslam/lib/orbslam/
ldd /workspace/install_rgbd/orbslam/lib/orbslam/rgbd_hil | grep -E 'libORB_SLAM2|libg2o|libDBoW2|opencv_core|not found'
echo BUILD_EXIT=0
