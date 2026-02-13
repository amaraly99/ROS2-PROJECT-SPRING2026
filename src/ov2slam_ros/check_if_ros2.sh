#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-.}"

ok()   { echo "✅ $1"; }
warn() { echo "⚠️  $1"; }
bad()  { echo "❌ $1"; }
die()  { bad "$1"; exit 2; }

echo
echo "=================================================="
figlet "ROS2 CHECKER FOR OV2SLAM"
echo "=================================================="
echo "📁 Scanning: $ROOT"
echo

# ---------- helpers ----------
# Grep C/C++ sources while excluding common junk dirs
scan_sources() {
  local pattern="$1"
  grep -RIn --include=\*.{h,hpp,hh,c,cc,cpp,cxx} -E "$pattern" "$ROOT" 2>/dev/null || true
}

# Filter out commented occurrences in grep output lines formatted as: file:line:content
# Removes lines where content starts with // (optionally preceded by whitespace)
filter_commented_cpp() {
  grep -v ':[0-9]\+:[[:space:]]*//' || true
}

# Filter out commented occurrences in CMake grep output lines formatted as: file:line:content
filter_commented_cmake() {
  grep -v '^[^:]*:[0-9]\+:[[:space:]]*#' || true
}

# ---------- 1) package.xml ----------
pkg_xml="$ROOT/package.xml"
if [[ ! -f "$pkg_xml" ]]; then
  warn "package.xml not found at: $pkg_xml (skipping package.xml checks)"
else
  echo "📦 package.xml checks"

  # Hard fail on ROS1 buildtool
  if grep -qE '<buildtool_depend>[[:space:]]*catkin[[:space:]]*</buildtool_depend>' "$pkg_xml"; then
    die "package.xml declares ROS1 buildtool: catkin"
  fi

  # Hard fail on common ROS1 deps (these indicate ROS1 package metadata)
  if grep -qE '<depend>[[:space:]]*(roscpp|rospy|roslib|tf)[[:space:]]*</depend>' "$pkg_xml"; then
    die "package.xml contains ROS1 deps (roscpp/rospy/roslib/tf)"
  fi

  # Positive signal: ament_cmake or ament_python
  if grep -qE '<buildtool_depend>[[:space:]]*ament_(cmake|python)[[:space:]]*</buildtool_depend>' "$pkg_xml"; then
    ok "package.xml uses ament_* buildtool"
  else
    warn "package.xml does not explicitly declare ament_cmake/ament_python"
  fi

  # Positive signal: rclcpp or rclpy
  if grep -qE '<depend>[[:space:]]*(rclcpp|rclpy)[[:space:]]*</depend>' "$pkg_xml"; then
    ok "package.xml declares ROS2 client library (rclcpp/rclpy)"
  else
    warn "package.xml does not declare rclcpp/rclpy (maybe core library only)"
  fi
  echo
fi

# ---------- 2) CMake checks ----------
echo "🛠️  CMake checks"

# Fail on active catkin usage
catkin_cmake="$(grep -RIn --include=CMakeLists.txt --include=\*.cmake -E 'find_package\([[:space:]]*catkin|catkin_package\(' "$ROOT" 2>/dev/null \
  | filter_commented_cmake)"
if [[ -n "$catkin_cmake" ]]; then
  echo "$catkin_cmake"
  die "Active catkin usage found in CMake (ROS1)"
else
  ok "No active catkin usage in CMake"
fi

# Positive ROS2 signals in CMake
ament_cmake="$(grep -RIn --include=CMakeLists.txt --include=\*.cmake -E 'find_package\([[:space:]]*ament_cmake|ament_package\(' "$ROOT" 2>/dev/null \
  | filter_commented_cmake || true)"
if [[ -n "$ament_cmake" ]]; then
  ok "ROS2 ament usage found in CMake"
else
  warn "No ament_cmake/ament_package found in CMake (maybe not a ROS2 package or using non-ament build)"
fi
echo

# ---------- 3) Source checks ----------
echo "🧾 Source checks"

# Active ROS1 includes
ros_inc="$(scan_sources '#include[[:space:]]*<[[:space:]]*ros/' | filter_commented_cpp)"
tf1_inc="$(scan_sources '#include[[:space:]]*<[[:space:]]*tf/'  | filter_commented_cpp)"

if [[ -n "$ros_inc" ]]; then
  echo "$ros_inc"
  die "Active ROS1 include detected: <ros/...>"
else
  ok "No active <ros/...> includes"
fi

if [[ -n "$tf1_inc" ]]; then
  echo "$tf1_inc"
  die "Active ROS1 include detected: <tf/...>"
else
  ok "No active <tf/...> includes"
fi

# Active ROS1 namespaces/usages (best-effort; can false-positive in comments/strings)
ros1_api="$(scan_sources '(^|[^A-Za-z0-9_])(ros::|tf::|rosbag::|nodelet|dynamic_reconfigure)([^A-Za-z0-9_]|$)' | filter_commented_cpp)"
if [[ -n "$ros1_api" ]]; then
  echo "$ros1_api"
  die "ROS1 API usage detected (ros:: / tf:: / etc.)"
else
  ok "No obvious ROS1 API usage (ros::/tf::/etc.)"
fi

# Positive ROS2 signal in sources
ros2_api="$(scan_sources '(rclcpp::|RCLCPP_|tf2_ros|tf2::)' | filter_commented_cpp || true)"
if [[ -n "$ros2_api" ]]; then
  ok "ROS2 API usage found (rclcpp/tf2) 🎉"
else
  warn "No obvious ROS2 API usage found in sources (could be core library only)"
fi

echo
echo "🎯 Verdict: If you saw only ✅ (and maybe ⚠️), you're ROS2-clean (no active ROS1)."
echo
