cmake_minimum_required(VERSION 3.16)
project(visp_servo CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(servo_core REQUIRED)
find_package(Eigen3 REQUIRED)

# ── ViSP (the actual visual-servoing library — TS1 needs it) ──
find_package(VISP REQUIRED)

# Modular layout: thin main → controller → {ibvs_law math, depth/* modules}.
add_executable(visp_servo_node
  src/visp_servo_main.cpp
  src/ibvs_controller.cpp
  src/ibvs_law.cpp
  src/depth/slam_depth_source.cpp
  src/slam_pose_source.cpp
)
target_include_directories(visp_servo_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${VISP_INCLUDE_DIRS}
)
target_link_libraries(visp_servo_node ${VISP_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(visp_servo_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  yolo_msgs
  servo_core
)
target_compile_options(visp_servo_node PRIVATE -O2)

install(TARGETS visp_servo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()