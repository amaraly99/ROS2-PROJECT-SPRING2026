cmake_minimum_required(VERSION 3.16)
project(servo_core CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)

# Shared FSM/filter/instrumentation library. Downstream test-subject
# packages (visp_servo = IBVS, hil_servo = proportional) link this and
# inject their IServoController.
add_library(servo_core SHARED
  src/servo_fsm_node.cpp
)
target_include_directories(servo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(servo_core
  rclcpp
  geometry_msgs
  std_msgs
  yolo_msgs
)
target_compile_options(servo_core PRIVATE -O2)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS servo_core
  EXPORT export_servo_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_servo_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs std_msgs yolo_msgs)
ament_export_include_directories(include)
ament_package()