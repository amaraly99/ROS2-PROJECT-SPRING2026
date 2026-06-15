cmake_minimum_required(VERSION 3.16)
project(hil_servo CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(servo_core REQUIRED)

add_executable(hil_servo_node
  src/hil_servo_main.cpp
  src/proportional_controller.cpp
)
target_include_directories(hil_servo_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(hil_servo_node
  rclcpp
  geometry_msgs
  std_msgs
  yolo_msgs
  servo_core
)
target_compile_options(hil_servo_node PRIVATE -O2)

install(TARGETS hil_servo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()