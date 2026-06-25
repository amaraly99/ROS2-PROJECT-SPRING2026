cmake_minimum_required(VERSION 3.16)
project(visp_pbvs_servo CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
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

# PBVS controller: main + single controller translation unit.
add_executable(visp_pbvs_node
  src/visp_pbvs_main.cpp
  src/pbvs_controller.cpp
)
target_include_directories(visp_pbvs_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${VISP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(visp_pbvs_node ${VISP_LIBRARIES} Eigen3::Eigen ${OpenCV_LIBS})
ament_target_dependencies(visp_pbvs_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  yolo_msgs
  servo_core
)
target_compile_options(visp_pbvs_node PRIVATE -O2)

install(TARGETS visp_pbvs_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()