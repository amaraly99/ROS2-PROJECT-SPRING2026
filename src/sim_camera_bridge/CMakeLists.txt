cmake_minimum_required(VERSION 3.16)
project(sim_camera_bridge CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge   REQUIRED)

set(OpenCV_DIR "/workspace/opencv/build" CACHE PATH "")
find_package(OpenCV REQUIRED)

add_executable(sim_camera_bridge_node src/sim_camera_bridge_node.cpp)

target_include_directories(sim_camera_bridge_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../ovcam_producer/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(sim_camera_bridge_node ${OpenCV_LIBS} rt pthread)
ament_target_dependencies(sim_camera_bridge_node rclcpp sensor_msgs cv_bridge)
target_compile_options(sim_camera_bridge_node PRIVATE -O2)

install(TARGETS sim_camera_bridge_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()
