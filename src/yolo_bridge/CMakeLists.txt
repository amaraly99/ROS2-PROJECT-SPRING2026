cmake_minimum_required(VERSION 3.16)
project(yolo_bridge)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yolo_msgs   REQUIRED)

add_executable(yolo_bridge_node src/yolo_bridge_node.cpp)

target_include_directories(yolo_bridge_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../yolo_producer/include
)
target_link_libraries(yolo_bridge_node rt pthread)
ament_target_dependencies(yolo_bridge_node rclcpp sensor_msgs yolo_msgs)

install(TARGETS yolo_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()
