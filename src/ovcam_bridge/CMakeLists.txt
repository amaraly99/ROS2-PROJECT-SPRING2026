cmake_minimum_required(VERSION 3.16)
project(ovcam_bridge CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ovcam_bridge_node src/ovcam_bridge_node.cpp)

target_include_directories(ovcam_bridge_node PRIVATE
    # points directly at producer's include folder — no copy needed
    ${CMAKE_CURRENT_SOURCE_DIR}/../ovcam_producer/include
)
target_link_libraries(ovcam_bridge_node rt pthread)
ament_target_dependencies(ovcam_bridge_node rclcpp sensor_msgs)
target_compile_options(ovcam_bridge_node PRIVATE -O3)

install(TARGETS ovcam_bridge_node DESTINATION lib/${PROJECT_NAME})
ament_package()
