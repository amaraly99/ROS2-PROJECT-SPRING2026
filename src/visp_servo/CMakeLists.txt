cmake_minimum_required(VERSION 3.16)
project(visp_servo CXX)
set(CMAKE_CXX_STANDARD 17)

# ── ROS 2 dependencies ─────────────────────────────────────────
find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge         REQUIRED)
find_package(image_transport   REQUIRED)
find_package(yolo_msgs         REQUIRED)
find_package(tf2_ros           REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── ViSP ────────────────────────────────────────────────────────
find_package(VISP REQUIRED)
find_package(Eigen3 REQUIRED)

# ── OpenCV (custom build in /workspace/opencv/build) ────────────
set(OpenCV_DIR "/workspace/opencv/build" CACHE PATH "")
find_package(OpenCV REQUIRED)

# ── Executable ──────────────────────────────────────────────────
add_executable(visp_servo_node src/visp_servo_node.cpp)

target_include_directories(visp_servo_node PRIVATE
    ${VISP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(visp_servo_node
    ${VISP_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

ament_target_dependencies(visp_servo_node
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    visualization_msgs
    cv_bridge
    image_transport
    yolo_msgs
    tf2_ros
    tf2_geometry_msgs
)

target_compile_options(visp_servo_node PRIVATE -O2)

install(TARGETS visp_servo_node DESTINATION lib/${PROJECT_NAME})

# ── Install launch + config ─────────────────────────────────────
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()
