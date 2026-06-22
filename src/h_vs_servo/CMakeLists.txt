cmake_minimum_required(VERSION 3.8)
project(h_vs_servo)

# ── Dependencies ──────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(servo_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# ── Core math library (pure Eigen, zero ROS — copied from h_vs) ──
add_library(homography_2d_vs
    src/homography_2d_vs.cpp)
target_include_directories(homography_2d_vs PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS})

# ── Controller adapter library ────────────────────────────────────
add_library(hvs_controller
    src/hvs_controller.cpp)
target_include_directories(hvs_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(hvs_controller rclcpp servo_core)
target_link_libraries(hvs_controller homography_2d_vs ${OpenCV_LIBS})

# ── Executable ────────────────────────────────────────────────────
add_executable(h_vs_servo_node src/h_vs_servo_main.cpp)
ament_target_dependencies(h_vs_servo_node rclcpp servo_core)
target_link_libraries(h_vs_servo_node hvs_controller)

# ── Install ───────────────────────────────────────────────────────
install(TARGETS h_vs_servo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()