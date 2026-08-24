cmake_minimum_required(VERSION 3.10)
project(ros2_orb_slam3)

# Make sure to set this path before building the  
# set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/") # Must be set to match your installation

# Enforce use of C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) # REDUNDANT?

# find dependencies
find_package(Python3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # HIL: PoseStamped for /slam/pose
find_package(message_filters REQUIRED)
# find_package(your_custom_msg_interface REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.6 REQUIRED) # In Ubuntu 24.04 4.6 or greater is required
find_package(Eigen3 3.3.0 REQUIRED) # Matched with Sophus
# Prefer the vendored Pangolin build tree when this package is built by itself.
find_package(
  Pangolin QUIET CONFIG
  PATHS
    "${PROJECT_SOURCE_DIR}/Pangolin/build"
    "${PROJECT_SOURCE_DIR}/../Pangolin/build"
  NO_DEFAULT_PATH
)
if(NOT Pangolin_FOUND)
  find_package(Pangolin REQUIRED)
endif()
find_package(image_transport REQUIRED)

if(Pangolin_FOUND AND NOT Pangolin_INCLUDE_DIRS)
  set(_pangolin_include_dirs "")
  foreach(_pangolin_target IN LISTS Pangolin_LIBRARIES)
    if(TARGET "${_pangolin_target}")
      get_target_property(_target_includes "${_pangolin_target}" INTERFACE_INCLUDE_DIRECTORIES)
      if(_target_includes)
        list(APPEND _pangolin_include_dirs ${_target_includes})
      endif()
    endif()
  endforeach()
  if(_pangolin_include_dirs)
    list(REMOVE_DUPLICATES _pangolin_include_dirs)
    set(Pangolin_INCLUDE_DIRS "${_pangolin_include_dirs}")
  endif()
  unset(_pangolin_include_dirs)
  unset(_target_includes)
  unset(_pangolin_target)
endif()

# Header file locations [C++ node]
include_directories(include) # Add .hpp, .h files from include/ros2_orb_slam3
include_directories(${EIGEN3_INCLUDE_DIRS}) # Include headers for eigen3
include_directories(${Pangolin_INCLUDE_DIRS}) # include headers for pangolin
include_directories(${OpenCV_INCLUDE_DIRS}) # include headers for pangolin

add_subdirectory(orb_slam3/Thirdparty/DBoW2)
add_subdirectory(orb_slam3/Thirdparty/g2o)

# ORB-SLAM3 includes [VSLAM library]
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o
  ${ament_INCLUDE_DIRS}
)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs   # HIL: PoseStamped for /slam/pose
  message_filters
  # your_custom_msg_interface
  cv_bridge
  image_transport
  OpenCV
  Eigen3
  Pangolin
)

#* ORB SLAM3 as a shared library
# Whenever you add a new .h, .hpp or .cc, .cpp file in the ros2_orb_slam3/orb_slam3/include and os2_orb_slam3/orb_slam3/src directories, make sure to add them here as shown below
add_library(orb_slam3_lib SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
  orb_slam3/include/System.h
  orb_slam3/include/Tracking.h
  orb_slam3/include/LocalMapping.h
  orb_slam3/include/LoopClosing.h
  orb_slam3/include/ORBextractor.h
  orb_slam3/include/ORBmatcher.h
  orb_slam3/include/FrameDrawer.h
  orb_slam3/include/Converter.h
  orb_slam3/include/MapPoint.h
  orb_slam3/include/KeyFrame.h
  orb_slam3/include/Atlas.h
  orb_slam3/include/Map.h
  orb_slam3/include/MapDrawer.h
  orb_slam3/include/Optimizer.h
  orb_slam3/include/Frame.h
  orb_slam3/include/KeyFrameDatabase.h
  orb_slam3/include/Sim3Solver.h
  orb_slam3/include/Viewer.h
  orb_slam3/include/ImuTypes.h
  orb_slam3/include/G2oTypes.h
  orb_slam3/include/CameraModels/GeometricCamera.h
  orb_slam3/include/CameraModels/Pinhole.h
  orb_slam3/include/CameraModels/KannalaBrandt8.h
  orb_slam3/include/OptimizableTypes.h
  orb_slam3/include/MLPnPsolver.h
  orb_slam3/include/GeometricTools.h
  orb_slam3/include/TwoViewReconstruction.h
  orb_slam3/include/SerializationUtils.h
  orb_slam3/include/Config.h
  orb_slam3/include/Settings.h
)

target_compile_definitions(orb_slam3_lib PRIVATE REGISTER_TIMES)

set_target_properties(orb_slam3_lib PROPERTIES VERSION "${orb_slam3_lib_VERSION}") # TODO need to findout why this is required

ament_target_dependencies(orb_slam3_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Link libraries
target_link_libraries(orb_slam3_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  DBoW2
  g2o
  -lboost_system
  -lboost_serialization
  -lcrypto 
)


# Add executable
add_executable(mono_node_cpp
  src/mono_example.cpp
  src/common.cpp
)

add_executable(stereo_node_cpp
  src/stereo_example.cpp
  src/common.cpp
)

ament_target_dependencies(mono_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(stereo_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(mono_node_cpp PUBLIC orb_slam3_lib) # Link a node with the internal shared library
target_link_libraries(stereo_node_cpp PUBLIC orb_slam3_lib)


# Install all the header files in package/package/include
install (DIRECTORY include/ 
        DESTINATION include
)

install(DIRECTORY 
  config
  dist
  launch
  node_modules
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
 )

# Install our node and library
install(TARGETS mono_node_cpp stereo_node_cpp orb_slam3_lib DBoW2 g2o
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Tell downstream packages where to find the headers
ament_export_include_directories(include)

# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  orb_slam3_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Python node
# Install Python modules, submodules
ament_python_install_package(${PROJECT_NAME}) # Install the modules in ros2_orb_slam3/ros2_orb_slam3 folder

# Install Python executibles
install(PROGRAMS
  scripts/mono_driver_node.py
  scripts/stereo_driver_node.py
  scripts/rosbag_adapter_node.py
  scripts/evaluate_mh05.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
