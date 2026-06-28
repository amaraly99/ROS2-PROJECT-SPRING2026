cmake_minimum_required(VERSION 3.5)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(INSTALL_INCLUDE_DIR "include")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

find_package(OpenCV 3.0 REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
)


set (H_FILES
include/System.h
include/BenchmarkUtils.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/ORBVocabulary.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Map.h
include/Optimizer.h
include/PnPsolver.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Initializer.h
)

set (SRC_FILES
src/System.cc
src/BenchmarkUtils.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
)


add_library(${PROJECT_NAME} SHARED
${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
DBoW2
g2o
)


# Set standard installation directories
set(RUNTIME_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
set(LIBRARY_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(INCLUDES_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
set(INCLUDES_INSTALL_DIR ${INCLUDES_DESTINATION}/ORB_SLAM2)


# This relative path allows installed files to be relocatable.
set( CMAKECONFIG_INSTALL_DIR lib/cmake/${PROJECT_NAME} )
file( RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
    "${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}"
)

set( EXPORT_LIB_INC_DIR "${PROJECT_SOURCE_DIR};${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )

configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}Config.cmake.in
${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY IMMEDIATE )

# Install tree config
set( EXPORT_LIB_INC_DIR "\${PROJECT_CMAKE_DIR}/${REL_INCLUDE_DIR}/${PROJECT_NAME}" )
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}Config.cmake.in
${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake @ONLY )

export( PACKAGE ${PROJECT_NAME} )


install(FILES ${H_FILES}
DESTINATION ${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}/ORB_SLAM2
)
install(DIRECTORY Thirdparty
DESTINATION ${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}/ORB_SLAM2
)


install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  INCLUDES DESTINATION ${INCLUDES_DESTINATION}
)

## Install CMake config
install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)
