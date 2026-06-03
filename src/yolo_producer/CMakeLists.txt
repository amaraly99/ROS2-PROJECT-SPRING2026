cmake_minimum_required(VERSION 3.16)
project(yolo_producer CXX)
set(CMAKE_CXX_STANDARD 17)

# This project is a host-side Python program.
# CMake only compiles yolo_shm_test (a quick sanity check for the header).
# The actual producer is yolo_producer.py — run directly with Python.

add_executable(yolo_shm_test src/shm_test.cpp)
target_include_directories(yolo_shm_test PRIVATE include)
target_link_libraries(yolo_shm_test rt pthread)
target_compile_options(yolo_shm_test PRIVATE -O2)
