cmake_minimum_required(VERSION 3.16)
project(ovcam_producer CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LC REQUIRED libcamera)  # finds libcamera-dev

add_executable(ovcam_producer src/producer.cpp)

target_include_directories(ovcam_producer PRIVATE
    include          # finds ovcam_shm.hpp
    ${LC_INCLUDE_DIRS}
)
target_link_libraries(ovcam_producer
    ${LC_LIBRARIES}
    rt       # shm_open, sem_open
    pthread  # std::thread
)
target_compile_options(ovcam_producer PRIVATE -O3 -march=native)
