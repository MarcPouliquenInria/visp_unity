project(cubeTrackingMbtAprilTag)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_sensor visp_gui visp_vision visp_mbt visp_detection)

include_directories(${VISP_INCLUDE_DIRS})

add_executable(mbtCubeTracking main.cpp)
target_link_libraries(mbtCubeTracking ${VISP_LIBRARIES})

