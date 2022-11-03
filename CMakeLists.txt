cmake_minimum_required(VERSION 3.0.2)
project(perception-heightmap-old)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#add_subdirectory(oldmap_msgs)
add_subdirectory(oldmap_core)
add_subdirectory(oldmap_ros)
add_subdirectory(oldmap_rviz_plugin)
