cmake_minimum_required(VERSION 3.0.2)
project(camel-perception-heightmap)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#add_subdirectory(heightmap_msgs)
add_subdirectory(heightmap_core)
add_subdirectory(heightmap_ros)
add_subdirectory(heightmap_rviz_plugin)
