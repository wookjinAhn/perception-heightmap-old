cmake_minimum_required(VERSION 3.2.0)
project(oldmap_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
    Oldmap.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
#    INCLUDE_DIRS
#        include
    #LIBRARIES
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        std_msgs
    #DEPENDS
)

