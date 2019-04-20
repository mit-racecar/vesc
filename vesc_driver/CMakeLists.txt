cmake_minimum_required(VERSION 2.8.3)
project(vesc_driver)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###########
## Build ##
###########

include_directories(include)

SET(libs roslib roscpp)

# node executable
rosbuild_add_executable(vesc_driver_node
                        src/vesc_driver_node.cpp
                        src/vesc_driver.cpp
                        src/vesc_interface.cpp
                        src/vesc_packet.cpp
                        src/vesc_packet_factory.cpp)
target_link_libraries(vesc_driver_node ${libs})

# nodelet library
rosbuild_add_library(vesc_driver_nodelet
                     src/vesc_driver_nodelet.cpp
                     src/vesc_driver.cpp
                     src/vesc_interface.cpp
                     src/vesc_packet.cpp
                     src/vesc_packet_factory.cpp)
target_link_libraries(vesc_driver_nodelet ${libs})
