cmake_minimum_required(VERSION 2.8.3)
project(vesc_ackermann)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###########
## Build ##
###########

include_directories(include)

SET(libs roslib roscpp boost_system)

# node executable
rosbuild_add_executable(vesc_to_odom_node
                        src/vesc_to_odom_node.cpp
                        src/vesc_to_odom.cpp)
target_link_libraries(vesc_to_odom_node ${libs})

rosbuild_add_executable(ackermann_to_vesc_node
                        src/ackermann_to_vesc_node.cpp
                        src/ackermann_to_vesc.cpp)
target_link_libraries(ackermann_to_vesc_node ${libs})

# nodelet library
rosbuild_add_library(vesc_ackermann_nodelet
                     src/ackermann_to_vesc_nodelet.cpp
                     src/ackermann_to_vesc.cpp
                     src/vesc_to_odom_nodelet.cpp
                     src/vesc_to_odom.cpp)
target_link_libraries(vesc_ackermann_nodelet ${libs})
