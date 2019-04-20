cmake_minimum_required(VERSION 2.8.3)
project(vesc_msgs)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_genmsg()
