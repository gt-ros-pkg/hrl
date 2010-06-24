cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable (bin/pcd_read src/pcd_read.cpp)
target_link_libraries (bin/pcd_read pcl_io)

