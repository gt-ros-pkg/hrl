cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable (bin/pcd_read src/pcd_read.cpp)
target_link_libraries (bin/pcd_read pcl_io)

rosbuild_add_executable (bin/filter_voxel_grid src/filter_voxel_grid.cpp)
target_link_libraries (bin/filter_voxel_grid pcl_io pcl_filters)

rosbuild_add_executable (bin/og_sample
                         src/occupancy_grid.cpp
                         src/examples/og_sample.cpp)

rosbuild_add_executable (bin/og_difference
                         src/occupancy_grid.cpp
                         src/examples/og_difference.cpp)

rosbuild_add_executable (bin/og_to_pc
                         src/occupancy_grid.cpp
                         src/og_to_pc.cpp)

rosbuild_add_executable (bin/pc_to_og
                         src/occupancy_grid.cpp
                         src/pc_to_og.cpp)



