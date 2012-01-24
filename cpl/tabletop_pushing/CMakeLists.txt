cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
# set(ROS_BUILD_TYPE Debug)

# Build actions
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
#find_package(opencv2)

rosbuild_add_library(graphcut include/tabletop_pushing/extern/graphcut/graph.cpp include/tabletop_pushing/extern/graphcut/maxflow.cpp include/tabletop_pushing/extern/graphcut/LinkedBlockList.cpp include/tabletop_pushing/extern/graphcut/GCoptimization.cpp)
# rosbuild_add_executable(tabletop_pushing_perception_node src/tabletop_pushing_perception_node.cpp)
rosbuild_add_executable(object_singulation_node src/object_singulation_node.cpp)
# rosbuild_add_executable(region_tracking_node src/region_tracking.cpp)
# rosbuild_add_executable(bag_extractor src/extract_bag_images.cpp)
# rosbuild_add_executable(tabletop_segmenter_node src/ransac_segmenter.cpp)
# rosbuild_add_executable(offline_segmenter_node src/offline_segmenter.cpp)
# target_link_libraries(tabletop_pushing_perception_node graphcut)
