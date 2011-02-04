cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
rosbuild_gensrv()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

# Library
rosbuild_add_library(explore_lib src/explore_frontier.cpp src/Vector.cpp src/Line.cpp src/Path.cpp src/loop_closure.cpp)
set_target_properties(explore_lib
                      PROPERTIES OUTPUT_NAME "explore")
rosbuild_link_boost(explore_lib thread)

rosbuild_add_library(explore_action_lib src/explore_frontier.cpp src/Vector.cpp src/Line.cpp src/Path.cpp src/loop_closure.cpp src/explore.cpp)
set_target_properties(explore_action_lib
                      PROPERTIES OUTPUT_NAME "explore_action")
rosbuild_link_boost(explore_action_lib thread)


# Explore
rosbuild_add_executable(explore src/explore.cpp)
rosbuild_add_executable(explore_action src/explore_action.cpp)

target_link_libraries(explore explore_lib)
target_link_libraries(explore_action explore_action_lib)
