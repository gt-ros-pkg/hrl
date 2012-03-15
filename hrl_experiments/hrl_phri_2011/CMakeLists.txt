cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#find_package(SuiteSparse REQUIRED)
#include_directories(${CHOLMOD_INCLUDE_DIR} REQUIRED)
#MESSAGE(STATUS "include = ${CHOLMOD_INCLUDE_DIR}")
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

set(PROGRAMS pub_head ellipsoid_test interactive_tf interactive_ellipse ellipsoid_visualizer project_pc_ellipsoid data_extractor pc_head_stitcher function_extractor show_contact_cloud save_contact_cloud concat_clouds colorize_data_cloud density_estimation gray_reg_head)
#set(PROGRAMS ${PROGRAMS} gp_regression)
#rosbuild_add_executable(force_labeler src/force_labeler.cpp)
#rosbuild_add_executable(visualize_head src/visualize_head.cpp)
rosbuild_add_library(${PROJECT_NAME} src/pcl_basic.cpp src/pcl_features.cpp src/utils.cpp)

foreach(program ${PROGRAMS})
    rosbuild_add_executable(${program} src/${program}.cpp)
    target_link_libraries(${program} ${PROJECT_NAME})
endforeach(program)
#target_link_libraries(gp_regression ${CHOLMOD_LIBRARY})
