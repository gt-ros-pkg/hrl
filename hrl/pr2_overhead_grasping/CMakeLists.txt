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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(sensor_filter src/sensor_filter.cpp)
#rosbuild_add_executable(collision_monitor_node src/collision_monitor_node.cpp)
#rosbuild_add_library(random_forest src/random_forest.cpp)
rosbuild_add_library(collision_detection src/sensor_filter.cpp)
#rosbuild_add_library(collision_detection src/random_forest.cpp)
#rosbuild_add_library(collision_detection src/random_forest.cpp)
rosbuild_add_library(collision_detection src/collision_monitor.cpp)
rosbuild_add_library(collision_detection src/force_torque_monitor.cpp)
rosbuild_add_compile_flags (collision_detection ${SSE_FLAGS})
rosbuild_add_openmp_flags (collision_detection)
rosbuild_add_executable(random_forest src/random_forest.cpp)
rosbuild_add_compile_flags (random_forest ${SSE_FLAGS})
rosbuild_add_openmp_flags (random_forest)
rosbuild_add_executable(mahalanobis_dist src/mahalanobis_dist.cpp)
rosbuild_add_compile_flags (mahalanobis_dist ${SSE_FLAGS})
rosbuild_add_openmp_flags (mahalanobis_dist)
#rosbuild_add_link_flags(random_forest "-pg")
#rosbuild_add_compile_flags(random_forest "-pg")

rosbuild_add_executable(sensor_filter_visualizer src/sensor_filter_visualizer.cpp)
