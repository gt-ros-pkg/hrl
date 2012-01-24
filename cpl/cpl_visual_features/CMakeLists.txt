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

#find_package(opencv2)

# Build the libraries
rosbuild_add_library(cpl_visual_features_base
  src/features/gabor_filter_bank.cpp
  src/features/lm_filter_bank.cpp)

rosbuild_add_library(cpl_visual_features_saliency src/saliency/center_surround.cpp)

rosbuild_add_library(cpl_visual_features_motion src/motion/dense_lk.cpp src/motion/feature_tracker.cpp)

# Build the executables
rosbuild_add_executable(test_offline src/test_offline.cpp)

rosbuild_add_executable(sliding_window  src/sliding_window.cpp)

rosbuild_add_executable(texture_test  src/texture_test.cpp)
rosbuild_add_executable(train_textons src/train_textons.cpp)
rosbuild_add_executable(run_saliency src/run_saliency.cpp)
rosbuild_add_executable(train_sift_codebook src/sift_des_codebook_generation.cpp)
rosbuild_add_executable(saliency_depth_node src/depth_saliency_node.cpp)
rosbuild_add_executable(feature_tracker src/feature_tracker.cpp)

target_link_libraries(sliding_window cpl_visual_features_base cpl_visual_features_saliency)
target_link_libraries(texture_test cpl_visual_features_base cpl_visual_features_saliency)
target_link_libraries(train_textons cpl_visual_features_base cpl_visual_features_saliency)
target_link_libraries(run_saliency cpl_visual_features_base cpl_visual_features_saliency)
target_link_libraries(saliency_depth_node cpl_visual_features_base cpl_visual_features_saliency)
target_link_libraries(train_sift_codebook cpl_visual_features_base)