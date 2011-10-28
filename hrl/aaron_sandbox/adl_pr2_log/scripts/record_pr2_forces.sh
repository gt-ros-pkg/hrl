#!/bin/bash -x
rosbag record /adl_wrench_posearray /netft_gravity_zeroing/wrench_zeroed /netft_gravity_zeroing/wrench_markers -O $1
