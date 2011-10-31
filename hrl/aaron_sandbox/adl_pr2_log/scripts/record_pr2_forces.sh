#!/bin/bash -x
rosbag record /adl_wrench_posearray /netft_gravity_zeroing/wrench_zeroed /netft_gravity_zeroing/wrench_markers /pr2_netft/wrench_raw /frame/ellipse_frame /frame/l_gripper_shaver45_frame /frame/torso_lift_link -O $1
