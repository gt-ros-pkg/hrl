#!/bin/bash
source /opt/ros/electric/setup.bash
export ROS_PACKAGE_PATH="/home/pgrice/svn/gt-ros-pkg/hrl/wouse:$ROS_PACKAGE_PATH"
export ROS_MASTER_URI=http://monty1.hsi.gatech.edu:11311
env
python ~/svn/gt-ros-pkg/hrl/wouse/scripts/reset_wouse.py
