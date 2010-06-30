#!/bin/sh
rxplot -b 20 /accelerometer/l_gripper_motor/samples[0]/x:y:z &
rxplot -b 20 /accelerometer/r_gripper_motor/samples[0]/x:y:z &
roslaunch omni_teleop stereo_omni_pr2.launch
