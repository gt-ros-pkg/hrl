#!/bin/bash

roslaunch ./pr2_param.launch &
sleep 10

gazebo ./pr2_arm.world &
sleep 15

roslaunch ./pr2_controller.launch

