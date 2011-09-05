#!/bin/bash -x
rosbag record /tf /tool_netft/wrench_raw /tool_netft_zeroer/wrench_markers /tool_netft_zeroer/wrench_zeroed /head/pose /adl2/pose -O $1
