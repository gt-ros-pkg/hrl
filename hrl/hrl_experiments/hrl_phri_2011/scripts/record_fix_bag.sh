#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosbag record /adl2/pose /head/pose /tf /tool_netft/wrench_raw /tool_netft_zeroer/wrench_markers /tool_netft_zeroer/wrench_zeroed -O $dir/${people[$1]}_${tools[$2]}_${places[$3]}_fixed2.bag &
roslaunch hrl_netft netft_zero_fix.launch tool:=${tools[$2]}
