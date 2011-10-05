#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosbag play $dir/${people[$1]}_${tools[$2]}_${places[$3]}_fixed.bag --topics /tf /adl2/pose /head/pose /tool_netft/wrench_raw
