#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosbag record -l 1 /stitched_head -O $dir/${people[$1]}_head_stitched.bag
