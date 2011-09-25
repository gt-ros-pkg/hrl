#!/bin/bash 
dir=`rospack find hrl_phri_2011`/data
set -x
rosbag record -l 1 /stitched_head -O $dir/$1_head_stitched.bag
