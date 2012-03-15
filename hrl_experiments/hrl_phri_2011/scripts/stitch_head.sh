#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
roslaunch hrl_phri_2011 pc_head_stitcher.launch bag:=$dir/${people[$1]}_pc_captures.bag
