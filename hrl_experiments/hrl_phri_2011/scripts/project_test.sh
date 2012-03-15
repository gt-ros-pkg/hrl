#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 pub_head $dir/${people[$1]}_head_stitched.bag /stitched_head /base_link 1 &
rosrun hrl_phri_2011 project_pc_ellipsoid $dir/${people[$1]}_head_stitched.bag $dir/${people[$1]}_ellipsoid_registration.bag $dir/${people[$1]}_${tools[$2]}_${places[$3]}_processed.bag
