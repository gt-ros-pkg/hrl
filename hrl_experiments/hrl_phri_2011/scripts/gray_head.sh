#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 gray_reg_head $dir/${people[$1]}_head_stitched.bag $dir/sub1_ellipsoid_registration.bag $dir/${people[$1]}_head_stitched_gray.bag
