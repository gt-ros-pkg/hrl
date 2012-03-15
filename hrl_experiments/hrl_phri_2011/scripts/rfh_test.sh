#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 pub_head $dir/${people[$1]}_head_stitched.bag /stitched_head /base_link 1 &
rosrun hrl_phri_2011 interactive_ellipse base_link ellipse_frame 20 $dir/${people[$2]}_ellipsoid_registration.bag $dir/${people[$1]}_ellipsoid_registration.bag 1 &
rosrun hrl_phri_2011 ellipsoid_visualizer $dir/${people[$2]}_head_stitched.bag $dir/${people[$2]}_ellipsoid_registration.bag
