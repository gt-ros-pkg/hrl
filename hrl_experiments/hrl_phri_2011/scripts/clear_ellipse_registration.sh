#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 interactive_ellipse base_link ellipse_frame 20 $dir/${people[$1]}_ellipsoid_registration.bag
