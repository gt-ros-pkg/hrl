#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosbag record /ellipsoid_params -l 1 -O $dir/${people[$1]}_ellipsoid_registration.bag
