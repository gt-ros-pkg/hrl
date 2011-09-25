#!/bin/bash 
dir=`rospack find hrl_phri_2011`/data
set -x
rosbag record /ellipsoid_params -l 1 -O $dir/$1_ellipsoid_registration.bag
