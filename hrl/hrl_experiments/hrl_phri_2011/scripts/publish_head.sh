#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 pub_head $dir/${people[$1]}_head_stitched.bag /stitched_head /base_link 20
