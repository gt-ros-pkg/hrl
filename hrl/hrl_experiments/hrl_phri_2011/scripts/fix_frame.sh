#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 fix_gravity.py $dir/${people[$1]}_${tools[$2]}_${places[$3]}.bag
