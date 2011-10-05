#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
mv $dir/${people[$1]}_${tools[$2]}_${places[$3]}.bag $dir/${people[$1]}_${tools[$2]}_${places[$3]}.bag.orig
rosbag fix $dir/${people[$1]}_${tools[$2]}_${places[$3]}_fixed2.bag $dir/${people[$1]}_${tools[$2]}_${places[$3]}.bag
