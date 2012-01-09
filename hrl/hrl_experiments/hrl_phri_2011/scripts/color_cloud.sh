#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
types=("cheek_contact_cloud" "nose_contact_cloud" "chin_contact_cloud" "all_contact_cloud" )
set -x
rosrun hrl_phri_2011 colorize_data_cloud ${dir}/${people[$1]}_${tools[$2]}_${types[$3]}.bag ${dir}/${people[$1]}_${tools[$2]}_${types[$3]}_colored.bag
