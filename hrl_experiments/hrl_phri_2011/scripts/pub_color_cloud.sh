#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
types=("cheek_contact_cloud" "nose_contact_cloud" "chin_contact_cloud" "concat_contact_clouds" )
set -x
rosrun hrl_phri_2011 pub_head ${dir}/${people[$1]}_${tools[$2]}_${types[$3]}_colored.bag /contact_cloud /base_link 1
