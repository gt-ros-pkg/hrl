#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
args=("$@")
ccargs=""
for i in `seq 2 $#`
do
    ccargs="${dir}/${people[$1]}_${tools[$2]}_${places[${args[$i]}]}_contact_cloud.bag ${ccargs}"
done
set -x
rosrun hrl_phri_2011 concat_clouds /contact_cloud $ccargs ${dir}/${people[$1]}_${tools[$2]}_concat_contact_clouds.bag
