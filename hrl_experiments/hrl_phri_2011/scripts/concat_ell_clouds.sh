#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
args=("$@")
ccargs=""
for i in `seq 4 $#`
do
    ccargs="${dir}/${people[${args[$i-1]}]}_${tools[$1]}_${places[$2]}_${functions[$3]}_ell_face_data_cloud.bag ${ccargs}"
done
set -x
rosrun hrl_phri_2011 concat_clouds /data_cloud $ccargs ${dir}/${tools[$1]}_${places[$2]}_${functions[$3]}_concat_ell_clouds.bag
