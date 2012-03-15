#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
args=("$@")
ccargs=""
study_users="4 5 7 8 9 10 11 13"
set -x
for user in `echo $study_users`
do
    rosrun hrl_phri_2011 extract_ell_function.sh $user $1 $2 $3
#ccargs="${dir}/${people[${args[$i-1]}]}_${tools[$1]}_${places[$2]}_${functions[$3]}_ell_data_cloud.bag ${ccargs}"
done
rosrun hrl_phri_2011 concat_ell_clouds.sh $1 $2 $3 $study_users
rosrun hrl_phri_2011 color_combo_cloud.sh $1 $2 $3 
