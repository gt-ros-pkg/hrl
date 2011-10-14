#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
args=("$@")
ccargs=""
study_users=("4" "5" "7" "8" "9" "10" "11" "13")
num_users=${#study_users[@]}
#multipliers=( "0.6050" "0.4263" "0.2705" "0.3242" "0.2679" "0.8833" "0.3389" "0.5271" )
multipliers=( "1" "1" "1" "1" "1" "1" "1" "1" )
set -x
for (( i=0; i<${num_users}; i++ ));
do
    rosrun hrl_phri_2011 extract_ell_face_function.sh ${study_users[$i]} $1 $2 $3 ${multipliers[$i]}
done
rosrun hrl_phri_2011 density_est_all.sh $1 $2 $3 ${study_users[*]}
#rosrun hrl_phri_2011 concat_ell_face_clouds.sh $1 $2 $3 ${study_users[*]}
#rosrun hrl_phri_2011 color_combo_face_cloud.sh $1 $2 $3 
