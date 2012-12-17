#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
args=("$@")
ccargs=""

if [ $1 == 0 ]; then
    # subjects for wiping:
    posh=0.02
    ptrim=0.50
    study_users=( "${study_users_wiping[@]}" )
    echo $study_users
    if [ $2 == 0 ]; then
        multipliers=( "${cheek_multipliers[@]}" )
    fi
    if [ $2 == 1 ]; then
        multipliers=( "${nose_multipliers[@]}" )
    fi
    if [ $2 == 2 ]; then
        multipliers=( "${chin_multipliers[@]}" )
    fi
else
    # subjects for shaving:
    posh=0.02
    ptrim=0.50
    study_users=( "${study_users_shaving[@]}" )
    multipliers=( "${shaving_multipliers[@]}" )
fi

#multipliers=( "1" "1" "1" "1" "1" "1" "1" "1" )

num_users=${#study_users[@]}
set -x
for (( i=0; i<${num_users}; i++ ));
do
    rosrun hrl_phri_2011 extract_ell_face_function.sh ${study_users[$i]} $1 $2 $3 ${multipliers[$i]}
done
rosrun hrl_phri_2011 density_est_all.sh $1 $2 $3 ${study_users[*]}
#rosrun hrl_phri_2011 concat_ell_face_clouds.sh $1 $2 $3 ${study_users[*]}
#rosrun hrl_phri_2011 color_combo_face_cloud.sh $1 $2 $3 
