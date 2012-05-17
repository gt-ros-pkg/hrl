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
    multipliers=( "${shaver_multipliers[@]}" )
fi

#multipliers=( "1" "1" "1" "1" "1" "1" "1" "1" )

num_users=${#study_users[@]}
set -x
for (( i=0; i<${num_users}; i++ ));
do
    rosrun hrl_phri_2011 function_extractor $dir/${people[${study_users[$i]}]}_${tools[$1]}_${places[$2]}_processed_norms.bag ${functions[$3]} $dir/${people[${study_users[$i]}]}_${tools[$1]}_${places[$2]}_${functions[$3]}_ell_face_data_cloud.bag $dir/${people[${study_users[$i]}]}_${tools[$1]}_${places[$2]}_${functions[$3]}_ell_face_data_cloud_nonproj.bag _force_thresh:=0.5 _time_thresh:=0.2 _multiplier:=${multipliers[$i]}
done
rosrun hrl_phri_2011 new_density_est_all.sh $1 $2 $3 $posh $ptrim ${study_users[*]}
#rosrun hrl_phri_2011 concat_ell_face_clouds.sh $1 $2 $3 ${study_users[*]}
#rosrun hrl_phri_2011 color_combo_face_cloud.sh $1 $2 $3 
