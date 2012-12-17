#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x

noise=0.001;

if [ $1 == 0 ]; then
    # subjects for wiping:
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
    study_users=( "${study_users_shaving[@]}" )
    multipliers=( "${shaving_multipliers[@]}" )
fi

num_users=${#study_users[@]}

out_bags="";
set -x
for (( i=0; i<${num_users}; i++ ));
do
    echo $i
    in_bag=$dir/${people[${study_users[$i]}]}_${tools[$1]}_${places[$2]}_processed_norms.bag;
    out_bag=$dir/${people[${study_users[$i]}]}_${tools[$1]}_${places[$2]}_${functions[$3]}_ell_data_cloud_noisy.bag;
    rosrun hrl_phri_2011 function_extractor $in_bag ${functions[$3]} $out_bag $dir/sub1_ellipsoid_registration.bag $dir/${people[4]}_head_stitched.bag $noise _force_thresh:=0.5 _time_thresh:=0.2 _multiplier:=${multipliers[$i]}
    out_bags="${out_bags} ${out_bag}";
done

rosrun hrl_phri_2011 concat_clouds /data_cloud $out_bags ${dir}/${tools[$1]}_${places[$2]}_concat_noisy_clouds.bag
