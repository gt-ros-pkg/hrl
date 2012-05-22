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
rosrun hrl_phri_2011 gray_reg_head $dir/sub1_head_stitched.bag $dir/sub1_ellipsoid_registration.bag $dir/sub1_head_stitched_gray.bag
rosrun hrl_phri_2011 density_estimation $dir/sub1_head_stitched_gray.bag $ccargs ${dir}/${tools[$1]}_${places[$2]}_${functions[$3]}_density_est.bag _target_force:=0.5 _pilot_ph:=$posh _pilot_fh:=0.1 _percent_trim:=$ptrim _use_min:=1
