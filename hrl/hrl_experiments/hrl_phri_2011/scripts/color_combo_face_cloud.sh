#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
types=("cheek_contact_cloud" "nose_contact_cloud" "chin_contact_cloud" "concat_contact_clouds" )
set -x
rosrun hrl_phri_2011 colorize_data_cloud ${dir}/${tools[$1]}_${places[$2]}_${functions[$3]}_concat_ell_face_clouds.bag ${dir}/${tools[$1]}_${places[$2]}_${functions[$3]}_concat_ell_face_clouds_colored.bag
