#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 function_extractor $dir/${people[$1]}_${tools[$2]}_${places[$3]}_processed_norms.bag ${functions[$4]} $dir/${people[$1]}_${tools[$2]}_${places[$3]}_${functions[$4]}_ell_face_data_cloud.bag $dir/sub1_ellipsoid_registration.bag $dir/${people[4]}_head_stitched.bag _force_thresh:=0.5 _time_thresh:=0.2 _multiplier:=$5
