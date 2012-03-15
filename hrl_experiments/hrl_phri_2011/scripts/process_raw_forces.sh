#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 data_extractor $dir/${people[$1]}_head_stitched.bag $dir/${people[$1]}_${tools[$2]}_${places[$3]}_processed_norms.bag 1 $dir/${people[$1]}_${tools[$2]}_${places[$3]}_contact_registration.bag 1
