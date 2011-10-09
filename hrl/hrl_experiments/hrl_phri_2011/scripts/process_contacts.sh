#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 data_extractor $dir/${people[$1]}_head_stitched.bag $dir/${people[$1]}_${tools[$2]}_${places[$3]}_contacts.bag 0
rosrun hrl_phri_2011 save_contact_cloud.sh $1 $2 $3 
rosrun hrl_phri_2011 color_cloud.sh $1 $2 $3 
