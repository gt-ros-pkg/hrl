#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
treatments=( "train" "1" "2" "3" "4" )
set -x
rosrun hrl_phri_2011 convert_pr2_data_to_mat.py $dir/${people[$1]}_${tools[$2]}_pr2_treatment_${treatments[$3]}.bag
