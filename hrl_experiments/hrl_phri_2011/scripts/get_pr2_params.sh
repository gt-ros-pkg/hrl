#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosrun hrl_phri_2011 analyze_pr2_interaction.py $dir/${people[$1]}_${tools[$2]}_pr2_treatment_train.bag
