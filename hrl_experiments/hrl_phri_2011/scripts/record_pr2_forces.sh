#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
rosbag record /tf /head/pose /pr2_antenna/pose /l_cart/x_err /l_cart/state/xd /l_cart/state/x /l_cart/state/tau /l_cart/sensor_raw_ft /l_cart/sensor_ft /l_cart/qd /l_cart/k_effective /l_cart/f_err /l_cart/f_cmd -O $dir/${people[$1]}_${tools[$2]}_pr2_treatment_$3.bag
