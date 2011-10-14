#!/bin/bash 
pkg=`rospack find hrl_phri_2011`
source $pkg/scripts/variables.sh
set -x
for i in `seq 0 4`
do
    rosrun hrl_phri_2011 convert_pr2_forces.sh $1 0 ${i}
done
