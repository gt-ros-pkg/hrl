#!/bin/bash -x

# uses avi_to_pngs.sh and pngs_to_pose_pkl.sh to go from avis to the
# pkls with force, mechanism angle etc. for multiple mechanisms.
# 
# This script will take a very long time to run. You want to let it
# run overnight. (it can take longer than that too)

#
# Usage: ./avi_to_pkls_multiple_mechanism.sh <mechanism 1 directory> <directory 2> ...
#

ARGC=$#

if [ $ARGC = 0 ]; then
    echo "Usage: ./avi_to_pkls_multiple_mechanism.sh <mechanism 1 directory> <directory 2> ..."
    exit
fi

for d in $@
do
    ./avi_to_pngs.sh $d
done

for d in $@
do
    ./pngs_to_pose_pkl.sh $d
done


