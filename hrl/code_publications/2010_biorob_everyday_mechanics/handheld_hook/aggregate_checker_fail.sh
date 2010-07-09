#!/bin/bash -x

ARGC=$#
if [ $ARGC = 0 ]; then
    echo "Usage: ./aggregate_checker_fail.sh <mechanism 1 directory> <directory 2> ..."
    exit
fi

mkdir aggregated_checker_fail

for d in $@
do
    for f in `ls $d/*/ -d`
    do
        python log_images.py -d $f -b
        cp $f/checker_fail/* aggregated_checker_fail
    done
done


