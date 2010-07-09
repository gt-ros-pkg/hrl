#!/bin/bash -x
#
# Usage: ./annotate_multiple.sh <mechanism 1 directory> <directory 2> ...
#

ARGC=$#

if [ $ARGC = 0 ]; then
    echo "Usage: ./annotate_multiple.sh <mechanism 1 directory> <directory 2> ..."
    exit
fi

for d in $@
do
    python annotate_images.py -d $d
done

