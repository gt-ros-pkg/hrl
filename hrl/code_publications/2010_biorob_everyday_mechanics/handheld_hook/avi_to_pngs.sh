#!/bin/bash -x

if [ "$1" = "" ]; then
    echo Give mechanism name
    exit
fi


for f in `ls $1/*/ -d`
do
    if [ -f $f/00000.png ]
    then
        echo $f already has pngs from avi
    else
        python log_images.py -d $f -c
    fi
done


