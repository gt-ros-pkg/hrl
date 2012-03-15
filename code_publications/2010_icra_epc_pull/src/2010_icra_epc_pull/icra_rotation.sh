#!/bin/bash

if [ "$1" = "" ]; then
    echo Give an experiment number
    exit
fi

if [ "$2" = "" ]; then
    echo Give a position number [1,2,3,4]
    exit
fi


if [ "$3" = "" ]; then
    echo Give a hook angle
    exit
fi

if [ "$4" = "" ]; then
    echo Give info string
    exit
fi

if [ "$5" = "" ]; then
    echo Give zenither height
    exit
fi




mkdir $1
python kinematics_estimator_least_squares.py &
python hook_and_pull.py --lpi --ha=$3 --info $4 -p$2 -z$5
kill %
mv hook_plane_scan*.pkl pull_trajectories_*.pkl pose_dict*.pkl mechanism_trajectories_robot*.pkl $1



