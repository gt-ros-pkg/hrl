#!/bin/bash

if [ "$1" = "" ]; then
    echo Give an experiment number
    exit
fi

mkdir $1
python ../handheld_hook/initialise_experiment.py
python collect_data.py
mv mechanism_info*.pkl poses_dict.pkl ft_log*.pkl spring_scale*.pkl $1
mv *.png $1



