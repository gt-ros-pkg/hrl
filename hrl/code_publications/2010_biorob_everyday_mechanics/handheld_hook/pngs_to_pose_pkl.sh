#!/bin/bash -x

if [ "$1" = "" ]; then
    echo Give mechanism name
    exit
fi

A=`pwd`

#roslaunch sturm_kinematics.launch &
#sleep 30
#roslaunch modeling_forces checkerboard.launch &
#sleep 30

for f in `ls $1/0*/ -d`
do
    if [ -f $f/poses_dict.pkl ]
    then
        echo $f already has poses from pngs
    else
        cd $f
        python $A/log_images.py -d ./ -p &
        python $A/checkerboard_poses_to_pickle.py
        cd $A
    fi
    python analyse_logs.py --time_check --savefig --dir $f
    python analyse_logs.py --sync --dir $f
    python analyse_logs.py --split --dir $f
#    python analyse_logs.py -c --savefig --dir $f
#    python analyse_logs.py -f --savefig --dir $f
done


#killall python
#sleep 30


