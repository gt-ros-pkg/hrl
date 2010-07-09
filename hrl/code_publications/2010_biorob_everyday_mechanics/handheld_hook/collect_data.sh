#!/bin/bash -x

if [ "$1" = "" ]; then
    echo Give a mechanism name
    exit
fi

if [ "$2" = "" ]; then
    echo enter number of trials
    exit
fi


A=`pwd`
mkdir $1
cd $1

if [ -f mechanism_info.pkl ]
then
    echo You have already entered mechanism details for $1
else
    python $A/initialise_experiment.py
fi

NMECH=0
for f in `ls */ -d`
do
    let NMECH=$NMECH+1
done

N=0
while [ $N -lt $2 ]
do
    let N=$N+1
    let NMECH=$NMECH+1
    echo =========================================
    echo OPEN, WAIT, AND THEN CLOSE THE MECHANISM
    echo =========================================
    D=`printf "%04d" $NMECH`
    mkdir $D
    cd $D
    python $A/log_ft_data.py -l -r &
    python $A/log_images.py -l
    cp ../mechanism_info.pkl .
    cd -
    sleep 3
done



