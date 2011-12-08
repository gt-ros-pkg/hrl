#!/bin/bash

# NOT TRUE ANY MORE:
# $1 => tagid stripped (eg. 'person')
# $2 => capture number (eg. 0 for "best position")
# $3 => actual tagid (eg. 'person      ')
# $4 => explore_radius in cm (eg. 400)

# python sm_head_capture.py --fname hrl_$4_$1_$2_headpre
# ./sm_servo_capture.py --fname hrl_$4_$1_$2 --tag "$3"  # automatically adds _servo
# python sm_head_capture.py --fname hrl_$4_$1_$2_headpost


# $1 => search rotation #
# $2 => object #

# woot_150_0_tag_CordlessPhon.yaml

if [ "$1" == "0" ]; then
    echo 'Processing object 0'
    yaml="woot_150_$2_tag_OrangeMedBot.yaml"
    headpre="woot_150_$2_tag_OrangeMedBot_headpre"
    headpost="woot_150_$2_tag_OrangeMedBot_headpost"
    bag="woot_150_$2_tag_OrangeMedBot"
    tagid="OrangeMedBot"
elif [ "$1" == "1" ]; then
    echo 'Processing object 1'
    yaml="woot_150_$2_tag_TravisTVremo.yaml"
    headpre="woot_150_$2_tag_TravisTVremo_headpre"
    headpost="woot_150_$2_tag_TravisTVremo_headpost"
    bag="woot_150_$2_tag_TravisTVremo"
    tagid="TravisTVremo"
elif [ "$1" == "2" ]; then
    echo 'Processing object 2'
    yaml="woot_150_$2_tag_RedBottle.yaml"
    headpre="woot_150_$2_tag_RedBottle_headpre"
    headpost="woot_150_$2_tag_RedBottle_headpost"
    bag="woot_150_$2_tag_RedBottle"
    tagid="RedBottle   "
elif [ "$1" == "3" ]; then
    echo 'Processing object 3'
    yaml="woot_150_$2_tag_OnMetalKeys.yaml"
    headpre="woot_150_$2_tag_OnMetalKeys_headpre"
    headpost="woot_150_$2_tag_OnMetalKeys_headpost"
    bag="woot_150_$2_tag_OnMetalKeys"
    tagid="OnMetalKeys "
elif [ "$1" == "4" ]; then
    echo 'Processing object 4'
    yaml="woot_150_$2_tag_WhiteMedsBot.yaml"
    headpre="woot_150_$2_tag_WhiteMedsBot_headpre"
    headpost="woot_150_$2_tag_WhiteMedsBot_headpost"
    bag="woot_150_$2_tag_WhiteMedsBot"
    tagid="WhiteMedsBot"
elif [ "$1" == "5" ]; then
    echo 'Processing object 5'
    yaml="woot_150_$2_tag_BlueMedsBox.yaml"
    headpre="woot_150_$2_tag_BlueMedsBox_headpre"
    headpost="woot_150_$2_tag_BlueMedsBox_headpost"
    bag="woot_150_$2_tag_BlueMedsBox"
    tagid="BlueMedsBox "
elif [ "$1" == "6" ]; then
    echo 'Processing object 6'
    yaml="woot_150_$2_tag_TeddyBearToy.yaml"
    headpre="woot_150_$2_tag_TeddyBearToy_headpre"
    headpost="woot_150_$2_tag_TeddyBearToy_headpost"
    bag="woot_150_$2_tag_TeddyBearToy"
    tagid="TeddyBearToy"
elif [ "$1" == "7" ]; then
    echo 'Processing object 7'
    yaml="woot_150_$2_tag_CordlessPhon.yaml"
    headpre="woot_150_$2_tag_CordlessPhon_headpre"
    headpost="woot_150_$2_tag_CordlessPhon_headpost"
    bag="woot_150_$2_tag_CordlessPhon"
    tagid="CordlessPhon"
else
    echo 'Processing object 8'
    yaml="woot_150_$2_tag_BlueHairBrus.yaml"
    headpre="woot_150_$2_tag_BlueHairBrus_headpre"
    headpost="woot_150_$2_tag_BlueHairBrus_headpost"
    bag="woot_150_$2_tag_BlueHairBrus"
    tagid="BlueHairBrus"
fi

yaml="search_aware_home/$yaml"
echo "$yaml"
if [ -a $yaml ]; then
    echo "WORKING:"
    echo -e "\t$yaml"
    echo -e "\t$headpre"
    echo -e "\t$headpost"
    echo -e "\t$bag"
    echo -e '\n'

    rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped  --latch -f "$yaml"
    # python sm_head_capture.py --fname "$headpre"
    # rosservice call /rfid_orient/flap "''" 0.0
    # rosservice call /rfid_orient/orient "\'$tagid\'"
    ./sm_servo_capture.py --fname "$bag" --tag "$tagid"
    python sm_head_capture.py --fname "$headpost" 
    rosrun tf tf_echo /map /base_link | tee "search_aware_home/${bag}_end.txt"
else
    echo 'File: $yaml does not exist. Skipping'
fi


# rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped  --latch -f woot_tag__person.yaml
# python sm_head_capture.py --fname hrl_$4_$1_$2_headpre
# ./sm_servo_capture.py --fname hrl_$4_$1_$2 --tag "$3"  # automatically adds _servo
# python sm_head_capture.py --fname hrl_$4_$1_$2_headpost
