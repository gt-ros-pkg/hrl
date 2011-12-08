#!/bin/bash

# sample call: ./cap_pose.sh 2.0 1.0 0.0 1.570796

echo 'Called with args: ' $1 $2 'as <x,y>'

rosservice call /rotate_backup/navstack $1 $2 0.0 0.0
rosservice call /rfid_orient/bag "['rosbag', 'record', '/tf', '/rfid/ears_reader_arr', '-o', '/u/travis/svn/robot1/src/projects/rfid_people_following/src/rfid_people_following/data/OnMetal/']"
rosservice call /rfid_orient/flap "'OnMetal     '"
rosservice call /rfid_orient/bag "[]"

rosservice call /rotate_backup/navstack $1 $2 0.0 1.570796
rosservice call /rfid_orient/bag "['rosbag', 'record', '/tf', '/rfid/ears_reader_arr', '-o', '/u/travis/svn/robot1/src/projects/rfid_people_following/src/rfid_people_following/data/OnMetal/']"
rosservice call /rfid_orient/flap "'OnMetal     '"
rosservice call /rfid_orient/bag "[]"

rosservice call /rotate_backup/navstack $1 $2 0.0 3.141592
rosservice call /rfid_orient/bag "['rosbag', 'record', '/tf', '/rfid/ears_reader_arr', '-o', '/u/travis/svn/robot1/src/projects/rfid_people_following/src/rfid_people_following/data/OnMetal/']"
rosservice call /rfid_orient/flap "'OnMetal     '"
rosservice call /rfid_orient/bag "[]"

rosservice call /rotate_backup/navstack $1 $2 0.0 4.712388
rosservice call /rfid_orient/bag "['rosbag', 'record', '/tf', '/rfid/ears_reader_arr', '-o', '/u/travis/svn/robot1/src/projects/rfid_people_following/src/rfid_people_following/data/OnMetal/']"
rosservice call /rfid_orient/flap "'OnMetal     '"
rosservice call /rfid_orient/bag "[]"