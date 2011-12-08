#!/bin/bash

rosservice call /rfid_orient/bag "['rosbag', 'record', '/tf', '/rfid/ears_reader_arr', '-o', '/u/travis/svn/robot1/src/projects/rfid_people_following/src/rfid_people_following/data/OnMetal/']"
rosservice call /rfid_orient/flap "'OnMetal     '"
rosservice call /rfid_orient/bag "[]"

