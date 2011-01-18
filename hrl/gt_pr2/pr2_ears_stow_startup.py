#!/usr/bin/python

import roslib
roslib.load_manifest( 'robotis' )
import rospy
import robotis.lib_robotis as rs

import time
import math

if __name__ == '__main__':
    rospy.logout( 'pr2_ears_stow_startup: Stowing RFID Ears Servos' )
    
    dyn_right = rs.USB2Dynamixel_Device( '/dev/robot/servo1' )
    pan_right = rs.Robotis_Servo( dyn_right, 29 )
    tilt_right = rs.Robotis_Servo( dyn_right, 30 )

    dyn_left = rs.USB2Dynamixel_Device( '/dev/robot/servo0' )
    pan_left = rs.Robotis_Servo( dyn_left, 27 )
    tilt_left = rs.Robotis_Servo( dyn_left, 28 )

    pan_right.move_angle( 1.370, math.radians( 10 ), False )
    tilt_right.move_angle( 0.0, math.radians( 10 ), False )

    pan_left.move_angle( -1.370, math.radians( 10 ), False )
    tilt_left.move_angle( 0.0, math.radians( 10 ), False )

    while pan_left.is_moving() or pan_right.is_moving():
        time.sleep( 0.1 )
    time.sleep(1.0)

