#!/usr/bin/python

import roslib
roslib.load_manifest( 'robotis' )
import rospy
import robotis.lib_robotis as rs

import time
import math

if __name__ == '__main__':
    rospy.logout( 'pr2_ears_stow_startup: Stowing RFID Ears Servos' )
    
    rospy.logout( 'pr2_ears_stow_startup: Connecting to servos' )
    dyn_left = rs.USB2Dynamixel_Device( '/dev/robot/servo1' )
    pan_left = rs.Robotis_Servo( dyn_left, 29 )
    tilt_left = rs.Robotis_Servo( dyn_left, 30 )

    dyn_right = rs.USB2Dynamixel_Device( '/dev/robot/servo0' )
    pan_right = rs.Robotis_Servo( dyn_right, 27 )
    tilt_right = rs.Robotis_Servo( dyn_right, 28 )

    # Hack to prevent the right servo from shaking.
    pan_right.write_address( 27, [3] ) # change the right pan compliance region
    pan_right.write_address( 26, [3] ) # change the right pan compliance region

    rospy.logout( 'pr2_ears_stow_startup: Commanding stow positions' )
    pan_left.move_angle( 1.370, math.radians( 10 ), False )
    tilt_left.move_angle( 0.0, math.radians( 10 ), False )

    pan_right.move_angle( -1.370, math.radians( 10 ), False )
    tilt_right.move_angle( 0.0, math.radians( 10 ), False )

    rospy.logout( 'pr2_ears_stow_startup: Done' )

    time.sleep(1.0)

