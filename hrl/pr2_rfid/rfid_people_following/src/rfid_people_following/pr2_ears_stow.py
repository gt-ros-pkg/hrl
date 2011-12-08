#!/usr/bin/python

import roslib
roslib.load_manifest( 'robotis' )
import rospy
import robotis.ros_robotis as rr

import time
import math

if __name__ == '__main__':
    p_left = rr.ROS_Robotis_Client( 'left_pan' )
    t_left = rr.ROS_Robotis_Client( 'left_tilt' )
    
    p_left.move_angle( -1.370, math.radians(10), blocking = False )
    t_left.move_angle( 0.0, math.radians(10), blocking = False )


    p_right = rr.ROS_Robotis_Client( 'right_pan' )
    t_right = rr.ROS_Robotis_Client( 'right_tilt' )
    
    p_right.move_angle( 1.370, math.radians(10), blocking = False )
    t_right.move_angle( 0.0, math.radians(10), blocking = False )

