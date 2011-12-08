#!/usr/bin/python

# Merges functionality from ros_M5e in hrl_rfid package.
#     if opt.device == 'ears':
#         print 'Starting Ears RFID Services'
#         ros_rfid = ROS_M5e( name = 'ears', readPwr = opt.power,
#                             portStr = '/dev/robot/RFIDreader',
#                             antFuncs = [EleLeftEar, EleRightEar],
#                             callbacks = [] )
#         rospy.spin()
#         ros_rfid.stop()


import roslib
roslib.load_manifest( 'robotis' )
roslib.load_manifest( 'hrl_rfid' )
roslib.load_manifest( 'rfid_people_following' )
import rospy

import time
from threading import Thread

# The Servers need to distinguish b/w real vs. simulated!
#   They implement same services / functions. (Clients unaffected)
import os
if os.environ.has_key('ROBOT') and os.environ['ROBOT'] == 'sim':
    import rfid_people_following.robotis_servo_sim as rr
    import rfid_people_following.M5e_reader_sim as rM5e
    
else:
    import robotis.ros_robotis as rr
    import hrl_rfid.ros_M5e as rM5e


if __name__ == '__main__':
#     p_right = rr.ROS_Robotis_Poller( '/dev/robot/servo1', [29,30], ['right_pan', 'right_tilt'] )
#     p_left = rr.ROS_Robotis_Poller( '/dev/robot/servo0', [27,28], ['left_pan', 'left_tilt'] )
    p_right = rr.ROS_Robotis_Poller( '/dev/robot/servo1', [29,30], ['right_pan', 'right_tilt'] )
    p_left = rr.ROS_Robotis_Poller( '/dev/robot/servo0', [27,28], ['left_pan', 'left_tilt'] )
#     p_right = rr.ROS_Robotis_Poller( '/dev/robot/servo0', [29,30], ['right_pan', 'right_tilt'] )
#     p_left = rr.ROS_Robotis_Poller( '/dev/robot/servo1', [27,28], ['left_pan', 'left_tilt'] )
    ros_rfid = rM5e.ROS_M5e( name = 'ears', readPwr = 3000,
                             portStr = '/dev/robot/RFIDreader',
                             antFuncs = [ rM5e.EleLeftEar, rM5e.EleRightEar ],
                             callbacks = [] )
    rospy.spin()
    
    ros_rfid.stop()
    p_right.stop()
    p_left.stop()

