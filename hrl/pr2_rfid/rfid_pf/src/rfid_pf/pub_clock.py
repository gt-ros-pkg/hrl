#!/usr/bin/python

import time

import roslib
roslib.load_manifest( 'rosgraph_msgs' )
roslib.load_manifest( 'rospy' )
import rospy

from rosgraph_msgs.msg import Clock

rospy.init_node( 'clock_pub' )
time.sleep( 0.2 )

pub = rospy.Publisher( '/clock', Clock )

while not rospy.is_shutdown():
    pub.publish( Clock().clock.from_sec( time.time() ) )
    time.sleep( 0.001 )
