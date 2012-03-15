#!/usr/bin/python

import roslib
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('tf')
import rospy
import tf
from geometry_msgs.msg import PointStamped

import time

rospy.init_node('ground_truth_tag_pose')
listener = tf.TransformListener()


# listener.waitForTransform('/l_gripper_tool_frame', '/ear_antenna_left',
#                           rospy.Time(0), timeout = rospy.Duration(10) )
listener.waitForTransform('/l_gripper_tool_frame', '/map',
                          rospy.Time(0), timeout = rospy.Duration(10) )

rate = rospy.Rate(2)
while not rospy.is_shutdown():
    p = PointStamped()
    p.header.stamp = rospy.Time(0)
    p.header.frame_id =  '/l_gripper_tool_frame'
    p.point.x = 0.015 # Frame is actually in middle of fingers.  Move it out to tips
    #p_earl = listener.transformPoint('/ear_antenna_left', p)
    p_earl = listener.transformPoint('/map', p)
    print '<x,y,z> = <%2.3f, %2.3f, %2.3f> ' % ( p_earl.point.x, p_earl.point.y, p_earl.point.z )

    rate.sleep()
