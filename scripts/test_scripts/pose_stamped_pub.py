#!/usr/bin/env python

import roslib
roslib.load_manifest('hrl_tactile_controller')
import rospy

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import std_msgs.msg
import geometry_msgs.msg

def getMessageHeader():
  header = std_msgs.msg.Header()
  header.stamp = rospy.get_rostime()
  header.frame_id = "torso_lift_link"
  return header



pub = rospy.Publisher('/mpc_trajectory/waypoints/pose', geometry_msgs.msg.PoseStamped)
rospy.init_node('mpc_goal_publisher')

r = rospy.Rate(0.5)

while not rospy.is_shutdown():
  msg = geometry_msgs.msg.PoseStamped()
  msg.header = getMessageHeader()
  msg.pose.orientation.w = 1.0
  print msg
  pub.publish(msg)
  r.sleep()
