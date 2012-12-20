#!/usr/bin/env python

import roslib
roslib.load_manifest('hrl_tactile_controller')
import rospy

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import std_msgs.msg

def getMessageHeader():
  header = std_msgs.msg.Header()
  header.stamp = rospy.get_rostime()
  header.frame_id = "torso_lift_link"
  return header



ta_pub = rospy.Publisher('/skin/test_taxel_array', haptic_msgs.TaxelArray)
rospy.init_node('test_taxel_array_publisher')

r = rospy.Rate(0.5)

while not rospy.is_shutdown():
  ta_msg = haptic_msgs.TaxelArray()
  ta_msg.header = getMessageHeader()
  #print ta_msg
  ta_pub.publish(ta_msg)
  r.sleep()
