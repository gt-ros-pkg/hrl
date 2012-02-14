#!/usr/bin/env python

import roslib; roslib.load_manifest('assistive_teleop')
import rospy
from geometry_msgs.msg import PointStamped, Point, Quaternion
from std_msgs.msg import String
from tf import TransformListener, transformations as tft



class MirrorDirector(object):
    def __init__(self):
        rospy.Subscriber('adjust_mirror', PointStamped , self.adjust_mirror)



    def adjust_mirror(self, msg):
        frame = msg.header.frame_id
        point = msg.point


