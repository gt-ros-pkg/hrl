#!/usr/bin/env python

import roslib; roslib.load_manifest('assistive_teleop')
import rospy
from geometry_msgs.msg import PointStamped, Point, Quaternion
from std_msgs.msg import String
from tf import TransformListener, transformations as tft

#goal = (0,0,0) ellipse frame == (0,0,x), rotating only about (0,0,0) tool frame, then apply linear offset

class MirrorDirector(object):
    def __init__(self):
        rospy.Subscriber('adjust_mirror', PointStamped , self.adjust_mirror)
        self.tfl = TransformListener()


    def adjust_mirror(self, msg):
        frame = msg.header.frame_id
        point = msg.point	
	
	




