#!/usr/bin/python

import roslib; roslib.load_manifest('assistive_teleop')
import rospy

import actionlib

class Pr2Gripper():

    def __init__(self, arm):
       pass


    def grab(self, gain, blocking=False):
        pass

    def release(self):
        pass


if __name__=='__main__':
    gripper = Pr2Gripper()
    while not rospy.is_shutdown():
        rospy.spin()
