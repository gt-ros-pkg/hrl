#!/usr/bin/python

import roslib; roslib.load_manifest('assistive_teleop')
import rospy

import actionlib
from pr2_gripper_sensor_msgs.msg import (
    PR2GripperGrabAction, PR2GripperGrabCommand
    )

class Pr2Gripper():
    def __init__(self, arm):
        self.arm = arm
        self.grab_ac = actionlib.SimpleActionClient('grab_ac',
                                                    PR2GripperGrabAction)
        rospy.loginfo("Waiting for " + self.arm + "_arm server")
        if self.grab_ac.wait_for_server(rospy.Duration(50)):
            rospy.loginfo("Found move_" + self.arm + "_arm server")
        else:
            rospy.logwarn("Cannot find move_" + self.arm + "_arm server")
       
    def grab(self, gain=0.03, blocking=False, block_timeout=20):
        grab_goal = PR2GripperGrabCommand(gain)
        self.grab_ac.send_goal(grab_goal)
        if blocking:
            self.grab_ac.wait_for_result(rospy.Duration(block_timeout))

    def release(self):
        pass


if __name__=='__main__':
    rospy.init_node('gripper_sensor_intermediary')
    gripper = Pr2Gripper('right')
    gripper.grab()
    while not rospy.is_shutdown():
        rospy.spin()
