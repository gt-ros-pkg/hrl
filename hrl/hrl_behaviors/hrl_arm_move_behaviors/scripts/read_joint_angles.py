#! /usr/bin/python

import random
import sys
import numpy as np

import roslib; roslib.load_manifest("hrl_arm_move_behaviors")
import rospy

from hrl_arm_move_behaviors.pr2_arm_base import PR2ArmBase

rospy.init_node('read_joint_angles')
pab = PR2ArmBase('r')

for i in range(1000):
    print pab.get_joint_angles(wrapped=True)
    rospy.sleep(0.1)

