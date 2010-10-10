#!/usr/bin/python
import roslib; roslib.load_manifest("hrl_pr2_lib")

import rospy
from hrl_pr2_lib.simple_arm_manager import SimpleArmManager, SmoothMoveArmTrajectory

import random

import copy

node_name = "test_simple_arm_manager" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

if __name__ == "__main__":
    rospy.init_node(node_name, anonymous = True)
    log("Node initialized")

#   sam = SimpleArmManager()
#   while True:
#       print sam.FK(0, sam.get_joint_angles(0))
#       rospy.sleep(0.1)

#   while True:
#       pos, rot = sam.FK(0, sam.get_joint_angles(0))
#       print "FK", pos, rot 
#       print "IK", sam.IK(0, pos, rot, sam.get_joint_angles(0))
#       rospy.sleep(0.1)

#   while True:
#       pos, rot = sam.FK(0, sam.get_joint_angles(0))
#       pos[2,0] -= 0.05
#       for i in range(3):
#           rot[i,0] = 0.0
#       rot[3,0] = 1.0
#       print sam.IK(0, pos, rot, sam.get_joint_angles(0))

#   pos, rot = sam.FK(0, sam.get_joint_angles(0))
#   pos[2,0] += 0.15
#   for i in range(3):
#       rot[i,0] = 0.0
#   rot[3,0] = 1.0
#   sam.move_arm(0, pos, rot, 1.0)
    
#   while True:
#       pos, rot = sam.FK(0, sam.get_joint_angles(0))
#       rand = random.randint(0, 1)
#       if rand == 0:
#           delta = -0.1
#       else:
#           delta = 0.1
#       rand = random.randint(0, 2)
#       pos[rand,0] += delta 
#       for i in range(3):
#           rot[i,0] = 0.0
#       rot[3,0] = 1.0
#       if sam.can_move_arm(0, pos, rot, 1.0):
#           sam.move_arm(0, pos, rot, 1.0)
#       rospy.sleep(1.1)

#   pos, rot = sam.FK(0, sam.get_joint_angles(0))
#   altpos = copy.copy(pos)
#   pos[1] += 0.2
#   altpos[2] -= 0.2
#   sam.move_arm(0, pos, rot, 1.0)
#   rospy.sleep(0.2)
#   sam.move_arm(0, altpos, rot, 1.0)

    smat = SmoothMoveArmTrajectory()
    while True:
        smat.beg_arm_traj(0, 0.2, dir=(0.,0.,1.), dur=None)
        rospy.sleep(5.15)
        smat.beg_arm_traj(0, 0.2, dir=(0.,0.,-1.), dur=None)
        rospy.sleep(5.15)
