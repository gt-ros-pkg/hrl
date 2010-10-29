#!/usr/bin/python
import roslib; roslib.load_manifest("hrl_pr2_lib")

import rospy
from hrl_pr2_lib.pr2_arms import PR2Arms
from hrl_lib.transforms import rotX, rotY, rotZ
import numpy as np

import random

import copy

node_name = "test_pr2_arms" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

if __name__ == "__main__":
    rospy.init_node(node_name, anonymous = True)
    log("Node initialized")

    arms = PR2Arms()

#   while not rospy.is_shutdown():
#       print arms.FK(0, arms.get_joint_angles(0))
#       rospy.sleep(0.1)

#   while not rospy.is_shutdown():
#       print arms.get_joint_angles(0)
#       rospy.sleep(0.1)


#   while not rospy.is_shutdown():
#       pos = (0.52, -0.3, -0.1)
#       rot = rotY(np.pi / 2.)
#       arms.move_arm(0, pos, rot, 4.)
#       arms.wait_for_arm_completion(0)
#       rospy.sleep(0.5)
#       angs = arms.get_joint_angles(0)
#       angs[1] -= 0.15
#       if angs[1] < -.54:
#           angs[1] = -.54
#       angs[2] -= 0.15
#       if angs[2] < -3.9:
#           angs[2] = -3.9
#       angs[5] += 0.15
#       if angs[5] > 0.:
#           angs[5] = 0.
#       arms.set_joint_angles(0, angs, 4.)
#       arms.wait_for_arm_completion(0)
#       rospy.sleep(0.5)

    while not rospy.is_shutdown():
        angs = arms.get_joint_angles(0)
        pos, rot = arms.FK(0, angs)
        print "FK", pos, rot 
        ik_angs = arms.IK(0, pos, rot, angs)
        print "pos", pos
        print "IK", ik_angs
        print "Angles", angs
        if ik_angs is not None:
            diffs_sqrd = [(ik_angs[i] - angs[i]) ** 2 for i in range(len(angs))]
            print "Squared differences between IK and original angles:", diffs_sqrd
            print "Sum squared differences between IK and original angles:", sum(diffs_sqrd)
        rospy.sleep(0.1)

#   print arms.create_JTG(0, [[0.2] * 7], [1.])
#   arms.move_arm(0, (0.7, 0.0, 0.035), rotY(np.pi / 2.), 4.)

#   while not rospy.is_shutdown():
#       pos, rot = arms.FK(0, arms.get_joint_angles(0))
#       pos[2,0] -= 0.05
#       for i in range(3):
#           rot[i,0] = 0.0
#       rot[3,0] = 1.0
#       print arms.IK(0, pos, rot, arms.get_joint_angles(0))

#   pos, rot = arms.FK(0, arms.get_joint_angles(0))
#   pos[2,0] += 0.15
#   for i in range(3):
#       rot[i,0] = 0.0
#   rot[3,0] = 1.0
#   arms.move_arm(0, pos, rot, 1.0)
    
#   while not rospy.is_shutdown():
#       pos, rot = arms.FK(0, arms.get_joint_angles(0))
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
#       if arms.can_move_arm(0, pos, rot, 1.0):
#           arms.move_arm(0, pos, rot, 1.0)
#       rospy.sleep(1.1)

#   pos, rot = arms.FK(0, arms.get_joint_angles(0))
#   altpos = copy.copy(pos)
#   pos[1] += 0.2
#   altpos[2] -= 0.2
#   arms.move_arm(0, pos, rot, 1.0)
#   rospy.sleep(0.2)
#   arms.move_arm(0, altpos, rot, 1.0)

#   while not rospy.is_shutdown():
#       arms.smooth_linear_arm_trajectory(0, 0.3, dir=(0.,0.,1.), dur=None)
#       rospy.sleep(7.15)
#       arms.smooth_linear_arm_trajectory(0, 0.3, dir=(0.,0.,-1.), dur=None)
#       rospy.sleep(7.15)

#   deltas = np.linspace(0.004, 0.009, 10)
#   j = 0
#   while not rospy.is_shutdown():
#       arms.smooth_linear_arm_trajectory(0, 0.3, dir=(0.,0.,1.), dur=None, delta=deltas[j])
#       print "delta =", deltas[j]
#       rospy.sleep(7.15)
#       arms.smooth_linear_arm_trajectory(0, 0.3, dir=(0.,0.,-1.), dur=None, delta=deltas[j])
#       print "delta =", deltas[j]
#       rospy.sleep(7.15)
#       j += 1

#   jerks = np.linspace(0.008, 0.03, 10)
#   j = 0
#   while not rospy.is_shutdown():
#       arms.smooth_linear_arm_trajectory(0, 0.3, dir=(0.,0.,-1.), dur=None, max_jerk=jerks[j])
#       print "jerk =", jerks[j]
#       rospy.sleep(7.15)
#       arms.smooth_linear_arm_trajectory(0, 0.3, dir=(0.,0.,1.), dur=None, max_jerk=jerks[j])
#       print "jerk =", jerks[j]
#       rospy.sleep(7.15)
#       j += 1
