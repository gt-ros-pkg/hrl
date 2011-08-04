#! /usr/bin/python

import sys
import numpy as np

import roslib; roslib.load_manifest("hrl_pr2_arms")
import rospy
import tf.transformations as tf_trans

from hrl_pr2_arms.pr2_arm import PR2Arm, create_pr2_arm
from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory, PR2ArmJTranspose
from hrl_pr2_arms.pr2_arm import PR2ArmJInverse, PR2ArmJTransposeTask

from spheroid_space import SpheroidSpace

def main():
    rospy.init_node("pr2_arm_test")
    arm = sys.argv[1]
    jnt_arm = create_pr2_arm(arm, arm_type=PR2ArmJointTrajectory)
    kin = jnt_arm.kinematics

    ellipse_rot = np.mat([[-1., 0., 0.], [0., -1., 0.], [0., 0., 1.]])
    sspace = SpheroidSpace(0.15, np.mat([0.78, -0.18, 0.1]).T, ellipse_rot)

    uvp = np.array([1.0, np.pi/2, 0.0])
    uvp_delta = np.array([0.0, 0.6, 0.6])
    pos, rot = sspace.spheroidal_to_pose(uvp + uvp_delta)
    print pos, rot

    #q_ik = kin.IK_search(pos, rot)
    q_ik = kin.IK(pos, rot, jnt_arm.get_joint_angles())
    
    if q_ik is not None:
        jnt_arm.set_ep(q_ik, 5)
    else:
        print "IK failure"

if __name__ == "__main__":
    main()
