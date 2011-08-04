#! /usr/bin/python

import sys
import numpy as np

import roslib; roslib.load_manifest("hrl_pr2_arms")
import rospy

#from hrl_pr2_arms.pid_controller import PIDController
from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory, PR2ArmJTranspose
from hrl_pr2_arms.pr2_arm import PR2ArmJInverse, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_arm_kinematics import PR2ArmKinematics

def main():
    rospy.init_node("pr2_arm_test")
    arm = sys.argv[1]
    mode = sys.argv[2]
    assert arm in ['r', 'l']
    assert mode in ['joint1', 'cart1', 'cart2', 'cart3']

    if mode == 'joint1':
        pr2_r_kinematics = PR2ArmKinematics('r')
        pr2_joint_arm = PR2ArmJointTrajectory('r', pr2_r_kinematics)
        pr2_joint_arm.set_ep([0.1]*7, 15)

    if mode == 'cart1':
        pr2_r_kinematics = PR2ArmKinematics('r')
        pr2_jtrans_arm = PR2ArmJTranspose('r', pr2_r_kinematics)
        pos = np.mat([[0.6, 0.0, 0.0]]).T
        rot = np.mat(np.eye(3))
        pr2_jtrans_arm.set_ep((pos, rot), 5.)

    if mode == 'cart2':
        pr2_r_kinematics = PR2ArmKinematics('r')
        pr2_jttask_arm = PR2ArmJTransposeTask('r', pr2_r_kinematics)
        pr2_jttask_arm.set_gains([30., 300., 300., 50., 50., 50.],
                                 [3.]*6, 'r_gripper_tool_frame')

    if mode == 'cart3':
        pr2_r_kinematics = PR2ArmKinematics('r')
        pr2_jttask_arm = PR2ArmJTransposeTask('r', pr2_r_kinematics)
        pos_a = np.mat([[0.0, 0.1, 0.0]]).T
        rot_a = np.mat(np.eye(3))
        pos_b = np.mat([[0.6, 0.0, 0.0]]).T
        rot_b = np.mat([[1.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0],
                        [0.0, 1.0, 0.0]])
        print zip(*pr2_jttask_arm.interpolate_ep((pos_a, rot_a), (pos_b, rot_b), 10))
        

if __name__ == "__main__":
    main()
