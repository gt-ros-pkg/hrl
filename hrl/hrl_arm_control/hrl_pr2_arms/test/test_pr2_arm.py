#! /usr/bin/python

import sys
import numpy as np

import roslib; roslib.load_manifest("hrl_pr2_arms")
import rospy
import tf.transformations as tf_trans

from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory, PR2ArmJTranspose
from hrl_pr2_arms.pr2_arm import PR2ArmJInverse, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_arm_kinematics import PR2ArmKinematics

def main():
    rospy.init_node("pr2_arm_test")
    arm = sys.argv[1]
    mode = sys.argv[2]
    assert arm in ['r', 'l']
    assert mode in ['joint1', 'cart1', 'cart2', 'cart3', 'cart4']

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
        pr2_jttask_arm.set_gains([0., 60., 60., 50., 50., 50.],
                                 [3., 0.1, 0.1, 3., 3., 3.], 'r_gripper_tool_frame')

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
        
    if mode == 'cart4':
        pr2_r_kinematics = PR2ArmKinematics('r')
        pr2_jttask_arm = PR2ArmJTransposeTask('r', pr2_r_kinematics)
        pos_a = np.mat([[0.0, 0.1, 0.0]]).T
        rot_a = np.mat(tf_trans.random_rotation_matrix())[:3,:3]
        pos_b = np.mat([[0.6, 0.0, 0.0]]).T
        rot_b = np.mat(tf_trans.euler_matrix(0.5, 0.8, 0.5))[:3,:3]
        ep_err = pr2_jttask_arm.ep_error((pos_a, rot_a), (pos_b, rot_b))
        err_mat = np.mat(tf_trans.euler_matrix(ep_err[3,0], ep_err[4,0], ep_err[5,0]))[:3, :3]
        print "err_mat", err_mat
        diff_mat = rot_b.T * rot_a
        print "diff_mat", diff_mat
        rx, ry, rz = tf_trans.euler_from_matrix(diff_mat)
        print ep_err[3:6,0].T
        print rx, ry, rz
        print diff_mat - err_mat

if __name__ == "__main__":
    main()
