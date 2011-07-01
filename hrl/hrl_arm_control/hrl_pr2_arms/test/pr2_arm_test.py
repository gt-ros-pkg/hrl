#! /usr/bin/python

import sys

import roslib; roslib.load_manifest("hrl_pr2_arms")
import rospy

#from hrl_pr2_arms.pid_controller import PIDController
from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_arm_kinematics import PR2ArmKinematics

def main():
    rospy.init_node("pr2_arm_test")
    arm = sys.argv[1]
    mode = sys.argv[2]
    assert arm in ['r', 'l']
    assert mode in ['jt1']

    if mode == 'jt1':
        pr2_r_kinematics = PR2ArmKinematics('r')
        pr2_jt_arm = PR2ArmJointTrajectory('r', pr2_r_kinematics)
        pr2_jt_arm.set_ep([0.1]*7, 15)

    if mode == 'jt2':
        pass

if __name__ == "__main__":
    main()
