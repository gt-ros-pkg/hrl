#! /usr/bin/python

import numpy as np, math
import sys
import pygame as pg

import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy

from hrl_pr2_arms.pid_joint_control import PIDJCFactory
from hrl_pr2_arms.pr2_arm_kinematics import PR2ArmKinematics
from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory
from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions

def main():
    rospy.init_node('test_pid_joint_control')

    pid_jc_factory = PIDJCFactory('r', [0.05]*7, [0.25]*7, 
                                  [0.04]*7, [0.02]*7, [0.070]*7, [0.4]*7, 
                                  10., 0.1)

    q = [-0.2795571923841168, 0.9737173354972255, 0.60073767302223002, -1.445674103265479, -6.0812198392665415, -1.1110966461101544, 0.3056365620075685]
    q = [-0.1, 0.0, 0.0, -0.5, 0.0, -0.4, 0.0]
    pid_jc = pid_jc_factory.create_pid_jc(q)

    epc = EPC('test_pid_joint_control')
    epc.epc_motion(pid_jc, 0.1, 50)
    

if __name__ == '__main__':
    main()
