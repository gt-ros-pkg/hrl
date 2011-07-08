#! /usr/bin/python

import numpy as np, math
import sys
import pygame as pg

import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy

from hrl_pr2_arms.pid_joint_control import PIDJCFactory
from hrl_pr2_arms.pr2_arm_kinematics import PR2ArmKinematics
from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory
from equilibrium_point_control.timer import Timer
from equilibrium_point_control.epc import EPGenerator, EPC, EPStopConditions

def main():
    rospy.init_node('test_pid_joint_control')

    pid_jc_factory = PIDJCFactory('r', [0.03]*7, [0.4]*7, [0.3]*7, [0.08]*7, [1.]*7, 10.)

    pid_jc = pid_jc_factory.create_pid_jc([0.]*7)

    epc = EPC('test_pid_joint_control')
    epc.epc_motion(pid_jc, 0.1, 50)
    

if __name__ == '__main__':
    main()
