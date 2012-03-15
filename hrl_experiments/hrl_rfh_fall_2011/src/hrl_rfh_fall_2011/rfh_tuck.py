#! /usr/bin/python

import numpy as np
import pygame as pg

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('hrl_rfh_fall_2011')

import rospy

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from hrl_generic_arms.ep_trajectory_controller import EPArmController

def main():
    rospy.init_node("test_ellipsoid_controller")
    ctrl_switcher = ControllerSwitcher()

    ctrl_switcher.carefree_switch('l', '%s_arm_controller', None)
    ctrl_switcher.carefree_switch('r', '%s_arm_controller', None)
    rospy.sleep(1)
    l_arm = create_pr2_arm('l', PR2ArmJointTrajectory)
    r_arm = create_pr2_arm('r', PR2ArmJointTrajectory)

    l_tuck_angles = [ 0.14228106,  1.29643293,  1.78480255, -1.56470338,  1.16505304,
                     -0.09788312,  0.23542476]
    r_tuck_angles = [ 0.01289596,  1.02437885, -1.34551339, -1.78272859,  0.38331793,
                     -1.28334274,  0.02605728]
    l_joint_controller = EPArmController(l_arm, 0.1, "l_joint_ep_controller")
    r_joint_controller = EPArmController(r_arm, 0.1, "r_joint_ep_controller")
    l_joint_controller.execute_interpolated_ep(l_tuck_angles, 10)
    r_joint_controller.execute_interpolated_ep(r_tuck_angles, 10)

if __name__ == "__main__":
    main()
