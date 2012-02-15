#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest('hrl_pr2_arms')

import rospy
import tf
import tf.transformations as tf_trans

from hrl_generic_arms.ep_trajectory_controller import EPArmController
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

def main():
    rospy.init_node("servo_prepare")
    ctrl_switcher = ControllerSwitcher()
    ctrl_switcher.carefree_switch('r', '%s_arm_controller')
    ctrl_switcher.carefree_switch('l', '%s_arm_controller')
    r_arm = create_pr2_arm('r', PR2ArmJointTrajectory)
    l_arm = create_pr2_arm('l', PR2ArmJointTrajectory)
    r_arm.wait_for_joint_angles()
    l_arm.wait_for_joint_angles()

    r_ep_arm_ctrl = EPArmController(r_arm)
    l_ep_arm_ctrl = EPArmController(l_arm)
    r_ep_arm_ctrl.execute_interpolated_ep([-1.91,  1.25, -1.93, -1.53,  0.33, -0.03, 0.0],
                                          15, blocking=False)
    l_ep_arm_ctrl.execute_interpolated_ep([1.91,  1.25,  1.93, -1.53, -0.33, -0.03, -3.09],
                                          15, blocking=True)

    
    

if __name__ == "__main__":
    main()
