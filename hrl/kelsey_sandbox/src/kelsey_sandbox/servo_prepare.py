#! /usr/bin/python

import numpy as np
import sys

import roslib
roslib.load_manifest('hrl_pr2_arms')

import rospy
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Bool

from hrl_generic_arms.ep_trajectory_controller import EPArmController
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

joint_ctrl = '%s_arm_controller'

def setup_servoing_arms(msg):
    ctrl_switcher = ControllerSwitcher()
    ctrl_switcher.carefree_switch('r', joint_ctrl, reset=False)
    ctrl_switcher.carefree_switch('l', joint_ctrl, reset=False)
    r_arm = create_pr2_arm('r', PR2ArmJointTrajectory, controller_name=joint_ctrl)
    l_arm = create_pr2_arm('l', PR2ArmJointTrajectory, controller_name=joint_ctrl)

    r_ep_arm_ctrl = EPArmController(r_arm)
    l_ep_arm_ctrl = EPArmController(l_arm)
    r_ep_arm_ctrl.execute_interpolated_ep([-1.91,  1.25, -1.93, -1.53,  0.33, -0.03, 0.0],
                                          15, blocking=False)
    l_ep_arm_ctrl.execute_interpolated_ep([1.91,  1.25,  1.93, -1.53, -0.33, -0.03, -3.09],
                                          15, blocking=True)

def main():
    rospy.init_node("servo_prepare")
    if len(sys.argv) < 2:
        print "-s for server, -p for play"
    if sys.argv[1] == "-s":
        rospy.Subscriber("/pr2_ar_servo/arms_setup", Bool, setup_servoing_arms)
    if sys.argv[1] == "-p":
        setup_servoing_arms(None)
    rospy.spin()

if __name__ == "__main__":
    main()
