#! /usr/bin/python

import numpy as np
import pygame as pg

import roslib
roslib.load_manifest('hrl_rfh_fall_2011')

import rospy

from hrl_rfh_fall_2011.ellipsoid_controller import EllipsoidController
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmCartesianPostureBase, PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from hrl_generic_arms.ep_trajectory_controller import EPArmController
from equilibrium_point_control.ep_control import EPC

def main():
    rospy.init_node("test_ellipsoid_controller")
    ctrl_switcher = ControllerSwitcher()

    ctrl_switcher.carefree_switch('l', '%s_arm_controller', None)
    rospy.sleep(1)
    joint_arm = create_pr2_arm('l', PR2ArmJointTrajectory)

    setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
    #joint_controller = EPArmController(joint_arm, 0.1, "joint_ep_controller")
    #joint_controller.execute_interpolated_ep(setup_angles, 10)

    ctrl_switcher.carefree_switch('l', '%s_cart', None)
    rospy.sleep(1)

    cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, end_link="%s_gripper_shaver45_frame")
    cart_arm.set_posture(setup_angles)
#cart_arm.set_posture(cart_arm.get_joint_angles(wrapped=False))
    cart_arm.set_gains([110, 600, 600, 40, 40, 40], [15, 15, 15, 1.2, 1.2, 1.2])
    ell_controller = EllipsoidController(cart_arm)
    ell_controller.reset_arm_orientation(8)

    pg.init()
    screen = pg.display.set_mode((300,300))
    while not rospy.is_shutdown():
        pg.event.get()
        keys = pg.key.get_pressed()
        dur = 5
        if True:
            r = np.random.randint(6)
            if r == 0:
                ell_controller.command_move([ 1,  0,  0], dur)
            elif r == 1:
                ell_controller.command_move([-1,  0,  0], dur)
            elif r == 2:
                ell_controller.command_move([ 0,  1,  0], dur)
            elif r == 3:
                ell_controller.command_move([ 0, -1,  0], dur)
            elif r == 4:
                ell_controller.command_move([ 0,  0,  1], dur)
            elif r == 5:
                ell_controller.command_move([ 0,  0, -1], dur)
            rospy.sleep(1)
        else:
            if keys[pg.K_w]:
                ell_controller.command_move([-1,  0,  0], dur)
            if keys[pg.K_s]:
                ell_controller.command_move([ 1,  0,  0], dur)
            if keys[pg.K_a]:
                ell_controller.command_move([ 0,  1,  0], dur)
            if keys[pg.K_d]:
                ell_controller.command_move([ 0, -1,  0], dur)
            if keys[pg.K_q]:
                ell_controller.command_move([ 0,  0,  1], dur)
            if keys[pg.K_e]:
                ell_controller.command_move([ 0,  0, -1], dur)
        rospy.sleep(0.05)

if __name__ == "__main__":
    main()
