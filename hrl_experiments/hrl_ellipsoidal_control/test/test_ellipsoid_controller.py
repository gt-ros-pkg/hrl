#! /usr/bin/python

import numpy as np
import pygame as pg

import roslib
roslib.load_manifest('hrl_ellipsoidal_control')

import rospy

from hrl_ellipsoidal_control.ellipsoid_controller import EllipsoidController
from hrl_pr2_arms.pr2_arm import PR2ArmCartesianPostureBase, PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from hrl_generic_arms.ep_trajectory_controller import EPArmController

LOCAL_VELOCITY = 0.0025
LATITUDE_STEP = 0.12
LONGITUDE_STEP = 0.06
HEIGHT_STEP = 0.17
keys_dict = {pg.K_w : 'UP', pg.K_s : 'DOWN', pg.K_a : 'LEFT',
             pg.K_d : 'RIGHT', pg.K_q : 'OUT', pg.K_e : 'IN'}
key_params = {'UP' : (-LONGITUDE_STEP, 0, 0), 'DOWN' : (LONGITUDE_STEP, 0, 0),
              'RIGHT' : (0, -LATITUDE_STEP, 0), 'LEFT' : (0, LATITUDE_STEP, 0),
              'IN' : (0, 0, -HEIGHT_STEP), 'OUT' : (0, 0, HEIGHT_STEP)}

def main():
    rospy.init_node("test_ellipsoid_controller")
    ctrl_switcher = ControllerSwitcher()

    if False:
        ctrl_switcher.carefree_switch('l', '%s_arm_controller', None)
        rospy.sleep(1)
        joint_arm = create_pr2_arm('l', PR2ArmJointTrajectory)

        setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
        joint_arm.set_ep(setup_angles, 5)
        rospy.sleep(5)

        ctrl_switcher.carefree_switch('l', '%s_cart_jt_task', 
                                      "$(find hrl_rfh_fall_2011)/params/l_jt_task_shaver45.yaml") 
        rospy.sleep(1)

    cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, 
                              controller_name='%s_cart_jt_task', 
                              end_link="%s_gripper_shaver45_frame")

    ell_controller = EllipsoidController(cart_arm)
    rospy.sleep(1)
    if False:
        cart_arm.set_posture(setup_angles)
        setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
        cart_arm.set_posture(setup_angles)
        cart_arm.set_gains([200, 800, 800, 80, 80, 80], [15, 15, 15, 1.2, 1.2, 1.2])
        ell_controller.reset_arm_orientation(8)

    pg.init()
    screen = pg.display.set_mode((300,300))
    while not rospy.is_shutdown():
        pg.event.get()
        keys = pg.key.get_pressed()
        button_press = None
        for key in keys_dict:
            if keys[key]:
                button_press = keys_dict[key]
                change_ep = key_params[button_press]
                break
        if button_press is not None:
            ell_controller.execute_ell_move(change_ep, (0, 0, 0), np.pi, LOCAL_VELOCITY)
            
        rospy.sleep(0.05)

if __name__ == "__main__":
    main()
