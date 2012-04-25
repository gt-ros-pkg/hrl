#! /usr/bin/python

import numpy as np
import pygame as pg

import roslib
roslib.load_manifest('hrl_ellipsoidal_control')

import rospy
import tf.transformations as tf_trans

from hrl_ellipsoidal_control.ellipsoid_controller import EllipsoidController
from hrl_pr2_arms.pr2_arm import PR2ArmCartesianPostureBase, PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from hrl_generic_arms.ep_trajectory_controller import EPArmController

move_keys_dict = {pg.K_w : 'UP', pg.K_s : 'DOWN', pg.K_a : 'LEFT',
             pg.K_d : 'RIGHT', pg.K_q : 'OUT', pg.K_e : 'IN'}
LOCAL_VELOCITY = 0.0025
LATITUDE_STEP = 0.12
LONGITUDE_STEP = 0.06
HEIGHT_STEP = 0.17
move_key_params = {'UP' : (-LONGITUDE_STEP, 0, 0), 'DOWN' : (LONGITUDE_STEP, 0, 0),
                   'RIGHT' : (0, -LATITUDE_STEP, 0), 'LEFT' : (0, LATITUDE_STEP, 0),
                   'IN' : (0, 0, -HEIGHT_STEP), 'OUT' : (0, 0, HEIGHT_STEP)}

ROT_VELOCITY = 0.002
ROLL_STEP = np.pi/12
PITCH_STEP = np.pi/12
YAW_STEP = np.pi/12
rot_key_params = {'IN' : (-ROLL_STEP, 0, 0), 'OUT' : (ROLL_STEP, 0, 0),
                  'UP' : (0, PITCH_STEP, 0), 'DOWN' : (0, -PITCH_STEP, 0),
                  'LEFT' : (0, 0, -YAW_STEP), 'RIGHT' : (0, 0, YAW_STEP)}

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
        #cart_arm.set_posture(setup_angles)
        setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
        cart_arm.set_posture(setup_angles)
        cart_arm.set_gains([200, 800, 800, 80, 80, 80], [15, 15, 15, 1.2, 1.2, 1.2])
        ell_controller.reset_arm_orientation(8)

    if False:
        for i in range(5):
            ell_controller.execute_rotation((-np.pi/10, 0, 0), 0.01)
            ell_controller.execute_rotation((0, 0, -np.pi/10), 0.01)
            ell_controller.execute_rotation((0, -np.pi/10, 0), 0.01)
            ell_controller.execute_rotation((0, 0, np.pi/10), 0.01)
            ell_controller.execute_rotation((np.pi/10, 0, 0), 0.01)
            ell_controller.execute_rotation((0, np.pi/10, 0), 0.01)
        return

    quat_gripper_rot = tf_trans.quaternion_from_euler(np.pi, 0, 0)

    pg.init()
    screen = pg.display.set_mode((300,300))
    while not rospy.is_shutdown():
        pg.event.get()
        keys = pg.key.get_pressed()
        for key in move_keys_dict:
            if keys[key]:
                button_press = move_keys_dict[key]
                if not (keys[pg.K_CAPSLOCK] or keys[pg.K_LSHIFT] or keys[pg.K_RSHIFT]):
                    change_ep = move_key_params[button_press]
                    ell_controller.execute_ell_move(change_ep, (0, 0, 0), quat_gripper_rot, LOCAL_VELOCITY)
                    break
                else:
                    change_ep = rot_key_params[button_press]
                    ell_controller.execute_rotation(change_ep, ROT_VELOCITY)
                    break
            
        rospy.sleep(0.05)

if __name__ == "__main__":
    main()
