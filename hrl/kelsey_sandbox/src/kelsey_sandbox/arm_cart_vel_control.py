#! /usr/bin/python

import numpy as np
from collections import deque
from threading import Lock

import roslib
roslib.load_manifest("hrl_pr2_arms")
import tf.transformations as tf_trans

import rospy
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory, PR2ArmCartesianPostureBase
from hrl_generic_arms.controllers import PIDController

rate = 100.

# controller gains
kp_proportional = 0.084
kp_integral = 0.35
kp_init_integral = 0.005
kp_constant = 0.002
kp_max_ctrl = 0.30

kr_proportional = 0.160
kr_integral = 0.200
kr_init_integral = 0.05
kr_constant = 0.020
kr_max_ctrl = 0.40

class PR2ArmCartVelocityController(object):
    def __init__(self, arm, velocity_rot=None):
        self.arm = arm
        if velocity_rot is None:
            self.velocity_rot = np.mat(np.eye(3))
        else:
            self.velocity_rot = velocity_rot
        self._is_moving = False
        self.lock = Lock()

    def update_velocity_rot(self, velocity_rot):
        with self.lock:
            self.velocity_rot = np.mat(velocity_rot)

    def move_velocity(self, velocity=0.015, is_translation=True, velocity_rot=None):
        if velocity_rot is not None:
            self.update_velocity_rot(velocity_rot)
        pos_i_des, rot_i_des = self.arm.get_ep()
        pos_i_act, rot_i_act = self.arm.get_end_effector_pose()

        # this is the current residiual error in the controller at the start
        pos_err = pos_i_act - pos_i_des 
        rot_err = rot_i_act.T * rot_i_des

        if is_translation:
            pos_vel_des = velocity
            pid_ctrl = PIDController(rate=rate, k_p=kp_proportional, k_i=kp_integral, 
                                     i_max=None, init_integ=np.sign(pos_vel_des) * kp_init_integral, 
                                     saturation=kp_max_ctrl, 
                                     feed_forward=np.sign(pos_vel_des) * kp_constant,
                                     name="arm_vel")
        else:
            rot_vel_des = velocity
            pid_ctrl = PIDController(rate=rate, k_p=kr_proportional, k_i=kr_integral, 
                                     i_max=None, init_integ=np.sign(rot_vel_des) * kr_init_integral, 
                                     saturation=kr_max_ctrl, 
                                     feed_forward=np.sign(rot_vel_des) * kr_constant,
                                     name="arm_vel")
        vels = deque([np.array([0]*6)]*40)
        r = rospy.Rate(rate)
        self._is_moving = True
        while not rospy.is_shutdown() and self._is_moving:
            with self.lock:
                vel_rot = self.velocity_rot.copy()
            cur_pos, cur_rot = self.arm.get_end_effector_pose()

            # hacky velocity filter
            xd_act = self.arm.get_controller_state()['xd_act']
            vels.append(xd_act)
            vels.popleft()
            vel_filt = np.mat(np.mean(vels, 0)).T
            x_vel_filt = (vel_rot.T * vel_filt[:3,0])[0,0]
            roll_vel_filt = (vel_rot.T * vel_filt[3:,0])[0,0]

            if is_translation:
                # PI velocity controller for position
                pos_ctrl = pid_ctrl.update_state(pos_vel_des - x_vel_filt)
                pos_des = vel_rot * (np.mat([pos_ctrl, 0, 0]).T + 
                                     np.mat(np.diag([1, 0, 0])) * vel_rot.T * (cur_pos - pos_err) +
                                     np.mat(np.diag([0, 1, 1])) * vel_rot.T * pos_i_des)

                rot_des = rot_i_des # don't change rotation

            if not is_translation:
                rot_des_vel_frame = np.mat(np.eye(4))
                rot_des_vel_frame[:3,:3] = cur_rot * rot_err * vel_rot
                roll_des_vel_frame, r2, r3 = tf_trans.euler_from_matrix(rot_des_vel_frame)

                # PI velocity controller for rotation
                rot_ctrl = pid_ctrl.update_state(rot_vel_des + roll_vel_filt)
                print roll_vel_filt, rot_vel_des, rot_vel_des - roll_vel_filt

                roll_ctrl_des = roll_des_vel_frame + rot_ctrl
                r1, pitch_i_des, yaw_i_des = tf_trans.euler_from_matrix(rot_i_des * vel_rot)
                rot_des = np.mat(tf_trans.euler_matrix(roll_ctrl_des, pitch_i_des, yaw_i_des)[:3,:3]) * vel_rot.T

                pos_des = pos_i_des # don't change translation

            self.arm.set_ep((pos_des, rot_des), 1)
            r.sleep()
        self.arm.set_ep(self.arm.get_ep(), 1)

    def stop_moving(self):
        self._is_moving = False

def main():
    rospy.init_node("arm_cart_vel_control")
    if True:
        ctrl_switcher = ControllerSwitcher()
        ctrl_switcher.carefree_switch('r', '%s_arm_controller', 
                           '$(find hrl_pr2_arms)/params/joint_traj_params_electric.yaml')
        rospy.sleep(0.5)
        ctrl_switcher.carefree_switch('r', '%s_joint_controller_low', 
                           '$(find hrl_pr2_arms)/params/joint_traj_params_electric_low.yaml')
        r_arm_js = create_pr2_arm('r', PR2ArmJointTrajectory, controller_name='%s_joint_controller_low')
        q = [-0.34781704,  0.27341079, -1.75392154, -2.08626393, -3.43756314, -1.82146607, -1.85187734]
        r_arm_js.set_ep(q, 3) 
        rospy.sleep(6)
        ctrl_switcher.carefree_switch('r', '%s_cart_low_rfh',
                                      '$(find kelsey_sandbox)/params/j_transpose_low_rfh.yaml')

    r_arm = create_pr2_arm('r', PR2ArmCartesianPostureBase)
    r_arm.set_posture()
    rospy.sleep(0.2)
    vel_ctrl = PR2ArmCartVelocityController(r_arm)
    #vel_frame = tf_trans.euler_matrix(0, -np.pi/2, 0)[:3,:3]
    #vel_ctrl.move_velocity(velocity_rot=vel_frame, velocity=0.005, is_translation=False)
    vel_frame = tf_trans.euler_matrix(0, 0, np.pi/2)[:3,:3]
    vel_ctrl.move_velocity(velocity_rot=vel_frame, velocity=0.10, is_translation=False)


if __name__ == "__main__":
    main()
