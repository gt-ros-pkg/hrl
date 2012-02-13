#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
roslib.load_manifest("hrl_pr2_arms")
import rospy
from std_msgs.msg import String
import tf.transformations as tf_trans

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory, PR2ArmCartesianPostureBase
from arm_cart_vel_control import PR2ArmCartVelocityController
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

MOVE_BUTTONS = {'translate_up' : (True, tf_trans.euler_matrix(0, -np.pi/2, 0)[:3,:3]), 
                'translate_down' : (True, tf_trans.euler_matrix(0, np.pi/2, 0)[:3,:3]), 
                'translate_left' : (True, tf_trans.euler_matrix(0, 0, np.pi/2)[:3,:3]), 
                'translate_right' : (True, tf_trans.euler_matrix(0, 0, -np.pi/2)[:3,:3]), 
                'translate_in' : (True, tf_trans.euler_matrix(0, 0, np.pi)[:3,:3]), 
                'translate_out' : (True, tf_trans.euler_matrix(0, 0, 0)[:3,:3]),
                'rotate_x_pos' : (False, tf_trans.euler_matrix(0, 0, 0)[:3,:3]),
                'rotate_x_neg' : (False, tf_trans.euler_matrix(0, 0, np.pi)[:3,:3]),
                'rotate_y_pos' : (False, tf_trans.euler_matrix(0, 0, np.pi/2)[:3,:3]),
                'rotate_y_neg' : (False, tf_trans.euler_matrix(0, 0, -np.pi/2)[:3,:3]),
                'rotate_z_pos' : (False, tf_trans.euler_matrix(0, -np.pi/2, 0)[:3,:3]),
                'rotate_z_neg' : (False, tf_trans.euler_matrix(0, np.pi/2, 0)[:3,:3])}

MONITOR_RATE = 20.
MOVE_STATE_TOPIC = "/arm_control_gui/move_state"
LOAD_ARM_TOPIC = "/arm_control_gui/load_arm"

class ArmCartCtrlBackend(object):
    def __init__(self, r_arm, l_arm, monitor_rate, misses_allowed=5):
        self.misses_allowed = misses_allowed
        self.is_move_connected = False
        self.last_move_time = 0.
        self.misses = 0
        self.current_arm = "l"
        self.arm_switch = False
        self.topic_cmd = ""
        self.active_cmd = ""

        self.r_vel_ctrl = PR2ArmCartVelocityController(r_arm)
        self.l_vel_ctrl = PR2ArmCartVelocityController(l_arm)
        self.trans_vel = 0.01
        self.rot_vel = 0.10

        rospy.Subscriber(MOVE_STATE_TOPIC, String, self.move_state_cb)
        rospy.Subscriber(LOAD_ARM_TOPIC, String, self.load_arm_cb)

    def move_state_cb(self, msg):
        self.is_move_connected = True
        self.last_move_time = rospy.get_time()
        self.misses = 0
        self.topic_cmd = msg.data

    def load_arm_cb(self, msg):
        new_arm = msg.data
        if self.current_arm != new_arm:
            self.arm_switch = True

    def check_arm_switched(self):
        if self.arm_switch:
            self.arm_switch = False
            return True
        return False

    def check_move_state(self):
        if rospy.get_time() - self.last_move_time > 1. / MONITOR_RATE:
            self.misses += 1
            if self.misses > self.misses_allowed:
                self.is_move_connected = False
        return self.is_move_connected

    def backend_loop(self):
        #vel_frame = tf_trans.euler_matrix(0, -np.pi/2, 0)[:3,:3]
        #vel_ctrl.move_velocity(velocity_rot=vel_frame, velocity=0.005, is_translation=False)
        
        vel_ctrl = self.r_vel_ctrl
        r = rospy.Rate(MONITOR_RATE)
        while not rospy.is_shutdown():
            if self.check_move_state():
                if self.topic_cmd != self.active_cmd and self.topic_cmd in MOVE_BUTTONS:
                    self.active_cmd = self.topic_cmd
                    is_translation, vel_frame = MOVE_BUTTONS[self.topic_cmd]
                    if is_translation:
                        vel_val = self.trans_vel
                    else:
                        vel_val = self.rot_vel

                    def move_vel_thread(te):
                        vel_ctrl.move_velocity(velocity_rot=vel_frame, velocity=vel_val, 
                                               is_translation=is_translation)
                    move_vel_timer = rospy.Timer(rospy.Duration(0.05), move_vel_thread, oneshot=True)
                if self.topic_cmd == "":
                    self.active_cmd = ""
                    vel_ctrl.stop_moving()
            else:
                vel_ctrl.stop_moving()

            r.sleep()

def main():
    rospy.init_node("arm_cart_control_backend")
    ctrl_switcher = ControllerSwitcher()
    if False:
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
    l_arm = None
    #l_arm = create_pr2_arm('l', PR2ArmCartesianPostureBase)
    #l_arm.set_posture()
    rospy.sleep(0.2)
    cart_ctrl = ArmCartCtrlBackend(r_arm, l_arm, MONITOR_RATE)
    cart_ctrl.backend_loop()

if __name__ == "__main__":
    main()
