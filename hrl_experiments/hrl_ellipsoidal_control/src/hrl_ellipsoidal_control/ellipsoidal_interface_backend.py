#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("hrl_ellipsoidal_control")
import rospy
import tf.transformations as tf_trans
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse

from hrl_ellipsoidal_control.ellipsoid_controller import EllipsoidController
from hrl_pr2_arms.pr2_arm import PR2ArmCartesianPostureBase, PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

class ControllerInterfaceBackend(object):
    def __init__(self, name):
        self.ctrl_name = name
        self.button_clk_sub = rospy.Subscriber("/arm_cart_ctrl_gui/button_clk", String,
                                               self._button_clk_cb)
        self.buttons_enable_srv = rospy.ServiceProxy("/arm_cart_ctrl_gui/buttons_enable", Empty)
        self.buttons_disable_srv = rospy.ServiceProxy("/arm_cart_ctrl_gui/buttons_disable", Empty)
        self.set_ctrl_name_pub = rospy.Publisher("/arm_cart_ctrl_gui/set_controller_name", String)
        self.set_status_pub = rospy.Publisher("/arm_cart_ctrl_gui/set_status", String)

        rospy.sleep(0.1)
        self.set_ctrl_name_pub.publish(self.ctrl_name)
        self.buttons_enable_srv()

    def _button_clk_cb(self, msg):
        self.set_ctrl_name_pub.publish(self.ctrl_name)
        self.set_status_pub.publish("Moving arm.")
        self.buttons_disable_srv()
        self.run_controller(msg.data)
        self.set_status_pub.publish("Finished moving arm.")
        rospy.sleep(0.1)
        self.buttons_enable_srv()

    def run_controller(self, button_press):
        rospy.logerr("run_controller not implemented!")

LOCAL_VELOCITY = 0.0025
LATITUDE_STEP = 0.12
LONGITUDE_STEP = 0.06
HEIGHT_STEP = 0.17
trans_params = {'translate_up' : (-LONGITUDE_STEP, 0, 0),   'translate_down' : (LONGITUDE_STEP, 0, 0),
                'translate_right' : (0, -LATITUDE_STEP, 0), 'translate_left' : (0, LATITUDE_STEP, 0),
                'translate_in' : (0, 0, HEIGHT_STEP),      'translate_out' : (0, 0, -HEIGHT_STEP)}

ROT_VELOCITY = 0.002
ROLL_STEP = np.pi/12
PITCH_STEP = np.pi/12
YAW_STEP = np.pi/12
rot_params = {'rotate_x_pos' : (-ROLL_STEP, 0, 0), 'rotate_x_neg' : (ROLL_STEP, 0, 0),
              'rotate_y_pos' : (0, PITCH_STEP, 0), 'rotate_y_neg' : (0, -PITCH_STEP, 0),
              'rotate_z_pos' : (0, 0, -YAW_STEP),  'rotate_z_neg' : (0, 0, YAW_STEP)}

class EllipsoidInterfaceBackend(ControllerInterfaceBackend):
    def __init__(self):
        super(EllipsoidInterfaceBackend, self).__init__("Ellipsoid Controller")
        self.cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, 
                                       controller_name='%s_cart_jt_task', 
                                       end_link="%s_gripper_shaver45_frame")

        self.ell_controller = EllipsoidController(self.cart_arm)
        rospy.sleep(1)
        if False:
            #cart_arm.set_posture(setup_angles)
            setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
            self.cart_arm.set_posture(setup_angles)
            self.cart_arm.set_gains([200, 800, 800, 80, 80, 80], [15, 15, 15, 1.2, 1.2, 1.2])
            self.ell_controller.reset_arm_orientation(8)

    def run_controller(self, button_press):
        quat_gripper_rot = tf_trans.quaternion_from_euler(np.pi, 0, 0)
        if button_press in trans_params:
            change_trans_ep = trans_params[button_press]
            self.ell_controller.execute_ell_move((change_trans_ep, (0, 0, 0)), ((0, 0, 0), 0), 
                                                 quat_gripper_rot, LOCAL_VELOCITY)
        elif button_press in rot_params:
            change_rot_ep = rot_params[button_press]
            self.ell_controller.execute_ell_move(((0, 0, 0), change_rot_ep), ((0, 0, 0), 0), 
                                                 quat_gripper_rot, ROT_VELOCITY)
        elif button_press == "reset_rotation":
            self.ell_controller.execute_ell_move(((0, 0, 0), np.mat(np.eye(3))), ((0, 0, 0), 1), 
                                                 quat_gripper_rot, ROT_VELOCITY)

def main():
    rospy.init_node("ellipsoidal_controller_backend")

    if False:
        ctrl_switcher = ControllerSwitcher()
        ctrl_switcher.carefree_switch('l', '%s_arm_controller', None)
        rospy.sleep(1)
        joint_arm = create_pr2_arm('l', PR2ArmJointTrajectory)

        setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
        joint_arm.set_ep(setup_angles, 5)
        rospy.sleep(5)

        ctrl_switcher.carefree_switch('l', '%s_cart_jt_task', 
                                      "$(find hrl_rfh_fall_2011)/params/l_jt_task_shaver45.yaml") 
        rospy.sleep(1)

    ell_backend = EllipsoidInterfaceBackend()
    rospy.spin()

if __name__ == "__main__":
    main()
