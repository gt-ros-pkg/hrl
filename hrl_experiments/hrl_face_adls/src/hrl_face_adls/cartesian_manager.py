#!/usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("hrl_face_adls")

import rospy
from geometry_msgs.msg import TwistStamped

from hrl_ellipsoidal_control.controller_base import CartesianStepController
from pykdl_utils.pr2_kin import kin_from_param
from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmCartesianBase, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from face_adls_manager import async_call

VELOCITY = 0.001

class CartesianControllerManager(object):
    def __init__(self, arm_char):
        self.arm_char = arm_char
        self.kin = None
        self.cart_ctrl = CartesianStepController()
        self.ctrl_switcher = ControllerSwitcher()
        self.command_move_sub = rospy.Subscriber("/face_adls/%s_cart_move" % arm_char, TwistStamped, 
                                                 async_call(self.command_move_cb))

    def enable_controller(self, end_link="%s_gripper_shaver45_frame",
                          ctrl_params="$(find hrl_face_adls)/params/l_jt_task_shaver45.yaml",
                          ctrl_name='%s_cart_jt_task'):

        self.ctrl_switcher.carefree_switch(self.arm_char, ctrl_name, ctrl_params, reset=False)
        rospy.sleep(0.2)
        cart_arm = create_pr2_arm(self.arm_char, PR2ArmJTransposeTask, 
                                  controller_name=ctrl_name, 
                                  end_link=end_link, timeout=5)
        self.cart_ctrl.set_arm(cart_arm)
        self.arm = cart_arm

    def command_move_cb(self, msg):
        if self.kin is None or msg.header.frame_id not in self.kin.get_segment_names():
            self.kin = kin_from_param("torso_lift_link", msg.header.frame_id)
        torso_pos_ep, torso_rot_ep = self.arm.get_ep()
        torso_B_ref = self.kin.forward_filled(base_segment="torso_lift_link", 
                                              target_segment=msg.header.frame_id)
        _, torso_rot_ref = PoseConverter.to_pos_rot(torso_B_ref)
        ref_pos_off, ref_rot_off = PoseConverter.to_pos_rot(msg)
        change_pos = torso_rot_ref * ref_pos_off
        ep_rot_ref = torso_rot_ep.T * torso_rot_ref
        change_rot = ep_rot_ref * ref_rot_off * ep_rot_ref.T
        _, change_rot_rpy = PoseConverter.to_pos_euler(np.mat([0]*3).T, change_rot)
        change_rot_rpy = [0, 0, 0]
        self.cart_ctrl.execute_cart_move((change_pos.T.A[0], change_rot_rpy), ((0, 0, 0), 0), 
                                         velocity=VELOCITY, blocking=True)

def main():
    rospy.init_node("cartesian_manager")
    cart_man = CartesianControllerManager('r')
    cart_man.enable_controller(end_link="r_gripper_tool_frame", ctrl_name="r_cart_jt_task_tool",
                               ctrl_params="$(find hrl_face_adls)/params/r_jt_task_tool.yaml")
    rospy.spin()

if __name__ == "__main__":
    main()
