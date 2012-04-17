#! /usr/bin/python

import sys
import numpy as np
import roslib
roslib.load_manifest('hrl_ellipsoidal_control')
roslib.load_manifest("pr2_controllers_msgs")

import rospy
import actionlib

from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

from hrl_pr2_arms.pr2_arm import PR2ArmCartesianPostureBase, PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from hrl_ellipsoidal_control.ellipsoidal_interface_backend import EllipsoidalInterfaceBackend
from hrl_ellipsoidal_control.cartesian_interface_backend import CartesianInterfaceBackend

SETUP_ANGLES = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]

class EllipsoidControlExperiment(object):
    def __init__(self, interface_backend):
        self.ctrl_switcher = ControllerSwitcher()
        self.torso_sac = actionlib.SimpleActionClient('torso_controller/position_joint_action',
                                                      SingleJointPositionAction)
        self.torso_sac.wait_for_server()
        self.gripper_sac = actionlib.SimpleActionClient(
                                          '/l_gripper_controller/gripper_action',
                                          Pr2GripperCommandAction)
        self.gripper_sac.wait_for_server()
        self.cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, 
                                       controller_name='%s_cart_jt_task', 
                                       end_link="%s_gripper_shaver45_frame")
        self.backend = interface_backend(self.cart_arm)
        rospy.loginfo("[ell_control_experiment] EllipsoidControlExperiment ready.")

    def adjust_torso(self):
        # move torso up
        tgoal = SingleJointPositionGoal()
        tgoal.position = 0.238  # all the way up is 0.300
        tgoal.min_duration = rospy.Duration( 2.0 )
        tgoal.max_velocity = 1.0
        self.torso_sac.send_goal_and_wait(tgoal)

    def close_gripper(self):
        ggoal = Pr2GripperCommandGoal()
        ggoal.command.position = 0.0
        ggoal.command.max_effort = 10.0
        self.gripper_sac.send_goal_and_wait(ggoal)

    def move_to_setup_pose(self):
        self.ctrl_switcher.carefree_switch('l', '%s_arm_controller', None)
        rospy.sleep(1)
        joint_arm = create_pr2_arm('l', PR2ArmJointTrajectory)

        setup_angles = SETUP_ANGLES
        joint_arm.set_ep(setup_angles, 5)
        rospy.sleep(6)

    def setup_controller(self):
        self.ctrl_switcher.carefree_switch('l', '%s_cart_jt_task', 
                                           "$(find hrl_rfh_fall_2011)/params/l_jt_task_shaver45.yaml") 
        rospy.sleep(2)
        setup_angles = SETUP_ANGLES
        self.cart_arm.set_posture(setup_angles)
        self.cart_arm.set_gains([200, 800, 800, 80, 80, 80], [15, 15, 15, 1.2, 1.2, 1.2])
        self.backend.ell_controller.reset_arm_orientation(8)

    def run_experiment(self):
        self.backend.disable_interface("Setting up arm.")
        self.adjust_torso()
        self.close_gripper()
        self.move_to_setup_pose()
        self.setup_controller()
        self.backend.enable_interface("Controller ready.")

def main():
    if sys.argv[1] == "ell":
        interface_backend = EllipsoidalInterfaceBackend
    elif sys.argv[1] == "cart":
        interface_backend = CartesianInterfaceBackend
    else:
        print "Argument must be either 'ell' or 'cart'"
    rospy.init_node("ell_control_experiment")
    ece = EllipsoidControlExperiment(interface_backend)
    ece.run_experiment()
    rospy.spin()
    ece.backend.print_statistics()
    if len(sys.argv) >= 3:
        ece.backend.save_statistics(sys.argv[2])

if __name__ == "__main__":
    main()
