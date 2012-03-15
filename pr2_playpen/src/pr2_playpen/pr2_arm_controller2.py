#######################################################################
#
#   USE pr2_object_manipulation/pr2_gripper_reactive_approach/controller_manager.py
#   That code has much of the ideas at the bottom, with more.
#
#######################################################################






# TODO Update code to throw points one at a time.  Sections are labled: "Hack"

import numpy as np, math
from threading import RLock, Timer
import sys

import roslib; roslib.load_manifest('hrl_pr2_lib')
import tf

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus

from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand
from geometry_msgs.msg import PoseStamped

from teleop_controllers.msg import JTTeleopControllerState

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import hrl_lib.transforms as tr
import time

import tf.transformations as tftrans
import types


node_name = "pr2_arms" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

##
# Class for simple management of the arms and grippers.
# Provides functionality for moving the arms, opening and closing
# the grippers, performing IK, and other functionality as it is
# developed.
class PR2Arms(object):

    ##
    # Initializes all of the servers, clients, and variables
    #
    # @param send_delay send trajectory points send_delay nanoseconds into the future
    # @param gripper_point given the frame of the wrist_roll_link, this point offsets
    #                      the location used in FK and IK, preferably to the tip of the
    #                      gripper
    def __init__(self, send_delay=50000000, gripper_point=(0.23,0.0,0.0),
                 force_torque = False):
        log("Loading PR2Arms")

        self.send_delay = send_delay
        self.off_point = gripper_point
        self.gripper_action_client = [actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction),actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)]

        self.gripper_action_client[0].wait_for_server()
        self.gripper_action_client[1].wait_for_server()

        self.arm_state_lock = [RLock(), RLock()]
        self.r_arm_cart_pub = rospy.Publisher('/r_cart/command_pose', PoseStamped)
        self.l_arm_cart_pub = rospy.Publisher('/l_cart/command_pose', PoseStamped)

        rospy.Subscriber('/r_cart/state', JTTeleopControllerState, self.r_cart_state_cb)
        rospy.Subscriber('/l_cart/state', JTTeleopControllerState, self.l_cart_state_cb)

        self.tf_lstnr = tf.TransformListener()

        rospy.sleep(1.)

        log("Finished loading SimpleArmManger")

    def r_cart_state_cb(self, msg):
        trans, quat = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                 'r_gripper_tool_frame', rospy.Time(0))
        rot = tr.quaternion_to_matrix(quat)
        tip = np.matrix([0.12, 0., 0.]).T
        self.r_ee_pos = rot*tip + np.matrix(trans).T
        self.r_ee_rot = rot

        ros_pt = msg.x_desi_filtered.pose.position
        x, y, z = ros_pt.x, ros_pt.y, ros_pt.z
        self.r_cep_pos = np.matrix([x, y, z]).T
        pt = rot.T * (np.matrix([x,y,z]).T - np.matrix(trans).T)
        pt = pt + tip
        self.r_cep_pos_hooktip = rot*pt + np.matrix(trans).T
        ros_quat = msg.x_desi_filtered.pose.orientation
        quat = (ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w)
        self.r_cep_rot = tr.quaternion_to_matrix(quat)

    def l_cart_state_cb(self, msg):
        ros_pt = msg.x_desi_filtered.pose.position
        self.l_cep_pos = np.matrix([ros_pt.x, ros_pt.y, ros_pt.z]).T
        ros_quat = msg.x_desi_filtered.pose.orientation
        quat = (ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w)
        self.l_cep_rot = tr.quaternion_to_matrix(quat)

    def get_ee_jtt(self, arm):
        if arm == 0:
            return self.r_ee_pos, self.r_ee_rot
        else:
            return self.l_ee_pos, self.l_ee_rot

    def get_cep_jtt(self, arm, hook_tip = False):
        if arm == 0:
            if hook_tip:
                return self.r_cep_pos_hooktip, self.r_cep_rot
            else:
                return self.r_cep_pos, self.r_cep_rot
        else:
            return self.l_cep_pos, self.l_cep_rot

    # set a cep using the Jacobian Transpose controller.
    def set_cep_jtt(self, arm, p, rot=None):
        if arm != 1:
            arm = 0
        ps = PoseStamped()
        ps.header.stamp = rospy.rostime.get_rostime()
        ps.header.frame_id = 'torso_lift_link'
 
        ps.pose.position.x = p[0,0]
        ps.pose.position.y = p[1,0]
        ps.pose.position.z = p[2,0]
 
        if rot == None:
            if arm == 0:
                rot = self.r_cep_rot
            else:
                rot = self.l_cep_rot

        quat = tr.matrix_to_quaternion(rot)
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        if arm == 0:
            self.r_arm_cart_pub.publish(ps)
        else:
            self.l_arm_cart_pub.publish(ps)

    # rotational interpolation unimplemented.
    def go_cep_jtt(self, arm, p):
        step_size = 0.01
        sleep_time = 0.1
        cep_p, cep_rot = self.get_cep_jtt(arm)
        unit_vec = (p-cep_p)
        unit_vec = unit_vec / np.linalg.norm(unit_vec)
        while np.linalg.norm(p-cep_p) > step_size:
            cep_p += unit_vec * step_size
            self.set_cep_jtt(arm, cep_p)
            rospy.sleep(sleep_time)
        self.set_cep_jtt(arm, p)
        rospy.sleep(sleep_time)



# TODO Evaluate gripper functions and parameters

    ##
    # Move the gripper the given amount with given amount of effort
    #
    # @param arm 0 for right, 1 for left
    # @param amount the amount the gripper should be opened
    # @param effort - supposed to be in Newtons. (-ve number => max effort)
    def move_gripper(self, arm, amount=0.08, effort = 15):
        self.gripper_action_client[arm].send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=amount, max_effort = effort)))

    ##
    # Open the gripper
    #
    # @param arm 0 for right, 1 for left
    def open_gripper(self, arm):
        self.move_gripper(arm, 0.08, -1)

    ##
    # Close the gripper
    #
    # @param arm 0 for right, 1 for left
    def close_gripper(self, arm, effort = 15):
        self.move_gripper(arm, 0.0, effort)

    # def get_wrist_force(self, arm):
    #     pass

    ######################################################
    # More specific functionality
    ######################################################

if __name__ == '__main__':
    rospy.init_node(node_name, anonymous = True)
    log("Node initialized")

    pr2_arm = PR2Arms()

#    #------- testing set JEP ---------------
#    raw_input('Hit ENTER to begin')
    r_arm, l_arm = 0, 1
#    cep_p, cep_rot = pr2_arm.get_cep_jtt(r_arm)
#    print 'cep_p:', cep_p.A1
#
#    for i in range(5):
#        cep_p[0,0] += 0.01
#        raw_input('Hit ENTER to move')
#        pr2_arm.set_cep_jtt(r_arm, cep_p)


    raw_input('Hit ENTER to move')
    p1 = np.matrix([0.62, 0.0, 0.16]).T
    pr2_arm.go_cep_jtt(r_arm, p1)

    #rospy.sleep(10)
    #pr2_arm.close_gripper(r_arm, effort = -1)
    raw_input('Hit ENTER to move')
    p2 = np.matrix([0.600+0.06, 0.106, -0.32]).T
    pr2_arm.go_cep_jtt(r_arm, p2)

    raw_input('Hit ENTER to move')
    pr2_arm.go_cep_jtt(r_arm, p1)

    raw_input('Hit ENTER to go home')
    home = np.matrix([0.23, -0.6, -0.05]).T
    pr2_arm.go_cep_jtt(r_arm, home)


