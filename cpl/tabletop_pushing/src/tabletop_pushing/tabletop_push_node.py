#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, Georgia Institute of Technology
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  * Neither the name of the Georgia Institute of Technology nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('tabletop_pushing')
import rospy
import hrl_pr2_lib.linear_move as lm
import hrl_pr2_lib.pr2 as pr2
import hrl_lib.tf_utils as tfu
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from geometry_msgs.msg import PoseStamped
from pr2_controllers_msgs.msg import *
import tf
import numpy as np
from tabletop_pushing.srv import *
from math import sin, cos, pi
import sys

# Setup joints stolen from Kelsey's code.
LEFT_ARM_SETUP_JOINTS = np.matrix([[1.32734204881265387, -0.34601608409943324,
                                    1.4620635485239604, -1.2729772622637399,
                                    7.5123303230158518, -1.5570651396529178,
                                    -5.5929916630672727]]).T
RIGHT_ARM_SETUP_JOINTS = np.matrix([[-1.32734204881265387, -0.34601608409943324,
                           -1.4620635485239604, -1.2729772622637399,
                           -7.5123303230158518, -1.5570651396529178,
                           -7.163787989862169]]).T
LEFT_ARM_READY_JOINTS = np.matrix([[0.42427649, 0.0656137,
                                    1.43411927, -2.11931035,
                                    -15.78839978, -1.64163257,
                                    -17.2947453]]).T
RIGHT_ARM_READY_JOINTS = np.matrix([[-0.42427649, 0.0656137,
                                     -1.43411927, -2.11931035,
                                     15.78839978, -1.64163257,
                                     8.64421842e+01]]).T


READY_POSE_MOVE_THRESH = 0.5

class TabletopPushNode:

    def __init__(self):
        rospy.init_node('tabletop_push_node', log_level=rospy.DEBUG)
        self.torso_z_offset = rospy.get_param('~torso_z_offset', 0.15)
        self.look_pt_x = rospy.get_param('~look_point_x', 0.45)
        self.head_pose_cam_frame = rospy.get_param('~head_pose_cam_frame',
                                                   'openni_rgb_frame')
        self.default_torso_height = rospy.get_param('~default_torso_height',
                                                    0.2)
        self.gripper_raise_dist = rospy.get_param('~graipper_post_push_raise_dist',
                                                  0.05)
        use_slip = rospy.get_param('~use_slip_detection', 1)

        self.tf_listener = tf.TransformListener()

        # Set joint gains
        prefix = roslib.packages.get_pkg_dir('hrl_pr2_arms')+'/params/'
        cs = ControllerSwitcher()
        rospy.loginfo(cs.switch("r_arm_controller", "r_arm_controller",
                                prefix + "pr2_arm_controllers_push.yaml"))
        rospy.loginfo(cs.switch("l_arm_controller", "l_arm_controller",
                                prefix + "pr2_arm_controllers_push.yaml"))

        # Setup arms
        rospy.loginfo('Creating pr2 object')
        self.robot = pr2.PR2(self.tf_listener, arms=True, base=False)
        rospy.loginfo('Setting up left arm move')
        self.left_arm_move = lm.LinearReactiveMovement('l', self.robot,
                                                       self.tf_listener,
                                                       use_slip, use_slip)
        rospy.loginfo('Setting up right arm move')
        self.right_arm_move = lm.LinearReactiveMovement('r', self.robot,
                                                        self.tf_listener,
                                                        use_slip, use_slip)


        self.push_pose_proxy = rospy.ServiceProxy('get_push_pose', PushPose)
        self.gripper_push_service = rospy.Service('gripper_push',
                                                  GripperPush,
                                                  self.gripper_push_action)
        self.gripper_pre_push_service = rospy.Service('gripper_pre_push',
                                                  GripperPush,
                                                  self.gripper_pre_push_action)
        self.gripper_post_push_service = rospy.Service('gripper_post_push',
                                                       GripperPush,
                                                       self.gripper_post_push_action)
        self.gripper_sweep_service = rospy.Service('gripper_sweep',
                                                   GripperPush,
                                                   self.gripper_sweep_action)
        self.gripper_pre_sweep_service = rospy.Service('gripper_pre_sweep',
                                                       GripperPush,
                                                       self.gripper_pre_sweep_action)
        self.gripper_post_sweep_service = rospy.Service('gripper_post_sweep',
                                                        GripperPush,
                                                        self.gripper_post_sweep_action)
        self.overhead_push_service = rospy.Service('overhead_push',
                                                   GripperPush,
                                                   self.overhead_push_action)
        self.overhead_pre_push_service = rospy.Service('overhead_pre_push',
                                                       GripperPush,
                                                       self.overhead_pre_push_action)
        self.overhead_post_push_service = rospy.Service('overhead_post_push',
                                                        GripperPush,
                                                        self.overhead_post_push_action)
        self.raise_and_look_serice = rospy.Service('raise_and_look',
                                                   RaiseAndLook,
                                                   self.raise_and_look_action)

    #
    # Arm pose initialization functions
    #
    def init_arm_pose(self, force_ready=False, which_arm='l'):
        '''
        Move the arm to the initial pose to be out of the way for viewing the
        tabletop
        '''
        if which_arm == 'l':
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            robot_gripper = self.robot.left_gripper
            ready_joints = LEFT_ARM_READY_JOINTS
            setup_joints = LEFT_ARM_SETUP_JOINTS
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            robot_gripper = self.robot.right_gripper
            ready_joints = RIGHT_ARM_READY_JOINTS
            setup_joints = RIGHT_ARM_SETUP_JOINTS

        ready_diff = np.linalg.norm(pr2.diff_arm_pose(robot_arm.pose(),
                                                      ready_joints))
        push_arm.set_movement_mode_ik()

        rospy.loginfo('Moving %s_arm to setup pose' % which_arm)
        robot_arm.set_pose(setup_joints, nsecs=2.0, block=True)
        rospy.loginfo('Moved %s_arm to setup pose' % which_arm)

        rospy.loginfo('Closing %s_gripper' % which_arm)
        res = robot_gripper.close(block=True)
        rospy.loginfo('Closed %s_gripper' % which_arm)
        push_arm.pressure_listener.rezero()

    def reset_arm_pose(self, force_ready=False, which_arm='l'):
        '''
        Move the arm to the initial pose to be out of the way for viewing the
        tabletop
        '''
        if which_arm == 'l':
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            robot_gripper = self.robot.left_gripper
            ready_joints = LEFT_ARM_READY_JOINTS
            setup_joints = LEFT_ARM_SETUP_JOINTS
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            robot_gripper = self.robot.right_gripper
            ready_joints = RIGHT_ARM_READY_JOINTS
            setup_joints = RIGHT_ARM_SETUP_JOINTS
        push_arm.pressure_listener.rezero()

        ready_diff = np.linalg.norm(pr2.diff_arm_pose(robot_arm.pose(),
                                                      ready_joints))
        push_arm.set_movement_mode_ik()

        # Choose to move to ready first, if it is closer, then move to init
        moved_ready = False
        if force_ready or ready_diff > READY_POSE_MOVE_THRESH or force_ready:
            rospy.loginfo('Moving %s_arm to ready pose' % which_arm)
            robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)
            rospy.loginfo('Moved %s_arm to ready pose' % which_arm)
            moved_ready = True
        else:
            rospy.loginfo('Arm in ready pose')


        rospy.loginfo('Moving %s_arm to setup pose' % which_arm)
        robot_arm.set_pose(setup_joints, nsecs=2.0, block=True)
        rospy.loginfo('Moved %s_arm to setup pose' % which_arm)

    #
    # Behavior functions
    #
    def gripper_push_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
        push_arm.pressure_listener.rezero()

        # Push in a straight line
        rospy.loginfo('Pushing forward')
        r, pos_error = push_arm.move_relative_gripper(
            np.matrix([push_dist, 0.0, 0.0]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done pushing forward')

        response.dist_pushed = push_dist - pos_error
        return response

    def gripper_pre_push_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        rospy.loginfo('Have a request frame of: ' + str(push_frame));
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist
        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'

        if request.arm_init:
            rospy.loginfo('Moving %s_arm to ready pose' % which_arm)
            push_arm.set_movement_mode_ik()
            robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)

        orientation = tf.transformations.quaternion_from_euler(0.0, 0.0,
                                                               wrist_yaw)
        pose = np.matrix([start_point.x, start_point.y, start_point.z])
        rot = np.matrix([orientation])

        # Move to start pose
        loc = [pose, rot]
        push_arm.set_movement_mode_cart()
        push_arm.move_absolute(loc, stop='pressure', frame=push_frame)
        rospy.loginfo('Done moving to start point')
        return response

    def gripper_post_push_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'

        # Retract in a straight line
        rospy.loginfo('Moving gripper up')
        push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, self.gripper_raise_dist]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done moving up')

        rospy.loginfo('Moving gripper backwards')
        push_arm.move_relative_gripper(
            np.matrix([-push_dist, 0.0, 0.0]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done moving backwards')

        if request.arm_reset:
            self.reset_arm_pose(True, which_arm)
        return response

    def gripper_sweep_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
            wrist_roll = -pi
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
            wrist_roll = 0.0
        push_arm.pressure_listener.rezero()

        # NOTE: because of the wrist roll orientation, +Z at the gripper
        # equates to negative Y in the torso_lift_link at 0.0 yaw
        # So we flip the push_dist to make things look like one would expect
        rospy.loginfo('Sweeping in')
        r, pos_error = push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, -push_dist]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done sweeping in')

        response.dist_pushed = push_dist - pos_error
        return response

    def gripper_pre_sweep_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
            wrist_roll = -pi
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
            wrist_roll = 0.0

        if request.arm_init:
            rospy.loginfo('Moving %s_arm to ready pose' % which_arm)
            push_arm.set_movement_mode_ik()
            robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)

        orientation = tf.transformations.quaternion_from_euler(0.5*pi, 0.0,
                                                               wrist_yaw)
        pose = np.matrix([start_point.x, start_point.y, start_point.z])
        rot = np.matrix([orientation])

        if request.arm_init:
            # Rotate wrist before moving to position
            rospy.loginfo('Rotating wrist for sweep')
            arm_pose = robot_arm.pose()
            arm_pose[-1] =  wrist_roll
            robot_arm.set_pose(arm_pose, nsecs=2.0, block=True)

        # Move to offset pose
        loc = [pose, rot]
        push_arm.set_movement_mode_cart()
        push_arm.move_absolute(loc, stop='pressure', frame=push_frame)
        rospy.loginfo('Done moving to start point')

        return response

    def gripper_post_sweep_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
            wrist_roll = -pi
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
            wrist_roll = 0.0

        rospy.loginfo('Sweeping outward')
        push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, (push_dist)]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done sweeping outward')

        if request.arm_reset:
            self.reset_arm_pose(True, which_arm)
        return response

    def overhead_push_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
            wrist_pitch = 0.5*pi
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
            wrist_pitch = -0.5*pi

        push_arm.pressure_listener.rezero()

        rospy.loginfo('Pushing forward')
        r, pos_error = push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, push_dist]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done pushing forward')

        response.dist_pushed = push_dist - pos_error
        return response

    def overhead_pre_push_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
            wrist_pitch = 0.5*pi
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
            wrist_pitch = -0.5*pi

        if request.arm_init:
            rospy.loginfo('Moving %s_arm to ready pose' % which_arm)
            push_arm.set_movement_mode_ik()
            robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)

        orientation = tf.transformations.quaternion_from_euler(0.0, 0.5*pi,
                                                               wrist_yaw)
        pose = np.matrix([start_point.x, start_point.y, start_point.z])
        rot = np.matrix([orientation])

        if request.arm_init:
            # Rotate wrist before moving to position
            rospy.loginfo('Rotating elbow for overhead push')
            arm_pose = robot_arm.pose()
            arm_pose[-3] =  wrist_pitch
            robot_arm.set_pose(arm_pose, nsecs=2.0, block=True)

        # Move to offset pose
        loc = [pose, rot]
        push_arm.set_movement_mode_cart()
        push_arm.move_absolute(loc, stop='pressure', frame=push_frame)
        rospy.loginfo('Done moving to start point')

        return response

    def overhead_post_push_action(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
            wrist_pitch = 0.5*pi
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
            wrist_pitch = -0.5*pi

        rospy.loginfo('Moving gripper up')
        push_arm.move_relative_gripper(
            np.matrix([-self.gripper_raise_dist, 0.0, 0.0]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done moving up')
        rospy.loginfo('Pushing reverse')
        push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, -push_dist]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done pushing reverse')

        if request.arm_reset:
            self.reset_arm_pose(True, which_arm)
        return response

    def gripper_push_action_complete(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
        push_arm.pressure_listener.rezero()

        if request.arm_init:
            rospy.loginfo('Moving %s_arm to ready pose' % which_arm)
            push_arm.set_movement_mode_ik()
            robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)

        orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, wrist_yaw)
        pose = np.matrix([start_point.x, start_point.y, start_point.z])
        rot = np.matrix([orientation])

        # Move to start pose
        loc = [pose, rot]
        push_arm.set_movement_mode_cart()
        push_arm.move_absolute(loc, stop='pressure', frame=push_frame)
        rospy.loginfo('Done moving to start point')

        # Push in a straight line
        rospy.loginfo('Pushing forward')
        r, pos_error = push_arm.move_relative_gripper(
            np.matrix([push_dist, 0.0, 0.0]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done pushing forward')

        rospy.loginfo('Moving gripper backwards')
        push_arm.move_relative_gripper(
            np.matrix([-push_dist, 0.0, 0.0]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done moving backwards')

        if request.arm_reset:
            self.reset_arm_pose(True, which_arm)

        response.dist_pushed = push_dist - pos_error
        return response

    def gripper_sweep_action_complete(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
            wrist_roll = -pi
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
            wrist_roll = 0.0
        push_arm.pressure_listener.rezero()

        if request.arm_init:
            rospy.loginfo('Moving %s_arm to ready pose' % which_arm)
            push_arm.set_movement_mode_ik()
            robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)

        orientation = tf.transformations.quaternion_from_euler(0.5*pi, 0.0,
                                                               wrist_yaw)
        pose = np.matrix([start_point.x, start_point.y, start_point.z])
        rot = np.matrix([orientation])

        if request.arm_init:
            # Rotate wrist before moving to position
            rospy.loginfo('Rotating wrist for sweep')
            arm_pose = robot_arm.pose()
            arm_pose[-1] =  wrist_roll
            robot_arm.set_pose(arm_pose, nsecs=2.0, block=True)

        # Move to offset pose
        loc = [pose, rot]
        push_arm.set_movement_mode_cart()
        push_arm.move_absolute(loc, stop='pressure', frame=push_frame)
        rospy.loginfo('Done moving to start point')

        # NOTE: because of the wrist roll orientation, +Z at the gripper
        # equates to negative Y in the torso_lift_link at 0.0 yaw
        # So we flip the push_dist to make things look like one would expect
        rospy.loginfo('Sweeping in')
        r, pos_error = push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, -push_dist]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done sweeping in')

        rospy.loginfo('Sweeping outward')
        push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, (push_dist)]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done sweeping outward')

        if request.arm_reset:
            self.reset_arm_pose(True, which_arm)
        response.dist_pushed = push_dist - pos_error
        return response

    def overhead_push_action_complete(self, request):
        response = GripperPushResponse()
        push_frame = request.start_point.header.frame_id
        start_point = request.start_point.point
        wrist_yaw = request.wrist_yaw
        push_dist = request.desired_push_dist

        if request.left_arm:
            push_arm = self.left_arm_move
            robot_arm = self.robot.left
            ready_joints = LEFT_ARM_READY_JOINTS
            which_arm = 'l'
            wrist_pitch = 0.5*pi
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            which_arm = 'r'
            wrist_pitch = -0.5*pi

        push_arm.pressure_listener.rezero()
        if request.arm_init:
            rospy.loginfo('Moving %s_arm to ready pose' % which_arm)
            push_arm.set_movement_mode_ik()
            robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)

        orientation = tf.transformations.quaternion_from_euler(0.0, 0.5*pi,
                                                               wrist_yaw)
        pose = np.matrix([start_point.x, start_point.y, start_point.z])
        rot = np.matrix([orientation])

        if request.arm_init:
            # Rotate wrist before moving to position
            rospy.loginfo('Rotating elbow for overhead push')
            arm_pose = robot_arm.pose()
            arm_pose[-3] =  wrist_pitch
            robot_arm.set_pose(arm_pose, nsecs=2.0, block=True)

        # Move to offset pose
        loc = [pose, rot]
        push_arm.set_movement_mode_cart()
        push_arm.move_absolute(loc, stop='pressure', frame=push_frame)
        rospy.loginfo('Done moving to start point')

        # Push inward
        rospy.loginfo('Pushing forward')
        r, pos_error = push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, push_dist]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done pushing forward')

        rospy.loginfo('Pushing reverse')
        push_arm.move_relative_gripper(
            np.matrix([0.0, 0.0, -push_dist]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done pushing reverse')

        if request.arm_reset:
            self.reset_arm_pose(True, which_arm)
        response.dist_pushed = push_dist - pos_error
        return response

    def raise_and_look_action(self, request):
        '''
        Service callback to raise the spine to a specific height relative to the
        table height and tilt the head so that the Kinect views the table
        '''
        if request.init_arms:
            self.init_arms()

        if request.point_head_only:
            response = RaiseAndLookResponse()
            response.head_succeeded = self.init_head_pose(request.camera_frame)
            return response

        # Get torso_lift_link position in base_link frame
        (trans, rot) = self.tf_listener.lookupTransform('/base_link',
                                                        '/torso_lift_link',
                                                        rospy.Time(0))
        lift_link_z = trans[2]

        # tabletop position in base_link frame
        table_base = self.tf_listener.transformPose('/base_link',
                                                    request.table_centroid)
        table_z = table_base.pose.position.z
        goal_lift_link_z = table_z + self.torso_z_offset
        lift_link_delta_z = goal_lift_link_z - lift_link_z
        rospy.loginfo('Torso height (m): ' + str(lift_link_z))
        rospy.loginfo('Table height (m): ' + str(table_z))
        rospy.loginfo('Torso goal height (m): ' + str(goal_lift_link_z))
        rospy.loginfo('Torso delta (m): ' + str(lift_link_delta_z))

        # Set goal height based on passed on table height
        # TODO: Set these better
        torso_max = 0.3
        torso_min = 0.01
        current_torso_position = np.asarray(self.robot.torso.pose()).ravel()[0]
        torso_goal_position = current_torso_position + lift_link_delta_z
        torso_goal_position = (max(min(torso_max, torso_goal_position),
                                   torso_min))
        rospy.loginfo('Moving torso to ' + str(torso_goal_position))
        # Multiply by 2.0, because of units of spine
        self.robot.torso.set_pose(torso_goal_position)

        rospy.loginfo('Got torso client result')
        new_torso_position = np.asarray(self.robot.torso.pose()).ravel()[0]
        rospy.loginfo('New torso position is: ' + str(new_torso_position))

        # Get torso_lift_link position in base_link frame
        (new_trans, rot) = self.tf_listener.lookupTransform('/base_link',
                                                            '/torso_lift_link',
                                                            rospy.Time(0))
        new_lift_link_z = new_trans[2]
        rospy.loginfo('New Torso height (m): ' + str(new_lift_link_z))
        # tabletop position in base_link frame
        new_table_base = self.tf_listener.transformPose('/base_link',
                                                        request.table_centroid)
        new_table_z = new_table_base.pose.position.z
        rospy.loginfo('New Table height: ' + str(new_table_z))

        # Point the head at the table centroid
        # NOTE: Should we fix the tilt angle instead for consistency?
        look_pt = np.asmatrix([self.look_pt_x,
                               0.0,
                               -self.torso_z_offset])
        rospy.loginfo('Point head at ' + str(look_pt))
        head_res = self.robot.head.look_at(look_pt,
                                           request.table_centroid.header.frame_id,
                                           request.camera_frame)
        response = RaiseAndLookResponse()
        if head_res:
            rospy.loginfo('Succeeded in pointing head')
            response.head_succeeded = True
        else:
            rospy.loginfo('Failed to point head')
            response.head_succeeded = False
        return response

    #
    # Util Functions
    #
    def print_pose(self):
        pose = self.robot.left.pose()
        rospy.loginfo('Left arm pose: ' + str(pose))
        cart_pose = self.left_arm_move.arm_obj.pose_cartesian_tf()
        rospy.loginfo('Left Cart_position: ' + str(cart_pose[0]))
        rospy.loginfo('Left Cart_orientation: ' + str(cart_pose[1]))
        cart_pose = self.right_arm_move.arm_obj.pose_cartesian_tf()
        pose = self.robot.right.pose()
        rospy.loginfo('Right arm pose: ' + str(pose))
        rospy.loginfo('Right Cart_position: ' + str(cart_pose[0]))
        rospy.loginfo('Right Cart_orientation: ' + str(cart_pose[1]))

    def init_head_pose(self, camera_frame):
        look_pt = np.asmatrix([self.look_pt_x, 0.0, -self.torso_z_offset])
        rospy.loginfo('Point head at ' + str(look_pt))
        head_res = self.robot.head.look_at(look_pt,
                                           'torso_lift_link',
                                           camera_frame)
        if head_res:
            rospy.loginfo('Succeeded in pointing head')
            return True
        else:
            rospy.loginfo('Failed to point head')
            return False

    def init_spine_pose(self):
        rospy.loginfo('Setting spine height to '+str(self.default_torso_height))
        self.robot.torso.set_pose(self.default_torso_height)
        new_torso_position = np.asarray(self.robot.torso.pose()).ravel()[0]
        rospy.loginfo('New spine height is ' + str(new_torso_position))

    def init_arms(self):
        self.init_arm_pose(True, which_arm='r')
        self.init_arm_pose(True, which_arm='l')
        rospy.loginfo('Done initializing arms')

    #
    # Main Control Loop
    #
    def run(self):
        '''
        Main control loop for the node
        '''
        self.init_spine_pose()
        self.init_head_pose(self.head_pose_cam_frame)
        self.init_arms()
        rospy.spin()

if __name__ == '__main__':
    node = TabletopPushNode()
    node.run()
