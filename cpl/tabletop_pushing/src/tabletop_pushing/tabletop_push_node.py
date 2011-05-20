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
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from tabletop_pushing.srv import *
from math import sin, cos, pi
import sys

LEFT_ARM_INIT_JOINTS = np.matrix([[7.0e-01, 3.65531104e-01,
                                   1.68462256e+00, -2.2,
                                   2.17482262e+02, -1.41818799e+00,
                                   -8.64819949e+01]]).T
LEFT_ARM_MIDDLE_JOINTS = np.matrix([[7.0e-01, -1.30025955e-01,
                                    1.56307360e+00, -2.2,
                                    2.16694211e+02, -1.45799866e+00,
                                    -8.64421842e+01]]).T
LEFT_ARM_READY_JOINTS = np.matrix([[5.0e-01, -1.30025955e-01,
                                    1.56307360e+00, -1.81768523e+00,
                                    2.16694211e+02, -1.45799866e+00,
                                    -8.64421842e+01]]).T

RIGHT_ARM_INIT_JOINTS = np.matrix([[-7.0e-01, 3.65531104e-01,
                                    -1.68462256e+00, -2.2,
                                    -2.17482262e+02, -1.41818799e+00,
                                    -8.64819949e+01]]).T
RIGHT_ARM_MIDDLE_JOINTS = np.matrix([[-7.0e-01, -1.30025955e-01,
                                    -1.56307360e+00, -2.2,
                                    -2.16694211e+02, -1.45799866e+00,
                                    -8.64421842e+01]]).T
RIGHT_ARM_READY_JOINTS = np.matrix([[-5.0e-01, -1.30025955e-01,
                                    -1.56307360e+00, -1.81768523e+00,
                                    -2.16694211e+02, -1.45799866e+00,
                                    -8.64421842e+01]]).T


INIT_POSE_MOVE_THRESH = 0.5
READY_POSE_MOVE_THRESH = 0.5

class TabletopPushNode:

    def __init__(self, no_arms = False):
        rospy.init_node('tabletop_push_node', log_level=rospy.DEBUG)
        self.gripper_x_offset = rospy.get_param('~gripper_push_start_x_offset',
                                                0.03)
        self.gripper_z_offset = rospy.get_param('~gripper_push_start_z_offset',
                                                0.03)
        use_slip = rospy.get_param('~use_slip_detection', 1)

        self.tf_listener = tf.TransformListener()

        self.no_arms = no_arms

        if not no_arms:
            rospy.loginfo('Creating pr2 object')
            self.robot = pr2.PR2(self.tf_listener, arms=True, base=False)
            rospy.loginfo("Setting up left arm move")
            self.left_arm_move = lm.LinearReactiveMovement('l', self.robot,
                                                           self.tf_listener,
                                                           use_slip, use_slip)
            rospy.loginfo("Setting up right arm move")
            self.right_arm_move = lm.LinearReactiveMovement('r', self.robot,
                                                            self.tf_listener,
                                                            use_slip, use_slip)


        self.push_pose_proxy = rospy.ServiceProxy("get_push_pose", PushPose)
        self.gripper_push_service = rospy.Service('gripper_push',
                                                  GripperPush,
                                                  self.gripper_push_action)
        self.gripper_sweep_service = rospy.Service('gripper_sweep',
                                                   GripperPush,
                                                   self.gripper_sweep_action)

    #
    # Behavior functions
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
            init_joints = LEFT_ARM_INIT_JOINTS
            middle_joints = LEFT_ARM_MIDDLE_JOINTS
            ready_joints = LEFT_ARM_READY_JOINTS
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            robot_gripper = self.robot.right_gripper
            init_joints = RIGHT_ARM_INIT_JOINTS
            middle_joints = RIGHT_ARM_MIDDLE_JOINTS
            ready_joints = RIGHT_ARM_READY_JOINTS

        init_diff = np.linalg.norm(pr2.diff_arm_pose(robot_arm.pose(),
                                                     init_joints))
        ready_diff = np.linalg.norm(pr2.diff_arm_pose(robot_arm.pose(),
                                                      ready_joints))
        push_arm.set_movement_mode_ik()

        # Choose to move to ready first, if it is closer, then move to init
        # TODO: This could be smarter, i.e., don't move up, then down
        moved_ready = False
        if force_ready or ready_diff < init_diff:
            if ready_diff > READY_POSE_MOVE_THRESH or force_ready:
                rospy.loginfo('Moving %s_arm to ready pose' % which_arm)
                robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)
                rospy.loginfo('Moved %s_arm to ready pose' % which_arm)
                moved_ready = True
            else:
                rospy.loginfo('Arm in ready pose')

        # Test for closeness before moving
        if not moved_ready and init_diff < INIT_POSE_MOVE_THRESH:
            rospy.loginfo('Arm in init pose')
            rospy.loginfo('init_diff is: '+str(init_diff))
        else:
            rospy.loginfo('Moving %s_arm to middle pose' % which_arm)
            robot_arm.set_pose(middle_joints, nsecs=2.0, block=True)
            rospy.loginfo('Moved %s_arm to middle pose' % which_arm)

            rospy.loginfo('Moving %s_arm to init pose' % which_arm)
            robot_arm.set_pose(init_joints, nsecs=2.0, block=True)
            rospy.loginfo('Moved %s_arm to init pose' % which_arm)

        # TODO: Check to see if the gripper needs to be closed
        # rospy.loginfo('Opening gripper')
        # res = robot_gripper.open(block=True)
        # rospy.loginfo('Gripper status is: ' + str(res))

        rospy.loginfo('Closing gripper')
        res = robot_gripper.close(block=True)
        rospy.loginfo('Gripper status is: ' + str(res))

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
            middle_joints = LEFT_ARM_MIDDLE_JOINTS
            which_arm = 'l'
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            middle_joints = RIGHT_ARM_MIDDLE_JOINTS
            which_arm = 'r'

        # Offset to higher position to miss the table, but were lower to avoid
        # seeing the arm for now.
        rospy.loginfo("Moving %s_arm to middle pose" % which_arm)
        push_arm.set_movement_mode_ik()
        robot_arm.set_pose(middle_joints, nsecs=2.0, block=True)

        rospy.loginfo("Moving %s_arm to ready pose" % which_arm)
        push_arm.set_movement_mode_ik()
        robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)

        orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, wrist_yaw)
        pose = np.matrix([start_point.x, start_point.y, start_point.z])
        rot = np.matrix([orientation])

        # rospy.loginfo('Pose: ' + str(pose))
        # rospy.loginfo('Ort: ' + str(rot))

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
            np.matrix([(-push_dist + pos_error), 0.0, 0.0]).T,
            stop='pressure', pressure=5000)
        rospy.loginfo('Done moving backwards')

        self.init_arm_pose(force_ready=True)
        response.dist_pushed = push_dist - pos_error
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
            middle_joints = LEFT_ARM_MIDDLE_JOINTS
            push_dir = -1
            which_arm = 'l'
        else:
            push_arm = self.right_arm_move
            robot_arm = self.robot.right
            ready_joints = RIGHT_ARM_READY_JOINTS
            middle_joints = RIGHT_ARM_MIDDLE_JOINTS
            push_dir = +1
            which_arm = 'r'

        # Offset to higher position to miss the table, but were lower to avoid
        # seeing the arm for now.
        rospy.loginfo("Moving %s_arm to middle pose" % which_arm)
        push_arm.set_movement_mode_ik()
        robot_arm.set_pose(middle_joints, nsecs=2.0, block=True)

        rospy.loginfo("Moving %s_arm to ready pose" % which_arm)
        push_arm.set_movement_mode_ik()
        robot_arm.set_pose(ready_joints, nsecs=2.0, block=True)

        orientation = tf.transformations.quaternion_from_euler(0.5*pi, 0.0,
                                                               wrist_yaw)
        pose = np.matrix([start_point.x, start_point.y, start_point.z])
        rot = np.matrix([orientation])

        # TODO: Rotate wrist before moving to position
        # arm_pose = robot_arm.pose()
        # arm_pose[-1] = 0.5*pi
        # push_arm.set_movement_mode_ik()
        # robot_arm.set_pose(arm_pose, nsecs=2.0, block=True)

        # Move to offset pose
        loc = [pose, rot]
        push_arm.set_movement_mode_cart()
        push_arm.move_absolute(loc, stop='pressure', frame=push_frame)
        rospy.loginfo('Done moving to start point')

        # Sweep inward
        # TODO: Change this to split the push_dist in x and y depending on yaw
        # and move relative gripper?
        rospy.loginfo('Sweeping in')
        r, pos_error = push_arm.move_relative_base(
            np.matrix([0.0, push_dir*push_dist, 0.0]).T,
            #'l_wrist_roll_link',
            stop='pressure', pressure=5000)
        rospy.loginfo('Done sweeping in')

        rospy.loginfo('Sweeping outward')
        push_arm.move_relative_base(
            np.matrix([0.0, (-push_dir*push_dist + pos_error), 0.0]).T,
            #'l_wrist_roll_link',
            stop='pressure', pressure=5000)
        rospy.loginfo('Done sweeping outward')

        self.init_arm_pose(force_ready=True)
        response.dist_pushed = push_dist - pos_error
        return response

    #
    # Util Functions
    #
    def print_pose(self):
        cart_pose = self.left_arm_move.current_location()
        pose = self.robot.left.pose()
        rospy.loginfo(str(pose))
        rospy.loginfo('Left Cart_position: ' + str(cart_pose[0]))
        rospy.loginfo('Left Cart_orientation: ' + str(cart_pose[1]))
        cart_pose = self.right_arm_move.current_location()
        pose = self.robot.right.pose()
        rospy.loginfo(str(pose))
        rospy.loginfo('Right Cart_position: ' + str(cart_pose[0]))
        rospy.loginfo('Right Cart_orientation: ' + str(cart_pose[1]))

    #
    # Main Control Loop
    #
    def run(self):
        """
        Main control loop for the node
        """
        if not self.no_arms:
            self.init_arm_pose(True, which_arm='r')
            self.init_arm_pose(True, which_arm='l')
        rospy.spin()

if __name__ == '__main__':
    node = TabletopPushNode(no_arms=False)
    node.run()
