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

class TabletopExecutive:

    def __init__(self):
        # TODO: Replace these parameters with learned / perceived values
        # The offsets should be removed and learned implicitly
        self.gripper_push_dist = rospy.get_param('~gripper_push_dist',
                                                 0.15)
        self.gripper_sweep_dist = rospy.get_param('~gripper_sweep_dist',
                                                 0.15)
        self.gripper_x_offset = rospy.get_param('~gripper_push_start_x_offset',
                                                -0.03)
        self.gripper_z_offset = rospy.get_param('~gripper_push_start_z_offset',
                                                0.00)

        self.sweep_x_offset = rospy.get_param('~gripper_sweep_start_x_offset',
                                              0.10)
        self.sweep_y_offset = rospy.get_param('~gripper_sweep_start_y_offset',
                                              0.05)
        self.sweep_z_offset = rospy.get_param('~gripper_sweep_start_z_offset',
                                              0.04)

        self.push_pose_proxy = rospy.ServiceProxy('get_push_pose', PushPose)
        self.gripper_push_proxy = rospy.ServiceProxy('gripper_push',GripperPush)
        self.gripper_sweep_proxy = rospy.ServiceProxy('gripper_sweep',GripperPush)

    def run(self, num_pushes, num_sweeps):
        for i in xrange(num_pushes):
            self.push_object(self.gripper_push_dist)
        for i in xrange(num_sweeps):
            self.sweep_object(2.0*self.gripper_push_dist)


    def push_object(self, push_dist):
        # Make push_pose service request
        pose_req = PushPoseRequest()
        pose_req.use_laser = False

        # Get push pose
        rospy.loginfo("Calling push pose service")
        try:
            pose_res = self.push_pose_proxy(pose_req)
        except rospy.ServiceException, e:
            rospy.logwarn("Service did not process request: %s"%str(e))
            return

        # Convert pose response to correct push request format
        push_req = GripperPushRequest()
        push_req.start_point.header = pose_res.push_pose.header
        push_req.start_point.point = pose_res.push_pose.pose.position

        # TODO: Correctly set the wrist yaw
        # orientation = pose_res.push_pose.pose.orientation
        wrist_yaw = 0 # 0.25*pi # 0.5*pi
        push_req.wrist_yaw = wrist_yaw
        push_req.desired_push_dist = push_dist

        # TODO: Remove these offsets and incorporate them directly into the perceptual inference
        # Offset pose to not hit the object immediately
        push_req.start_point.point.x += self.gripper_x_offset*cos(wrist_yaw)
        push_req.start_point.point.y += self.gripper_x_offset*sin(wrist_yaw)
        push_req.start_point.point.z += self.gripper_z_offset

        # TODO: Correctly pick which arm to use
        push_req.left_arm = True
        push_req.right_arm = False

        # Call push service
        push_res = self.gripper_push_proxy(push_req)

    def sweep_object(self, push_dist):
        # TODO: Set this up once the push works
        # Make push_pose service request
        pose_req = PushPoseRequest()
        pose_req.use_laser = False

        # Get push pose
        rospy.loginfo("Calling push pose service")
        try:
            pose_res = self.push_pose_proxy(pose_req)
        except rospy.ServiceException, e:
            rospy.logwarn("Service did not process request: %s"%str(e))
            return

        # Convert pose response to correct push request format
        sweep_req = GripperPushRequest()
        sweep_req.start_point.header = pose_res.push_pose.header
        sweep_req.start_point.point = pose_res.push_pose.pose.position
        # rospy.loginfo('Push pose point:' + str(sweep_req.start_point.point))
        # TODO: Correctly set the wrist yaw
        # orientation = pose_res.push_pose.pose.orientation
        wrist_yaw = 0 # 0.25*pi # 0.5*pi
        sweep_req.wrist_yaw = wrist_yaw
        sweep_req.desired_push_dist = push_dist

        # TODO: Correctly pick which arm to use
        sweep_req.left_arm = True
        sweep_req.right_arm = False

        # TODO: Flip the sign depending on which arm is chosen
        # TODO: Remove these offsets and incorporate them directly into the perceptual inference
        # Offset pose to not hit the object immediately
        sweep_req.start_point.point.x += (self.sweep_x_offset*cos(wrist_yaw) +
                                          self.sweep_y_offset*sin(wrist_yaw))
        sweep_req.start_point.point.y += (self.sweep_x_offset*sin(wrist_yaw) +
                                          self.sweep_y_offset*cos(wrist_yaw))
        sweep_req.start_point.point.z += self.sweep_z_offset

        # rospy.loginfo('Sweep start point:' + str(sweep_req.start_point.point))

        # Call push service
        sweep_res = self.gripper_sweep_proxy(sweep_req)

if __name__ == '__main__':
    node = TabletopExecutive()
    node.run(3,2)
