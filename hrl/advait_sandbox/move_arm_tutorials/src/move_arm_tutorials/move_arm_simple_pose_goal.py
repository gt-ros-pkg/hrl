#
# Copyright (c) 2010, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#

# Author: Advait Jain (advait@cc.gatech.edu), Healthcare Robotics Lab, Georgia Tech


import roslib; roslib.load_manifest('move_arm_tutorials')

import rospy
import actionlib

import geometric_shapes_msgs

from move_arm_msgs.msg import MoveArmAction
from move_arm_msgs.msg import MoveArmGoal
from motion_planning_msgs.msg import SimplePoseConstraint

from motion_planning_msgs.msg import PositionConstraint
from motion_planning_msgs.msg import OrientationConstraint

from actionlib_msgs.msg import GoalStatus

def pose_constraint_to_position_orientation_constraints(pose_constraint):
    position_constraint = PositionConstraint()
    orientation_constraint = OrientationConstraint()
    position_constraint.header = pose_constraint.header
    position_constraint.link_name = pose_constraint.link_name
    position_constraint.position = pose_constraint.pose.position

    position_constraint.constraint_region_shape.type = geometric_shapes_msgs.msg.Shape.BOX
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

    position_constraint.constraint_region_orientation.x = 0.0
    position_constraint.constraint_region_orientation.y = 0.0
    position_constraint.constraint_region_orientation.z = 0.0
    position_constraint.constraint_region_orientation.w = 1.0

    position_constraint.weight = 1.0

    orientation_constraint.header = pose_constraint.header
    orientation_constraint.link_name = pose_constraint.link_name
    orientation_constraint.orientation = pose_constraint.pose.orientation
    orientation_constraint.type = pose_constraint.orientation_constraint_type

    orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
    orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
    orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
    orientation_constraint.weight = 1.0

    return position_constraint, orientation_constraint

def add_goal_constraint_to_move_arm_goal(pose_constraint, move_arm_goal):
    position_constraint, orientation_constraint = pose_constraint_to_position_orientation_constraints(pose_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)


if __name__ == '__main__':
    rospy.init_node('move_arm_pose_goal_test')

    move_arm = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
    move_arm.wait_for_server()
    rospy.loginfo('Connected to server')

    goalA = MoveArmGoal()
    goalA.motion_plan_request.group_name = 'right_arm'
    goalA.motion_plan_request.num_planning_attempts = 1
    goalA.motion_plan_request.planner_id = ''
    goalA.planner_service_name = 'ompl_planning/plan_kinematic_path'
    goalA.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)

    desired_pose = SimplePoseConstraint()
    desired_pose.header.frame_id = 'torso_lift_link'
    desired_pose.link_name = 'r_gripper_l_fingertip_link'
    desired_pose.pose.position.x = 0.75
    desired_pose.pose.position.y = -0.188
    desired_pose.pose.position.z = 0

    desired_pose.pose.orientation.x = 0.0
    desired_pose.pose.orientation.y = 0.0
    desired_pose.pose.orientation.z = 0.0
    desired_pose.pose.orientation.w = 1.0

    desired_pose.absolute_position_tolerance.x = 0.02
    desired_pose.absolute_position_tolerance.y = 0.02
    desired_pose.absolute_position_tolerance.z = 0.02

    desired_pose.absolute_roll_tolerance = 0.04
    desired_pose.absolute_pitch_tolerance = 0.04
    desired_pose.absolute_yaw_tolerance = 0.04

    add_goal_constraint_to_move_arm_goal(desired_pose, goalA)

    move_arm.send_goal(goalA)
    finished_within_time = move_arm.wait_for_result(rospy.Duration(200.0))

    if not finished_within_time:
        move_arm.cancel_goal()
        rospy.loginfo("Timed out achieving goal A")
    else:
        state = move_arm.get_state()

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Action finished and was successful.')
        else:
            rospy.loginfo('Action failed: %d'%(state))



