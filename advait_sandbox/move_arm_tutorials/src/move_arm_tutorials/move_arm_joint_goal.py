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

from motion_planning_msgs.msg import JointConstraint

from actionlib_msgs.msg import GoalStatus


if __name__ == '__main__':
    import hrl_lib.transforms as tr

    rospy.init_node('move_arm_joint_goal_test')

    move_arm = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
    move_arm.wait_for_server()
    rospy.loginfo('Connected to server')

    goalB = MoveArmGoal()

    names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint',
            'r_upper_arm_roll_joint', 'r_elbow_flex_joint',
            'r_forearm_roll_joint', 'r_wrist_flex_joint',
            'r_wrist_roll_joint']

    goalB.motion_plan_request.group_name = 'right_arm'
    goalB.motion_plan_request.num_planning_attempts = 1
    goalB.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)

    goalB.motion_plan_request.planner_id = ''
    goalB.planner_service_name = 'ompl_planning/plan_kinematic_path'


    import roslib; roslib.load_manifest('darpa_m3')
    import sandbox_advait.pr2_arms as pa
    pr2_arms = pa.PR2Arms()
    raw_input('Move arm to goal location and hit ENTER')
    q = pr2_arms.get_joint_angles(0)
    raw_input('Move arm to start location and hit ENTER')

    q[6] = tr.angle_within_mod180(q[6])
    q[4] = tr.angle_within_mod180(q[4])
    for i in range(7):
        jc = JointConstraint()
        jc.joint_name = names[i]
        jc.position = q[i]
        jc.tolerance_below = 0.1
        jc.tolerance_above = 0.1
        goalB.motion_plan_request.goal_constraints.joint_constraints.append(jc)

    move_arm.send_goal(goalB)
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


