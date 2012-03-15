#! /usr/bin/python
import sys
import numpy as np

import roslib; roslib.load_manifest("hrl_arm_move_behaviors")
import rospy
import actionlib
import tf
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from hrl_arm_move_behaviors.msg import EPCDirectMoveAction, EPCDirectMoveGoal
from hrl_arm_move_behaviors.msg import EPCLinearMoveAction, EPCLinearMoveGoal
from hrl_arm_move_behaviors.pr2_arm_base import PR2ArmBase

def main():
    rospy.init_node('test_epc_move_actionlib')
    args = sys.argv
    tf_listener = tf.TransformListener()
    dm_client = actionlib.SimpleActionClient(args[1] + '_epc_move/direct_move', 
                                             EPCDirectMoveAction)
    dm_client.wait_for_server()
    lm_client = actionlib.SimpleActionClient(args[1] + '_epc_move/linear_move', 
                                             EPCLinearMoveAction)
    lm_client.wait_for_server()

    raw_input("Do direct move")
    # test direct move
    pos, quat = tf_listener.lookupTransform('torso_lift_link', 
                                            args[1] + '_gripper_tool_frame', rospy.Time())
    new_pos = np.array(pos) + [-0.12, -0.03, -0.02]
    target_pose = PoseStamped()
    target_pose.pose = Pose(Point(*new_pos), Quaternion(*quat))
    target_pose.header.frame_id = 'torso_lift_link'
    target_pose.header.stamp = rospy.Time.now()
    tool_frame = args[1] + '_gripper_tool_frame'
    dm_goal = EPCDirectMoveGoal(target_pose, tool_frame, True, 0.02, 0.35, 0.2, 1.0, 5)
    dm_client.send_goal(dm_goal)

    dm_client.wait_for_result()
    print "Result:", dm_client.get_result().result

    raw_input("Do linear move")
    # test linear move
    lm_goal = EPCLinearMoveGoal()
    pos, quat = tf_listener.lookupTransform('torso_lift_link', 
                                            args[1] + '_gripper_tool_frame', rospy.Time())
    start_pose, end_pose = PoseStamped(), PoseStamped()
    start_pose.pose = Pose(Point(*pos), Quaternion(*quat))
    start_pose.header.frame_id = 'torso_lift_link'
    start_pose.header.stamp = rospy.Time.now()
    new_pos = np.array(pos) + [0.10,  0.00, 0.00]
    end_pose.pose = Pose(Point(*new_pos), Quaternion(*quat))
    end_pose.header.frame_id = 'torso_lift_link'
    end_pose.header.stamp = rospy.Time.now()
    tool_frame = args[1] + '_gripper_tool_frame'
    lm_goal = EPCLinearMoveGoal(start_pose, end_pose, tool_frame, True, 0.03, 0.1, 1.0, 3)
    lm_goal.reset_controllers = False
    lm_client.send_goal(lm_goal)

    lm_client.wait_for_result()
    print "Result:", lm_client.get_result().result

if __name__ == '__main__':
    main()
