#! /usr/bin/python

import numpy as np
import copy

import roslib; roslib.load_manifest('hrl_arm_move_behaviors')
import rospy
import actionlib
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import std_srvs.srv
import tf

from hrl_arm_move_behaviors.msg import EPCDirectMoveAction, EPCDirectMoveResult
from hrl_arm_move_behaviors.msg import EPCLinearMoveAction, EPCLinearMoveResult
from hrl_arm_move_behaviors.epc_move import EPCMove
import hrl_arm_move_behaviors.util as util

class EPCMoveActionlib(object):
    def __init__(self):
        self.arm = rospy.get_param("~arm", default="r")

        self.tf_listener = tf.TransformListener()
        self.epc_move = EPCMove(self.arm, self.tf_listener)

        self.direct_move_act = actionlib.SimpleActionServer("~direct_move", 
                                                         EPCDirectMoveAction, 
                                                         self.execute_direct_move, False)
        self.direct_move_act.start()

        self.linear_move_act = actionlib.SimpleActionServer("~linear_move", 
                                                               EPCLinearMoveAction, 
                                                               self.execute_linear_move, False)
        self.linear_move_act.start()

        #self.pix3_pub = rospy.Publisher("/lin_pose3", PoseStamped)
        rospy.loginfo("[epc_move_actionlib] EPCMoveActionlib loaded.")

    def execute_direct_move(self, goal):
        rospy.loginfo("[epc_move_actionlib] Execute direct move.")
        result = EPCDirectMoveResult()

        transformed_pose_stamped = self.tf_listener.transformPose("torso_lift_link", goal.target_pose)
        transformed_pose = util.pose_msg_to_mat(transformed_pose_stamped)

        if goal.reset_controllers:
            self.epc_move.reset_controllers()

        def preempt_callback(err_pos, err_ang):
            if self.direct_move_act.is_preempt_requested():
                return "preempted"
            return None

        result.result = self.epc_move.direct_move(transformed_pose, goal.tool_frame,
                                                  goal.err_pos_goal, goal.err_ang_goal,
                                                  goal.err_pos_max, goal.err_ang_max, 
                                                  goal.steady_steps, preempt_callback)

        rospy.loginfo("[epc_move_actionlib] Direct move completed with result '%s'." %
                      result.result)

        if result.result == "succeeded":
            self.direct_move_act.set_succeeded(result)
        elif result.result == "preempted":
            self.direct_move_act.set_preempted(result)
        else:
            self.direct_move_act.set_aborted(result)

    def execute_linear_move(self, goal):
        rospy.loginfo("[epc_move_actionlib] Execute linear move.")
        result = EPCLinearMoveResult()

        start_pose_stamped = self.tf_listener.transformPose("torso_lift_link", goal.start_pose)
        start_pose = util.pose_msg_to_mat(start_pose_stamped)
        end_pose_stamped = self.tf_listener.transformPose("torso_lift_link", goal.end_pose)
        end_pose = util.pose_msg_to_mat(end_pose_stamped)

        if goal.reset_controllers:
            self.epc_move.reset_controllers()

        def preempt_callback(err_pos, err_ang):
            if self.linear_move_act.is_preempt_requested():
                return "preempted"
            return None

        result.result = self.epc_move.linear_move(start_pose, end_pose, goal.tool_frame,
                                                  goal.velocity, 
                                                  goal.err_pos_max, goal.err_ang_max, 
                                                  goal.setup_steps, preempt_callback)

        rospy.loginfo("[epc_move_actionlib] Linear move completed with result '%s'." %
                      result.result)

        if result.result == "succeeded":
            self.linear_move_act.set_succeeded(result)
        elif result.result == "preempted":
            self.linear_move_act.set_preempted(result)
        else:
            self.linear_move_act.set_aborted(result)


def main():
    rospy.init_node("epc_move_actionlib", anonymous=True)
    ema = EPCMoveActionlib()
    rospy.spin()

if __name__ == "__main__":
    main()
