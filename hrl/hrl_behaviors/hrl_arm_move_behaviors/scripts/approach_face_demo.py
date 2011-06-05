#! /usr/bin/python
import random
import sys
import numpy as np

import roslib; roslib.load_manifest("hrl_arm_move_behaviors")
import rospy
import actionlib
import tf
import tf.transformations as tf_trans
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from hrl_arm_move_behaviors.msg import LinearMoveRelativeGoal, LinearMoveRelativeAction
from hrl_arm_move_behaviors.msg import LinearMoveRelativeResult, LinearMoveRelativeSetupGoal
from hrl_arm_move_behaviors.msg import LinearMoveRelativeSetupAction, LinearMoveRelativeSetupResult

class TestLinearMove(object):
    def __init__(self):
        args = sys.argv
        self.move_setup_client = actionlib.SimpleActionClient(
                                              args[1] + '_linear_arm_move/move_relative_setup', 
                                              LinearMoveRelativeSetupAction)
        self.move_setup_client.wait_for_server()
        self.move_client = actionlib.SimpleActionClient(
                                              args[1] + '_linear_arm_move/move_relative', 
                                              LinearMoveRelativeAction)
        self.tf_listener = tf.TransformListener()
        self.move_client.wait_for_server()
        rospy.Subscriber("/pixel3d", 
                         PoseStamped, self.pixel23d_callback)
        self.pix1_pub = rospy.Publisher("/lin_pose1", PoseStamped)
        self.pix2_pub = rospy.Publisher("/lin_pose2", PoseStamped)

    def pixel23d_callback(self, msg):
        msg.header.stamp = rospy.Time.now()
        self.tf_listener.waitForTransform("torso_lift_link", msg.header.frame_id, msg.header.stamp, rospy.Duration(4))
        msg = self.tf_listener.transformPose("torso_lift_link", msg)
        x, y, z = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.pix1_pub.publish(msg)
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, 
                msg.pose.orientation.z, msg.pose.orientation.w]
        quat_flip_rot = tf_trans.quaternion_from_euler(0.0, np.pi/2.0, 0.0)
        quat_flipped = tf_trans.quaternion_multiply(quat, quat_flip_rot)
        mat_flipped = tf_trans.quaternion_matrix(quat_flipped)
        rot_angle = np.arctan(-mat_flipped[2,1] / mat_flipped[2,2])
        quat_ortho_rot = tf_trans.quaternion_from_euler(rot_angle + np.pi, 0.0, 0.0)
        quat_flipped_ortho = tf_trans.quaternion_multiply(quat_flipped, quat_ortho_rot)
        (msg.pose.orientation.x, msg.pose.orientation.y, 
         msg.pose.orientation.z, msg.pose.orientation.w) = quat_flipped_ortho

        setup_goal = LinearMoveRelativeSetupGoal()
        setup_goal.goal_pose = msg
        setup_goal.approach_dist = 0.3
        self.pix2_pub.publish(setup_goal.goal_pose)
        self.move_setup_client.send_goal(setup_goal)
        self.move_setup_client.wait_for_result()
        setup_result = self.move_setup_client.get_result()
        if self.move_setup_client.get_state() == GoalStatus.ABORTED:
            print "No IK solution for setup"
            return
        rospy.sleep(1.5)

        move_goal = LinearMoveRelativeGoal()
        move_goal.distance = 0.02
        move_goal.goal_pose = PoseStamped()
        move_goal.goal_pose.header.frame_id = msg.header.frame_id
        move_goal.goal_pose.header.stamp = rospy.Time.now()
        move_goal.goal_pose.pose.position = Point(x, y, z)
        move_goal.goal_pose.pose.orientation = Quaternion(*quat_flipped_ortho)
        self.move_client.send_goal(move_goal)
        self.move_client.wait_for_result()
        move_result = self.move_client.get_result()
        if self.move_client.get_state() == GoalStatus.ABORTED:
            print "No IK solution for move"
            return

def main():
    rospy.init_node('test_linear_move')
    tlm = TestLinearMove()
    rospy.spin()

if __name__ == '__main__':
    main()
