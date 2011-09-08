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

class TouchFace(object):
    def __init__(self, arm):
        self.move_setup_client = actionlib.SimpleActionClient(
                                              arm + '_linear_arm_move/move_relative_setup', 
                                              LinearMoveRelativeSetupAction)
        self.move_setup_client.wait_for_server()
        self.move_client = actionlib.SimpleActionClient(
                                              arm + '_linear_arm_move/move_relative', 
                                              LinearMoveRelativeAction)
        self.tf_listener = tf.TransformListener()
        self.move_client.wait_for_server()
        rospy.Subscriber("/pixel3d", 
                         PoseStamped, self.pixel_2_3d_callback)
        self.pix1_pub = rospy.Publisher("/lin_pose1", PoseStamped)
        self.pix2_pub = rospy.Publisher("/lin_pose2", PoseStamped)

    def pixel_2_3d_callback(self, msg):
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
        msg.pose.orientation = Quaternion(*quat_flipped_ortho)
        touch_face_at_pose(msg)

    def touch_face_at_pose(self, pose_stamped):
        setup_goal = LinearMoveRelativeSetupGoal()
        setup_goal.goal_pose = pose_stamped
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
        move_goal.goal_pose = pose_stamped
        move_goal.goal_pose.header.stamp = rospy.Time.now()
        self.move_client.send_goal(move_goal)
        self.move_client.wait_for_result()
        move_result = self.move_client.get_result()
        if self.move_client.get_state() == GoalStatus.ABORTED:
            print "No IK solution for move"
            return

def main():
    rospy.init_node('touch_face')
    touch_face = TouchFace(sys.argv[1])
    rospy.spin()

if __name__ == '__main__':
    main()
