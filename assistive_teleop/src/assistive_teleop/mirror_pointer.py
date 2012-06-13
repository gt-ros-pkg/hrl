#!/usr/bin/env python

import math
import numpy as np

import roslib; roslib.load_manifest('assistive_teleop')
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PointStamped
from tf import TransformListener, transformations as tft
import pose_utils as pu


class MirrorPointer(object):
    def __init__(self):
        self.tf = TransformListener()
        self.active = True
        self.arm_pose = PoseStamped()
        self.goal_pub = rospy.Publisher('mirror_goal', PoseStamped, latch=True)
        rospy.Subscriber('/head_position', PoseStamped, self.head_pose_cb)

    def curr_pose(self):
        return self.tf.lookupTransform("torso_lift_link",
                                        "r_wrist_roll_link",
                                        rospy.Time(0))

    def head_pose_cb(self, msg):
        if not self.active:
            return
        tp = self.tf.transformPose('/torso_lift_link', msg)
        target_pt = PointStamped(tp.header, tp.pose.position)
        #(ctrans, crot) = self.curr_pose()
        goal = PoseStamped()
        #goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = '/r_gripper_tool_frame'
        goal.pose.position = Point(0.15,0,0)
        goal.pose.orientation = Quaternion(*tft.random_quaternion())
        self.mirror_pub.publish(goal)
        gp = self.tf.transformPose('/torso_lift_link', goal)
        pu.aim_pose_to(gp, target_pt, (1,0,0))
        self.goal_pub.publish(gp)

if __name__=='__main__':
    rospy.init_node('mirror_pointer')
    mp = MirrorPointer()
    rospy.sleep(1)
    mp.head_pose_cb(mp.head_pose)
    rospy.spin()
