#!/usr/bin/env python

import math

import roslib; roslib.load_manifest('assistive_teleop')
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from tf import TransformListener, transformations as tft


class MirrorPointer(object):
    def __init__(self):
        self.tf = TransformListener()
        self.active = True
        self.arm_pose = PoseStamped()
        self.head_pose = PoseStamped()
        self.head_pose.header.frame_id = 'torso_lift_link'
        self.head_pose.pose.position = Point(0.3,0.7,0.9)
        self.head_pose.pose.orientation = Quaternion(1,0,0,0)
        self.test_pub = rospy.Publisher('mirror_test_pose', PoseStamped, latch=True)
        self.test_pub.publish(self.head_pose)
        self.goal_pub = rospy.Publisher('mirror_goal', PoseStamped, latch=True)
        
        rospy.Subscriber('/head_position', PoseStamped, self.head_pose_cb)

    def curr_pose(self):
        return self.tf.lookupTransform("torso_lift_link",
                                        "r_wrist_roll_link",
                                        rospy.Time(0))

    def head_pose_cb(self, msg):
        if not self.active:
            return
        #t = self.tf.getLatestCommonTime(msg.header.frame_id, '/r_wrist_roll_link')
        #print "Latest common time: ", t.to_sec()
        tp = self.tf.transformPose('/r_wrist_roll_link', msg)
        trans = (tp.pose.position.x, tp.pose.position.y, tp.pose.position.z)
        print "Trans: %s, %s, %s" %trans
        rotx = math.atan2(trans[2],trans[1])

        print "X Rot: ", rotx*(180./math.pi)
        roty = math.atan2(trans[0],trans[2])
        print "Y Rot: ", roty*(180./math.pi)
        rotz = math.atan2(trans[1],trans[0])
        print "Z Rot: ", rotz*(180./math.pi)
        quat = tft.quaternion_from_euler(rotx, roty, rotz)
        (ctrans, crot) = self.curr_pose()
        point_quat = tft.quaternion_multiply(crot,quat)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'torso_lift_link'
        goal.pose.position = Point(*ctrans)
        goal.pose.orientation = Quaternion(*point_quat)
        self.goal_pub.publish(goal)

if __name__=='__main__':
    rospy.init_node('mirror_pointer')
    mp = MirrorPointer()
    rospy.sleep(1)
    mp.head_pose_cb(mp.head_pose)
    rospy.spin()
