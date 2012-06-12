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
        self.head_pose.pose.position = Point(1,0.5,0.5)
        self.head_pose.pose.orientaiton = Quaternion(1,0,0,0)
        self.test_pub = rospy.Publisher('mirror_test_pose', PoseStamped, latch=True)
        self.test_pub.publish(self.head_pose)
        self.goal_pub = rospy.Publisher('mirror_goal', PoseStamped, latch=True)
        
        rospy.subscribe('/head_position', PoseStamped, head_pose_cb)

    def curr_pose(self):
        (trans,rot) = self.tf.lookupTransform("/torso_lift_link",
                                            "/r_wrist_roll_link",
                                            rospy.Time(0))
        return (trans, rot)
        #cp = PoseStamped()
        #cp.header.stamp = rospy.Time.now()
        #cp.header.frame_id = '/torso_lift_link'
        #cp.pose.position = Point(*trans)
        #cp.pose.orientation = Quaternion(*rot)
        #return cp
        
    def head_pose_cb(self, msg):
        if not self.active:
            return
        (trans, rot) = self.tf.transformPose(msg.header.frame_id,
                                              '/r_gripper_tool_frame',
                                              rospy.Time.now())
        rotz = math.atan2(trans[1],trans[0])
        roty = math.atan2(trans[0],trans[2])
        quat = tft.quaternion_from_euler((0., roty, rotz))
        (ctrans, crot) = curr_pose()
        point_quat = tft.quaternion_multiply(crot,quat)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'torso_lift_link'
        goal.pose.position = Point(*ctrans)
        goal.pose.orientation = Quaternion(*point_quat)




if __name__=='__main__':
    rospy.init_node('mirror_pointer')
    mp = MirrorPointer()
    mp.head_pose_cb(mp.head_pose)
    rospy.spin()
