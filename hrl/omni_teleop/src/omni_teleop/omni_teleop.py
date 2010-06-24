#!/usr/bin/env python  
import roslib
roslib.load_manifest('omni_teleop')

import rospy
import tf
import math
import actionlib

from geometry_msgs.msg import PoseStamped
from phantom_omni.msg import PhantomButtonEvent

from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

class OmniPR2Teleop:
    def __init__(self):
        rospy.init_node('omni_frames')
        self.tfbroadcast = tf.TransformBroadcaster()
        self.left_cart_pub = rospy.Publisher('l_cart/command_pose', PoseStamped)
        self.l_gripper_server = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.l_gripper_server.wait_for_server()
        rospy.Subscriber("omni1_button", PhantomButtonEvent, self.omni1_button)

        self.l_gripper_pose = None
        self.r_gripper_pose = None
        self.l_gripper_open = False
        self.l_gripper_close = False

    def omni1_button(self, msg):
        #white button closes
        MAX_EFFORT = 40

        if msg.white_button == 1:
            self.l_gripper_server.send_goal(Pr2GripperCommandGoal(
                                 Pr2GripperCommand(position = 0.0, max_effort = MAX_EFFORT)),
                                 done_cb = self.l_gripper_done,
                                 feedback_cb=self.l_gripper_cb)
            self.l_gripper_close = True
        elif msg.white_button == 0 and self.l_gripper_pose != None and self.l_gripper_close:
            self.l_gripper_server.send_goal(Pr2GripperCommandGoal(
                                 Pr2GripperCommand(position = self.l_gripper_pose-.01, max_effort = MAX_EFFORT))
                                 )
            self.l_gripper_close = False
            self.l_gripper_pose = None
            
        #grey button opens
        if msg.grey_button == 1:
            self.l_gripper_server.send_goal(Pr2GripperCommandGoal(
                                 Pr2GripperCommand(position = 0.09, max_effort = MAX_EFFORT)),
                                 done_cb = self.l_gripper_done,
                                 feedback_cb=self.l_gripper_cb)
            self.l_gripper_open = True
        elif msg.grey_button == 0 and self.l_gripper_pose != None and self.l_gripper_open:
            self.l_gripper_server.send_goal(Pr2GripperCommandGoal(
                                 Pr2GripperCommand(position = self.l_gripper_pose+.01, max_effort = MAX_EFFORT)))
            self.l_gripper_open = False
            self.l_gripper_pose = None

    def l_gripper_done(self, state, msg):
        self.l_gripper_pose = msg.position

    def l_gripper_cb(self, msg):
        self.l_gripper_pose = msg.position
        self.l_effort = msg.effort

    def r_gripper_cb(self, msg):
        self.r_gripper_pose = msg.position
        self.l_gripper_effort = msg.effort

    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            self.tfbroadcast.sendTransform((0.6, 0.3, -0.3),
                             tuple(tf.transformations.quaternion_from_euler(0, 0, math.pi/2)),
                             rospy.Time.now(),
                             "omni1",
                             "torso_lift_link")

            self.tfbroadcast.sendTransform((0.6, -0.3, -0.3),
                             tuple(tf.transformations.quaternion_from_euler(0, 0, math.pi/2)),
                             rospy.Time.now(),
                             "omni2",
                             "torso_lift_link")

            ps = PoseStamped()
            ps.header.frame_id = 'omni1_link6'
            ps.header.stamp = rospy.get_rostime()
            ps.pose.position.x = -.134 
            ps.pose.position.y = 0
            ps.pose.position.z = 0
            q = tf.transformations.quaternion_from_euler(0, math.pi, 0)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            self.left_cart_pub.publish(ps)

            rate.sleep()


if __name__ == '__main__':
    o = OmniPR2Teleop()
    o.run()


