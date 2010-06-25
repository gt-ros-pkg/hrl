#!/usr/bin/env python  
import roslib
roslib.load_manifest('omni_teleop')

import rospy
import tf
import math
import actionlib
import tf.transformations as tr
import numpy as np

from geometry_msgs.msg import PoseStamped
from phantom_omni.msg import PhantomButtonEvent

from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

class OmniButtonHandler:
    def __init__(self, topic, controller):
        #rospy.Subscriber("omni1_button", PhantomButtonEvent, self.omni1_button)
        rospy.Subscriber(topic, PhantomButtonEvent, self.button_event)
        #self.gripper_server = actionlib.SimpleActionClient(controller + 'l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.gripper_server = actionlib.SimpleActionClient(controller + '/gripper_action', Pr2GripperCommandAction)
        self.gripper_server.wait_for_server()
        self.gripper_pose = None
        self.gripper_open = False
        self.gripper_close = False
        self.MAX_EFFORT = 40

    def button_event(self, msg):
        #white button closes

        if msg.white_button == 1:
            self.gripper_server.send_goal(Pr2GripperCommandGoal(
                                 Pr2GripperCommand(position = 0.0, max_effort = self.MAX_EFFORT)),
                                 done_cb = self.gripper_done_cb,
                                 feedback_cb=self.gripper_fb_cb)
            self.gripper_close = True
        elif msg.white_button == 0 and self.gripper_pose != None and self.gripper_close:
            self.gripper_server.send_goal(Pr2GripperCommandGoal(
                                 Pr2GripperCommand(position = self.l_gripper_pose-.01, max_effort = self.MAX_EFFORT))
                                 )
            self.gripper_close = False
            self.gripper_pose = None
            
        #grey button opens
        if msg.grey_button == 1:
            self.gripper_server.send_goal(Pr2GripperCommandGoal(
                                 Pr2GripperCommand(position = 0.09, max_effort = self.MAX_EFFORT)),
                                 done_cb = self.gripper_done_cb,
                                 feedback_cb=self.gripper_fb_cb)
            self.l_gripper_open = True
        elif msg.grey_button == 0 and self.gripper_pose != None and self.l_gripper_open:
            self.gripper_server.send_goal(Pr2GripperCommandGoal(
                                 Pr2GripperCommand(position = self.gripper_pose+.01, max_effort = self.MAX_EFFORT)))
            self.l_gripper_open = False
            self.gripper_pose = None

    def gripper_done_cb(self, state, msg):
        self.gripper_pose = msg.position

    def gripper_fb_cb(self, msg):
        self.gripper_pose = msg.position
        self.effort = msg.effort

class OmniPR2Teleop:
    def __init__(self):
        rospy.init_node('omni_frames')
        self.tfbroadcast = tf.TransformBroadcaster()
        self.tflistener = tf.TransformListener()

        #self.left_cart_pub = rospy.Publisher('l_cart/command_pose', PoseStamped)
        self.left_cart_pub = rospy.Publisher('pr2_right', PoseStamped)
        self.omni1_button_handler = OmniButtonHandler('omni1_button', 'l_gripper_controller')
        self.omni2_button_handler = OmniButtonHandler('omni2_button', 'r_gripper_controller')


    def run(self):
        print 'OmniPR2Teleop: running...'
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

            #Get the pose for tip (link6) in link0, multiply by scaling factor, publish back into command_pose
            tip_6 = np.matrix(tr.translation_matrix([-.134, 0, 0])) * np.matrix(tr.quaternion_matrix(tr.quaternion_from_euler(0, math.pi, 0)))
            (trans1, quat1) = self.tflistener.lookupTransform('/omni1_link0', '/omni1_link6', rospy.Time(0))
            z_T_6 = np.matrix(tr.translation_matrix(trans1)) * np.matrix(tr.quaternion_matrix(quat1)) 
            tip_0 = z_T_6 * tip_6

            tip_t = tr.translation_from_matrix(tip_0)
            tip_q = tr.quaternion_from_matrix(tip_0)
            #print tip_t, tip_q, ' == ', tr.quaternion_from_euler(0, math.pi, 0)

            ps = PoseStamped()
            ps.header.frame_id = 'omni1_link0'
            ps.header.stamp = rospy.get_rostime()
            ps.pose.position.x = 1. * tip_t[0] 
            ps.pose.position.y = 1. * tip_t[1]
            ps.pose.position.z = 1. * tip_t[2]
            q = tip_q
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            self.left_cart_pub.publish(ps)
            rate.sleep()


if __name__ == '__main__':
    o = OmniPR2Teleop()
    o.run()


