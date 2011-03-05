#!/usr/bin/env python  
import roslib
roslib.load_manifest('pr2_playpen')

import rospy
import tf
import math
import tf.transformations as tr
import numpy as np
import hrl_lib.tf_utils as tfu
#import threading
from geometry_msgs.msg import PoseStamped
#from phantom_omni.msg import OmniFeedback
from teleop_controllers.msg import JTTeleopControllerState
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
import actionlib
###COULD use the accelerometer in a smart way too if want to....
#from pr2_msgs.msg import AccelerometerState


def limit_range(numb, lower, upper):
    if lower > upper:
        raise RuntimeError('lower bound > upper bound, wa?')
    return max(min(numb, upper), lower)


class ControlPR2Arm:

    def __init__(self, pr2_control_topic, # = 'l_cart/command_pose',
            gripper_control_topic,  #'l_gripper_controller'
            gripper_tip_frame, #'l_gripper_tool_frame'
            center_in_torso_frame, #=[1,.3,-1], 
            tfbroadcast, #=None,
            tflistener): #=None):

        self.zero_out_forces = True

        self.X_BOUNDS = [.3, 1.5] #Bound translation of PR2 arms in the X direction (in torso link frame)
        self.center_in_torso_frame = center_in_torso_frame
        self.tflistener = tflistener
        self.tfbroadcast = tfbroadcast
        self.gripper_tip_frame = gripper_tip_frame
        self.prev_dt = 0.0
        self.tip_t = np.zeros((3,1))
        self.tip_q = np.zeros((4,1))
        self.gripper_server = actionlib.SimpleActionClient(gripper_control_topic+'/gripper_action', Pr2GripperCommandAction)
        self.gripper_server.wait_for_server()
        rate = rospy.Rate(100.0)

        success = False
        while not success and (not rospy.is_shutdown()):
            rate.sleep()
            try:
                print "need to implement tf stuff still"
                success = True
            except tf.LookupException, e:
                pass
            except tf.ConnectivityException, e:
                pass
        rospy.loginfo('Finished linking frame (but not really)')

        self.pr2_pub = rospy.Publisher(pr2_control_topic, PoseStamped)

#####################################################################################################################
########################I will need a static transform like this in the launch file from the calibration#############
    # Link this omni to pr2 frame
#     def send_transform_to_link_omni_and_pr2_frame(self):
#         self.tfbroadcast.sendTransform(self.center_in_torso_frame,
#                          tuple(tf.transformations.quaternion_from_euler(0, 0, 0)),
#                          rospy.Time.now(),
#                          self.omni_name,
#                          "/torso_lift_link")
####################################################################################################################
####################################################################################################################


    ##
    # Transform from omni base link to torso lift link taking into account scaling
    def torso_T_playpen(self, omni_tip, msg_frame):
        #Transform into link0 so we can scale
        #print 'torso_t_omni', self.omni_name + '_link0', msg_frame
        z_T_6 = tfu.transform(self.omni_name + '_link0', msg_frame, self.tflistener)
        tip_0 = z_T_6 * omni_tip
        tip_0t = (np.array(tr.translation_from_matrix(tip_0)) * np.array(self.scale_omni_l0)).tolist()
        tip_0q = tr.quaternion_from_matrix(tip_0)

        #Transform into torso frame so we can bound arm workspace
        tll_T_0 = tfu.transform('/torso_lift_link', self.omni_name + '_link0', self.tflistener)
        tip_torso_mat = tll_T_0 * tfu.tf_as_matrix([tip_0t, tip_0q])
        tip_t, tip_q = tfu.matrix_as_tf(tip_torso_mat)
        tip_t[0] = limit_range(tip_t[0], self.X_BOUNDS[0], self.X_BOUNDS[1])
        self.tip_t = tip_t
        self.tip_q = tip_q

    ##
    # Transfrom from torso lift link to link omni base link, taking into account scaling
    def playpen_T_torso(self, torso_mat):
        l0_mat = tfu.transform(self.omni_name + '_link0', '/torso_lift_link', self.tflistener) * torso_mat
        l0_t = (np.array(tr.translation_from_matrix(l0_mat)) / np.array(self.scale_omni_l0)).tolist()
        l0_q = tr.quaternion_from_matrix(l0_mat)
        omni_pt_mat = tfu.transform(self.omni_name, self.omni_name + '_link0', self.tflistener) * tfu.tf_as_matrix((l0_t, l0_q))
        return tfu.matrix_as_tf(omni_pt_mat)

    ##
    # Callback for pose of omni
    def cmd_pose(self, tip_t, tip_q):
            #Get the omni's tip pose in the PR2's torso frame
#         tip_omni, msg_frame = tfu.posestamped_as_matrix(msg)
#         self.torso_T_playpen(tip_omni, msg_frame)
        # tip_t = self.tip_t
        # tip_q = self.tip_q
        #Publish new arm pose
        ps = PoseStamped()
        ps.header.frame_id = '/torso_lift_link'
        ps.header.stamp = rospy.get_rostime()
        ps.pose.position.x = tip_t[0]
        ps.pose.position.y = tip_t[1]
        ps.pose.position.z = tip_t[2]
        ps.pose.orientation.x = tip_q[0]
        ps.pose.orientation.y = tip_q[1]
        ps.pose.orientation.z = tip_q[2]
        ps.pose.orientation.w = tip_q[3]
        self.pr2_pub.publish(ps)

#         if self.zero_out_forces:
#             wr = OmniFeedback()
#             wr.force.x = 0 
#             wr.force.y = 0 
#             wr.force.z = 0 
#             self.omni_fb.publish(wr)
#             self.zero_out_forces = False

class PR2Playpen:
    def __init__(self):
        rospy.init_node('playpen_simple_grasper')
        self.tfbroadcast = tf.TransformBroadcaster()
        self.tflistener = tf.TransformListener()

        self.left_controller = ControlPR2Arm(
            pr2_control_topic = 'l_cart/command_pose',
            gripper_control_topic = 'l_gripper_controller',
            gripper_tip_frame = 'l_gripper_tool_frame',
            center_in_torso_frame = [1.2, .3, -1], 
            tfbroadcast=self.tfbroadcast,
            tflistener=self.tflistener)
#             if ff == True:
#                 self.left_feedback = ForceFeedbackFilter(wrench_topic = '/l_cart/state',
#                                              dest_frame = '/omni2_sensable',
#                                              wrench_frame = '/l_gripper_tool_frame', 
#                                              force_feedback_topic = 'omni2_force_feedback',
#                                              tflistener = self.tflistener,
#                                              kp_name = '/l_cart/cart_gains/trans/p',
#                                              kd_name = '/l_cart/cart_gains/trans/d')
#                 self.ff_list.append(self.left_feedback)
        
        self.right_controller = ControlPR2Arm(
            pr2_control_topic = 'r_cart/command_pose',
            gripper_control_topic = 'r_gripper_controller',
            gripper_tip_frame = 'r_gripper_tool_frame',
            center_in_torso_frame = [1.2, -.3, -1], 
            tfbroadcast=self.tfbroadcast,
            tflistener=self.tflistener)
#             if ff == True:
#                 self.right_feedback = ForceFeedbackFilter(wrench_topic = '/r_cart/state',
#                       dest_frame = '/omni1_sensable',
#                       wrench_frame = '/r_gripper_tool_frame', 
#                       force_feedback_topic = 'omni1_force_feedback',
#                       tflistener = self.tflistener,
#                       kp_name = '/r_cart/cart_gains/trans/p',
#                       kd_name = '/r_cart/cart_gains/trans/d')
#                 self.ff_list.append(self.right_feedback)
        

    def run(self):
        rate = rospy.Rate(100.0)
        rospy.loginfo('running...')
        rospy.spin()


if __name__ == '__main__':
    o = PR2Playpen()
#    o.run()
####Default out of workspace positions
    tip_t_r = np.array([0.23, -0.6, -0.05])
    tip_q_r = np.array([-0.51, 0.54, 0.48, 0.46])
    tip_t_l = np.array([0.23, 0.6, -0.05])
    tip_q_l = np.array([-0.51, 0.54, 0.48, 0.46])

    o.right_controller.gripper_server.send_goal(Pr2GripperCommandGoal(
                                                Pr2GripperCommand(position = 0.09, max_effort = 40)), 
                                                done_cb = None, feedback_cb = None)

#    while not rospy.is_shutdown():

    # o.left_controller.cmd_pose(tip_t_l, tip_q_l)

    # cur_time = rospy.Time.to_sec(rospy.Time.now())
    # while rospy.Time.to_sec(rospy.Time.now())-cur_time < 1:
    #     o.right_controller.cmd_pose(tip_t_r, tip_q_r)

    while not rospy.is_shutdown():
        o.left_controller.cmd_pose(tip_t_l, tip_q_l)
        o.right_controller.cmd_pose(tip_t_r, tip_q_r)
