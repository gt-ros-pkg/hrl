#!/usr/bin/env python  
import roslib
roslib.load_manifest('pr2_omni_teleop')

import rospy
import tf
import math
import actionlib
import tf.transformations as tr
import numpy as np
import hrl_lib.tf_utils as tfu
import coefficients as coeff
#import threading
from geometry_msgs.msg import PoseStamped
from phantom_omni.msg import OmniFeedback
from teleop_controllers.msg import JTTeleopControllerState
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

###COULD use the accelerometer in a smart way too if want to....
#from pr2_msgs.msg import AccelerometerState


def limit_range(numb, lower, upper):
    if lower > upper:
        raise RuntimeError('lower bound > upper bound, wa?')
    return max(min(numb, upper), lower)


class ControlPR2Arm:

    def __init__(self, omni_name, #='omni1', 
            pr2_control_topic, # = 'l_cart/command_pose',
            gripper_control_topic,  #'l_gripper_controller'
            gripper_tip_frame, #'l_gripper_tool_frame'
            center_in_torso_frame, #=[1,.3,-1], 
            scaling_in_base_frame, #=[3.5, 3., 5.],
            tfbroadcast, #=None,
            tflistener): #=None):

        #threading.Thread.__init__(self)
        self.enabled = False
        self.zero_out_forces = True

        self.X_BOUNDS = [.3, 1.5] #Bound translation of PR2 arms in the X direction (in torso link frame)
        self.kPos = 15.
        self.kVel = 0.5
        self.kPos_close = 70.
        self.omni_name = omni_name
        self.center_in_torso_frame = center_in_torso_frame
        self.scaling_in_base_frame = scaling_in_base_frame
        self.tflistener = tflistener
        self.tfbroadcast = tfbroadcast
        self.gripper_tip_frame = gripper_tip_frame
        self.prev_dt = 0.0
        self.tip_tt = np.zeros((3,1))
        self.tip_tq = np.zeros((4,1))
        rate = rospy.Rate(100.0)

        rospy.loginfo('Attempting to link ' + omni_name + ' to the PR2\'s torso frame.')
        success = False
        while not success and (not rospy.is_shutdown()):
            self.send_transform_to_link_omni_and_pr2_frame()
            rate.sleep()
            try:
                self.scale_omni_l0 = np.abs(self.l0_rotate_base(self.scaling_in_base_frame))
                success = True
            except tf.LookupException, e:
                pass
            except tf.ConnectivityException, e:
                pass
        rospy.loginfo('Finished linking frame for %s' % omni_name)

        self.omni_fb = rospy.Publisher(self.omni_name + '_force_feedback', OmniFeedback)
        self.pr2_pub = rospy.Publisher(pr2_control_topic, PoseStamped)
        self.scale_omni_l0 = np.abs(self.l0_rotate_base(self.scaling_in_base_frame))

        rospy.Subscriber(self.omni_name + '_pose', PoseStamped, self.omni_pose_cb)
        self.gripper_handler = GripperOmniHandler(self.omni_name + '_button', gripper_control_topic)

    ##
    # Link this omni to pr2 frame
    def send_transform_to_link_omni_and_pr2_frame(self):
        self.tfbroadcast.sendTransform(self.center_in_torso_frame,
                         tuple(tf.transformations.quaternion_from_euler(0, 0, 0)),
                         rospy.Time.now(),
                         self.omni_name,
                         "/torso_lift_link")
    ##
    # Rotate a vector from omni_link0 to base_footprint frame
    def l0_rotate_base(self, vec_base):
        m = tfu.translation_matrix(vec_base)
        vec_l0 = tr.translation_from_matrix((tfu.rotate('/base_footprint', '/torso_lift_link', self.tflistener) * \
                                             tfu.rotate('/torso_lift_link', self.omni_name + '_link0', self.tflistener)).T * m)
        return np.array(vec_l0)

    ##
    # Transform from omni base link to torso lift link taking into account scaling
    def torso_T_omni(self, omni_tip, msg_frame):
        #Transform into link0 so we can scale
        #print 'torso_t_omni', self.omni_name + '_link0', msg_frame
        z_T_6 = tfu.transform(self.omni_name + '_link0', msg_frame, self.tflistener)
        tip_0 = z_T_6 * omni_tip
        tip_0t = (np.array(tr.translation_from_matrix(tip_0)) * np.array(self.scale_omni_l0)).tolist()
        tip_0q = tr.quaternion_from_matrix(tip_0)

        #Transform into torso frame so we can bound arm workspace
        tll_T_0 = tfu.transform('/torso_lift_link', self.omni_name + '_link0', self.tflistener)
        tip_torso_mat = tll_T_0 * tfu.tf_as_matrix([tip_0t, tip_0q])
        tip_tt, tip_tq = tfu.matrix_as_tf(tip_torso_mat)
        tip_tt[0] = limit_range(tip_tt[0], self.X_BOUNDS[0], self.X_BOUNDS[1])
        self.tip_tt = tip_tt
        self.tip_tq = tip_tq

    ##
    # Transfrom from torso lift link to link omni base link, taking into account scaling
    def omni_T_torso(self, torso_mat):
        l0_mat = tfu.transform(self.omni_name + '_link0', '/torso_lift_link', self.tflistener) * torso_mat
        l0_t = (np.array(tr.translation_from_matrix(l0_mat)) / np.array(self.scale_omni_l0)).tolist()
        l0_q = tr.quaternion_from_matrix(l0_mat)
        omni_pt_mat = tfu.transform(self.omni_name, self.omni_name + '_link0', self.tflistener) * tfu.tf_as_matrix((l0_t, l0_q))
        return tfu.matrix_as_tf(omni_pt_mat)


    ##
    # Set whether control should be enabled
    def set_control(self, s):
        self.enabled = s
        if self.enabled:
            self.gripper_handler.set_enabled(True)
            self.zero_out_forces = True
        else:
            self.gripper_handler.set_enabled(False)

    ##
    # Callback for pose of omni
    def omni_pose_cb(self, msg):
        if self.enabled:
            #Get the omni's tip pose in the PR2's torso frame
            tip_omni, msg_frame = tfu.posestamped_as_matrix(msg)
            self.torso_T_omni(tip_omni, msg_frame)
            tip_tt = self.tip_tt
            tip_tq = self.tip_tq
            #Publish new arm pose
            ps = PoseStamped()
            ps.header.frame_id = '/torso_lift_link'
            ps.header.stamp = rospy.get_rostime()
            ps.pose.position.x = tip_tt[0]
            ps.pose.position.y = tip_tt[1]
            ps.pose.position.z = tip_tt[2]
            ps.pose.orientation.x = tip_tq[0]
            ps.pose.orientation.y = tip_tq[1]
            ps.pose.orientation.z = tip_tq[2]
            ps.pose.orientation.w = tip_tq[3]
            self.pr2_pub.publish(ps)
            if self.zero_out_forces:
                wr = OmniFeedback()
                wr.force.x = 0 
                wr.force.y = 0 
                wr.force.z = 0 
                self.omni_fb.publish(wr)
                self.zero_out_forces = False
        else:
            #this is a zero order hold publishing the last received values until the control loop is active again
            tip_tt = self.tip_tt
            tip_tq = self.tip_tq
            ps = PoseStamped()
            ps.header.frame_id = '/torso_lift_link'
            ps.header.stamp = rospy.get_rostime()
            ps.pose.position.x = tip_tt[0]
            ps.pose.position.y = tip_tt[1]
            ps.pose.position.z = tip_tt[2]
            ps.pose.orientation.x = tip_tq[0]
            ps.pose.orientation.y = tip_tq[1]
            ps.pose.orientation.z = tip_tq[2]
            ps.pose.orientation.w = tip_tq[3]
            self.pr2_pub.publish(ps)

            #this is to make the omni force well move if the arm has moved but the commanded
            #position of the arm has not changed
            tip_omni, msg_frame = tfu.posestamped_as_matrix(msg)
            m_o1 = tfu.transform(self.omni_name, msg_frame, self.tflistener) * tip_omni
            ee_point = np.matrix(tr.translation_from_matrix(m_o1)).T
            tip_torso = tfu.transform('/torso_lift_link', self.gripper_tip_frame, self.tflistener) \
                                  * tfu.tf_as_matrix(([0.,0.,0.], tr.quaternion_from_euler(0,0,0)))
            center_t, center_q = self.omni_T_torso(tip_torso)
            center_col_vec = np.matrix(center_t).T

            #Send force control info
            wr = OmniFeedback()
            # offsets (0, -.268, -.15) introduced by Hai in phantom driver
            # should be cleaned up at some point so that it is consistent with position returned by phantom -marc
            lock_pos = tr.translation_matrix(np.matrix([0,-.268,-.150]))*tfu.transform(self.omni_name+'_sensable', self.omni_name, self.tflistener)*np.row_stack((center_col_vec, np.matrix([1.])))
            wr.position.x = (lock_pos[0,0])*1000.0  #multiply by 1000 mm/m to get units phantom expects
            wr.position.y = (lock_pos[1,0])*1000.0 
            wr.position.z = (lock_pos[2,0])*1000.0 
            self.omni_fb.publish(wr)


class OmniPR2Teleop:
    def __init__(self, arm, ff):
        rospy.init_node('omni_frames')
        self.enabled = False
        self.tfbroadcast = tf.TransformBroadcaster()
        self.tflistener = tf.TransformListener()
        self.controller_list = []
        self.ff_list = []
        if arm == "l" or arm == "b":
            self.left_controller = ControlPR2Arm(
                                        omni_name ='omni2', 
                                        pr2_control_topic = 'l_cart/command_pose',
                                        gripper_control_topic = 'l_gripper_controller',
                                        gripper_tip_frame = 'l_gripper_tool_frame',
                                        center_in_torso_frame = [1.2, .3, -1], 
                                        scaling_in_base_frame = [3.5, 3., 5.],
                                        tfbroadcast=self.tfbroadcast,
                                        tflistener=self.tflistener)
            self.controller_list.append(self.left_controller)
            rospy.Subscriber('omni2_button', PhantomButtonEvent, self.omni_safety_lock_cb)
            if ff == True:
                self.left_feedback = ForceFeedbackFilter(wrench_topic = '/l_cart/state',
                                             dest_frame = '/omni2_sensable',
                                             wrench_frame = '/l_gripper_tool_frame', 
                                             force_feedback_topic = 'omni2_force_feedback',
                                             tflistener = self.tflistener,
                                             kp_name = '/l_cart/cart_gains/trans/p',
                                             kd_name = '/l_cart/cart_gains/trans/d')
                self.ff_list.append(self.left_feedback)
        if arm == "r" or arm == "b":
            self.right_controller = ControlPR2Arm(
                                       omni_name ='omni1', 
                                       pr2_control_topic = 'r_cart/command_pose',
                                       gripper_control_topic = 'r_gripper_controller',
                                       gripper_tip_frame = 'r_gripper_tool_frame',
                                       center_in_torso_frame = [1.2, -.3, -1], 
                                       scaling_in_base_frame = [3.5, 3., 5.],
                                       tfbroadcast=self.tfbroadcast,
                                       tflistener=self.tflistener)
            rospy.Subscriber('omni1_button', PhantomButtonEvent, self.omni_safety_lock_cb)
            self.controller_list.append(self.right_controller)
            if ff == True:
                self.right_feedback = ForceFeedbackFilter(wrench_topic = '/r_cart/state',
                      dest_frame = '/omni1_sensable',
                      wrench_frame = '/r_gripper_tool_frame', 
                      force_feedback_topic = 'omni1_force_feedback',
                      tflistener = self.tflistener,
                      kp_name = '/r_cart/cart_gains/trans/p',
                      kd_name = '/r_cart/cart_gains/trans/d')
                self.ff_list.append(self.right_feedback)
        
        self.set_state(False)

    def run(self):
        rate = rospy.Rate(100.0)
        rospy.loginfo('running...')
        while not rospy.is_shutdown():
            for cont in self.controller_list:
                cont.send_transform_to_link_omni_and_pr2_frame()


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    
    p.add_option('--arms', action='store', type='string', dest='arm', default='r', 
                 help='this allows initialization of right, left or both arms with an input of r, l or b respectively')
    p.add_option('--ff', action='store', type='int', dest='ff', default='0',
                 help='enter 1 to activate force feedback, 0 otherwise')

    opt, args = p.parse_args()

    o = OmniPR2Teleop(opt.arm, opt.ff)
    o.run()



