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
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from phantom_omni.msg import PhantomButtonEvent
from teleop_controllers.msg import JTTeleopControllerState
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_msgs.msg import AccelerometerState


class ForceFeedbackFilter:

    def __init__(self, wrench_topic, dest_frame, wrench_frame, force_feedback_topic, tflistener, kp_name, kd_name):
        self.wrench_frame = wrench_frame
        self.dest_frame = dest_frame
        self.tflistener = tflistener
        self.omni_fb = rospy.Publisher(force_feedback_topic, Wrench)

##################NEED TO MODIFY WHERE IT IS PUBLISHING TO CHECK BOTH ARMS AT SAME TIME##########
        self.filtered_fb = rospy.Publisher('/test_filtered_fb', Wrench)
################################################################################################
        #REMOVE references to self.filtered_fb after check that everything is working
        rospy.Subscriber(wrench_topic, JTTeleopControllerState, self.wrench_callback)
        self.enable = False
        self.FIR = coeff.coefficients
        self.history = np.matrix(np.zeros((3,17)))
        self.prev_time = rospy.Time.now().nsecs*1e-9
        self.prev_dt = 0.0
        self.omni_max_limit = np.array([5., 5., 5.])
        self.omni_min_limit = np.array([-5., -5., -5.])
        self.kp = rospy.get_param(kp_name)
        self.kd = rospy.get_param(kd_name)
        self.force_scaling = -0.025
        self.force_old = np.zeros(3)

    def set_force_scaling(self, scalar):
        self.force_scaling = -1.*scalar

    def set_enable(self, v):
        self.enable = v

    def wrench_callback(self, state):
        #calculating force estimate from position error (does not compensate for motion error due to dynamics)
        x_err = np.matrix([state.x_err.linear.x, state.x_err.linear.y, state.x_err.linear.z]).T
        x_dot = np.matrix([state.xd.linear.x, state.xd.linear.y, state.xd.linear.z]).T
        feedback = -1.0*self.kp*x_err-self.kd*x_dot

        #currently the calculated force feedback is published but not to omni, we use the velocity limited state from the controller
        wr_ee = [state.F.force.x, state.F.force.y, state.F.force.z]
        
        #this is a simple FIR filter designed in Matlab to smooth the force estimate
        shift_right = np.array(self.history[:,0:self.FIR.size-1])
        new_col = np.array(wr_ee).reshape((3,1))
        self.history = np.matrix(np.hstack((new_col, shift_right)))
        wr_ee_filt = self.history*self.FIR

        #find and use the rotation matrix from wrench to torso

        wr = Wrench()
        wr.force.x = wr_ee_filt[0,0]
        wr.force.y = wr_ee_filt[1,0]
        wr.force.z = wr_ee_filt[2,0]
        
        test = Wrench()
        test.force.x = feedback[0,0] #wr.force.x-self.force_old[0]
        test.force.y = feedback[1,0] #wr.force.y-self.force_old[1]
        test.force.z = feedback[2,0] #wr.force.z-self.force_old[2]
             
        #publishing of two different wrenches calculated, DELETE first when all finished
        self.filtered_fb.publish(wr)

#this could be used for trying to damp the force feedback, didn't work very well
#         self.force_old[0] = wr.force.x
#         self.force_old[1] = wr.force.y
#         self.force_old[2] = wr.force.z
#         dt = rospy.Time.now().nsecs*1e-9-self.prev_time
#         self.prev_time = rospy.Time.now().nsecs*1e-9
#         print "time step: ", dt



class FF_PR2Teleop:
    def __init__(self):
        rospy.init_node('pr2_omni_force_feedback')
        self.tfbroadcast = tf.TransformBroadcaster()

        self.tflistener = tf.TransformListener()
#         self.left_feedback = ForceFeedbackFilter(wrench_topic = '/l_cart/state', #'/l_cart/test/wrench_unfiltered', #
#             dest_frame = '/omni1_sensable',
#             wrench_frame = '/l_gripper_tool_frame', 
#             force_feedback_topic = 'omni1_force_feedback',
#             tflistener = self.tflistener,
#             kp_name = '/l_cart/cart_gains/trans/p',
#             kd_name = '/l_cart/cart_gains/trans/d')
        self.right_feedback = ForceFeedbackFilter(wrench_topic = '/r_cart/state', #'/l_cart/test/wrench_unfiltered', #
            dest_frame = '/omni2_sensable',
            wrench_frame = '/r_gripper_tool_frame', 
            force_feedback_topic = 'omni2_force_feedback',
            tflistener = self.tflistener,
            kp_name = '/r_cart/cart_gains/trans/p',
            kd_name = '/r_cart/cart_gains/trans/d')

    def run(self):
        rate = rospy.Rate(100.0)
        rospy.loginfo('running...')
        rospy.spin()
 
if __name__ == '__main__':
    #o = ForceFeedbackFilter()
    #rospy.spin()
    o = FF_PR2Teleop()
    o.run()
    #while not rospy.is_shutdown():
    #    time.sleep(1.0)


