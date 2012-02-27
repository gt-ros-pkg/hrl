#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, Georgia Institute of Technology
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  * Neither the name of the Georgia Institute of Technology nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('visual_servo')
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
import tf
import tf.transformations as tf_trans
import sys
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from visual_servo.srv import *

class VisualServoExecutionNode:
    def move(self):
        rospy.loginfo('Switching the Arm Controller')
        ctrl_switcher = ControllerSwitcher()
        ctrl_switcher.carefree_switch('l', '%s_cart', '$(find visual_servo)/params/cart_param.yaml')
        rospy.sleep(0.5)

        rospy.loginfo('Creating the Publisher')
        pub = rospy.Publisher('l_cart/command', Twist)

        rospy.loginfo('Waiting for Visual Servo Node Service')
        rospy.wait_for_service('visual_servo_twist')
        
        rospy.loginfo('Hooking up Service Proxy to the Visual Servo Twist')
        service = rospy.ServiceProxy('visual_servo_twist', VisualServoTwist)
             

        pose = Twist()
        pose.linear.x = 0.1
        pose.linear.y = -0.2
        pose.linear.z = 0.0
        pose.angular.x = 0.0
        pose.angular.y = 0.0
        pose.angular.z = 0.0
        while not rospy.is_shutdown():
            try:
                resp = service()
                pose.linear.x = resp.vx
                pose.linear.y = resp.vy
                pose.linear.z = resp.vz
                pose.angular.x = resp.wx
                pose.angular.y = resp.wy
                pose.angular.z = resp.wz       
                pub.publish(pose)
                rospy.sleep(1.0) 
            except rospy.ServiceException, e:
                print "Service Call Failed: %s"%e

if __name__ == '__main__':
    try:
        rospy.init_node('vs_execute_node', log_level=rospy.DEBUG)
        node = VisualServoExecutionNode()
        node.move()
    except rospy.ROSInterruptException: pass
