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
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
import tf
import tf.transformations as tf_trans
import sys
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from visual_servo.srv import *
from array import *
import atexit

def goodbye():
  rospy.logwarn("Program Terminating: Reset all velocities to 0")
  try:
    cs = ControllerSwitcher()
    cs.carefree_switch('r', '%s_cart', '$(find visual_servo)/params/rmc_cartTwist_params.yaml')
    rospy.sleep(0.5)
    pub = rospy.Publisher('r_cart/command', Twist)
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    pub.publish(twist) 
  except rospy.ROSInterruptException,e: print "Could not terminate properly. Please panic and hit the reset button\n>> Reason:%s"%e

#atexit.register(goodbye)
class VisualServoExecutionNode:
  def initial_arm(self, cs):
    rospy.loginfo('Setting to Initial pose...')
    cs.carefree_switch('r', '%s_arm', '$(find visual_servo)/params/rmc_joint_trajectory_params.yaml' )
    rospy.sleep(0.5)
    pub = rospy.Publisher('r_arm/command', JointTrajectory)
    msg = JointTrajectory()
    pts = JointTrajectoryPoint()
    pts.positions = array('d', [-0.20, +0.056137,
                                    -1.53411927, -1.35,
                                    -15.85, -1.582,
                                    -1.72])
    pts.time_from_start = rospy.Duration(10)
    msg.points = [pts]
    msg.joint_names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint','r_elbow_flex_joint', 'r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint']

    sys.stdout.write('Moving... ')
    pub.publish(msg)
    rospy.sleep(0.5)
    pub.publish(msg)
    print 'Done'

  def move(self):
		
    rospy.init_node('vs_execute_node', log_level=rospy.DEBUG)
    cs = ControllerSwitcher()
    # self.initial_arm(cs)
    
    rospy.sleep(3.0)
    cs.carefree_switch('r', '%s_cart', '$(find visual_servo)/params/pr2mc_jinv_params.yaml')
    rospy.sleep(0.5)
    pub = rospy.Publisher('r_cart/commandTwist', TwistStamped)
    msg = TwistStamped()
    msg.header.frame_id = '/torso_lift_link'
    msg.header.stamp = rospy.Time(0)
    msg.twist.linear.x = 0.00
    msg.twist.linear.y = 0.10
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = 0.0
    pub.publish(msg)
    rospy.sleep(0.5)
    pub.publish(msg)
    '''
    cs.carefree_switch('r', '%s_cart', '$(find visual_servo)/params/rmc_cartTwist_params.yaml')
    rospy.sleep(0.5)
    pub = rospy.Publisher('r_cart/command', Twist)
    pose = Twist()
    pose.linear.x = 0.00
    pose.linear.y = -0.2
    pose.linear.z = 0.0
    pose.angular.x = 0.0
    pose.angular.y = 0.0
    pose.angular.z = 0.0
    pub.publish(pose)
    rospy.sleep(0.5)
    pub.publish(pose)
    '''
    '''
    rospy.loginfo('Waiting for Visual Servo Node Service')
    rospy.wait_for_service('visual_servo_twist')
    rospy.loginfo('Hooking up Service Proxy to the Visual Servo Twist')
    service = rospy.ServiceProxy('visual_servo_twist', VisualServoTwist)
    
    while not rospy.is_shutdown():
      try:
        resp = service()							
        pose.linear.x = resp.vx
        pose.linear.y = resp.vy
        pose.linear.z = resp.vz
        pose.angular.x = resp.wx
        pose.angular.y = resp.wy
        pose.angular.z = resp.wz
        rospy.loginfo(pose)
        # pub.publish(pose)
        rospy.sleep(0.3) 
      except rospy.ServiceException, e: pass 
'''
if __name__ == '__main__':
	try:
		node = VisualServoExecutionNode()
		node.move()
	except rospy.ROSInterruptException: pass
