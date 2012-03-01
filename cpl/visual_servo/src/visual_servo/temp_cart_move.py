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

import roslib; roslib.load_manifest('tabletop_pushing')
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped
import tf
import tf.transformations as tf_trans
import sys
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

class TestNode:
     def talk(self):
        # pub = rospy.Publisher('command', PoseStamped)
        isStamped = True;
        if (isStamped):
            rospy.loginfo('created the controller')
            ctrl_swtch = ControllerSwitcher()
            # note: more smooth but see some jerking robot_mechanism_controllers/JTCartesianControllers
            ctrl_swtch.carefree_switch('r', '%s_cart', '$(find hrl_pr2_arms)/params/j_transpose_params_low.yaml')

            # note: very jerky... pr2_manipulation_controllers/JTTaskControllers
            # ctrl_swtch.carefree_switch('r', '%s_cart', '$(find hrl_pr2_arms)/params/j_transpose_task_params_high.yaml')
            # ctrl_swtch.carefree_switch('r', '%s_cart', '$(find tabletop_pushing)/params/j_transpose_params_high.yaml')

            rospy.sleep(1.0)

            rospy.loginfo('initializing the test node')
            pub = rospy.Publisher('r_cart/command_pose', PoseStamped)

            rospy.loginfo('Finally Publishing')
            pose = PoseStamped()
            pose.header.frame_id = '/torso_lift_link'
            pose.header.stamp = rospy.Time(0)
            pose.pose.position.x = 0.1
            pose.pose.position.y = -0.2
            pose.pose.position.z = -0.2
            pose.pose.orientation.x = 0.5
            pose.pose.orientation.y = 0.5
            pose.pose.orientation.z = 0.5
            pose.pose.orientation.w = 0.5 
            while not rospy.is_shutdown():
                pose.pose.position.x = pose.pose.position.x + 0.025
            	rospy.loginfo('Publishing following message: %s'%pose)
            	pub.publish(pose)
            	rospy.sleep(0.5) 
       
        else:
            
            # note: Does not work (wiki says it takes in Geomtry_msgs/Pose, but when you run it, there is some conflict)
            # Robot_mechanism_controllers/CartesianPoseControllers
            rospy.loginfo('created the controller')
            ctrl_swtch = ControllerSwitcher()
            ctrl_swtch.carefree_switch('r', '%s_cart', '$(find tabletop_pushing)/params/rmc_cartPose_params.yaml')
            rospy.sleep(1.0)

            rospy.loginfo('initializing the test node')
            pub = rospy.Publisher('r_cart/command', Pose)

            rospy.loginfo('Finally Publishing')
            pose = Pose()
            pose.position.x = 0.3
            pose.position.y = -0.2
            pose.position.z = -0.20
            pose.orientation.x = 0.7071
            pose.orientation.y = 0 
            pose.orientation.z = 0 
            pose.orientation.w = 0.7071 
            while not rospy.is_shutdown():
                pose.position.x = pose.position.x + 0.05
            	rospy.loginfo('Publishing following message: %s'%pose)
            	pub.publish(pose)
            	rospy.sleep(0.5) 





if __name__ == '__main__':
    try:
        rospy.init_node('test_node', log_level=rospy.DEBUG)
        node = TestNode()
        node.talk()
    except rospy.ROSInterruptException: pass
