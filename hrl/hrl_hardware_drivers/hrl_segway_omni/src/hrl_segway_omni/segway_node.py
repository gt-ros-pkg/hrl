#! /usr/bin/python
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  \author Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)
#  \author Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)

import roslib; roslib.load_manifest('hrl_segway_omni')
from hrl_lib.msg import PlanarBaseVel
from geometry_msgs.msg import Twist
import rospy
import segway
import numpy as np

def callback(cmd):
    #print 'segway_node:', cmd.xvel, cmd.yvel, cmd.angular_velocity
    mecanum.set_velocity(cmd.xvel, cmd.yvel, cmd.angular_velocity)

def callback_ros(cmd):
    #print 'segway_node:', cmd.linear.x, cmd.linear.y, cmd.angular.z
    avel = cmd.angular.z * 0.5
    avel = np.clip(avel,-0.2,0.2)
    mecanum.set_velocity(cmd.linear.x, cmd.linear.y, avel)

rospy.init_node("segway_node", anonymous=False)
rospy.Subscriber("base", PlanarBaseVel, callback, None, 1)
rospy.Subscriber("cmd_vel", Twist, callback_ros, None, 1)
#mecanum = segway.Mecanum(init_ros_node=False)
mecanum = segway.Mecanum()
rospy.spin()

