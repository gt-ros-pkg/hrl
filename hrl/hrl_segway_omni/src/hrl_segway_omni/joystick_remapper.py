#!/usr/bin/python
#
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

#  \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
#  \author Cressel Anderson (Healthcare Robotics Lab, Georgia Tech.)
#  \author Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)

import time
import roslib
roslib.load_manifest('hrl_segway_omni')
import rospy

from joy.msg import Joy
from geometry_msgs.msg import Twist

def joy_cb(data):
    global tw
#    print 'data.axes:', data.axes
#    print 'data.buttons:', data.buttons

    dead_man_pressed = (data.buttons[4] == 1)

    xvel = data.axes[1] * max_xvel
    yvel = data.axes[0] * max_yvel
    avel = data.axes[2] * max_avel

    tw = Twist()
    if dead_man_pressed:
        tw.linear.x = xvel
        tw.linear.y = yvel
        tw.angular.z = avel

dead_man_pressed = False
max_xvel = 0.25
max_yvel = 0.2
max_avel = 0.5

rospy.init_node("cody_joystick_remapper", anonymous=False)
pub = rospy.Publisher('cmd_vel', Twist)
rospy.Subscriber('joy', Joy, joy_cb, None, 1)

rospy.logout('Ready to Joystick the Segway')

tw = Twist()

while not rospy.is_shutdown():
    rospy.sleep(0.1)
    pub.publish(tw)






