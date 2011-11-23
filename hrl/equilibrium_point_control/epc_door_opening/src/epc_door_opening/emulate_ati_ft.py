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

# Author: Advait Jain

import numpy as np, math
import copy
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_door_opening')
import rospy

from hrl_msgs.msg import FloatArray
from geometry_msgs.msg import Twist

def ft_cb(data):
    lock.acquire()
    ft_val[0] = data.linear.x
    ft_val[1] = data.linear.y
    ft_val[2] = data.linear.z
    ft_val[3] = data.angular.x
    ft_val[4] = data.angular.y
    ft_val[5] = data.angular.z
    lock.release()

if __name__ == '__main__':
    import sys

    rospy.init_node('ati_ft_emulator')

    print sys.argv

    # 4 for roslaunch
    if len(sys.argv) != 2 and len(sys.argv) != 4:
        rospy.logerr('Need to pass the topic name on the command line.  Exiting...')
        sys.exit()
    
    topic = sys.argv[1]

    lock = RLock()
    ft_val = [0.] * 6
    pub = rospy.Subscriber('/r_cart/state/wrench', Twist, ft_cb)
    pub = rospy.Publisher(topic, FloatArray)
    rospy.loginfo('Started the ATI FT emulator.')
    rt = rospy.Rate(100)
    while not rospy.is_shutdown():
        lock.acquire()
        send_ft_val = copy.copy(ft_val)
        lock.release()
        fa = FloatArray(rospy.Header(stamp=rospy.Time.now()), send_ft_val)
        pub.publish(fa)
        rt.sleep()


