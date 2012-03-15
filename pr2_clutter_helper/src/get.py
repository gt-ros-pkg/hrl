#!/usr/bin/python
#
# Copyright (c) 2010, Georgia Tech Research Corporation
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

#  \author Jason Okerman (Healthcare Robotics Lab, Georgia Tech.)


"""
Grab data from topic <given> of generic <type>.
"""
import roslib; roslib.load_manifest('pr2_clutter_helper')
#depnds on sensor_msgs, opencv2, cv_bridge
import rospy
from sensor_msgs.msg import PointCloud
import cv #version 2.1 expected
from cv_bridge import CvBridge
#may need image_transport?

import time #sleep
from random import * #randint

TOPIC_NAME = "tilt_laser_cloud"
global_ros_data = None
NODE_NAME = 'my_listener'
'''
   Comments:
   Traps exactly one message from the topic specified.  This is 
   To run 'main', this assumes the launch of "pr2_laser_snapshotter" with mapping to "tilt_laser_cloud"
   Basic use is to import and call "subscribe_once" method.
'''

def init(node_name = NODE_NAME):
    rospy.init_node(node_name, anonymous=True)

def myCallback(data):
    print 'callback called'
    global global_ros_data
    global_ros_data = data
    
##def listener():
##    rospy.init_node('my_listener', anonymous=True)
##    rospy.Subscriber("tilt_laser_cloud", PointCloud, cloudCallback)
##    rospy.spin()

def subscribe_once(topic_name=TOPIC_NAME, msg_type=PointCloud):
    init()
    #if not something, init.
    s = rospy.Subscriber(topic_name, msg_type, myCallback)
    #spin until 1 message is received.
    print 'sleeping now: waitin for topic', topic_name
    while (not global_ros_data):
        rospy.sleep(.1)
        if rospy.is_shutdown(): #catch a CTRL-C
            print '\n Forcing exit'
            return False; #check for 'false' on output to catch error results.
    print 'waking and exiting now'
    s.unregister()
    global global_ros_data
    return global_ros_data #value will be returned as 'global_ros_data'

if __name__ == 'main':
    data = subscribe_once(TOPIC_NAME, PointCloud)
    
    #assume global_ros_data is defined.
    #do something with the pointcloud
    for i in range(10):
        pt = data.points[i]
        print "(%f, %f, %f)" %(pt.x, pt.y, pt.z)
    
    
    
    
    

