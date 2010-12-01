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

import roslib; roslib.load_manifest('pr2_clutter_helper')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32
from random import *
from pr2_lcs_helper.srv import *

'''
    client to test broadcaster.
    
    NOT FINISHED YET, have not tested -- hope to soon!

'''


def local_rand_cloud():

    #Create a new cloud object and stuff it
    ROS_pointcloud = PointCloud() #sensor_msgs.msg.PointCloud()
    x = random()*5 #const offset easy to change.
    
    #stuff point cloud with random points in a rectangle which drifts over time.
    #Add an extra intensity channel as well.
    intensity_channel = ChannelFloat32()
    intensity_channel.name = 'intensities'
    for i in range(3000):
        intensity_channel.values += [random()]
   
    for i in range(3000):
        ROS_pointcloud.points += [Point32(random()*5-x,random(), intensity_channel.values[i])]
    #rgb color has maximum value of 0xFFFFFF
    rgb_channel = ChannelFloat32()
    rgb_channel.name = 'rgb'
    rgb_channel.values = normList(intensity_channel.values, 0xFF)
    
    #Separate r,g,b channels range betweeon 0 and 1.0.
    r, g, b = [], [], []
    for pt in ROS_pointcloud.points: 
        b += [pt.y]; 
        r += [pt.z];
        g += [0];
    r_channel = ChannelFloat32(name='r', values = normList(r) )
    g_channel = ChannelFloat32(name='g', values = normList(g) )
    b_channel = ChannelFloat32(name='b', values = b)
    
    ROS_pointcloud.channels = (intensity_channel, rgb_channel, r_channel, g_channel, b_channel)
    return ROS_pointcloud
    
def normList(L, normalizeTo=1):
    '''normalize values of a list to make its max = normalizeTo'''
    vMax = max(L)
    if vMax == 0: vMax = 1; #avoid divide by zero
    return [ x/(vMax*1.0)*normalizeTo for x in L]
    
def labeled_cloud_pub_client(cloud, frame):
    rospy.wait_for_service('pub_cloud_service')
    try:
        service = rospy.ServiceProxy('pub_cloud_service', CloudPub)
        cloud.header = rospy.Header(frame_id=frame)
        
        resp = service(cloud, frame)
        return resp.success #bool
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    
if __name__== "__main__":
    print "creating local random pointcloud"
    
    
    cloud = local_rand_cloud()
    
    print "sending cloud, receiving success/fail"
    
    frame_id = 'my_frame'
    success = labeled_cloud_pub_client(cloud, frame_id)
    print "status of success: ", success
    n =1
    print 'loop'
    while True:
        cloud = local_rand_cloud()
        rospy.sleep(.1)
        success = labeled_cloud_pub_client(cloud, frame_id)
        print "status of success: ", success, n
        n+=1
    
    
    
    
    
    
