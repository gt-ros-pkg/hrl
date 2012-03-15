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
from pr2_lcs_helper.srv import *
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32

'''
    This function publishes a sensor_msgs.msg.PointCloud object for rviz
    It doesn't import anything but generates points in a rectangular region and 
    lets this region move accross the screen over time. 

    The frame is '/my_frame', so rviz view frame must be changed accordingly.
    
    NOT FINISHED YET, have not tested -- hope to soon!

'''

pub=''

def handle_service_request(req):
    #<generate data to send in this function> 
    cloud = req.cloud
    try:
        #cloud = local_rand_cloud()
        #cloud.header.frame_id = '/my_frame'
        #cloud.header.stamp = rospy.Time.now()
        print 'sending cloud at time', cloud.header.stamp, '&', rospy.Time.now()
        global pub
        pub.publish(cloud)
        return  True 
    except:
        return  False


def start_server():
    rospy.init_node('cloud_broadcaster_node')
    s = rospy.Service('pub_cloud_service', CloudPub, handle_service_request)
    print "Ready to publish cloud"
    
    global pub
    pub = start_publisher()
    
    rospy.spin()
    #rospy.sleep(.5) #possibly publish periodically


def start_publisher():
    #global pub
    pub = rospy.Publisher('my_cloud_chatter', PointCloud)
    #rospy.init_node('point_cloud_broadcaster')
    ## while not rospy.is_shutdown():
    
    #cloud = local_rand_cloud()
    #cloud.header.frame_id = '/my_frame'
    #cloud.header.stamp = rospy.Time.now()
    #print 'NOT YET sending cloud at time', cloud.header.stamp
    #pub.publish(cloud)
    
    ## rospy.sleep(1) #one second
    return pub
           
       
if __name__ == '__main__':
    try:
        # talker()
        start_server()
        
    except rospy.ROSInterruptException: pass
    
'''
def local_rand_cloud()
    ### Create a new cloud object and stuff it
    from random import *

    ROS_pointcloud = PointCloud() #sensor_msgs.msg.PointCloud()
    x = 0 #const offset easy to change.
    
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
    ### normalize values of a list to make its max = normalizeTo
    vMax = max(L)
    if vMax == 0: vMax = 1; #avoid divide by zero
    return [ x/(vMax*1.0)*normalizeTo for x in L]    
    
'''
    
    

