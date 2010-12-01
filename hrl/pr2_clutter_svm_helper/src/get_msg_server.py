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
from pr2_lcs_helper.srv import * #specifically, get CameraInfo.srv"
from sensor_msgs.msg import CameraInfo, PointCloud, Image
#from geometry_msgs.msg import TransformStamped
from tf.msg import tfMessage

import sys

#depnds on sensor_msgs, opencv2, cv_bridge
import rospy

NODE_NAME = 'get_camera_info_server'
TOPIC_NAME = "wide_stereo/right/camera_info"
# Defaults for global values
global msg_type, srv_type, response_type
msg_type = CameraInfo
srv_type = GetCameraInfo
resp_type = GetCameraInfoResponse
global_ros_data = None

'''
   Comments:
   Traps exactly one message from the topic specified.  
   Subscribes to the topic, but returns as a service request.
'''

def sub_callback(data):
    print '[get] callback called'
    global global_ros_data
    global_ros_data = data
    
def subscribe_once(topic_name=TOPIC_NAME, once_msg_type=msg_type):
    s = rospy.Subscriber(topic_name, once_msg_type, sub_callback)
    #spin until 1 message is received.
    print 'sleeping now: waiting for topic', topic_name
    global global_ros_data
    while (not global_ros_data):
        rospy.sleep(.1)
        if rospy.is_shutdown(): #catch a CTRL-C
            print '\n Forcing exit'
            return False; #check for 'false' on output to catch error results.
    print 'Returning received message from:', topic_name
    s.unregister()
    
    return global_ros_data #value will be returned as 'global_ros_data'

def handle_service_request(req):
    topic_name = req.topic_name 
    #<ignore message type, must be set earlier.  Subscribe to topic.>
    data = subscribe_once(topic_name, msg_type)
    #<generate data to send in this function> 
    return resp_type( data )


''' Ex: node_name = my_node, service_name = 'get_camera_info'
'''
def get_one_server(node_name, service_name):
    rospy.init_node(node_name)
    s = rospy.Service(service_name, srv_type, handle_service_request)
    print "Ready to get", request
    rospy.spin()

#for arg in sys.argv:
#    print arg


if __name__ == '__main__':
    print 'starting get_server.py'
    #request = rospy.resolve_name('request') #wrong syntax
    for arg in sys.argv: print '[arg] ', arg
    
    try:
        request = sys.argv[1]  
        print '[get] using msg type of', request
    except:
        print '[get] using defaults of CameraInfo'
        request = 'CameraInfo'
    try: default_topic_name = sys.argv[2]
    except: print '[get] ignoring default topic name'    
    try: default_reference_frame = sys.argv[3]
    except: print '[get] ignoring default reference frame'  
    
    unique_node_name = request + '_server'
    unique_service_name = 'get_'+ request
    print 'starting service with name: "%s" ' %unique_service_name
    
    msg = {'CameraInfo':CameraInfo, 'Image':Image, 
             'PointCloud':PointCloud,        
             'tfMessage': tfMessage}
    srv = {'CameraInfo':GetCameraInfo, 'Image':GetImage, 
                'PointCloud':GetPointCloud,        
                'tfMessage': GettfMessage}
    resp = {'CameraInfo':GetCameraInfoResponse, 
            'Image':GetImageResponse, 
            'PointCloud':GetPointCloudResponse,        
            'tfMessage': GettfMessageResponse}
    try:
        #global msg_type, srv_type, response_type
        msg_type = msg[request]
        srv_type = srv[request]
        resp_type = resp[request]
    except:
        print '[get service] unable to handle requested service of type: ', request
        sys.exit()
    get_one_server(unique_node_name, unique_service_name)
    
    
    
    
    
    
    

