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
from sensor_msgs.msg import CameraInfo

import sys

import rospy
from pr2_lcs_helper.srv import * #Get four .srv definitions

''' Usage: topic_name = None for default topic (listed in launch file), or 
        a string refering to ROS topic; msg_type is string refering to  
        returned msg type; and srv_type is a srv object (not string) generally
        named 'Get'+msg_type
    This client should be run to demonstrate success of the get_msg_server.py service.
    
'''
def get_single_msg(topic_name, msg_type, srv_type):
    service_name = 'get_'+msg_type
    rospy.wait_for_service(service_name)
    try:
        get_one_msg = rospy.ServiceProxy(service_name, srv_type)
        return get_one_msg(topic_name, msg_type)
    except:
        print 'Failed to acquire message from topic', topic_name
        return None

def create_cam_model( info_msg):
    from image_geometry import PinholeCameraModel
    import numpy as np
    cam_model = PinholeCameraModel()
    
    #  <have an info message > 
    cam_model.fromCameraInfo(info_msg)
    print 'cam_model = ', np.array( cam_model.intrinsicMatrix() )
    print 'cam tf = ', cam_model.tfFrame()
    return cam_model

def grab_tf(child_id = '/wide_stereo_optical_frame', frame_id = '/base_footprint'):
    #hopefully unused 
    topic = 'tf'
    tf_msg = None
    #tf publishes tfMessage which is a container holding a TransformStamped
    n=0
    # < grab tf messages until correct one is fetched >
    while not tf_msg:
        print 'try #', n; n+=1
        resp = get_single_msg(topic, 'tfMessage', GettfMessage) 
        tf_msg = resp.transform
        if tf_msg.header.frame_id == frame_id and tf_msg.child_frame_id==child_id:
            break;
        else: tf_msg = None
    print 'Got a tf', tf_msg, tf_msg.child_frame_id
    return tf_msg

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    topic = '/wide_stereo/right/camera_info'
    resp = get_single_msg(topic, 'CameraInfo', GetCameraInfo) 
    info_msg = resp.info
    
    topic = '/wide_stereo/right/image_rect_color'
    resp = get_single_msg(topic, 'Image', GetImage) 
    image_msg = resp.image
    print 'got image with h=', image_msg.height

    topic = '/wide_stereo/right/image_rect_color'
    resp = get_single_msg(topic, 'Image', GetImage) 
    image_msg = resp.image

    print "creating_cam_model"
    cam_model = create_cam_model(info_msg)
    
    print "Done ; next is to write transform"
    



  
    
#---------------------------------------------    
'''
    frame_id = 'base_footprint'
    acquisition_time = info_msg.header.stamp #type is ros.Time
    timeout = rospy.Duration(10) #at most wait 5 seconds

    do_some_tf():
        cam_frame = '/wide_stereo_optical_frame' #cam_model.tfFrame()
        other_frame = '/base_footprint'
        print "cam model frame :", cam_model.tfFrame()
        import tf
        #  <frame1 to frame2, time, timeout >
        try:
            rospy.init_node('client_calling_tf')
            print 'make listener'
            tf_listener = tf.TransformListener()
            print 'wait'
            latest = rospy.Time(0) #hack -- actually want time synchronized.
            #tf_listener.waitForTransform(cam_frame, other_frame, acquisition_time, timeout);
            #(trans,rot)
            print 'get tf now'
            print 'skipping transform'
            #transform = tf_listener.lookupTransform(cam_frame, other_frame);
        except:
            print "[get] TF exception: "
            return
        print "the transform is ", transform
        return transform
         
    do_some_tf()
    print "tf done"
'''
    
    
    
    
    
    

