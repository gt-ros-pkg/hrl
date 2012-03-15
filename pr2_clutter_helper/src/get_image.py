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
#depnds on sensor_msgs, opencv2, cv_bridge
import rospy
from sensor_msgs.msg import Image as msgImage
import cv #version 2.1 expected
from cv_bridge import CvBridge
#may need image_transport?

import time #sleep
from random import * #randint

global_ros_image = None

def imageCallback(data):
    print 'callback called'
    #Note: data should be of type sensor_msgs.msg.Image
    #rospy.loginfo(rospy.get_name()+"I heard some data from camera")
    global global_ros_image
    global_ros_image = data

def listener():
    rospy.init_node('my_image_listener', anonymous=True)
    rospy.Subscriber("wide_stereo/right/image_rect_color", msgImage, imageCallback)
    rospy.spin()

if __name__ == '__main__':
    cv.NamedWindow('view')
    cv.StartWindowThread()
    rospy.init_node('my_image_listener', anonymous=True)
    a = rospy.Subscriber("wide_stereo/right/image_rect_color", msgImage, imageCallback)
    
    print 'sleeping now' #spin cycle.  Similar to "spin_once" ?
    while (not global_ros_image):
        rospy.sleep(.1)
        if rospy.is_shutdown():
           print '\nforcing an exit'
           break;
    print 'waking and exiting now'
    
    if not rospy.is_shutdown():
        a.unregister()
        
        br = CvBridge()
        im = br.imgmsg_to_cv(global_ros_image)
        cv.Erode(im, im, None, 5)
        cv.Dilate(im, im, None, 5)
        cv.ShowImage('view', im)  
        cv.WaitKey()
        cv.SaveImage('~/Desktop/this_is_a_temporary_image.png', im)
    
    #IDEA:  --  'sleep spin until callback received. then kill subscriber.

    
    
    
    
    
    
    

