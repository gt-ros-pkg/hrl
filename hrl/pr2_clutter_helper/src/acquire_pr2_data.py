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
This program imports sensor data from the PR2.
By default it will look for a cloud (/table_cloud), an image on wide_stereo
left, and the necessary transformations, and camera_info. 
It also can map the laser points into the image frame, and color the cloud with the projected camera image.

This function runs well stand-alone, or can be imported and its functions called separately.

Python implementation by: Jason Okerman
"""
import roslib; roslib.load_manifest('pr2_clutter_helper')

#import sys
#import os
#from optparse import OptionParser

import rospy
from sensor_msgs.msg import Image, PointCloud, CameraInfo, ChannelFloat32
from geometry_msgs.msg import PointStamped,  Point32
from std_msgs.msg import Bool
from image_geometry.cameramodels import PinholeCameraModel
from cv_bridge import CvBridge
import message_filters
import tf

import cv as roscv #CV 2.1 from ROS 
from random import random, randint
import numpy as np

class AcquireCloud:
    def __init__(self, return_empty=False):
        self.bool_pub = rospy.Publisher("table_bool", Bool)
        self.cloud_pub = rospy.Publisher("table_cloud_colored", PointCloud)
        self.img_pub = rospy.Publisher("table_image/color_rect_image", Image)
        self.cam_pub = rospy.Publisher("table_image/camera_info", CameraInfo)
        self.img_msg = None
        self.cam_msg = None
        self.cloud_msg = None
        self.tf_matrix = None
        self.tf_translation = None
        self.tf_quaternion = None
        self.camPts = []
        self.pts3d = None
        self.intensities = []
        self.camPts_idx = []  #Formerly 'idx_list'
        self.img = np.array([])
        self.N = 0 #number of 3d points.
        #Optional self.sub1, sub2, sub3 --> created by subscribe() function.
        
        self.tfListener = tf.TransformListener()
        self.fixed_frame = '/base_footprint'  #must match header on cloud
        self.target_frame = '/wide_stereo_optical_frame'
                           #'/wide_stereo_gazebo_r_stereo_camera_optical_frame'
        self.cam_topic = "/wide_stereo/left/camera_info"
        self.img_topic = "/wide_stereo/left/image_rect_color"
        self.cloud_topic = "/table_cloud"
        self.cvBridge = CvBridge()
        if not return_empty: #subscribe imediately - blocking function
            self.subscribe()
            self.wait_for_data()
            self.unsubscribe()
            self.get_tf()
            self.print_test() ## 
            self.pts3d, self.intensities = self.get_pts3d(self.cloud_msg)
            self.cam_model = self.get_cam_model(self.cam_msg)
            self.camPts, self.camPts_idx, self.colors, self.img_dotted = self.get_camPts()
            roscv.SaveImage('/home/jokerman/Desktop/most_recent_img_dotted.png', self.img_dotted)
            #show_image(self.img_dotted)
            
            
    ''' Not Internal. Passed a cloud_msg
        Does more than title: Sets interanal pts3d and also intensities
        If intensities is not part of cloud_msg, then intensities will be empty array.
    '''        
    def get_pts3d(self, cloud_msg):
        N = len(cloud_msg.points)
        pts3d = np.zeros([3,N])
        for i in range(N):
            row = cloud_msg.points[i]
            pts3d[:,i] = (row.x, row.y, row.z);
        intensities = np.zeros(N)            
        for ch in cloud_msg.channels:
            if ch.name == 'intensities':
                for i in range(N):
                    intensities[i]=ch.values[i]
            break;        
        return np.matrix(pts3d), intensities    
    
    def pub_cloud(self):
        publish_cloud(self.cloud_pub, self.fixed_frame, self.pts3d, intensities=self.intensities, colors=self.colors)    
    
    def pub_cloud_bound(self):
        crop = self.camPts_idx
        publish_cloud(self.cloud_pub, self.fixed_frame, self.pts3d[:,crop], intensities=self.intensities[crop], colors=self.colors[:,crop])  
    
    def pub_cam(self, pub=None, frame=None, cam_msg_template=None):
        if not pub: pub = self.cam_pub
        #Stuff a new camera message object
        if not cam_msg_template: c = self.cam_msg
        else: c = cam_msg_template
        msg = CameraInfo(D=c.D,K=c.K,R=c.R,P=c.P,height=c.height,width=c.width)
        if not frame: frame=self.target_frame
        msg.header.frame_id = frame;        
        msg.header.stamp=rospy.Time.now()
        pub.publish(msg)
        
    def pub_img(self, pub=None, frame=None, image_template=None):
        if not pub: pub=self.img_pub
        #Stuff a new camera message object
        if not image_template: a = self.img_msg
        else: a = image_template
        msg = Image(height=a.height,width=a.width,encoding=a.encoding,data=a.data,is_bigendian=a.is_bigendian)
        if not frame: frame=self.target_frame
        msg.header.frame_id = frame;        
        msg.header.stamp=rospy.Time.now()
        pub.publish(msg)
    
    def get_cam_model(self, cam_msg):
        ''' Not internal.  Uses: camera_info message
        '''
        cam_model = PinholeCameraModel()
        cam_model.fromCameraInfo(cam_msg)
        return cam_model
        
    def print_test(self):
        print '---'
        print '* tf_matrix =\n', self.tf_matrix
        if self.img_msg:
            print '* img_msg has size', self.img_msg.width,'x',self.img_msg.height, '\n'
        if self.cam_msg:
            print '* cam_msg has K = \n', np.round(self.cam_msg.K)
        if self.cloud_msg:
            print '* cloud_msg has size', len(self.cloud_msg.points)
            print '* Cloud frame id', self.cloud_msg.header.frame_id
            print '* cloud_msg has channels:',
            for ch in self.cloud_msg.channels: print "'"+ch.name+"'",
        print '\n---'
        
    def wait_for_data(self):
        print 'wait_for_data'
        while not rospy.is_shutdown() and not (self.cam_msg and self.img_msg and self.cloud_msg):
            rospy.sleep(.5)
            print '\nWaiting for:',
            if not self.cam_msg: print 'cam_msg,',
            if not self.img_msg: print 'img_msg,',
            if not self.cloud_msg: print 'cloud_msg',
            
    ''' Internal.  Sets self.sub1, sub2, sub3 to specific callbacks.
    '''
    def subscribe(self):
        self.sub1 = rospy.Subscriber(self.img_topic, Image, self.image_callback )
        self.sub2 = rospy.Subscriber(self.cloud_topic, PointCloud , self.cloud_callback )
        self.sub3 = rospy.Subscriber(self.cam_topic, CameraInfo, self.camera_callback )

    def unsubscribe(self):
        self.sub1.unregister()
        self.sub2.unregister()
        self.sub3.unregister()
        print 'unregistering self'

    def image_callback(self, img_msg):
        if not self.img_msg: self.img_msg = img_msg; print '\n* img'
        else: print '..',
    
    def camera_callback(self, cam_msg):
        if not self.cam_msg: self.cam_msg = cam_msg; print '\n* camera'
        else: print '.',
          
    def cloud_callback(self, cloud_msg):
        if not self.cloud_msg: self.cloud_msg = cloud_msg;  print '\n* cloud'
        else: print '...',

    def get_tf(self):
        self.h = rospy.Header(frame_id=self.fixed_frame)  
        self.tf_matrix = self.tfListener.asMatrix(self.target_frame, self.h)
        # Alternate form
        self.tf_translation = tf.transformations.translation_from_matrix(self.tf_matrix)
        self.tf_quaternion = tf.transformations.quaternion_from_matrix(self.tf_matrix)

    def callback(self, img_msg, cam_msg=None, cloud_msg=None):
        print 'callback starting'

    def test_publish(self):
        self.bool_pub.publish(bool(randint(0,1)) )
        print 'publish complete'
        
    
    def return_data_for_segmentation(self):
        '''Returns: img, --> Numpy image #Formerly ROS CV 2.1 type image
                    pts3d, --> 3xN numpy array of 3D points
                    intensities, --> N length numpy array
                    camPts, --> 2xN numpy array of camera points.  
                        Points are projection from 3D. May be outside frame of camera (need camPts_idx).
                    colors, --> 3xN numpy array (0,1,2) = (r,g,b)
                    camPts_idx (Formerly idx_list) --> boolean array of size N for cropping
        '''
        return self.img, self.pts3d, self.intensities, self.camPts, self.colors, self.camPts_idx

  
    ''' Internal. Uses: cloud , img_msg, camera_model, bounds=None:
        Does a lot more: camPts_idx -> boolean array, true if 3D point false in camera view
        self.img -> from img_msg (numpy array)
        img_dotted -> for calibration check           
        
        *** NOTE, in absence of working "transfromPointCloud" function, this only works if 
        cloud is passed already in optical camera frame.  :(
    '''
    def get_camPts_already_in_cam_frame(self):
        self.img = self.cvBridge.imgmsg_to_cv(self.img_msg)
        img_dotted = np.array(self.img) # = roscv.CloneImage(np.array(img)) doesn't 
                                      # work between versions cv 2.0 and 2.1
        height = self.img_msg.height; width = self.img_msg.width;
        N = len(self.cloud_msg.points)
        colors = np.zeros( (3,N) )
        camPts = np.zeros( (2,N) )
        camPts_idx = np.array([False for i in range(N)]) #list of boolean False's
        #billybob = tf.TransformerROS()
        #cloud_msg_cam_frame = billybob.transformPointCloud('base_footprint', self.cloud_msg)
        i=0
        for row in self.cloud_msg.points:
            pt3d = (row.x, row.y, row.z);
            uv = self.cam_model.project3dToPixel(pt3d)
            camPts[:,i] = uv
            x = uv[0]; y = uv[1];
            if (x >=0 and y >=0 and x<width and y <height):
                roscv.Circle(img_dotted, uv, 0, (255,100,100) )
                row = int(y); col = int(x);
                r,g,b = (0,1,2)
                color = roscv.Get2D(self.img, row, col)
                colors[r,i] = color[r]
                colors[g,i] = color[g]
                colors[b,i] = color[b]
                camPts_idx[i] = True
            #else: print 'point ',uv,'out of bounds'
            i+=1;
        return camPts, camPts_idx, colors, img_dotted 
  
    ''' Internal. Uses: cloud , img_msg, camera_model, tf_matrix, bounds=None:
        Does a lot more: camPts_idx -> boolean array, true if 3D point false in camera view
                         self.img -> from img_msg (numpy array)
                         img_dotted -> for calibration check        
    '''
    def get_camPts(self):
        self.img = self.cvBridge.imgmsg_to_cv(self.img_msg)
        img_dotted = np.array(self.img) # = roscv.CloneImage(np.array(img)) doesn't 
                                        # work between versions cv 2.0 and 2.1
        height = self.img_msg.height; width = self.img_msg.width;
        N = len(self.cloud_msg.points)
        #-------------
        camPts = np.zeros( (2,N) )
        pts3d_camframe = self.get_pts3d_camframe()
        for i in range(N): 
            pt3d = pts3d_camframe[0,i], pts3d_camframe[1,i], pts3d_camframe[2,i] ##
            uv = self.cam_model.project3dToPixel(pt3d)
            camPts[:,i] = uv #2D point in camera frame
        #----------
        colors = np.zeros( (3,N) )
        camPts_idx = np.array([False for i in range(N)]) #list of boolean False's
        for i in range(N):   ##
            x = int(camPts[0,i]); y = int(camPts[1,i]);
            if (x >=0 and y >=0 and x<width and y <height):
                row = int(y); col = int(x);
                r,g,b = (0,1,2)
                color = roscv.Get2D(self.img, row, col)
                colors[r,i] = color[r]
                colors[g,i] = color[g]
                colors[b,i] = color[b]
                camPts_idx[i] = True
                roscv.Circle(img_dotted, (x,y), 0, (50,250,50) )
                
        return camPts, camPts_idx, colors, img_dotted 
        
  
    ''' Internal. Uses: tf_matrix, cloud_msg, img_msg, bridge,
            Notes: camTlaser is now called tf_matrix
            #focus_matrix is like "K" but with cx and cy offset removed.
    '''
    def get_camPts_by_matrix(self):
        self.pts3d,_ = self.get_pts3d(self.cloud_msg)
        self.get_tf()
        K = np.matrix(self.cam_msg.K).reshape(3,3)
        cam_centers = ( K[0,2], K[1,2] ) #cx and cy
        # Keep fx, fy, but clear cx and cy.  Make homogeneous.
        (fx, fy) = K[0,0], K[1,1]
        focus_matrix =  np.matrix([[fx, 0, 0,   0],[0, fy, 0,   0],[0,  0, 1,   0]])
        #--------
        pts3d_camframe = apply_transform_matrix( self.tf_matrix, self.pts3d)
        camPts = focus_matrix* xyzToHomogenous(pts3d_camframe)
        (u,v,w) = (0,1,2)
        camPts[u] = camPts[u] / camPts[w] + cam_centers[u] #shift over
        camPts[v] = camPts[v] / camPts[w] + cam_centers[v] #shift over
        camPts = np.matrix( np.round(camPts[:2]), 'int')
        #As last step, ensure that camPts is 2xN and not 3xN, also round
        #--------
        self.img = self.cvBridge.imgmsg_to_cv(self.img_msg)
        img_dotted = np.array(self.img)
        height = self.img_msg.height; width = self.img_msg.width;
        N = len(self.cloud_msg.points)
        colors = np.zeros( (3,N) )
        camPts_idx = np.array([False for i in range(N)]) #list of boolean False's
        for i in range(N):   ##
            x = camPts[0,i]; y = camPts[1,i];
            if (x >=0 and y >=0 and x < width and y < height):
                roscv.Circle(img_dotted, (x,y), 0, (0,100,255) )
                row = int(y); col = int(x);
                r,g,b = (0,1,2)
                color = roscv.Get2D(self.img, row, col)
                colors[r,i] = color[r]
                colors[g,i] = color[g]
                colors[b,i] = color[b]
                camPts_idx[i] = True
        return camPts, camPts_idx, colors, img_dotted 
   
    def get_pts3d_camframe(self):
        if self.tf_matrix == None: self.get_tf()
        if self.pts3d == None: self.pts3d, self.intensities = self.get_pts3d(self.cloud_msg)
        #If having problems try inverted matrix: np.linalg.inv(self.tf_matrix)
        pts3d_camframe = apply_transform_matrix( self.tf_matrix, self.pts3d )
        return pts3d_camframe
   
   
   
   
   
#---------------------      

'''This appears multiply a Transform matrix by a 3xN-element point or point set.
   Note that xyzToHomogenous is made to convert 3XN matrix, to 4XN matrix in homog coords
'''
def apply_transform_matrix(T, p):
    pn = T * xyzToHomogenous(p)
    return pn[0:3] / pn[3]
    
"""This is redundantly defined in hrl_lib.transforms.py, as part of gt-ros-pkg
   convert 3XN matrix, to 4XN matrix in homogenous coords
"""
def xyzToHomogenous(v, floating_vector=False):
    if floating_vector == False:
        return np.row_stack((v, np.ones(v.shape[1])))
    else:
        return np.row_stack((v, np.zeros(v.shape[1])))
        
def publish_cloud(pub, frame, pts3d, intensities=None, labels=None, colors=None):
    cloud = convert_pointcloud_to_ROS(pts3d, intensities, labels, colors)
    if not frame: frame='/base_footprint'
    cloud.header.frame_id = frame
    cloud.header.stamp=rospy.Time.now()
    pub.publish(cloud)

def filter_example():  #THIS COULD WORK, BUT DOESN'T BECAUSE TIMESTAMPS DO NOT MATCH
    rospy.init_node('rosSegmentCloud')
    pos_pub = rospy.Publisher("face_position", PointStamped)
    print 'Filter the image topic and point cloud topic to come in together'
    image_sub = message_filters.Subscriber("/wide_stereo/left/image_rect_color", Image)
    cloud_sub = message_filters.Subscriber("/tilt_scan_cloud", PointCloud )
    camera_sub = message_filters.Subscriber("/wide_stereo/left/camera_info", CameraInfo)

    ts = message_filters.TimeSynchronizer([image_sub, camera_sub], 10)
    ts.registerCallback(cs.callback)
    
def show_image(img, window_name='img', wait=False):
    roscv.StartWindowThread(); roscv.NamedWindow(window_name)
    roscv.ShowImage(window_name, img)
    if wait: roscv.WaitKey()
    
def save_cloud_as_py(cloud, path = './saved_cloud.py'):
    f = open(path,'w')
    f.write('#number of points = %i \n' %len(cloud.points) )
    f.write('points = [')
    for row in cloud.points:
        f.write('\n    [%.5f, %.5f, %.5f ],' %(row.x, row.y, row.z) )
    f.write(']')
    #Optionally print intensity array also.
    for channel in cloud.channels:
        if channel.name == "intensities":
            f.write('\n\nintensities = [')
            for row in channel.values:
                f.write('%i, ' %int(row) )
            f.write(']\n\n') 
            break;
    f.close()
     
''' Expects pts3D and colors to be 3xN, intensities and labels to be N.
'''
def convert_pointcloud_to_ROS(pts3d, intensities = None, labels = None, colors=None, bgr=True):
    ROS_pointcloud = PointCloud()
    ROS_pointcloud.points = []
    ROS_pointcloud.channels = []
    (x,y,z) = (0,1,2)
    for pt in np.asarray(pts3d.T):
        ROS_pointcloud.points += [Point32(pt[x], pt[y], pt[z])]
        
    intensity_channel = ChannelFloat32(name = 'intensities')
    if intensities != None:
        for value in intensities:
            intensity_channel.values += [value]
        ROS_pointcloud.channels += [intensity_channel]
        
    label_channel = ChannelFloat32(name = 'labels')   
    if labels != None:
        for value in labels:
            label_channel.values += [value]  
        ROS_pointcloud.channels += [label_channel] 
            
    if colors != None:    
        r_ch = ChannelFloat32(name='r')   
        g_ch = ChannelFloat32(name='g')   
        b_ch = ChannelFloat32(name='b')   
        rgb_ch = ChannelFloat32(name='rgb')
        for (r,g,b) in np.asarray(colors.T):
            #NOTE: CV image is in bgr order.  Ros expects rgb order
            r_ch.values += [b/256]   
            g_ch.values += [g/256]  
            b_ch.values += [r/256]    
            rgb_color = int(b*0xFF*0xFF+ g*0xFF+r) #incorrect
            rgb_ch.values += [rgb_color]
        ROS_pointcloud.channels += [r_ch, g_ch, b_ch]
    return ROS_pointcloud
    
    #---------------        
    
def main():
    print 'Get Bridge'
    br = CvBridge()

    print 'Setup the ros node'
    rospy.init_node('rosSegmentCloud')

    print 'Setup the Acquire Object'
    ac = AcquireCloud()
    
    ac.print_test()
    
    show_image(ac.img, 'img')
    show_image(ac.img_dotted, 'dotted')
    ###tf_br = tf.TransformBroadcaster()
    
    print 'publish test'
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        ###tf_br.sendTransform(ac.tf_translation, ac.tf_quaternion, now, "table_camera", ac.fixed_frame)
        ### tf == sadness :(
        ac.pub_cloud_bound()
        
        im_msg = br.cv_to_imgmsg(ac.img_dotted)
        im_msg.header.frame_id = ac.target_frame ###"table_camera" 
        
        im_msg.header.stamp = now
        ac.img_pub.publish(im_msg)
        cam_msg = ac.cam_msg
        cam_msg.header = rospy.Header(frame_id=ac.target_frame, stamp=now)
        ###cam_msg.header = rospy.Header(frame_id="table_camera", stamp=now)
        ac.cam_pub.publish(cam_msg)
        
               
        ac.test_publish()
        print 'pub'
        rospy.sleep(1)

if __name__ == '__main__':
    main()   
    

    
