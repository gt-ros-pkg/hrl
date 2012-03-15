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

#  \author Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)

from laser_camera_segmentation.srv._Segmentation import *
import sensor_msgs

import numpy as np

import opencv as cv
import hrl_lib.util as ut
import opencv.highgui as highgui

from labeling.label_object import label_object

def convert_pointcloud_to_ROS(pts3d, intensities = None, labels = None):
    ROS_pointcloud = sensor_msgs.msg.PointCloud()
    ROS_pointcloud.points = []
    for point in np.asarray(pts3d.T):
        ROS_pointcloud.points += [geometry_msgs.msg.Point32(point[0],point[1],point[2])]
        
    intensity_channel = sensor_msgs.msg.ChannelFloat32()
    intensity_channel.name = 'intensities'
    if intensities != None:
        for value in intensities:
            intensity_channel.values += [value]
        
    label_channel = sensor_msgs.msg.ChannelFloat32()   
    label_channel.name = 'labels' 
    if labels != None:
        for value in labels:
            label_channel.values += [value]    
    
    
    ROS_pointcloud.channels = [intensity_channel, label_channel]
    return ROS_pointcloud

def convert_ROS_pointcloud_to_pointcloud(ROS_pointcloud):
    intensities = []
    labels = []
    pts3d = []
    for point in ROS_pointcloud.points:
        pts3d += [[point.x, point.y, point.z]]
    pts3d = np.array(pts3d).T
    #print pts3d
    
    for value in ROS_pointcloud.channels[0].values:
        intensities += [value]
    intensities = np.array(intensities)  
    
    for value in ROS_pointcloud.channels[1].values:
        labels += [value]
    labels = np.array(labels)  
    
    return pts3d, intensities, labels


def convert_cvimage_to_ROS(image):
    imgTmp = cv.cvCloneImage(image)
    imNP = ut.cv2np(imgTmp,format='BGR')
    ROS_image = np.reshape(imNP,(1,-1))[0]

    return ROS_image

def convert_ROS_image_to_cvimage(ROS_image, width, height):
    ROS_image = np.array(ROS_image, dtype='uint8')
    imNP = np.reshape(ROS_image,(height,width, 3))
    cvimage = ut.np2cv(imNP)

    return cvimage


#convert label_object to ROS geometry_msgs.msg.Polygon
def convert_polygon_to_ROS(polygon):
    ROS_polygon = geometry_msgs.msg.Polygon()
    ROS_polygon.points = []
    for point in polygon.get_points():
        ROS_polygon.points += [geometry_msgs.msg.Point32(point[0], point[1], 0.)]
    return ROS_polygon


def convert_ROS_polygon_to_polygon(ROS_polygon):
    polygon = label_object() 
    for point in ROS_polygon.points:
        polygon.add_point((point.x,point.y))
    return polygon
