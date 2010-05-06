#!/usr/bin/env python
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

import roslib; roslib.load_manifest('laser_camera_segmentation')

import rospy

from laser_camera_segmentation.srv._Segmentation import *
import sensor_msgs

from laser_camera_segmentation.ROS_interface_helper_functions import *

import laser_camera_segmentation.processor as processor
import laser_camera_segmentation.configuration as configuration

from labeling.scan_dataset import scan_dataset

def segment_pointcloud(request):
    #convert data from ROS
    pts3d, intensities, labels = convert_ROS_pointcloud_to_pointcloud(request.pointcloud)
    cvimage = convert_ROS_image_to_cvimage(request.image, request.imageWidth, request.imageHeight)
    polygon = convert_ROS_polygon_to_polygon(request.regionOfInterest2D)
    polygon.set_label('surface')
    print polygon, polygon.get_points()
    
    #create processor and configuration
    #cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/ROS_server_test', 'codyRobot')
    cfg = configuration.configuration('../data/ROS_server', 'dummyScanner')
    pc = processor.processor(cfg)
    
    pc.scan_dataset = scan_dataset()
    pc.scan_dataset.image_artag_filename = ''
    pc.scan_dataset.table_plane_translation = np.matrix([0,0,0]).T

    pc.scan_dataset.ground_plane_translation = np.matrix([0,0,request.laserHeightAboveGround]).T 
    pc.scan_dataset.ground_plane_rotation = ''
    #pc.scan_dataset.is_test_set = True
    pc.scan_dataset.is_labeled = True
    pc.scan_dataset.id = 'ROStest'
    
    pc.img = cvimage
    pc.image_angle = request.imageAngle
    pc.pts3d = pts3d
    pc.intensities = intensities
    pc.scan_indices = np.zeros(len(intensities))
    
    pc.scan_dataset.polygons.append(polygon)
    pc.do_all_point_cloud()
    pc.do_polygon_mapping()
    
    pc.scan_dataset.ground_plane_normal = np.matrix([0.,0.,1.]).T
    
    if request.numberOfPointsToClassify == -1:
        n = 999999999999
    else:
        n = request.numberOfPointsToClassify
    feature_data = pc.generate_features(n, False, True)
    labels, testresults = pc.load_classifier_and_test_on_dataset('all_post', feature_data)

    response = SegmentationResponse()
    response.pointcloud = convert_pointcloud_to_ROS(pc.pts3d_bound, pc.intensities_bound, labels)
    return response


def segmentation_server():
    rospy.init_node('segmentation_server')
    s = rospy.Service('segment_pointcloud', Segmentation, segment_pointcloud)
    print "Ready to segment pointclouds!"
    rospy.spin()

if __name__ == "__main__":
    segmentation_server()