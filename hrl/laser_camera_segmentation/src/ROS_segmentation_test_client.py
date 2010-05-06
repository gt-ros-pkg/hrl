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


import laser_camera_segmentation.processor as processor
import laser_camera_segmentation.configuration as configuration

#from placement import Placement
from labeling.label_object import label_object
from labeling.scan_dataset import scan_dataset
#
import numpy as np
import opencv as cv


from laser_camera_segmentation.ROS_interface_helper_functions import *

from laser_camera_segmentation.srv._Segmentation import *
import sensor_msgs



def get_test_data():
    cfg = configuration.configuration('../data/ROS_test_client/', 'dummyScanner')
    pc = processor.processor(cfg)
    pc.scan_dataset = scan_dataset()
    pc.scan_dataset.image_artag_filename = ''
    pc.scan_dataset.table_plane_translation = np.matrix([0,0,0]).T
    #approximate height of hokuyo above the ground, assume it to be fixed for now.
    pc.scan_dataset.ground_plane_translation = np.matrix([0,0,1.32]).T 
    pc.scan_dataset.ground_plane_rotation = ''
    #pc.scan_dataset.is_test_set = True
    pc.scan_dataset.is_labeled = True
    pc.scan_dataset.id = 'ROStest'
    pc.load_raw_data(pc.scan_dataset.id)
    image_size = cv.cvGetSize(pc.img) 

    #don't do any mapping to not change the pts3d!
    #create point cloud from raw range points
    (pc.pts3d, pc.scan_indices, pc.intensities) = pc.create_pointcloud(pc.laserscans, True, True)
    
    #region of interest in image pixels
    polygon = label_object()
    polygon.set_label('surface')
    width_min = image_size.width * 0.4
    width_max = image_size.width * 0.95
    height_min = image_size.height * 0.3
    height_max = image_size.height * 0.95
    polygon.add_point((width_min,height_min))
    polygon.add_point((width_min,height_max))
    polygon.add_point((width_max,height_max))
    polygon.add_point((width_max,height_min))    
    
    return pc.pts3d, pc.intensities, None, pc.img,  pc.image_angle, polygon


def test_segmentation():
    rospy.wait_for_service('segment_pointcloud')
    try:
        pts3d, intensities, labels, image, image_angle, polygon = get_test_data()
        ROS_pointcloud = convert_pointcloud_to_ROS(pts3d, intensities, labels)
        ROS_image = convert_cvimage_to_ROS(image)
        imageSize = cv.cvGetSize(image)
        
        ROS_polygon = convert_polygon_to_ROS(polygon)
        
        
        segment_pointcloud = rospy.ServiceProxy('segment_pointcloud', Segmentation)
        request = SegmentationRequest()
        request.imageWidth = 41
        request.pointcloud = ROS_pointcloud
        request.image = ROS_image
        request.imageWidth = imageSize.width
        request.imageHeight = imageSize.height
        request.imageAngle = image_angle
        request.regionOfInterest2D = ROS_polygon
        request.laserHeightAboveGround = 1.32
        request.numberOfPointsToClassify = 1000 #-1 == all
        
        response = segment_pointcloud(request)
        
        pts3d_bound, intensities, labels = convert_ROS_pointcloud_to_pointcloud(response.pointcloud)
        cfg = configuration.configuration('../data/ROS_test_client/', 'dummyScanner')
        pc = processor.processor(cfg)
        
        pc.scan_dataset = scan_dataset()
        pc.scan_dataset.image_artag_filename = ''
        pc.scan_dataset.table_plane_translation = np.matrix([0,0,0]).T
    
        pc.scan_dataset.ground_plane_translation = np.matrix([0,0,request.laserHeightAboveGround]).T 
        pc.scan_dataset.ground_plane_rotation = ''
        pc.scan_dataset.is_labeled = True
        pc.scan_dataset.id = 'ROStest'
        pc.image_angle = ''
        pc.pts3d_bound = pts3d_bound
        pc.map_polys = labels
        pc.scan_dataset.ground_plane_normal = np.matrix([0.,0.,1.]).T

        from enthought.mayavi import mlab
        pc.display_3d('labels')
        mlab.show()        
        
        
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#def usage():
#    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    rospy.init_node('segmentation_client')
#    if len(sys.argv) == 3:
#        x = int(sys.argv[1])
#        y = int(sys.argv[2])
#    else:
#        print usage()
#        sys.exit(1)
        
    print 'segmentation_client'
    test_segmentation()
