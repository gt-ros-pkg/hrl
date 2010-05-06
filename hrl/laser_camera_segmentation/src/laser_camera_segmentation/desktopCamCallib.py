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

#  \author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)
#  \author Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)

import roslib; roslib.load_manifest('laser_camera_segmentation')

if __name__ == '__main__':
    import scanr_transforms as trs
    import transforms as tr
    import numpy as np,math
    import webcam_config as cc
    import opencv as cv
    import opencv.highgui as hg
    import hrl_lib.util as ut
    import tilting_hokuyo.processing_3d as p3d
    import laser_cam_callib
    import pygame

    #seeds = np.array([1.0, 0.9, -1.7, 3.1, 0.061, 0.032, -0.027 ])
    #seeds = np.array([0.0, 0.0, -0.0, 0.0, 0.061, 0.032, -0.027 ])
   
    
    #seeds = np.array([0.8, 0.9, -1.7, 3.1, 0.061, 0.032, -0.035 ])
    
    #seeds = np.array([2.4000  ,  3.8000  , -2.9000   , 5.5000  ,  0.0600  ,  0.0300  , -0.0430  ])
   # seeds = np.array([2.2000  ,  2.0000  , -2.8000  ,  5.5000  ,   0.0500  ,  0.0300  , -0.0430    ])
# 2.0000    2.0000   -2.8000    5.5000    0.0550    0.0300  x  -0.0400
    #seeds = np.array([0.9000  ,  0.8000 ,  -2.2000 ,   3.1000 ,   0.0620 ,   0.0320,   -0.0270 ])
    seeds = np.array([  1.8000  ,  1.7000  , -2.6000 ,   4.7500 ,   0.0620  ,  0.0320  , -0.0270  ])







    
    deltas = np.array([0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001 ])
    #-1.0000  x   1.7000   -2.2000    6.4000   -0.0200    0.0300   -0.0430  

    names = ['Ry_0', 'Rz_0', 'Rx_-90', 'Rz_-90', 'dx', 'dy', 'dz']


    def camTlaser( res = np.zeros(6) ):
        rot = tr.Ry( math.radians( 0.0 + res[0] )) * tr.Rz( math.radians( 0.0 + res[1] )) * tr.Rx( math.radians( -90.0 + res[2] )) * tr.Rz( math.radians( -90.0 + res[3]))
        disp = np.matrix([ res[4], res[5], res[6] ]).T + np.matrix([ 0.0, 0.0, 0.0 ]).T
        return tr.composeHomogeneousTransform(rot, disp)

    cameraTlaser = trs.camTlaser

    cp = cc.webcam_parameters['DesktopWebcam']
    fx = cp['focal_length_x_in_pixels']
    fy = cp['focal_length_y_in_pixels']
    cam_proj_mat =  np.matrix([[fx, 0, 0,   0],
                               [0, fy, 0,   0],
                               [0,  0, 1,   0]])

    cam_centers = ( cp['optical_center_x_in_pixels'], cp['optical_center_y_in_pixels'] )

    
    #take image and scan
    import scanner  
    import configuration    
    #id = '2009Nov04_144041'
    id = '2009Nov04_135319'
    
    cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/labeling')
    img = hg.cvLoadImage(cfg.path + '/data/' + id + '_image.png')
    thok_dict = ut.load_pickle(cfg.path + '/data/' + id + '_laserscans.pkl')
    #cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/calib')
    #cfg.webcam_id = 0
    #sc = scanner.scanner(cfg)
    #sc.capture_and_save('calib', False)
    #img = hg.cvLoadImage('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/calib/data/calib_image.png')
    #thok_dict = ut.load_pickle('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/calib/data/calib_laserscans.pkl')
    poses, scans = thok_dict['laserscans'][0]
    points_cloud_laser = p3d.generate_pointcloud(poses, scans, math.radians(-180), math.radians(180), 
                                0, .035, max_dist=5.0, min_dist=.1)

    c = laser_cam_callib.Callib(cameraTlaser, seeds, deltas, names, points_cloud_laser, img, cam_proj_mat, cam_centers,1, id)
    
    while not c.reDraw():
        tmp = 1

    pygame.quit()
    
