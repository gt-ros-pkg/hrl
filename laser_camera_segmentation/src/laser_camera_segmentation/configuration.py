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


import numpy as np,math


class configuration(object):
    '''
    classdocs
    '''


    def __init__(self, path = '../data/', device = 'desktopScanner'):
        
        '''
        set default values
        '''
        self.path = path
        self.pointcloud_max_dist = 5.0
        self.pointcloud_min_dist = 0.1    
        
        self.device = device
        
        if device == 'desktopScanner':
            import webcam_config as cc
            self.webcam_id = 1
            
            #most code from travis scanr-class:
            # Initialize webcam
            self.cam_name = 'DesktopWebcam'
            cp = cc.webcam_parameters[self.cam_name]
            fx = cp['focal_length_x_in_pixels']
            fy = cp['focal_length_y_in_pixels']
            self.cam_proj_mat =  np.matrix([[fx, 0, 0,   0],
                                            [0, fy, 0,   0],
                                            [0,  0, 1,   0]])
    
            self.cam_centers = ( cp['optical_center_x_in_pixels'], cp['optical_center_y_in_pixels'] )
    
            # cam_vec came from a previous laser_cam_callibration
            #self.cam_vec = np.array([0.8, 0.9, -1.7, 3.1, 0.061, 0.032, -0.035 ])
            #self.cam_vec = np.array([1.2000,    1.2000 ,  -1.4000  ,  3.6000  ,  0.0600   , 0.0330   ,-0.0200])
            #self.cam_vec = np.array([0.9000  ,  0.8000 ,  -2.2000 ,   3.1000 ,   0.0620 ,   0.0320,   -0.0270 ])
            self.cam_vec = np.array([  1.8000  ,  1.7000  , -2.6000 ,   4.7500 ,   0.0620  ,  0.0320  , -0.0270  ])             
            
            #self.cam_vec = np.array([ 0.0  , 0.0  , 0.0 ,   0.0 ,   0.0 ,  0.0  , 0.0  ])                        
                       
            self.cam_deltas = np.array([0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001 ])
            self.cam_names = ['Ry_0', 'Rz_0', 'Rx_-90', 'Rz_-90', 'dx', 'dy', 'dz']
            import scanr_transforms as trs
            self.camTlaser = trs.camTlaser(self.cam_vec)
            
            
            self.scanner_metal_plate_offset = 0.05 #TODO
    
            # Initialize THOK
            self.thok_l1 = 0
            self.thok_l2 = 0.035
            self.thok_tilt_angles = (math.radians(40.0),math.radians(-40.0))
            self.thok_devname = '/dev/robot/desktopServos'
            self.thok_servonum = 19
            self.thok_hoknum = 0
            self.thok_scan_speed = math.radians(5.0)
            
        elif device == 'codyRobot':
            import hrl_camera.camera_config as cc
            
            self.webcam_id = 0
            
            #values from equilibrium_point_control/lpi.py
            self.cam_name = 'mekabotUTM'
            cp = cc.camera_parameters[self.cam_name]
            fx = cp['focal_length_x_in_pixels']
            fy = cp['focal_length_y_in_pixels']
            
            self.cam_proj_mat =  np.matrix([[fx, 0, 0,   0],
                                            [0, fy, 0,   0],
                                            [0,  0, 1,   0]])
            self.cam_centers = ( cp['optical_center_x_in_pixels'], cp['optical_center_y_in_pixels'] )
    

            #self.camTlaser = mcf.utmcam0Tglobal(mcf.globalTthok0(m),self.image_angle)
            
            
            # Initialize THOK
            self.thok_l1 = 0
            self.thok_l2 = -0.055
            self.thok_tilt_angles = (math.radians(40.0),math.radians(-40.0))
            self.thok_devname = '/dev/robot/servo0'
            self.thok_servonum = 5
            self.thok_hoknum = 0
            self.thok_scan_speed = math.radians(10.0) #speed=10 in lpi
            
        elif device == 'dummyScanner': #just for testing/demonstration without dependencies outside of gt-ros-pkgk
            self.webcam_id = 0
            
            #values from equilibrium_point_control/lpi.py
            self.cam_name = 'dummyUTM'
            import opencv as cv
            #values copied from Cody
            cp =     {'calibration_image_width' : 640.0,
            'calibration_image_height' : 480.0, 
            'focal_length_x_in_pixels' : 362.381,
            'focal_length_y_in_pixels' : 362.260,
            'optical_center_x_in_pixels' : 275.630,
            'optical_center_y_in_pixels' : 267.914,
            'lens_distortion_radial_1' :    -0.270544,
            'lens_distortion_radial_2' :    0.0530850,
            'lens_distortion_tangential_1' : 0,
            'lens_distortion_tangential_2' : 0,
            'opencv_bayer_pattern' :  cv.CV_BayerBG2BGR,
            'color': True,
            'uid': 8520228
            }
            fx = cp['focal_length_x_in_pixels']
            fy = cp['focal_length_y_in_pixels']
            
            self.cam_proj_mat =  np.matrix([[fx, 0, 0,   0],
                                            [0, fy, 0,   0],
                                            [0,  0, 1,   0]])
            self.cam_centers = ( cp['optical_center_x_in_pixels'], cp['optical_center_y_in_pixels'] )
    

            #self.camTlaser = mcf.utmcam0Tglobal(mcf.globalTthok0(m),self.image_angle)
            
            
            # Initialize THOK
            self.thok_l1 = 0
            self.thok_l2 = -0.055
            self.thok_tilt_angles = (math.radians(40.0),math.radians(-40.0))
            self.thok_devname = '/dev/robot/servo0'
            self.thok_servonum = 5
            self.thok_hoknum = 0
            self.thok_scan_speed = math.radians(10.0) #speed=10 in lpi

        else:
            print 'ERROR: unknown device',device