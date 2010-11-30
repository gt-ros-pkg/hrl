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
import roslib; roslib.load_manifest('tf')
from tf import transformations

# Optional imports below:
#     import scanr_transforms as trs [for camTlaser(cam_vec) , when "DesktopScanner"]
#  x  import codyRobot_camera_config as cc [for camera properties when "CodyRobot"]


# WARNING: THE "PR2" configurations are for a fixed demo setup.  Actual values are 
#   populated by "acquire_pr2_data.py", or something similar

class configuration(object):
    '''
    Define camera properties matrix: fx, fy, and optical center in x,y
    Define rotation/translation matrix between camera origin and laser cloud origin
    Three types of robot/device accepted:
        "desktopScanner" -- used to collect dataset of tablescans
        "codyRobot" -- demonstrated object placement in clutter first on this platform
        "dummyScanner" -- esentially same as codyRobot, with no dependancies.
        "Other" -- quits with warning
        "PR2" or  -- requires tf_msg (geometry_msgs.msg.StampedTransform) to be passed
            also.  This is because the Transform is dynamic, unlike our other robots which have had a
            fixed relative placement between the camera and tilting hokuyo.
        "PR2example" -- initialized with a fixed transform between pointcloud 
             and camera, good for example data only.              
    
    '''

    def __init__(self, path = '../data/', device = 'desktopScanner', tf_msg = None):
        '''
        set default values
        '''
        self.path = path
        self.pointcloud_max_dist = 5.0
        self.pointcloud_min_dist = 0.1    
        
        self.device = device
        
        if device == 'PR2' or device == 'PR2example':
        
            self.cam_name = 'wide_stereo/right/image_rect_color'
            fx = 428.48 #focal lengths in pixels
            fy = 428.35
            self.cam_proj_mat = np.matrix([[fx, 0, 0,   0],
                                           [0, fy, 0,   0],
                                           [0,  0, 1,   0]])
            cx = 323.4 #in pixels
            cy = 242.9 #in pixels
            self.cam_centers = (cx, cy)
            self.cam_image_height= 480 #px
            self.cam_image_width= 640 #px
            
            #Transform properties will depend on wide_stereo_optical_frame
            #    -to-base_footprint TF since the PR2
            #can move its head relative to the tilting hokuyo.
            #
            
            ######### EXAMPLE result from TF on PR2 ##############
            #  header: --
            #  frame_id: /base_footprint
            #    child_frame_id: /wide_stereo_optical_frame 
            #    transform: 
            #      translation: 
            x = 0.111181322026
            y= 0.0201393251186 #-0.09 #Using right camera is shifted over by 9cm.
            z= 1.39969502374 #+0.051 #- 1.32  #****MY data was in BASE_LINK FRAME??
            #      rotation:  (rotation same as optical frame of Gazebo_R and Gazebo_L_ optical)
            rx= -0.625821685412
            ry= 0.66370971141
            rz= -0.30689909515
            rw= 0.271384565597
            #euler_angles = transformations.euler_from_quaternion([rx,ry,rz,rw])
            #In euler: (-132, -1.4, -94) in degrees.
            #####################################################
            
            #kill soon
            # Initialize THOK
            self.thok_l1 = 0
            self.thok_l2 = -0.055
            self.thok_tilt_angles = (math.radians(40.0),math.radians(-40.0))
            self.thok_devname = '/dev/robot/servo0'
            self.thok_servonum = 5
            self.thok_hoknum = 0
            self.thok_scan_speed = math.radians(10.0) #speed=10 in lpi
            
            if device == 'PR2' and tf_msg: 
                #requires an appropriate TF message (type=TransformStamped) called tf_msg.
                #Condition: header.frame_id = '/base_footprint', child_frame_id = '/wide_stereo_optical_frame'
                t = tf_msg.transform.translation
                r = tf_msg.transform.rotation
                (x,y,z) = (t.x, t.y, t.z)
                (rx, ry, rz, rw) = (r.x, r.y, r.z, r.w)             
            T = transformations.translation_matrix([x,y,z])
            R = transformations.quaternion_matrix([rx,ry,rz,rw])
            print 'R=',R
            print 'T=',T
            M = np.matrix(R);
            M[:3,3] = np.matrix(T)[:3,3]
            print 'M=',M
            #hack
            M = np.linalg.inv(M)
            self.camTlaser = M
#   (wrong)  TRmatrix = [[-0.06939527, -0.66415251,  0.74436936,  0.11118132],
#                        [-0.99730322,  0.02832033, -0.06770713, -0.06986067],
#                        [ 0.02388707, -0.74706051, -0.66432673,  1.39969502],
#                        [ 0.        ,  0.        ,  0.        ,  1.        ]]
            #Result is a 4x4 array: [ R | t ]
            #                       [ 0 | 1 ]

#            np.array([[ 0.74436936,  0.06939527,  0.66415251,  0.        ],
#                      [-0.06770713,  0.99730322, -0.02832033,  0.        ],
#                      [-0.66432673, -0.02388707,  0.74706051,  0.        ],
#                      [ 0.        ,  0.        ,  0.        ,  1.        ]])
          
        elif device == 'desktopScanner':
            import scanr_transforms as trs
            self.webcam_id = 1
            
            #From webcam_config definition formerly in robot1-->hrl_lib
            #  Parameter definitions for camera used on desktopScanner
            webcam_parameters = {
                'DesktopWebcam':
                {  
                    'calibration_image_width' : 960.0,
                    'calibration_image_height' : 720.0, 
                    'focal_length_x_in_pixels' : 794.985,
                    'focal_length_y_in_pixels' : 797.122,
                    'optical_center_x_in_pixels' : 491.555,
                    'optical_center_y_in_pixels' : 344.289,
                    'lens_distortion_radial_1' :    0.0487641,
                    'lens_distortion_radial_2' :    -0.128722,
                    'lens_distortion_tangential_1' : 0,
                    'lens_distortion_tangential_2' : 0,
                    'opencv_bayer_pattern' : None,
                    'color': True,
                }
            }
            
            #most code from travis scanr-class:
            # Initialize webcam
            self.cam_name = 'DesktopWebcam'
            cp = webcam_parameters[self.cam_name]
            fx = cp['focal_length_x_in_pixels']
            fy = cp['focal_length_y_in_pixels']
            self.cam_proj_mat =  np.matrix([[fx, 0, 0,   0],
                                            [0, fy, 0,   0],
                                            [0,  0, 1,   0]])
    
            self.cam_centers = ( cp['optical_center_x_in_pixels'], cp['optical_center_y_in_pixels'] )
            self.cam_deltas = np.array([0.1, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001 ])
            self.cam_names = ['Ry_0', 'Rz_0', 'Rx_-90', 'Rz_-90', 'dx', 'dy', 'dz']
            self.cam_vec = np.array([  1.8000  ,  1.7000  , -2.6000 ,   4.7500 ,   0.0620  ,  0.0320  , -0.0270  ])             

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
                       
        elif device == 'codyRobot' or device == 'dummyScanner': 
            #just for testing/demonstration without dependencies outside of gt-ros-pkg            
            self.webcam_id = 0
            
            #values from equilibrium_point_control/lpi.py
            self.cam_name = 'mekabotUTM' #also: 'dummyUTM'

            #Values copied from Cody
            #Please update with current values if they are expected to have changed.
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
                      'opencv_bayer_pattern' :  48, #same as cv.CV_BayerBG2BGR
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
            print '[configuration] ERROR: configuration.py: Device "%s" not recognized:' %( device )
            print 'Exiting.  Cannot fetch transformation and camera properties for this device.'
            
            
