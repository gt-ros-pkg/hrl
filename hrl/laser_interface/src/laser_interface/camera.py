# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## @author Hai Nguyen/hai@gatech.edu
#import videre as vd
#import opencv as cv
#from opencv import highgui as hg
import cv
import numpy as np
import util as ut
import math as m
from pkg import *

###################################################################################################
# Cameras
#   The camera classes in this file are separated into two different types of
#   classes.  In the first group are hardware camera classes meant for
#   abstracting away hardware devices that provide a grid of pixels at every
#   time step.  The second group contain abstract mathematical camera classes
#   which satisfies the definition of a camera as a device that transform 3D
#   world points into 2D coordinates in a plane.
#
#   This was done so that different hardware camera classes can be swapped in
#   and out without requiring that the mathematical abstract camera be changed.
#   For example, to work off of a video recorded from disk the geometric camera class
#   can remain the same while the hardware camera class is swapped for one that
#   provide images from a file.
###################################################################################################

###################################################################################################
# Geometric reasoning functions 
###################################################################################################
def homo_transform3d(R, t):
    T = np.matrix(np.zeros((4,4)))
    T[0:3, 0:3] = R
    T[0:3, 3]   = t
    T[3,3]      = 1.0
    return T

#Rotates the coordinate frame counterclockwise
def Rx(a):
    return np.matrix([[1,  0,        0,],
                      [0,  m.cos(a),  m.sin(a)],
                      [0, -m.sin(a),  m.cos(a)]])

#Rotates the coordinate frame counterclockwise
def Ry(a):
    return np.matrix([[m.cos(a), 0, -m.sin(a)],
                      [0,       1,       0],
                      [m.sin(a), 0,  m.cos(a)]])

#Rotates the coordinate frame counterclockwise
def Rz(a):
    return np.matrix([[ m.cos(a), m.sin(a), 0],
                      [-m.sin(a), m.cos(a), 0],
                      [ 0,       0,       1]])

def rotation(rx,ry,rz):
    return Rx(rx) * Ry(ry) * Rz(rz)

def points_on_line(homogeneous2d_line, img_height):
    a = homogeneous2d_line[0,0]
    b = homogeneous2d_line[1,0]
    c = homogeneous2d_line[2,0]
    if b == 0:
        x = -c/a
        return [np.matrix([x, 0, 1]).T, 
                np.matrix([x, img_height, 1])]
    else:
        return [np.matrix([-c/a, 0, 1]), 
                np.matrix([(-b/a) * img_height - (c/a), img_height, 1])]

def tuple_to_homo(p):
    return np.matrix([p[0], p[1], 1]).T

def cvpoint_to_homo(p):
    return np.matrix([p.x, p.y, 1]).T

def homo_to_cvpoint(p):
    p = p / p[-1,0]
    #return cv.Point(int(round(p[0,0])), int(round(p[1,0])))
    return (int(round(p[0,0])), int(round(p[1,0])))

def rodrigues(vec):
    cvvec = ut.numpymat2cvmat(vec)
    dst   = cv.CreateMat(3, 3, cv.CV_32FC1)
    cv.Rodrigues2(cvvec, dst)
    return ut.cvmat2numpymat(dst).T

def normalize_homo_point(point):
    point = point / point[-1,0]
    return point

def homo_to_point(homo_point):
    p = normalize_homo_point(homo_point)
    return p[0:-1,:]

def point_to_homo(point):
    return np.concatenate((point, 1.0+np.zeros((1,point.shape[1]))), axis=0)

def general_projection_matrix():
    P = np.matrix(np.zeros((3,4)))
    P[0:3, 0:3] = np.eye(3)
    return P

###################################################################################################
# Hardware Cameras 
###################################################################################################
#class VidereStereo:
#    """
#       Videre stereo cam iterator version
#    """
#    def __init__(self, device_num = 0, gain=96, exposure=10):
#        self.camera = vd.VidereCap(device_num)
#        self.camera.set_mode(vd.NONE)
#        vd.set_gain(self.camera.capture, gain)
#        vd.set_exposure(self.camera.capture, exposure)
#
#    def next(self):
#        self.camera.process_frames()
#        return (self.camera.left_debayered, self.camera.right_debayered)
#
#    def __iter__(self):
#        return self

class StereoFile:
    def __init__(self, left_file, right_file):
        self.left  = cv.CreateFileCapture(left_file);
        self.right = cv.CreateFileCapture(right_file);
        #self.total_frames = hg.cvGetCaptureProperty(self.left, hg.CV_CAP_PROP_FRAME_COUNT)

    def next(self):
        tup = (cv.QueryFrame(self.left), cv.QueryFrame(self.right))
        #frame_numb = hg.cvGetCaptureProperty(self.left, hg.CV_CAP_PROP_POS_FRAMES )

        #if tup[0] == None or tup[1] == None or frame_numb >= self.total_frames:
        if tup[0] == None or tup[1] == None:
            raise StopIteration()
        return tup

    def __iter__(self):
        return self

#def single_eye(eye, device=0, gain=96, exposure=150):
#    vs = VidereStereo(device_num=device, gain=gain, exposure=exposure)
#    for l,r in vs:
#        if eye == 'left':
#            yield l
#        else:
#            yield r

def video_file(file):
     cap = cv.CreateFileCapture(file);
     f = cv.QueryFrame(cap)

     while f != None:
         yield f
         f = cv.QueryFrame(cap)

def strip_extension(filename):
    a = filename.split('.')
    return a[0]

def file_sequence(files, loader, extension=None):
    for f in files:
        if extension is not None:
            name = strip_extension(f)
            new_name = name + extension
            #print "file_sequence: loading ", new_name
            yield loader(new_name)
        else:
            #print "file_sequence: loading ", f
            yield loader(f)

###################################################################################################
# Geometric Cameras
###################################################################################################
class StereoCamera:
    def __init__(self, camera_left, camera_right, R, T):
        self.camera_left  = camera_left
        self.camera_right = camera_right 
        self.R = R
        self.T = T

    ##
    # @param x 
    # @param xprime 2x1 matrices
    def triangulate_3d(self, x, xprime):
        '''
        '''
        Klp = np.linalg.inv(self.camera_left.intrinsic_mat)
        Krp = np.linalg.inv(self.camera_right.intrinsic_mat)
    
        w1    = Klp * point_to_homo(x)
        w2    = Krp * point_to_homo(xprime)
    
        A     = np.concatenate((w1, -self.R.T * w2), axis=1)
        b     = self.T
    
        x_hat          = np.linalg.inv(A.T * A) * A.T * b
        left_estimate  = x_hat[0,0] * w1
        right_estimate = x_hat[1,0] * self.R * w2 + self.T
    
        p = (left_estimate + right_estimate)/2.0
        return {'point': p, 'error':np.linalg.norm(left_estimate- right_estimate)}


class ProjectiveCamera:

    ## 
    #  Format for parameters:
    #       focal_length_pixels        = (720.511045, 724.498871),
    #       optical_center_pixels      = (324.277843, 236.260833),
    #       lens_distortion_radial     = (-0.428665, 0.300025, -0.230655),
    #  
    def __init__(self, calibration_image_size, focal_length_pixels,
                  optical_center_pixels, lens_distortion_radial):
        self.calibration_image_size     =  calibration_image_size     
        self.focal_length_pixels        =  focal_length_pixels           
        self.optical_center_pixels      =  optical_center_pixels        
        self.lens_distortion_radial     =  lens_distortion_radial       
        self.intrinsic_mat        =    np.array([[focal_length_pixels[0], 0,                        optical_center_pixels[0]],
 			                                     [0,                        focal_length_pixels[1], optical_center_pixels[1]],
			                                     [0,                        0,                      1]]) 
        self.inv_intrinsic_mat    = np.linalg.inv(np.matrix(self.intrinsic_mat)) 

        self.intrinsic_mat_cv              = ut.numpymat2cvmat(self.intrinsic_mat) 
        self.inv_intrinsic_mat_cv          = ut.numpymat2cvmat(self.inv_intrinsic_mat)
        self.lens_distortion_radial_cv     = cv.CreateMat(4, 1, cv.CV_32FC1)
        self.lens_distortion_radial_cv[0,0] = lens_distortion_radial[0]
        self.lens_distortion_radial_cv[1,0] = lens_distortion_radial[1]
        self.lens_distortion_radial_cv[2,0] = 0
        #self.lens_distortion_radial_cv[2,0] = lens_distortion_radial[2]
        self.lens_distortion_radial_cv[3,0] = 0
        self.temp = None

        self.mapx = cv.CreateMat(int(calibration_image_size[1]), int(calibration_image_size[0]), cv.CV_32FC1)
        self.mapy = cv.CreateMat(int(calibration_image_size[1]), int(calibration_image_size[0]), cv.CV_32FC1)
        cv.InitUndistortMap(self.intrinsic_mat_cv, self.lens_distortion_radial_cv, self.mapx, self.mapy)


    def camera_matrix(self, g_T_c = homo_transform3d(R = np.eye(3), t = np.zeros((3,1)))):
        '''
            g_T_c - takes points in this camera's frame to global coordinate
        '''
        c_T_g = np.linalg.inv(g_T_c)
        #print 'c_T_g', c_T_g
        P     = general_projection_matrix()
        K     = np.matrix(self.intrinsic_mat)

        C     = K * P * c_T_g
        return C

    def undistort_img(self, image):
        if self.temp is None:
            self.temp = cv.CreateImage(cv.GetSize(image), 8, 3)
        #cv.cvUndistort2(image, self.temp, self.intrinsic_mat_cv, self.lens_distortion_radial_cv)
        cv.Remap(image, self.temp, self.mapx, self.mapy)
        return self.temp

class ROSProjectiveCamera(ProjectiveCamera):
    def __init__(self, name):
        master = rospy.getMaster()
        prefix = '/camera_models/' + name + '/'
        ProjectiveCamera.__init__(self, 
                (master[prefix + 'width'],    master[prefix + 'height']),
                (master[prefix + 'focal_x'],  master[prefix + 'focal_y']),
                (master[prefix + 'center_x'], master[prefix + 'center_y']),
                (master[prefix + 'radial_1'], master[prefix + 'radial_2']))

class ROSStereoCamera(StereoCamera):
    def __init__(self, name):
        master = rospy.getMaster()
        prefix = '/camera_models/' + name + '/'
        T      = np.matrix([master[prefix + 'translation_x'], master[prefix + 'translation_y'], master[prefix + 'translation_z']]).T
        R      = np.matrix([[master[prefix + 'rotation_0_0'], master[prefix + 'rotation_0_1'], master[prefix + 'rotation_0_2']],
                            [master[prefix + 'rotation_1_0'], master[prefix + 'rotation_1_1'], master[prefix + 'rotation_1_2']],
                            [master[prefix + 'rotation_2_0'], master[prefix + 'rotation_2_1'], master[prefix + 'rotation_2_2']]])
        left_cam  = master[prefix + 'left_cam']
        right_cam = master[prefix + 'right_cam'] 
        StereoCamera.__init__(self, ROSProjectiveCamera(left_cam), ROSProjectiveCamera(right_cam), R, T)

###################################################################################################
# HDR experiments
###################################################################################################
#class VidereHDR:
#    def __init__(self, device_num = 0):
#        self.camera = vd.VidereCap(device_num)
#        #self.camera.process_frames()
#        self.camera.set_mode(vd.NONE)
#        vd.set_gain(self.camera.capture, 96)
#        vd.set_exposure(self.camera.capture, 255)
#        self.camera.process_frames()
#        self.low_exposure_left  = cv.cvCreateImage(cv.cvGetSize(self.camera.left_debayered), 8, 3)
#        self.low_exposure_right = cv.cvCreateImage(cv.cvGetSize(self.camera.right_debayered), 8, 3)
#
#    def next(self):
#        vd.set_exposure(self.camera.capture, 55)
#        frames_wait = 3
#        for i in range(frames_wait):
#            self.camera.process_frames()
#        cv.cvCopy(self.camera.left_debayered, self.low_exposure_left)
#        cv.cvCopy(self.camera.right_debayered, self.low_exposure_right)
#
#        vd.set_exposure(self.camera.capture, 255)
#        for i in range(frames_wait):
#            self.camera.process_frames()
#        return (self.low_exposure_left, self.low_exposure_right, self.camera.left_debayered, self.camera.right_debayered)
#        #return (self.camera.left_debayered, self.camera.right_debayered)
#
#    def __iter__(self):
#        return self

###################################################################################################
# Camera calibrations 
###################################################################################################
KNOWN_CAMERAS = {}


if __name__ == "__main__":
    test = "test_gain"

    if 'test_hdr' == test:
        cv.NamedWindow("left_eye",  1)
        cv.NamedWindow("right_eye", 1)

        cv.NamedWindow("left_eye_low",  1)
        cv.NamedWindow("right_eye_low", 1)

        cam = VidereHDR(0)
        base_exposure = 100

        for ll, lr, l, r in cam:
            cv.ShowImage("left_eye_low", ll)
            cv.ShowImage("right_eye_low", lr)
            cv.ShowImage("left_eye", l)
            cv.ShowImage("right_eye", r)
            key = cv.WaitKey(10)

            if   'R' == key: #up
                base_exposure = base_exposure + 1
                vd.set_exposure(cam.camera.capture, base_exposure)
                print base_exposure
            elif 'T' == key:
                base_exposure = base_exposure - 1
                vd.set_exposure(cam.camera.capture, base_exposure)
                print base_exposure





































































































#KNOWN_CAMERAS['videre_left']    = ProjectiveCamera(calibration_image_size        = (640.0, 480.0), 
#                                                      focal_length_pixels        = (945.598886, 950.594644),
#                                                      optical_center_pixels      = (332.544501, 254.563215),
#                                                      lens_distortion_radial     = (-0.454865, 1.196744, -7.878979))
#KNOWN_CAMERAS['videre_right']   = ProjectiveCamera(calibration_image_size        = (640.0, 480.0), 
#                                                      focal_length_pixels        = (927.703322, 931.820423),
#                                                      optical_center_pixels      = (339.749173, 234.137828),
#                                                      lens_distortion_radial     = (-0.375167, -0.607004, 3.778784))
#KNOWN_CAMERAS['videre_stereo']  = StereoCamera(KNOWN_CAMERAS['videre_left'], KNOWN_CAMERAS['videre_right'], 
#                                     R = rodrigues(np.matrix([-0.009049, -0.003935, -0.000145])),
#                                     T = np.matrix([59.934176, 0.281676, -1.578173]).T/1000.0)
#
#
#KNOWN_CAMERAS['videre_left2']   = ProjectiveCamera(calibration_image_size       = (640.0, 480.0), 
#                                                   focal_length_pixels          = (962.472, 962.961),
#                                                   optical_center_pixels        = (312.410, 242.114),
#                                                   lens_distortion_radial       = (-0.441686, 0.302547, 0))
#KNOWN_CAMERAS['videre_right2']  = ProjectiveCamera(calibration_image_size       = (640.0, 480.0), 
#                                                     focal_length_pixels        = (947.233, 946.236),
#                                                     optical_center_pixels      = (337.313, 223.764),
#                                                     lens_distortion_radial     = (-0.419809, 0.246824, 0))
#KNOWN_CAMERAS['videre_stereo2'] = StereoCamera(KNOWN_CAMERAS['videre_left2'], KNOWN_CAMERAS['videre_right2'], 
#                                                R = np.matrix([[ 0.999802, 0.000869400, -0.0198607],
#                                                               [-0.000745101, 0.999980, 0.00626513],
#                                                               [0.0198657, -0.00624909, 0.999783]]),
#                                                T = np.matrix([60.6940, -0.401150, -2.25697]).T/1000.0)
#
#
#KNOWN_CAMERAS['f_left']         = ProjectiveCamera(calibration_image_size        = (640.0, 480.0), 
#                                                           focal_length_pixels          = (943.421, 952.346),
#                                                           optical_center_pixels      = (324.277843, 236.260833),
#                                                           lens_distortion_radial     = (-0.428665, 0.300025, -0.230655))
#KNOWN_CAMERAS['f_right']        = ProjectiveCamera(calibration_image_size        = (640.0, 480.0), 
#                                                           focal_length_pixels        = (947.324, 952.822),
#                                                           optical_center_pixels      = (344.965592, 255.324928),
#                                                           lens_distortion_radial     = (-0.381364, -0.029894, 0.450772))
#KNOWN_CAMERAS['f_stereo']       = StereoCamera(KNOWN_CAMERAS['f_left'], KNOWN_CAMERAS['f_right'], 
#                                          R = rodrigues(np.matrix([0,0,0.0])),
#                                          T = np.matrix([59.934176, 0.281676, -1.578173]).T/1000.0)
#
#
#KNOWN_CAMERAS['perfect-left']   = ProjectiveCamera(calibration_image_size        = (640.0, 480.0), 
#                                                      focal_length_pixels        = (720, 720),
#                                                      optical_center_pixels      = (320, 240),
#                                                      lens_distortion_radial     = (0.0, 0, 0))
#KNOWN_CAMERAS['perfect-right']  = ProjectiveCamera(calibration_image_size        = (640.0, 480.0), 
#                                                     focal_length_pixels        = (720, 720),
#                                                     optical_center_pixels      = (320, 240),
#                                                     lens_distortion_radial     = (0.0, 0, 0))
#KNOWN_CAMERAS['perfect-stereo'] = StereoCamera(KNOWN_CAMERAS['perfect-left'], KNOWN_CAMERAS['perfect-right'], 
#                                    R = rodrigues(np.matrix([0,0,0.0])),
#                                    T = np.matrix([60, 0, 0.0]).T/1000.0)



    #def undistort(self, homogeneous_point):
    #    homogeneous_point = homogeneous_point / homogeneous_point[-1,0]
    #    print '-------------------------------------------------'
    #    print 'homogeneous_point', homogeneous_point

    #    kappa1 = self.lens_distortion_radial[0]
    #    kappa2 = self.lens_distortion_radial[1]
    #    kappa3 = self.lens_distortion_radial[2]
    #    print 'kappas', kappa1, kappa2, kappa3

    #    xd = (homogeneous_point[0,0] - self.optical_center_pixels[0]) / self.focal_length_pixels[0]
    #    yd = (homogeneous_point[1,0] - self.optical_center_pixels[1]) / self.focal_length_pixels[1]
    #    print 'xd, yd', xd.T, yd.T

    #    r = np.power(np.power(xd,2) + np.power(yd,2), 0.5)
    #    denom = 1 + kappa1 * np.power(r,2) + kappa2 * np.power(r,4) + kappa3 * np.power(r,6)
    #    print 'denom'
    #    xu = ((xd / denom) * self.focal_length_pixels[0]) + self.optical_center_pixels[0]
    #    yu = ((yd / denom) * self.focal_length_pixels[1]) + self.optical_center_pixels[1]
    #    print 'xu, yu', xu.T, yu.T
    #    return np.matrix([xu, yu, 1]).T

    #def triangulate_3d(self, left_coordinate, right_coordinate):
    #    """ left_vector, right_coordinate -> cvPoint
    #        returns -> (3X1) vector in stereohead coordinate frame, error.
    #                   stereohead coord frame is as defined in transforms.py
    #        code copied and modified slightly from self.get_3d_coord
    #    """
    #    left_point  = ut.numpymat2cvmat(left_coordinate)
    #    right_point = ut.numpymat2cvmat(right_coordinate)

    #    right_vector  = cv.cvCreateMat(3,1,cv.CV_32F)
    #    left_vector   = cv.cvCreateMat(3,1,cv.CV_32F)
    #    output_vector = cv.cvCreateMat(3,1,cv.CV_32F)

    #    cv.cvGEMM( self.camera_left.inv_intrinsic_mat_cv, left_point, 1, None, 1, left_vector, 0 )   
    #    cv.cvGEMM( self.camera_right.inv_intrinsic_mat_cv, right_point, 1, None, 1,  right_vector, 0 )

    #    lv = np.matrix([left_vector[0], left_vector[1], left_vector[2]]).T
    #    rv = np.matrix([right_vector[0], right_vector[1], right_vector[2]]).T

    #    te = self.R * lv

    #    a = cv.cvCreateMat( 3, 2, cv.CV_32F )
    #    a[0,0], a[1,0], a[2,0] = te[0,0], te[1,0], te[2,0]
    #    a[0,1], a[1,1], a[2,1] = rv[0,0], rv[1,0], rv[2,0]

    #    params = cv.cvCreateMat( 2, 1, cv.CV_32F )

    #    #linear least squares [X1 -X2]*[al;bet]     - trans ~ 0
    #    #				      [left right]*[params] -translation ~0
    #    translation = cv.cvCreateMat(3,1,cv.CV_32F)
    #    translation[0] = -self.T[0,0]
    #    translation[1] = -self.T[1,0]
    #    translation[2] = -self.T[2,0]

    #    cv.cvSolve( a, translation, params , cv.CV_SVD )
    #    alpha = params[0]
    #    beta = -params[1]

    #    vec_l = alpha * te + self.T
    #    vec_r = beta * rv

    #    #compute reprojection error
    #    detection_error = np.linalg.norm(vec_r-vec_l)

    #    p = (vec_l + vec_r)/2.

    #    return {'point': p, 'error':detection_error}












        #  
        #  [right camera]
        #  pwidth 640 
        #  pheight 480 
        #  dpx 0.006000 
        #  dpy 0.006000 
        #  sx 1.000000 
        #  Cx 339.749173 
        #  Cy 234.137828 
        #  f  927.703322 
        #  fy 931.820423 
        #  alpha 0.000000 
        #  kappa1 -0.375167 
        #  kappa2 -0.607004 
        #  kappa3 3.778784 
        #  tau1 0.000000 
        #  tau2 0.000000 
        #  proj 
        #    9.510000e+02 0.000000e+00 3.112438e+02 -5.699740e+04 
        #    0.000000e+00 9.510000e+02 2.444868e+02 0.000000e+00 
        #    0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 
        #  rect 
        #    9.996422e-01 -4.699764e-03 2.633178e-02 
        #    4.818754e-03 9.999785e-01 -4.457230e-03 
        #    -2.631026e-02 4.582522e-03 9.996433e-01 



        #  [left camera]
        #  pwidth 640 
        #  pheight 480 
        #  dpx 0.006000 
        #  dpy 0.006000 
        #  sx 1.000000 
        #  f 945.598886 
        #  fy 950.594644 
        #  alpha 0.000000 
        #  kappa1 -0.454865 
        #  kappa2 1.196744 
        #  kappa3 -7.878979 
        #  tau1 0.000000 
        #  tau2 0.000000 
        #  proj 
        #    9.510000e+02 0.000000e+00 3.112438e+02 0.000000e+00 
        #    0.000000e+00 9.510000e+02 2.444868e+02 0.000000e+00 
        #    0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 
        #  rect 
        #    9.997387e-01 -4.774891e-03 2.235500e-02 
        #    4.673785e-03 9.999786e-01 4.572824e-03 
        #    -2.237635e-02 -4.467146e-03 9.997396e-01 
