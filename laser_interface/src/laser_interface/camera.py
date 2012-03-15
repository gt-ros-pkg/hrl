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
from pkg import *

import cv
import numpy as np
from sensor_msgs.msg import CameraInfo

import util as ut
import math as m
import time


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
class ROSStereoListener:
    def __init__(self, topics, rate=30.0, name='stereo_listener'):
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge, CvBridgeError
        import hrl_lib.rutils as ru
        self.listener = ru.GenericListener(name, [Image, Image], topics, rate)
        self.lbridge = CvBridge()
        self.rbridge = CvBridge()

    def next(self):
        #print 'ros_stereo_listener'
        lros, rros =  self.listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)
        lcv = self.lbridge.imgmsg_to_cv(lros, 'bgr8')
        rcv = self.rbridge.imgmsg_to_cv(rros, 'bgr8')
        return lcv, rcv


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

###################################################################################################
# Geometric Cameras
###################################################################################################
# StereoCamera(ROSCameraCalibration('/wide_stereo/left/image_rect_color'), ROSCameraCalibration('/wide_stereo/right/image_rect_color'))
class ROSStereoCalibration:
    def __init__(self, left_chan, right_chan):
        self.left  = ROSCameraCalibration(left_chan)
        self.right = ROSCameraCalibration(right_chan)
        while not self.right.has_msg:
            time.sleep(.2)

        self.R = np.matrix(np.eye(3))                  #Using rectified images, no R needed.
        self.T = self.right.P[:, 3].copy()             #Copy so that we don't modify P by accident
        self.T[0,0] = -self.T[0,0] / self.right.P[0,0] #Adjust for scaling term in projection matrix

    ##
    # @param x 
    # @param xright 2x1 matrices
    def triangulate_3d(self, xleft, xright):
        #Klp = np.linalg.inv(self.left.K)
        #Krp = np.linalg.inv(self.right.K)
        # Use the K embedded in P instead K as 
        # rectified images are the result of P
        Klp = np.linalg.inv(self.left.P[0:3, 0:3]) 
        Krp = np.linalg.inv(self.right.P[0:3, 0:3])
    
        wleft    = Klp * point_to_homo(xleft)
        wright    = Krp * point_to_homo(xright)

        print 'ROSStereoCalibration: xleft', wleft.T
        print 'ROSStereoCalibration: xright', wright.T

        A     = np.concatenate((wleft, -self.R * wright), axis=1)
        b     = self.T
    
        x_hat          = np.linalg.inv(A.T * A) * A.T * b
        left_estimate  = x_hat[0,0] * wleft
        right_estimate = x_hat[1,0] * self.R * wright + self.T
        print 'ROSStereoCalibration: alpha, beta', x_hat
        print 'ROSStereoCalibration: left est', left_estimate.T
        print 'ROSStereoCalibration: right est', right_estimate.T
        print 'A * x_hat', (A * x_hat).T
        print 'T', b.T
        print 'A*x-b', np.linalg.norm((A*x_hat) - b)
    
        p = (left_estimate + right_estimate)/2.0
        print 'ROSStereoCalibration: p', p.T
        return {'point': p, 'error':np.linalg.norm(left_estimate- right_estimate)}


class ROSCameraCalibration:
    def __init__(self, channel):
        rospy.Subscriber(channel, CameraInfo, self.camera_info)
        self.has_msg = False

    def camera_info(self, msg):
        self.distortion = np.matrix(msg.D)
        self.K = np.reshape(np.matrix(msg.K), (3,3))
        self.R = np.reshape(np.matrix(msg.R), (3,3))
        self.P = np.reshape(np.matrix(msg.P), (3,4))
        self.w = msg.width
        self.h = msg.height
        self.has_msg = True

    def project(self, p):
        return self.P * p



if __name__ == "__main__":
    KNOWN_CAMERAS = {}
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



