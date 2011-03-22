#
# Copyright (c) 2009, Georgia Tech Research Corporation
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

#  \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
#  \author Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)

import math
import numpy as np

import roslib; roslib.load_manifest('tf') # only depending on tf for transformations.py
import tf.transformations as tft

def homogeneousToxyz(p):
    return p[0:3,:] / p[3,:]

def xyzToHomogenous(v, floating_vector=False):
    """ convert 3XN matrix, to 4XN matrix in homogeneous coords
    """
#   v = np.matrix(v)
#   v = v.reshape(3,1)
    if floating_vector == False:
        return np.row_stack((v, np.ones(v.shape[1])))
    else:
        return np.row_stack((v, np.zeros(v.shape[1])))

def xyToxyz(v):
    '''convert 2XN matrix, to 3XN matrix in homogeneous coords
    '''
    return np.row_stack((v, np.zeros(v.shape[1])))


def xyToHomogenous(v):
    """ convert 2XN matrix to 4XN homogeneous
    """
#   p = np.matrix(p)
#   p = p.reshape(2,1)
#   return np.row_stack((p, np.matrix([0.,1.]).T))
    return np.row_stack((v, np.zeros(v.shape[1]), np.ones(v.shape[1])))

def invertHomogeneousTransform(t):
    """ Inverts homogeneous transform
    """
    rot = t[0:3, 0:3]
    disp = t[0:3, 3]
    scale = t[3,3]
    rot = rot.transpose()
    disp = -1*rot*disp
    te = np.matrix([0.,0.,0.,1./scale])
    t_inv = np.row_stack((np.column_stack((rot, disp)),te))
    return t_inv

def getRotSubMat(t):
    """ returns rotation submatrix from homogeneous transformation t
    """
    return t[0:3, 0:3]

def getDispSubMat(t):
    """ returns displacement submatrix from homogeneous transformation t
    """
    return t[0:3, 3]

def composeHomogeneousTransform(rot, disp):
    """ Composes homogeneous transform from rotation and disp
    """
    disp = np.matrix(disp)
    disp = disp.reshape(3,1)

    te = np.matrix([0.,0.,0.,1.])
    t = np.row_stack((np.column_stack((rot, disp)),te))
    return t

def angle_within_mod180(angle):
    ''' angle in radians.
        returns angle within -pi and +pi
    '''
    ang_deg = math.degrees(angle)%360
    if ang_deg > 180:
        ang_deg = ang_deg-360
    elif ang_deg <= -180:
        ang_deg = ang_deg+360

    return math.radians(ang_deg)

## returns equivalent angle in 1st or 4th quadrant.
# @param angle - in RADIANS
def angle_within_plus_minus_90(angle):
    ang_deg = math.degrees(angle)%360
    if ang_deg<=90.:
        return math.radians(ang_deg)
    elif ang_deg<=180:
        return math.radians(ang_deg-180)
    elif ang_deg<=270:
        return math.radians(ang_deg-180)
    else:
        return math.radians(ang_deg-360)

def Rx(theta):
    """ returns Rotation matrix which transforms from XYZ -> frame rotated by theta about the x-axis.
        2 <--- Rx  <--- 1
            theta is in radians.
            Rx = rotX'
    """
    st = math.sin(theta)
    ct = math.cos(theta)
    return np.matrix([[ 1.,  0., 0. ],
                      [ 0.,  ct, st ],
                      [ 0., -st, ct ]])

def Ry(theta):
    """ returns Rotation matrix which transforms from XYZ -> frame rotated by theta about the y-axis.
            theta is in radians.
            Ry = rotY'
    """
    st = math.sin(theta)
    ct = math.cos(theta)
    return np.matrix([[ ct, 0., -st ],
                      [ 0., 1.,  0. ],
                      [ st, 0.,  ct ]])

def Rz(theta):
    """ returns Rotation matrix which transforms from XYZ -> frame rotated by theta about the z-axis.
            theta is in radians.
            Rz = rotZ'
    """
    st = math.sin(theta)
    ct = math.cos(theta)
    return np.matrix([[  ct, st, 0. ],
                      [ -st, ct, 0. ],
                      [  0., 0., 1. ]])

def rotX(theta):
    """ returns Rotation matrix such that R*v -> v', v' is rotated about x axis through theta.
            theta is in radians.
            rotX = Rx'
    """
    st = math.sin(theta)
    ct = math.cos(theta)
    return np.matrix([[ 1., 0.,  0. ],
                      [ 0., ct, -st ],
                      [ 0., st,  ct ]])

def rotY(theta):
    """ returns Rotation matrix such that R*v -> v', v' is rotated about y axis through theta_d.
            theta is in radians.
            rotY = Ry'
    """
    st = math.sin(theta)
    ct = math.cos(theta)
    return np.matrix([[  ct, 0., st ],
                      [  0., 1., 0. ],
                      [ -st, 0., ct ]])

def rotZ(theta):
    """ returns Rotation matrix such that R*v -> v', v' is rotated about z axis through theta_d.
            theta is in radians.
            rotZ = Rz'
    """
    st = math.sin(theta)
    ct = math.cos(theta)
    return np.matrix([[ ct, -st, 0. ],
                      [ st,  ct, 0. ],
                      [ 0.,  0., 1. ]])


#-----------------   wrappers around functions in tf.transformation.py   -----------------

##
# compute rotation matrix from axis and angle.
# Example:
# a1 = rot_angle_direction(math.radians(30), np.matrix([0.,1.,0.]).T)
# a2 = Ry(math.radians(30))
# np.allclose(a1.T, a2) # result is True.
# @param angle - angle in RADIANS
# @param direction - 3x1 np matrix
# @return 3x3 np matrix that rotates a vector about the axis passing
# through the origin, in the given direction through angle.
def rot_angle_direction(angle, direction):
    direction = direction/np.linalg.norm(direction)
    r = tft.rotation_matrix(angle, direction.A1)
    return np.matrix(r[0:3,0:3])

##
# convert a quaternion to a 3x3 rotation matrix.
# @param q - quaternion (tf/transformation.py) (x,y,z,w)
# @return 3x3 rotation matrix (np matrix)
def quaternion_to_matrix(q):
    arr = tft.quaternion_matrix(q)
    return np.matrix(arr[0:3, 0:3])


def matrix_to_quaternion(r):
    m = np.column_stack((r, np.matrix([0.,0.,0.]).T))
    m = np.row_stack((m, np.matrix([0.,0.,0.,1.])))
    return tft.quaternion_from_matrix(m)

##
# convert rotation matrix to axis and angle.
# @param rmat - 3x3 np matrix.
def matrix_to_axis_angle(rmat):
    rmat = np.column_stack((rmat, np.matrix([0,0,0]).T))
    rmat = np.row_stack((rmat, np.matrix([0,0,0,1])))
    ang, direc, point = tft.rotation_from_matrix(rmat)
#    print 'point:', point
    return ang, direc




