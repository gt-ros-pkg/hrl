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

''' Coordinate frames for testing ROS-interface (copied from Cody).
'''

import numpy as np, math
import copy
import hrl_lib.transforms as tr

# dictionary for transforming different coorsinate frames to global coord frame
# global is NOT the world or fixed frame, its just a convenient global frame
_globalT = {
    'torso' : None,
    'thok0' : None,
    'utm0' : None,
    'utmcam0': None,
    'mecanum': None
}

def create_globalTDict():
    """ call the create functions for all the coord frames
    """
    createTorsoTransform()
    createThok0Transform()
    createUtm0Transform()
    createMecanumTransform()

def createTorsoTransform():
    ''' torso frame -> global frame
    '''
    disp = np.matrix([0.,0.,0.]).T
    rot = np.matrix(np.eye(3))
    t = tr.composeHomogeneousTransform(rot,disp)
    _globalT['torso'] = t

def createThok0Transform():
    ''' thok0 frame -> global frame
    '''
    disp = np.matrix([0.,0.,0.09]).T
    rot = np.matrix(np.eye(3))
    t = tr.composeHomogeneousTransform(rot,disp)
    _globalT['thok0'] = t

def createUtm0Transform():
    ''' utm0 frame -> global frame
    '''
    disp = copy.copy(tr.getDispSubMat(_globalT['thok0']))
    disp[2,0] += 0.055
    rot = np.matrix(np.eye(3))
    t = tr.composeHomogeneousTransform(rot,disp)
    _globalT['utm0'] = t

def createMecanumTransform():
    ''' mecanum frame -> global frame (ignores the zenither)
    '''
    disp = np.matrix([-0.25,0.,0.0]).T
    rot = np.matrix(np.eye(3))
    t = tr.composeHomogeneousTransform(rot,disp)
    _globalT['mecanum'] = t

create_globalTDict()


def globalTmecanum(p,floating_vector=False):
    ''' 3x1 vector from mecanum to global.
    '''
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_gl = _globalT['mecanum'] * p_hom
    if floating_vector == False:
        return p_gl[0:3]/p_gl[3]
    else:
        return p_gl[0:3]

def mecanumTglobal(p,floating_vector=False):
    ''' 3x1 vector from global to mecanum.
    '''
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_gl = tr.invertHomogeneousTransform(_globalT['mecanum']) * p_hom
    if floating_vector == False:
        return p_gl[0:3]/p_gl[3]
    else:
        return p_gl[0:3]

def globalTtorso(p,floating_vector=False):
    ''' 3x1 vector from torso to global.
    '''
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_gl = _globalT['torso'] * p_hom
    if floating_vector == False:
        return p_gl[0:3]/p_gl[3]
    else:
        return p_gl[0:3]

def torsoTglobal(p,floating_vector=False):
    ''' 3x1 vector from global to torso.
    '''
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_gl = tr.invertHomogeneousTransform(_globalT['torso']) * p_hom
    if floating_vector == False:
        return p_gl[0:3]/p_gl[3]
    else:
        return p_gl[0:3]

def globalTthok0(p,floating_vector=False):
    ''' 3x1 vector from thok0 to global.
    '''
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_gl = _globalT['thok0'] * p_hom
    if floating_vector == False:
        return p_gl[0:3]/p_gl[3]
    else:
        return p_gl[0:3]

def thok0Tglobal(p,floating_vector=False):
    ''' 3x1 vector from global to thok0.
    '''
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_gl = tr.invertHomogeneousTransform(_globalT['thok0']) * p_hom
    if floating_vector == False:
        return p_gl[0:3]/p_gl[3]
    else:
        return p_gl[0:3]

def globalTutm0(p,floating_vector=False):
    ''' 3x1 vector from utm0 to global.
    '''
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_gl = _globalT['utm0'] * p_hom
    if floating_vector == False:
        return p_gl[0:3]/p_gl[3]
    else:
        return p_gl[0:3]

def utm0Tglobal(p,floating_vector=False):
    ''' 3x1 vector from global to utm0.
    '''
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_gl = tr.invertHomogeneousTransform(_globalT['utm0']) * p_hom
    if floating_vector == False:
        return p_gl[0:3]/p_gl[3]
    else:
        return p_gl[0:3]

## transformation matrix to go from global to utmcam0 coord frame.
# @param ang - servo angle (in RADIANS)
# @return 4x4 transformation matrix.
def utmcam0Tglobal_mat(ang):
    thok0Tglobal_mat = tr.invertHomogeneousTransform(_globalT['thok0'])
    # servo angle.
    disp = np.matrix([0.,0.,0.]).T
    tmat = tr.composeHomogeneousTransform(tr.Ry(ang),disp)*thok0Tglobal_mat

    # cameraTlaser from thok_cam_calib.py
    x = 0.012
    y = -0.056
    z = 0.035
    r1 = 0.
    r2 = 0.
    r3 = -0.7
    disp = np.matrix([-x,-y,-z]).T
    r = tr.Rz(math.radians(-90))*tr.Ry(math.radians(90.))
    disp = r*disp
    r = r*tr.Rx(math.radians(r1))
    r = r*tr.Ry(math.radians(r2))
    r = r*tr.Rz(math.radians(r3))

    t = tr.composeHomogeneousTransform(r, disp)
    tmat = t*tmat
    return tmat

## global to utmcam0 coord frame.
# @param p - 3xN np matrix.
# @param ang - servo angle (in RADIANS)
# @param floating_vector - interpretation of p. False -> position vector. True -> floating vector (rotation only).
# @return 3xN np matrix in the new coord frame.
def utmcam0Tglobal(p,ang,floating_vector=False):
    t = utmcam0Tglobal_mat(ang)
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_c = t * p_hom
    if floating_vector == False:
        pt = p_c[0:3]/p_c[3]
    else:
        pt = p_c[0:3]

    return pt

## utmcam0 coord frame to global
# @param p - 3xN np matrix.
# @param ang - servo angle (in RADIANS)
# @param floating_vector - interpretation of p. False -> position vector. True -> floating vector (rotation only).
# @return 3xN np matrix in the new coord frame.
def globalTutmcam0(p,ang,floating_vector=False):
    t = utmcam0Tglobal_mat(ang)
    t = tr.invertHomogeneousTransform(t)
    p_hom = tr.xyzToHomogenous(p, floating_vector)
    p_c = t * p_hom
    if floating_vector == False:
        pt = p_c[0:3]/p_c[3]
    else:
        pt = p_c[0:3]

    return pt

