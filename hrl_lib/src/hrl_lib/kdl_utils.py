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


import numpy as np, math

import roslib
roslib.load_manifest('hrl_lib')

import PyKDL as kdl


## kdl vector -> 3x1 np matrix.
def kdl_vec_to_np(kdl_vec):
    v = np.matrix([kdl_vec[0],kdl_vec[1],kdl_vec[2]]).T
    return v

## kdl rotation matrix -> 3x3 numpy matrix.
def kdl_rot_to_np(kdl_rotation):
    m = kdl_rotation
    rot = np.matrix([[m[0,0],m[0,1],m[0,2]],
                     [m[1,0],m[1,1],m[1,2]],
                     [m[2,0],m[2,1],m[2,2]]])
    return rot

## 3x1 np vector -> KDL Vector
def np_vec_to_kdl(p):
    return kdl.Vector(p[0,0],p[1,0],p[2,0])

## 3x3 np rotation matrix -> KDL Rotation.
def np_rot_to_kdl(rot):
    return kdl.Rotation(rot[0,0],rot[0,1],rot[0,2],rot[1,0],rot[1,1],rot[1,2],rot[2,0],rot[2,1],rot[2,2])



