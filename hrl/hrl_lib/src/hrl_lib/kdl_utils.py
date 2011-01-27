
import numpy as np, math

import roslib
roslib.load_manifest('hrl_lib')

import rospy
import PyKDL as kdl


## kdl vector -> 3x1 np matrix.
def kdl_vec_to_np(kdl_vec):
    v = np.matrix([kdl_vec[0],kdl_vec[1],kdl_vec[2]]).T
    return v

## kdl rotation matrix -> 3x3 numpy matrix.
def kdl_rot_to_np(kdl_rotation):
    m = kdl_rotation
    rot = np.matrix([[m[0,0],m[1,0],m[2,0]],
                     [m[0,1],m[1,1],m[2,1]],
                     [m[0,2],m[1,2],m[2,2]]])
    return rot

## 3x1 np vector -> KDL Vector
def np_vec_to_kdl(p):
    return kdl.Vector(p[0,0],p[1,0],p[2,0])

## 3x3 np rotation matrix -> KDL Rotation.
def np_rot_to_kdl(rot):
    return kdl.Rotation(rot[0,0],rot[1,0],rot[2,0],rot[0,1],rot[1,1],rot[2,1],rot[0,2],rot[1,2],rot[2,2])



