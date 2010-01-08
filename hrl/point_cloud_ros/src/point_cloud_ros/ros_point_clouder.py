#!/usr/bin/python
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

## Testing point_cloud_mapping from python
## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)


import roslib; roslib.load_manifest('point_cloud_ros')

import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

import numpy as np, math
from numpy import pi
import sys, os, optparse, time
import copy

import hrl_lib.util as ut
import point_cloud_ros.point_cloud_utils as pcu

from enthought.mayavi import mlab


def SphereToCart(rho, theta, phi):
    x = rho * np.sin(phi) * np.cos(theta)
    y = rho * np.sin(phi) * np.sin(theta)
    z = rho * np.cos(phi)
    return (x,y,z)

def generate_sphere():
    pts = 4e3
    theta = np.random.rand(pts) * 2*pi
    phi = np.random.rand(pts) * pi
    rho = 1*np.ones(len(theta))

    x,y,z = SphereToCart(rho,theta,phi)

    pts = np.matrix(np.row_stack((x,y,z)))
    return pcu.np_points_to_ros(pts)

def plot_cloud(pts):
    x = pts[0,:].A1
    y = pts[1,:].A1
    z = pts[2,:].A1
    mlab.points3d(x,y,z,mode='point')
    mlab.show()

def plot_normals(pts,normals,curvature=None):

    x = pts[0,:].A1
    y = pts[1,:].A1
    z = pts[2,:].A1

    u = normals[0,:].A1
    v = normals[1,:].A1
    w = normals[2,:].A1

    if curvature != None:
        #mlab.points3d(x,y,z,curvature,mode='point',scale_factor=1.0)
        mlab.points3d(x,y,z,curvature,mode='sphere',scale_factor=0.1,mask_points=1)
        mlab.colorbar()
    else:
        mlab.points3d(x,y,z,mode='point')
    mlab.quiver3d(x,y,z,u,v,w,mask_points=16,scale_factor=0.1)
#    mlab.axes()
    mlab.show()

def downsample_cb(cloud_down):
    print 'downsample_cb got called.'
    pts = ros_pts_to_np(cloud_down.pts)
    x = pts[0,:].A1
    y = pts[1,:].A1
    z = pts[2,:].A1
    mlab.points3d(x,y,z,mode='point')
    mlab.show()

def normals_cb(normals_cloud):
    print 'normals_cb got called.'
    d = {}
    t0 = time.time()
    pts = ros_pts_to_np(normals_cloud.pts)
    t1 = time.time()
    print 'time to go from ROS point cloud to np matrx:', t1-t0
    d['pts'] = pts
    
    if normals_cloud.chan[0].name != 'nx':
        print '################################################################################'
        print 'synthetic_point_clouds.normals_cloud: DANGER DANGER normals_cloud.chan[0] is NOT nx, it is:', normals_cloud.chan[0].name
        print 'Exiting...'
        print '################################################################################'
        sys.exit()
        
    normals_list = []
    for i in range(3):
        normals_list.append(normals_cloud.chan[i].vals)

    d['normals'] = np.matrix(normals_list)
    d['curvature'] = normals_cloud.chan[3].vals

    print 'd[\'pts\'].shape:', d['pts'].shape
    print 'd[\'normals\'].shape:', d['normals'].shape
    ut.save_pickle(d, 'normals_cloud_'+ut.formatted_time()+'.pkl')




if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('--sphere', action='store_true', dest='sphere',
                 help='sample a sphere and publish the point cloud')
    p.add_option('--plot', action='store_true', dest='plot',
                 help='plot the result')
    p.add_option('-f', action='store', type='string',dest='fname',
                 default=None, help='pkl file with the normals.')
    p.add_option('--pc', action='store', type='string',dest='pc_fname',
                 default=None, help='pkl file with 3xN numpy matrix (numpy point cloud).')

    opt, args = p.parse_args()
    sphere_flag = opt.sphere
    plot_flag = opt.plot
    fname = opt.fname
    pc_fname = opt.pc_fname
    

    if sphere_flag or pc_fname!=None:
        rospy.init_node('point_cloud_tester', anonymous=True)
        pub = rospy.Publisher("tilt_laser_cloud", PointCloud)
        rospy.Subscriber("cloud_normals", PointCloud, normals_cb)
        rospy.Subscriber("cloud_downsampled", PointCloud, downsample_cb)

        time.sleep(1)

        if sphere_flag:
            pc = generate_sphere()

        if pc_fname != None:
            pts = ut.load_pickle(pc_fname)
            print 'before np_points_to_ros'
            t0 = time.time()
            pc = pcu.np_points_to_ros(pts)
            t1 = time.time()
            print 'time to go from numpy to ros:', t1-t0

            t0 = time.time()
            pcu.ros_pointcloud_to_np(pc)
            t1 = time.time()
            print 'time to go from ros to numpy:', t1-t0
            

        pub.publish(pc)
        rospy.spin()


    if plot_flag:
        if fname == None:
            print 'Please give a pkl file for plotting (-f option)'
            print 'Exiting...'
            sys.exit()

        d = ut.load_pickle(fname)
        plot_normals(d['pts'],d['normals'],d['curvature'])


