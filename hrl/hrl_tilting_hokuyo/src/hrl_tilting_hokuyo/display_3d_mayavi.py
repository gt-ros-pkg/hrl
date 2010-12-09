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

import roslib; roslib.load_manifest('hrl_tilting_hokuyo')

from enthought.mayavi import mlab

import sys,os,time
import optparse
import hrl_lib.util as ut
import numpy as np, math


color_list = [(1.,1.,1.),(1.,0.,0.),(0.,1.,0.),(0.,0.,1.),(1.,1.,0.),(1.,0.,1.),\
              (0.,1.,1.),(0.5,1.,0.5),(1.,0.5,0.5)]



##
# make a figure with a white background.
def white_bg():
    mlab.figure(fgcolor = (0,0,0), bgcolor = (1,1,1))

##
# save plot as a png
# @param name - file name
# size - (r,c) e.g. (1024, 768)
def savefig(name, size):
    mlab.savefig(name, size=size)

## plot 3D points connected to each other
#
# Check mlab.points3d documentation for details.
# @param pts - 3xN numpy matrix of points.
# @param color - 3 tuple of color. (float b/w 0 and 1)
# @param mode - how to display the points ('point','sphere','cube' etc.)
# @param scale_fator - controls size of the spheres. not sure what it means.
def plot(pts,color=(1.,1.,1.), scalar_list=None):
    if scalar_list != None:
        mlab.plot3d(pts[0,:].A1,pts[1,:].A1,pts[2,:].A1,scalar_list,
                    representation = 'wireframe', tube_radius = None)
        mlab.colorbar()
    else:
        mlab.plot3d(pts[0,:].A1,pts[1,:].A1,pts[2,:].A1,color=color,
                    representation = 'wireframe', tube_radius = None)

## plot 3D points as a cloud.
#
# Check mlab.points3d documentation for details.
# @param pts - 3xN numpy matrix of points.
# @param color - 3 tuple of color. (float b/w 0 and 1)
# @param mode - how to display the points ('point','sphere','cube' etc.)
# @param scale_fator - controls size of the spheres. not sure what it means.
def plot_points(pts,color=(1.,1.,1.),mode='point',scale_factor=0.01,scalar_list=None):
    if scalar_list != None:
        mlab.points3d(pts[0,:].A1,pts[1,:].A1,pts[2,:].A1,scalar_list,mode=mode,scale_factor=scale_factor)
        mlab.colorbar()
    else:
        mlab.points3d(pts[0,:].A1,pts[1,:].A1,pts[2,:].A1,mode=mode,color=color,scale_factor=scale_factor)


## Use mayavi2 to plot normals, and curvature of a point cloud.
# @param pts - 3xN np matrix
# @param normals - 3xN np matrix of surface normals at the points in pts.
# @param curvature - list of curvatures.
# @param mask_points - how many point to skip while drawint the normals
# @param color - of the arrows
# @param scale_factor - modulate size of arrows.
#
# Surface normals are plotted as arrows at the pts, curvature is colormapped and
# shown as spheres. The radius of the sphere also indicates the magnitude
# of the curvature. If curvature is None then it is not plotted. The pts
# are then displayed as pixels.
def plot_normals(pts, normals, curvature=None, mask_points=1,
                 color=(0.,1.,0.), scale_factor = 0.1):
    x = pts[0,:].A1
    y = pts[1,:].A1
    z = pts[2,:].A1

    u = normals[0,:].A1
    v = normals[1,:].A1
    w = normals[2,:].A1

    if curvature != None:
        curvature = np.array(curvature)
        #idxs = np.where(curvature>0.03)
        #mlab.points3d(x[idxs],y[idxs],z[idxs],curvature[idxs],mode='sphere',scale_factor=0.1,mask_points=1)
        mlab.points3d(x,y,z,curvature,mode='sphere',scale_factor=0.1,mask_points=1, color=color)
#        mlab.points3d(x,y,z,mode='point')
        mlab.colorbar()
    else:
        mlab.points3d(x,y,z,mode='point')
    mlab.quiver3d(x, y, z, u, v, w, mask_points=mask_points,
                  scale_factor=scale_factor, color=color)
#    mlab.axes()

## Plot a yellow cuboid.
# cuboid is defined by 12 tuples of corners that define the 12 edges,
# as returned by occupancy_grig.grid_lines() function.
def plot_cuboid(corner_tups):
    for tup in corner_tups:
        p1 = tup[0]
        p2 = tup[1]
        mlab.plot3d([p1[0,0],p2[0,0]],[p1[1,0],p2[1,0]],
                    [p1[2,0],p2[2,0]],color=(1.,1.,0.),
                    representation='wireframe',tube_radius=None)

## show the plot.
# call this function after plotting everything.
def show():
    mlab.show()



if __name__ == '__main__':
    p = optparse.OptionParser()

    p.add_option('-c', action='store', type='string', dest='pts_pkl',
                 help='pkl file with 3D points')
    p.add_option('-f', action='store', type='string', dest='dict_pkl',
                 help='pkl file with 3D dict')
    p.add_option('--save_cloud', action='store_true', dest='save_cloud',
                 help='pickle the point cloud (3xN matrix)')
    p.add_option('--pan_angle', action='store', type='float',
                 dest='max_pan_angle', default=60.0,
                 help='angle in DEGREES. points b/w (-pan_angle and +pan_angle) are displayed. [default=60.]')
    p.add_option('--max_dist', action='store', type='float',
                 dest='max_dist', default=3.0,
                 help='maximum distance (meters). Points further than this are discarded. [default=3.]')

    opt, args = p.parse_args()

    pts_pkl_fname = opt.pts_pkl
    dict_pkl_fname = opt.dict_pkl
    save_cloud_flag = opt.save_cloud
    max_pan_angle = opt.max_pan_angle
    max_dist = opt.max_dist

    if pts_pkl_fname != None:
        pts = ut.load_pickle(pts_pkl_fname)
    elif dict_pkl_fname != None:
        import tilting_hokuyo.processing_3d as p3d
        dict = ut.load_pickle(dict_pkl_fname)
        pts = p3d.generate_pointcloud(dict['pos_list'],dict['scan_list'],
                                      math.radians(-max_pan_angle),
                                      math.radians(max_pan_angle),
                                      dict['l1'],dict['l2'],
                                      min_tilt=math.radians(-90),max_tilt=math.radians(90))
    else:
        print 'Specify either a pts pkl or a dict pkl (-c or -f)'
        print 'Exiting...'
        sys.exit()

    dist_mat = ut.norm(pts)
    idxs = np.where(dist_mat<max_dist)[1]
    print 'pts.shape', pts.shape
    pts = pts[:,idxs.A1]
    print 'pts.shape', pts.shape

    if save_cloud_flag:
        ut.save_pickle(pts,'numpy_pc_'+ut.formatted_time()+'.pkl')

    plot_points(pts)
    mlab.show()


