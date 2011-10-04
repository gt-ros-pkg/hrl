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

from enthought.mayavi import mlab
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
# Check mlab.points3d documentation for details.
# @param pts - 3xN numpy matrix of points.
# @param color - 3 tuple of color. (float b/w 0 and 1)
# @param mode - how to display the points ('point','sphere','cube' etc.)
# @param scale_fator - controls size of the spheres. not sure what it means.
def plot_points(pts, color=(1.,1.,1.), scale_factor=0.02, mode='point', scalar_list=None):
    if scalar_list != None:
        mlab.points3d(pts[0,:].A1, pts[1,:].A1, pts[2,:].A1, scalar_list,mode=mode, scale_factor=scale_factor)
        mlab.colorbar()
    else:
        mlab.points3d(pts[0,:].A1, pts[1,:].A1, pts[2,:].A1, mode=mode, color=color, scale_factor=scale_factor)


## plot points and arrows.
# @param pts - 3xN np matrix
# @param vecs - 3xN np matrix of arrow vectors
# @param color - of the arrows
def plot_quiver(pts, vecs, color=(0.,1.,0.), arrow_scale = 0.05,
                point_mode='sphere', point_scale=0.002):
    x = pts[0,:].A1
    y = pts[1,:].A1
    z = pts[2,:].A1

    u = vecs[0,:].A1
    v = vecs[1,:].A1
    w = vecs[2,:].A1

    plot_points(pts, mode=point_mode, scale_factor=point_scale)
    mlab.quiver3d(x, y, z, u, v, w, scale_factor=arrow_scale,
                  color=color, scale_mode='vector')


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
    pts = np.matrix(np.random.uniform(size=(3,5000)))
    plot_points(pts)
    show()





