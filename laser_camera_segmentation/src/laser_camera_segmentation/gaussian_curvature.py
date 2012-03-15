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

#  \author Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)
import numpy as np
from hrl_lib.util import getTime
    
##
# Fits a plane
# 
# @param points 3xn numpy matrix representing points to be fitted
# @param d direction that normal calculated should point in
# @return normal and eigen value points (in descending order) 
def gaussian_curvature(points, p=np.matrix([0,0,1]).T):    
    assert(points.shape[0] == 3)    
    if points.shape[1] < 2:
        return np.matrix([0, 0, 0.0]).T, np.array([0, 0, 0.0])
    c = np.cov(points)
    try:
        u, s, vh = np.linalg.svd(c)
        u = np.matrix(u)  
    
        #select the 3rd eigenvector, assoc. with the smallest eigenvalue
        if u[:,2].T * p < 0:
            return -u[:,2], s
        else:
            return u[:,2], s
    except np.linalg.linalg.LinAlgError, e:
        print e
        print 'oh no!!!!!! why is this happening?'
        print points.shape
        print c        
        print points
    
def spread(points):
    assert(points.shape[0] == 3)
    if points.shape[1] < 2:
        return np.matrix([0, 0, 0.0]).T, np.array([0, 0, 0.0])
   
    c = np.cov(points)
    u, s, vh = np.linalg.svd(c)
    u = np.matrix(u)      
    return u[:,0:2], s[0:2]
    
    
##
# Calculates a coordinate system  & eigen values of given points matrix 
#
# @param points 3xn numpy matrix representing points to be fitted
# @param x first principle direction to match to
# @param z second direction to match to
# @return (3x3 matrix describe coordinate frame, list with associated eigen values) 
def gaussian_curvature2(points, x=np.matrix([1,0,0]).T, z=np.matrix([0,0,1]).T):
    assert(points.shape[0] == 3)
    #svd
    c = np.cov(points)
    u, s, vh = np.linalg.svd(c)
    u = np.matrix(u)  
      
    choices = range(3)
    #find correspondences
    z_distances = [np.degrees(np.arccos(np.abs(z.T * u[:,i])))[0,0] for i in choices]
    z_choice = choices[np.argmin(z_distances)]    
#    print z.T, z_distances, z_choice
#    print 'remaining', choices, 'removing', z_choice
    choices.remove(z_choice)    
    
    x_distances = [np.degrees(np.arccos(np.abs(x.T * u[:,i])))[0,0] for i in choices]
    x_choice = choices[np.argmin(x_distances)]
#    print x.T, x_distances, x_choice
#    print 'remaining', choices, 'removing', x_choice
    choices.remove(x_choice)    
    
    #reorder
    xv = u[:,x_choice]
    xs = s[x_choice]
    
    ys = s[choices[0]]
        
    zv = u[:,z_choice]
    zs = s[z_choice]
    
    #check directions
    if np.degrees(np.arccos(xv.T * x)) > 90.0:
        xv = -xv
    if np.degrees(np.arccos(zv.T * z)) > 90.0:
        zv = -zv
    
    #find third vector
    yv = np.matrix(np.cross(xv.T, zv.T)).T
    directions = np.concatenate([xv, yv, zv], axis=1)
    sd = [xs, ys, zs]
    if np.linalg.det(directions) < 0:
        directions[:,1] = -directions[:,1]
    assert np.linalg.det(directions) > 0        
    return directions, sd

        
def plot_axis(x,y, z, directions):
    from enthought.mayavi import mlab
    mlab.quiver3d(x, y, z, [directions[0,0]], [directions[1,0]], [directions[2,0]], scale_factor=1, color=(1,0,0))
    if directions.shape[1] > 1:
        mlab.quiver3d(x, y, z, [directions[0,1]], [directions[1,1]], [directions[2,1]], scale_factor=1, color=(0,1,0))
        mlab.quiver3d(x, y, z, [directions[0,2]], [directions[1,2]], [directions[2,2]], scale_factor=1, color=(0,0,1))        
    
def generate_pts():    
    #Generate random points on a disc
    radius = np.random.random((1, 2000))
    angle = np.random.random((1,2000)) * 2*np.pi
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    r = np.concatenate((x,y))    
    r[0,:] = r[0,:] * 2
    xy_plane = np.matrix(np.eye(3))[:,(0, 2)]
    pts = xy_plane * r
    return pts
    
    
def demo1():
    from enthought.mayavi import mlab
    pts = generate_pts()
    directions, magnitudes = gaussian_curvature(pts)
    print directions.T.A[0].tolist()

    print magnitudes.tolist()
    mlab.points3d(pts[0,:].A1, pts[1,:].A1, pts[2,:].A1, mode='sphere', scale_factor=.05)
    plot_axis(0,0,0, directions)
    plot_axis(2,0,0, np.eye(3))
    mlab.show()

def demo2():
    from enthought.mayavi import mlab
    pts = generate_pts()
    direction, magnitudes = gaussian_curvature(pts)    
    print 'eigen values', magnitudes.T
    mlab.points3d(pts[0,:].A1, pts[1,:].A1, pts[2,:].A1, mode='sphere', scale_factor=.05)
    plot_axis(0,0,0, direction)
    plot_axis(2,0,0, np.eye(3))
    mlab.show()    

if __name__ == '__main__':
    demo1()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
#    #U is unordered, we reorder it according to given x and z
#    u      = np.matrix(u)
#    vecs   = [u[:,i] / np.linalg.norm(u[:,i]) for i in range(3)]
#    s_list = [s[i] for i in range(3)]
#    canon  = [v / np.linalg.norm(v) for v in [x, z]]
#
#    ordered_vecs = []
#    ordered_s = [](x.T * u[
#    for idx, v in enumerate(vecs):    
#        angles = []        
#        for d in canon:
#            sep = np.degrees(np.arccos(d.T * v))[0,0]
#            if sep == 180:
#                sep = 0
#            angles.append(sep)            
#        min_idx = np.argmin(angles)        
#        ordered_vecs.append(vecs.pop(min_idx))
#        ordered_s.append(s_list.pop(min_idx))
#        
#    #pdb.set_trace()
    #ordered_vecs.append(np.matrix(np.cross(ordered_vecs[0].T, ordered_vecs[1].T)).T)
#    xe = ordered_vecs[0]
#    ye = ordered_vecs[2]
#    ze = ordered_vecs[1]
#    ordered_vecs = [xe, ye, ze]    
#        
#    ordered_s.append(s_list[0])
#    ordered_s = [ordered_s[0], ordered_s[2], ordered_s[1]]
#    ordered_s = np.matrix(ordered_s).T
#    print ordered_s
#    
#    reordered = np.concatenate(ordered_vecs, axis=1)
#    if np.linalg.det(reordered) < 0:
#        reordered[:,1] = -reordered[:,1]
#        ordered_s[1,0] = -ordered_s[1,0]
#        
#    return reordered, ordered_s     
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    