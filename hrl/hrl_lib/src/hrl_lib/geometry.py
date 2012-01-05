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

# convex_hull requires Shapely
import shapely.geometry as sg

import math
import numpy as np

# inverse of a 2x2 matrix
def inverse_2x2(a):
    return 1/np.linalg.det(a) * np.matrix([[a[1,1],-a[0,1]],[-a[1,0],a[0,0]]])

## compute point that is the projection of q onto the line passing
# through p1 and p2. 2x1 or 3x1 np matrices.
def project_point_on_line(q, p1, p2):
    v1 = p2-p1
    v1 = v1 / np.linalg.norm(v1)
    v2 = q - p1
    p = p1 + np.dot(v2.A1, v1.A1) * v1
    return p

# @param pts - Nx2 np array
# @return Mx2 np array
def convex_hull(pts):
    mp = sg.MultiPoint(pts)
    ch = mp.convex_hull
    if ch.__class__ == sg.polygon.Polygon:
        hull_pts = np.array(ch.exterior.coords)
    else:
        hull_pts = np.array(ch.coords)
    return hull_pts

# distance of a point from a curve defined by a series of points.
# pt - 2x1 or 3x1 np matrix
# pts_list - list of array-like of len 2 or 3
def distance_from_curve(pt, pts_list):
    spt = sg.Point(pt.A1)
    s_pts_list = sg.LineString(pts_list)
    return s_pts_list.distance(spt)

# distance of a point from a curve defined by a series of points.
# pt - 2x1 or 3x1 np matrix
# pts_list - list of array-like of len 2 or 3
def distance_along_curve(pt, pts_list):
    spt = sg.Point(pt.A1)
    s_pts_list = sg.LineString(pts_list)
    return s_pts_list.project(spt)

## return a point on the curve that is closest to the given point.
# pt - 2x1 or 3x1 np matrix
# pts_list - list of array-like of len 2 or 3
def project_point_on_curve(pt, pts_list):
    spt = sg.Point(pt.A1)
    s_pts_list = sg.LineString(pts_list)
    d = s_pts_list.project(spt)
    spt_new = s_pts_list.interpolate(d)
    return np.matrix(spt_new.coords[0]).T

## return point dist away from the start of the curve, measured along
# the curve.
# pts_list - list of array-like of len 2 or 3
# dist - float.
# normalized - see Shapely.interpolate documentation.
def get_point_along_curve(pts_list, dist, normalized=False):
    s_pts_list = sg.LineString(pts_list)
    spt_new = s_pts_list.interpolate(dist, normalized)
    return np.matrix(spt_new.coords[0]).T

def test_convex_hull():
    pp.axis('equal')
    pts = np.random.uniform(size=(40,2))
    h_pts = convex_hull(pts)
    print 'h_pts.shape:', h_pts.shape

    all_x = pts[:,0]
    all_y = pts[:,1]
    x_ch, y_ch = h_pts[:,0], h_pts[:,1]

    pp.plot(all_x, all_y, '.b', ms=5)
    pp.plot(x_ch, y_ch, '.-g', ms=5)

    pp.show()

def test_distance_from_curve():
    pt = np.matrix([0.8, 0.5, 0.3]).T
    pts_l = [[0, 0.], [1,0.], [1.,1.]]
    print 'distance_from_curve:', distance_from_curve(pt, pts_l)
    print 'distance_along_curve:', distance_along_curve(pt, pts_l)
    
    pt_proj = project_point_on_curve(pt, pts_l)

    pp.axis('equal')
    pp.plot([pt[0,0]], [pt[1,0]], 'go', ms=7)
    pp.plot([pt_proj[0,0]], [pt_proj[1,0]], 'yo', ms=7)
    pp.plot(*zip(*pts_l))
    pp.show()

# finds a,b such that x=ay+b (good if expected slope is large)
# x, y -- nX1 matrices
# returns a,b,mean residual
def fit_line_high_slope(x, y):
    A = np.column_stack((y, np.ones((y.shape[0],1))))
    x,resids,rank,s = np.linalg.linalg.lstsq(A, x)
    a = x[0,0]
    b = x[1,0]
    return a,b,resids/y.shape[0]

# finds a,b such that y=ax+b (good if expected slope is small)
# x, y -- nX1 matrices
# returns a,b,mean residual
def fit_line_low_slope(x, y):
    A = np.column_stack((x, np.ones((x.shape[0],1))))
    y,resids,rank,s = np.linalg.linalg.lstsq(A, y)
    a = y[0,0]
    b = y[1,0]
    return a,b,resids/x.shape[0]


if __name__ == '__main__':
    import matplotlib.pyplot as pp

    test_project_point_on_line = True
    test_convex_hull_flag = False
    test_distance_from_curve_flag = False

    if test_project_point_on_line:
        p1 = np.matrix([2,3.]).T
        p2 = np.matrix([-1.5,6.3]).T
        q = np.matrix([1.5,7.3]).T

        p = project_point_on_line(q, p1, p2)

        x = [p1[0,0], p2[0,0]]
        y = [p1[1,0], p2[1,0]]
        pp.plot(x, y, 'b', label='line')

        x = [q[0,0], p[0,0]]
        y = [q[1,0], p[1,0]]
        pp.plot(x, y, '-r.', ms=10, label='projection')

        pp.axis('equal')
        pp.legend()
        pp.show()

    if test_convex_hull_flag:
        test_convex_hull()

    if test_distance_from_curve_flag:
        test_distance_from_curve()



