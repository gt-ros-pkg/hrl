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




if __name__ == '__main__':
    import matplotlib.pyplot as pp

    test_project_point_on_line = False
    test_convex_hull_flag = True

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



