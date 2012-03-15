#
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

# \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)


import mekabot.hrl_robot as hr
import hrl_lib.util as ut, hrl_lib.transforms as tr

import math, numpy as np
import copy

import sys
sys.path.append('../')
import compliant_trajectories as ct
import segway_motion_calc as smc


def compute_workspace(z, hook_angle):
    firenze = hr.M3HrlRobot(connect=False)
    rot_mat = tr.Rz(hook_angle)*tr.Rx(math.radians(0))*tr.Ry(math.radians(-90))
    delta_list = [math.radians(d) for d in [0.,0.,0.,0.,10.,10.,10.]]
    x_list,y_list = [],[]
    if z < -0.4:
        xmin = 0.10
        xmax = 0.65
    else:
        xmin = 0.15
        xmax = 0.65

    for x in np.arange(xmin,xmax,.01):
        for y in np.arange(-0.1,-0.50,-0.01):
            if x<0.3 and y>-0.2:
                continue
            q = firenze.IK('right_arm',np.matrix([x,y,z]).T,rot_mat)
            if q != None and firenze.within_physical_limits_right(q,delta_list)==True:
                x_list.append(x)
                y_list.append(y)
    return np.matrix([x_list,y_list])

def create_workspace_dict():
    dd = {}
    ha_list = [math.radians(d) for d in [0.,90.,-90.]]
    for ha in ha_list:
        d = {}
        for z in np.arange(-0.1,-0.55,-0.01):
            print 'z:',z
            pts2d = compute_workspace(z,hook_angle=ha)
            d[z] = pts2d
        dd[ha] = {}
        dd[ha]['pts'] = d

    ut.save_pickle(dd,'workspace_dict_'+ut.formatted_time()+'.pkl')


def create_workspace_boundary(pkl_name):
    dd = ut.load_pickle(pkl_name)
    for ha in dd.keys():
        pts_dict = dd[ha]['pts']
        bndry_dict = {}
        for z in pts_dict.keys():
            print 'z:',z
            wrkspc = pts_dict[z]
            if wrkspc.shape[1] < 100:
                pts_dict.pop(z)
                continue
            bndry = smc.compute_boundary(wrkspc)
            bndry_dict[z] = bndry
        dd[ha]['bndry'] = bndry_dict
    ut.save_pickle(dd, pkl_name)



create_workspace_dict()

#if len(sys.argv) != 2:
#    print 'Usage:', sys.argv[0], '<wrkspc dict pkl>'
#    print 'Exiting ...'
#    sys.exit()
#create_workspace_boundary(sys.argv[1])



