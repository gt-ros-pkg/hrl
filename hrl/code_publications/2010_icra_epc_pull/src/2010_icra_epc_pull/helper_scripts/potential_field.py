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


import sys,time
import hrl_lib.util as ut, hrl_lib.transforms as tr
import matplotlib_util.util as mpu
import numpy as np, math

sys.path.append('../')
import segway_motion_calc as smc
import arm_trajectories as at

def plot_hook_translation(curr_pos_tl,cx_tl,cy_tl,cy_ts,
                          start_pos_ts,eq_pt_tl,bndry,wrkspc_pts):
    vt,a = smc.segway_motion_repulse(curr_pos_tl,cx_tl,cy_tl,cy_ts,
                        start_pos_ts,eq_pt_tl,bndry,wrkspc_pts)


    mpu.plot_yx(eq_pt_tl[1,:].A1, eq_pt_tl[0,:].A1, linewidth=2,
                color='g', scatter_size=15, label='Eq Pt')
    mpu.plot_yx(curr_pos_tl[1,:].A1, curr_pos_tl[0,:].A1, linewidth=0,
                color='b', scatter_size = 15, label = 'FK')
    mpu.plot_yx(bndry[1,:].A1, bndry[0,:].A1, linewidth=0, color='y',
                scatter_size=8)
    mpu.plot_yx([-0.2], [0.], linewidth=0, color='b', scatter_size=2)

    bndry_dist_eq = smc.dist_from_boundary(eq_pt_tl, bndry, wrkspc_pts) # signed
    bndry_dist_ee = smc.dist_from_boundary(curr_pos_tl, bndry, wrkspc_pts) # signed
    if bndry_dist_ee < bndry_dist_eq:
        p = curr_pos_tl
    else:
        p = eq_pt_tl
    pts_close = smc.pts_within_dist(p[0:2,:],bndry,0.01,0.1)
    mpu.plot_yx(pts_close[1,:].A1, pts_close[0,:].A1, linewidth=0,
                color='r', scatter_size = 8)

    nrm = np.linalg.norm(vt)
    vt = vt/nrm
    mpu.plot_quiver_yxv(p[1,:].A1, p[0,:].A1, vt, scale=12)

    mpu.show()

# only interested in the translation. so junk values for circle
# params are ok.
def plot_eq_pt_motion_tl():
    vec_list = []
    for i in range(len(ee_tl.p_list)):
#    for i in range(5):
        curr_pos_tl = np.matrix(ee_tl.p_list[i]).T
        eq_pt_tl = np.matrix(eq_tl.p_list[i]).T

        pts_ts = np.matrix(ee_ts.p_list[0:i+1]).T
        pts_2d_ts = pts_ts[0:2,:]
#        rad_opt,cx_ts,cy_ts = at.fit_circle(rad_guess,x_guess,y_guess,pts_2d_ts,
#                                         method='fmin_bfgs',verbose=False)
        rad_opt = 1.0
        cx_ts,cy_ts = 0.5,-1.3
        c_ts = np.matrix([cx_ts,cy_ts,0.]).T
        x,y,a = st.x_list[i],st.y_list[i],st.a_list[i]
        c_tl = smc.tlTts(c_ts,x,y,a)
        cx_tl,cy_tl = c_tl[0,0],c_tl[1,0]
        t0 = time.time()
        vt,a = smc.segway_motion_repulse(curr_pos_tl,cx_tl,cy_tl,cy_ts,
                            start_pos_ts,eq_pt_tl,bndry,wrkspc_pts)
        t1 = time.time()
#        print 'time to segway_motion_repulse:',t1-t0
        nrm = np.linalg.norm(vt)
#        if nrm > 0.005:
        vt = vt/nrm
        vec_list.append(vt.A1.tolist())

    v = np.matrix(vec_list).T

    eq_pts = np.matrix(eq_tl.p_list).T
    ee_pts = np.matrix(ee_tl.p_list).T
    mpu.plot_yx(eq_pts[1,:].A1,eq_pts[0,:].A1,linewidth=1,color='g',label='eq')
    mpu.plot_yx(ee_pts[1,:].A1,ee_pts[0,:].A1,linewidth=1,color='b',label='FK')
    mpu.plot_yx(bndry[1,:].A1,bndry[0,:].A1,linewidth=0,color='y')
    mpu.plot_quiver_yxv(eq_pts[1,:].A1,eq_pts[0,:].A1,v,scale=30)
    mpu.legend()
    mpu.show()

def plot_single_point():
    n_pts = 115
    pts_ts = np.matrix(ee_ts.p_list[0:n_pts]).T
    pts_2d_ts = pts_ts[0:2,:]
    rad_opt,cx_ts,cy_ts = at.fit_circle(rad_guess,x_guess,y_guess,pts_2d_ts,
                                     method='fmin_bfgs',verbose=False)
    print 'rad_opt,cx_ts,cy_ts:',rad_opt,cx_ts,cy_ts

    c_ts = np.matrix([cx_ts,cy_ts,0.]).T
    x,y,a = st.x_list[n_pts-1],st.y_list[n_pts-1],st.a_list[n_pts-1]
    c_tl = smc.tlTts(c_ts,x,y,a)
    cx_tl,cy_tl = c_tl[0,0],c_tl[1,0]

    curr_pos_tl = np.matrix(ee_tl.p_list[n_pts-1]).T
    eqpt_tl = np.matrix(eq_tl.p_list[n_pts-1]).T

    plot_hook_translation(curr_pos_tl,cx_tl,cy_tl,cy_ts,start_pos_ts,
                          eqpt_tl,bndry,wrkspc_pts)


def calc_motion_all():
    for i in range(len(ee_tl.p_list)):
        curr_pos_tl = np.matrix(ee_tl.p_list[i]).T
        eq_pt_tl = np.matrix(eq_tl.p_list[i]).T

        pts_ts = np.matrix(ee_ts.p_list[0:i+1]).T
        pts_2d_ts = pts_ts[0:2,:]
        rad_opt,cx_ts,cy_ts = at.fit_circle(rad_guess,x_guess,y_guess,pts_2d_ts,
                                         method='fmin_bfgs',verbose=False)
        c_ts = np.matrix([cx_ts,cy_ts,0.]).T
        x,y,a = st.x_list[i],st.y_list[i],st.a_list[i]
        c_tl = smc.tlTts(c_ts,x,y,a)
        cx_tl,cy_tl = c_tl[0,0],c_tl[1,0]
        vt,a = smc.segway_motion_repulse(curr_pos_tl,cx_tl,cy_tl,cy_ts,
                            start_pos_ts,eq_pt_tl,bndry)
        print 'angle:',math.degrees(a)



argv = sys.argv

fname = argv[1]
d = ut.load_pickle(fname)

st = d['segway']

ee_tl = at.joint_to_cartesian(d['actual'])
ee_ts = at.account_segway_motion(ee_tl,st)

eq_tl = at.joint_to_cartesian(d['eq_pt'])
eq_ts = at.account_segway_motion(eq_tl,st)

bndry = d['bndry']
wrkspc_pts = d['wrkspc']

rad_guess = 1.0
start_pos_ts = np.matrix(ee_ts.p_list[0]).T
x_guess = start_pos_ts[0,0]
y_guess = start_pos_ts[1,0] - rad_guess

plot_single_point()
#calc_motion_all()
#plot_eq_pt_motion_tl()

