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


import roslib
roslib.load_manifest('equilibrium_point_control')


import numpy as np, math
import scipy.optimize as so
import scipy.ndimage as ni
import matplotlib_util.util as mpu
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_hokuyo.hokuyo_processing as hp
import mekabot.coord_frames as mcf
import util as uto
from opencv.highgui import *

##
# @param pts - 2xN np matrix
# @return r,theta (two 1D np arrays)
def cartesian_to_polar(pts):
    r = ut.norm(pts).A1
    theta = np.arctan2(pts[1,:],pts[0,:]).A1
    return r,theta

##
# @param r - 1D np array
# @param theta - 1D np array (RADIANS)
# @return 2xN np matrix of cartesian points
def polar_to_cartesian(r,theta):
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    return np.matrix(np.row_stack((x,y)))

##
# mx,my,ma - motion of the robot
# cx,cy - axis of mechanism in robot frame.
# start_angle,end_angle - between 0 and 2*pi
def point_contained(mx,my,ma,cx,cy,rad,pts,start_angle,end_angle,buffer):
    if abs(mx)>0.2 or abs(my)>0.2 or abs(ma)>math.radians(40):
#        print 'too large a motion for point_contained'
        return np.array([[]])
    pts_t = pts + np.matrix([mx,my]).T
    pts_t = tr.Rz(-ma)[0:2,0:2]*pts_t
    r,t = cartesian_to_polar(pts_t-np.matrix([cx,cy]).T)
    t = np.mod(t,math.pi*2) # I want theta to be b/w 0 and 2pi
    if start_angle<end_angle:
        f = np.row_stack((r<rad+buffer,r>rad-buffer/2.,t<end_angle,t>start_angle))
    else:
        f = np.row_stack((r<rad+buffer,r>rad-buffer/2.,t<start_angle,t>end_angle))
    idxs = np.where(np.all(f,0))

    r_filt = r[idxs]
    t_filt = t[idxs]
    return polar_to_cartesian(r_filt,t_filt)+np.matrix([cx,cy]).T

def optimize_position(cx,cy,rad,curr_pos,eq_pos,pts,bndry,start_angle,
                      end_angle,buffer,tangential_force):
    scale_x,scale_y,scale_a = 1.,1.,1.
    b = min(abs(tangential_force),60.)
    if end_angle>start_angle:
#        min_alpha = math.radians(30)
        max_alpha = math.radians(90)
    else:
#        min_alpha = math.radians(-150)
        max_alpha = math.radians(-90)
    min_alpha = max_alpha - math.radians(60-b*0.7)

    dist_moved_weight = 0.4 - 0.3*b/60.
    alpha_weight = 0.4+1.0*b/60.
    bndry_weight = 1.
    pts_in_weight = 1.

    print 'OPTIMIZE_POSITION'
    print 'start_angle:', math.degrees(start_angle)
    print 'end_angle:', math.degrees(end_angle)
    print 'tangential_force:', tangential_force

    def error_function(params):
        mx,my,ma = params[0],params[1],params[2]
        mx,my,ma = mx/scale_x,my/scale_y,ma/scale_a
        #x,y = params[0],params[1]
        pts_in = point_contained(mx,my,ma,cx,cy,rad,pts,
                                 start_angle,end_angle,buffer)
        p = tr.Rz(ma)*curr_pos-np.matrix([mx,my,0.]).T
        p_eq = tr.Rz(ma)*eq_pos-np.matrix([mx,my,0.]).T

        dist_moved = math.sqrt(mx*mx+my*my)+abs(ma)*0.2
        dist_bndry = dist_from_boundary(p_eq,bndry,pts)
        alpha = math.pi-(start_angle-ma)

        if alpha<min_alpha:
            alpha_cost = min_alpha-alpha
        elif alpha>max_alpha:
            alpha_cost = alpha-max_alpha
        else:
            alpha_cost = 0.
        alpha_cost = alpha_cost * alpha_weight
        move_cost = dist_moved * dist_moved_weight
        bndry_cost = dist_bndry * bndry_weight
        pts_in_cost = pts_in.shape[1]/1000. * pts_in_weight

#        print '---------------------------------'
#        print 'alpha:',math.degrees(alpha)
#        print 'alpha_cost:',alpha_cost
#        print 'mx,my:',mx,my
#        print 'dist_moved:',dist_moved
#        print 'dist_bndry:',dist_bndry
#        print 'pts_in.shape[1]:',pts_in.shape[1]
#        print 'move_cost:', move_cost
#        print 'bndry_cost:',bndry_cost
#        print 'pts_in_cost:',pts_in_cost

#        return -pts_in.shape[1]+dist_moved - bndry_cost
        err = -pts_in_cost-bndry_cost+move_cost+alpha_cost
#        print 'error function value:',err
        return err

    params_1 = [0.,0.,0.]
    res = so.fmin_bfgs(error_function,params_1,full_output=1)

    r,f = res[0],res[1]

#    r,f,d = so.fmin_l_bfgs_b(error_function,params_1,approx_grad=True,
#                             bounds=[(-0.1*scale_x,0.1*scale_x),
#                                     (-0.1*scale_y,0.1*scale_y),
#                              (-math.radians(15)*scale_a,
#                                math.radians(15)*scale_a)],
#                             m=10, factr=10000000.0,
#                             pgtol=1.0000000000000001e-05,
#                             epsilon=0.0001, iprint=-1,
#                             maxfun=1000)
    opt_params = r
#    print 'optimized value:',f
    mx,my,ma =  opt_params[0]/scale_x,opt_params[1]/scale_y,\
                opt_params[2]/scale_a
    error_function(opt_params)
    return mx,my,ma
    #return opt_params[0],opt_params[1]

##
# compute the boundary of the 2D points. Making assumptions about
# the density of the points, tested with workspace_dict only.
# @param pts - 2xN np matrix
def compute_boundary(pts):
    npim1,nx,ny,br = hp.xy_map_to_np_image(pts,m_per_pixel=0.01,dilation_count=0,padding=10)
    npim1 = npim1/255
    npim = np.zeros(npim1.shape,dtype='int')
    npim[:,:] = npim1[:,:]

    connect_structure = np.empty((3,3),dtype='int')
    connect_structure[:,:] = 1
    erim = ni.binary_erosion(npim,connect_structure,iterations=1)
    bim = npim-erim
    tup = np.where(bim>0)
    bpts = np.row_stack((nx-tup[0],ny-tup[1]))*0.01 + br
#    cvim = uto.np2cv(bim)
#    cvSaveImage('boundary.png',cvim)
    return np.matrix(bpts)

##
#return 2x1 vector from closest boundary point
def vec_from_boundary(curr_pos,bndry):
    p = curr_pos[0:2,:]
    v = p-bndry
    min_idx = np.argmin(ut.norm(v))
    return v[:,min_idx]

##
#return distance from boundary. (-ve if outside the boundary)
# @param curr_pos - can be 3x1 np matrix
# @param bndry - boundary (2xN np matrix)
# @param pts - 2xN np matrix. pts whose boundary is bndry
def dist_from_boundary(curr_pos,bndry,pts):
    mv = vec_from_boundary(curr_pos,bndry)
#    spoly = sg.Polygon((bndry.T).tolist())
#    spt = sg.Point(curr_pos[0,0],curr_pos[1,0])
    d = np.linalg.norm(mv)

    p = curr_pos[0:2,:]
    v = p-pts
    min_dist = np.min(ut.norm(v))
#    print 'min_dist,d:',min_dist,d
#    print 'min_dist >= d',min_dist >= d-0.001
    if min_dist >= d-0.001:
#        print 'I predict outside workspace'
        d = -d

#    if spoly.contains(spt) == False:
#        print 'Shapely predicts outside workspace'
#        d = -d
    return d

##
# @param curr_pos - current location of end effector. 3x1 np matrix
# @param bndry - workspace boundary. 2xN np matrix
def close_to_boundary(curr_pos,bndry,pts,dist_thresh):
    min_dist = dist_from_boundary(curr_pos,bndry,pts)
    return min_dist <= dist_thresh

def visualize_boundary():
    d = ut.load_pickle('../../pkls/workspace_dict_2009Sep03_010426.pkl')
    z = -0.23
    k = d.keys()
    k_idx = np.argmin(np.abs(np.array(k)-z))
    pts = d[k[k_idx]]
    bpts = compute_boundary(pts)

    cl_list = []
    for pt in pts.T:
        if close_to_boundary(pt.T,bpts,dist_thresh=0.05)==True:
            cl_list.append(pt.A1.tolist())
    clpts = np.matrix(cl_list).T
    print 'clpts.shape:', clpts.shape

    mpu.plot_yx(pts[1,:].A1,pts[0,:].A1,linewidth=0)
    mpu.plot_yx(clpts[1,:].A1,clpts[0,:].A1,linewidth=0,color='r')
    mpu.plot_yx(bpts[1,:].A1,bpts[0,:].A1,linewidth=0,color='y')
    mpu.show()


## transform from torso start to torso local frame.
# @param pts - 3xN np matrix in ts coord frame.
# @param x,y,a - motion of the segway (in the ms frame)
# @return pts_tl
def tlTts(pts_ts,x,y,a):
    pts_ms = mcf.mecanumTglobal(mcf.globalTtorso(pts_ts))
    v_org_ms = np.matrix([x,y,0.]).T
    pts_ml = tr.Rz(a)*(pts_ms-v_org_ms)
    pts_tl = mcf.torsoTglobal(mcf.globalTmecanum(pts_ml))
    return pts_tl

## transform from torso local to torso start frame.
# @param pts - 3xN np matrix in tl coord frame.
# @param x,y,a - motion of the segway (in the ms frame)
# @return pts_ts
def tsTtl(pts_tl,x,y,a):
    pts_ml = mcf.mecanumTglobal(mcf.globalTtorso(pts_tl))
    v_org_ms = np.matrix([x,y,0.]).T
    pts_ms = tr.Rz(-a) * pts_ml + v_org_ms
    pts_ts = mcf.torsoTglobal(mcf.globalTmecanum(pts_ms))
    return pts_ts

## rotate vector from torso local to torso start frame.
# @param vecs_tl - 3xN np matrix in tl coord frame.
# @param a - motion of the segway (in the ms frame)
# @return vecs_ts
def tsRtl(vecs_tl, a):
    vecs_ml = mcf.mecanumTglobal(mcf.globalTtorso(vecs_tl, True), True)
    vecs_ms = tr.Rz(-a) * vecs_ml
    vecs_ts = mcf.torsoTglobal(mcf.globalTmecanum(vecs_ms, True), True)
    return vecs_ts

## rotate vector from torso local to torso start frame.
# @param vecs_tl - 3xN np matrix in tl coord frame.
# @param a - motion of the segway (in the ms frame)
# @return vecs_ts
def tlRts(vecs_ts, a):
    vecs_ms = mcf.mecanumTglobal(mcf.globalTtorso(vecs_ts, True), True)
    vecs_ml = tr.Rz(a) * vecs_ms
    vecs_tl = mcf.torsoTglobal(mcf.globalTmecanum(vecs_ml, True), True)
    return vecs_tl


def pts_within_dist(p,pts,min_dist,max_dist):
    v = p-pts
    d_arr = ut.norm(v).A1
    idxs = np.where(np.all(np.row_stack((d_arr<max_dist,d_arr>min_dist)),axis=0))
    pts_within = pts[:,idxs[0]]
    return pts_within


## apologies for the poor name. computes the translation of the torso
# frame that move the eq pt away from closest boundary and rotate such
# that local x axis is perp to mechanism returns 2x1 np matrix, angle
def segway_motion_repulse(curr_pos_tl, eq_pt_tl,bndry, all_pts):
    bndry_dist_eq = dist_from_boundary(eq_pt_tl,bndry,all_pts) # signed
    bndry_dist_ee = dist_from_boundary(curr_pos_tl,bndry,all_pts) # signed
    if bndry_dist_ee < bndry_dist_eq:
        p = curr_pos_tl[0:2,:]
        bndry_dist = bndry_dist_ee
    else:
        p = eq_pt_tl[0:2,:]
        bndry_dist = bndry_dist_eq

#    p = eq_pt_tl[0:2,:]
    pts_close = pts_within_dist(p,bndry,0.002,0.07)
    v = p-pts_close
    d_arr = ut.norm(v).A1
    v = v/d_arr
    v = v/d_arr # inverse distance weight
    resultant = v.sum(1)
    res_norm = np.linalg.norm(resultant)
    resultant = resultant/res_norm
    tvec = -resultant

    if bndry_dist < 0.:
        tvec = -tvec # eq pt was outside workspace polygon.

    if abs(bndry_dist)<0.01 or res_norm<0.01:
        # internal external test fails so falling back on
        # going to mean.
        m = all_pts.mean(1)
        tvec = m-p
        tvec = -tvec/np.linalg.norm(tvec)

    dist_move = 0.
    if bndry_dist > 0.05:
        dist_move = 0.
    else:
        dist_move = 1.

    tvec = tvec*dist_move # tvec is either a unit vec or zero vec.
    return tvec


if __name__ == '__main__':

    #d = ut.load_pickle('workspace_dict_2009Sep03_221107.pkl')
    d = ut.load_pickle('../../pkls/workspace_dict_2009Sep05_200116.pkl')
    z = -0.23
    k = d.keys()
    k_idx = np.argmin(np.abs(np.array(k)-z))
    pts = d[k[k_idx]]
#    visualize_boundary()

    for kk in k:
        pts = d[kk]
        bpts = compute_boundary(pts)
        cx,cy = 0.7,-0.6
        rad = 0.4
        
    #    pts_in = point_contained(cx,cy,0.,rad,pts,
    #                             start_angle=math.radians(140),
    #                             end_angle=math.radians(190))
        mpu.figure()
        mpu.plot_yx(pts[1,:].A1,pts[0,:].A1,linewidth=0)
        mpu.plot_yx(bpts[1,:].A1,bpts[0,:].A1,linewidth=0,color='y')
#    mpu.plot_yx(pts_in[1,:].A1,pts_in[0,:].A1,linewidth=0,color='g')
#    mpu.plot_yx([cy],[cx],linewidth=0,color='r')
    mpu.show()




### apologies for the poor name. computes the translation and rotation
## of the torso frame that move the eq pt away from closest boundary
## and rotate such that local x axis is perp to mechanism
## returns 2x1 np matrix, angle
#def segway_motion_repulse(curr_pos_tl,cx_tl,cy_tl,cy_ts,start_pos_ts,
#                          eq_pt_tl,bndry):
#    vec_bndry = vec_from_boundary(eq_pt_tl,bndry)
#    dist_boundary = np.linalg.norm(vec_bndry)
#    vec_bndry = vec_bndry/dist_boundary
#
#    radial_vec_tl = curr_pos_tl[0:2]-np.matrix([cx_tl,cy_tl]).T
#    radial_angle = math.atan2(radial_vec_tl[1,0],radial_vec_tl[0,0])
#    if cy_ts<start_pos_ts[1,0]:
#        err = radial_angle-math.pi/2
#    else:
#        err = radial_angle +math.pi/2
#    
#    a_torso = err
#    dist_move = max(0.15-dist_boundary,0.)
#    if dist_move < 0.04:
#        dist_move = 0.
#    hook_translation_tl = -vec_bndry*dist_move
#
##    print 'vec_bndry:',vec_bndry.A1.tolist()
##    print 'dist_boundary:',dist_boundary
#
#    return hook_translation_tl,a_torso


