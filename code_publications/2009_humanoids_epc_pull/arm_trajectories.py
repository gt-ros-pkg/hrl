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

# Author: Advait Jain

import roslib; roslib.load_manifest('2009_humanoids_epc_pull')

import scipy.optimize as so

import math, numpy as np
import pylab as pl
import sys, optparse, time
import copy

from enthought.mayavi import mlab

import mekabot.hrl_robot as hr
#import util as ut
import hrl_lib.util as ut, hrl_lib.transforms as tr
import matplotlib_util.util as mpu
import hrl_tilting_hokuyo.display_3d_mayavi as d3m

#import shapely.geometry as sg

class JointTrajectory():
    ''' class to store joint trajectories.
        data only - use for pickling.
    '''
    def __init__(self):
        self.time_list = [] # time in seconds
        self.q_list = [] #each element is a list of 7 joint angles.
        self.qdot_list = [] #each element is a list of 7 joint angles.

class CartesianTajectory():
    ''' class to store trajectory of cartesian points.
        data only - use for pickling
    '''
    def __init__(self):
        self.time_list = [] # time in seconds
        self.p_list = [] #each element is a list of 3 coordinates

class ForceTrajectory():
    ''' class to store time evolution of the force at the end effector.
        data only - use for pickling
    '''
    def __init__(self):
        self.time_list = [] # time in seconds
        self.f_list = [] #each element is a list of 3 coordinates


def joint_to_cartesian(traj):
    ''' traj - JointTrajectory
        returns CartesianTajectory after performing FK on traj.
    '''
    firenze = hr.M3HrlRobot(connect=False)
    pts = []
    for q in traj.q_list:
        p = firenze.FK('right_arm',q)
        pts.append(p.A1.tolist())

    ct = CartesianTajectory()
    ct.time_list = copy.copy(traj.time_list)
    ct.p_list = copy.copy(pts)
    #return np.matrix(pts).T
    return ct


def plot_forces_quiver(pos_traj,force_traj,color='k'):
    import arm_trajectories as at
    #if traj.__class__ == at.JointTrajectory:
    if isinstance(pos_traj,at.JointTrajectory):
        pos_traj = joint_to_cartesian(pos_traj)

    pts = np.matrix(pos_traj.p_list).T
    label_list = ['X coord (m)', 'Y coord (m)', 'Z coord (m)']
    x = pts[0,:].A1.tolist()
    y = pts[1,:].A1.tolist()

    forces = np.matrix(force_traj.f_list).T
    u = (-1*forces[0,:]).A1.tolist()
    v = (-1*forces[1,:]).A1.tolist()
    pl.quiver(x,y,u,v,width=0.002,color=color,scale=100.0)
#    pl.quiver(x,y,u,v,width=0.002,color=color)
    pl.axis('equal')

def plot_cartesian(traj,xaxis=None,yaxis=None,zaxis=None,color='b',label='_nolegend_',
                   linewidth=2,scatter_size=20):
    ''' xaxis - x axis for the graph (0,1 or 2)
        zaxis - for a 3d plot. not implemented.
    '''

    import arm_trajectories as at
    #if traj.__class__ == at.JointTrajectory:
    if isinstance(traj,at.JointTrajectory):
        traj = joint_to_cartesian(traj)

    pts = np.matrix(traj.p_list).T
    label_list = ['X coord (m)', 'Y coord (m)', 'Z coord (m)']
    x = pts[xaxis,:].A1.tolist()
    y = pts[yaxis,:].A1.tolist()

    if zaxis == None:
        pl.plot(x,y,c=color,linewidth=linewidth,label=label)
        pl.scatter(x,y,c=color,s=scatter_size,label='_nolegend_', linewidths=0)
        pl.xlabel(label_list[xaxis])
        pl.ylabel(label_list[yaxis])
        pl.legend(loc='best')
        pl.axis('equal')
    else:
        from numpy import array
        from enthought.mayavi.api import Engine
        engine = Engine()
        engine.start()
        if len(engine.scenes) == 0:
            engine.new_scene()

        z = pts[zaxis,:].A1.tolist()
        time_list = [t-traj.time_list[0] for t in traj.time_list]
        mlab.plot3d(x,y,z,time_list,tube_radius=None,line_width=4)
        mlab.axes()
        mlab.xlabel(label_list[xaxis])
        mlab.ylabel(label_list[yaxis])
        mlab.zlabel(label_list[zaxis])
        mlab.colorbar(title='Time')

        # ------------------------------------------- 
        axes = engine.scenes[0].children[0].children[0].children[1]
        axes.axes.position = array([ 0.,  0.])
        axes.axes.label_format = '%-#6.2g'
        axes.title_text_property.font_size=4

## compute the force that the arm would apply given the stiffness matrix
# @param  q_actual_traj - Joint Trajectory (actual angles.)
# @param  q_eq_traj - Joint Trajectory (equilibrium point angles.)
# @param torque_traj - JointTrajectory (torques measured at the joints.)
# @param rel_stiffness_list - list of 5 elements (stiffness numbers for the joints.)
# @return lots of things, look at the code.
def compute_forces(q_actual_traj,q_eq_traj,torque_traj,rel_stiffness_list):
    firenze = hr.M3HrlRobot(connect=False)

    d_gains_list_mN_deg_sec = [-100,-120,-15,-25,-1.25]
    d_gains_list = [180./1000.*s/math.pi for s in d_gains_list_mN_deg_sec]

    stiff_list_mNm_deg = [1800,1300,350,600,60]
    stiff_list_Nm_rad = [180./1000.*s/math.pi for s in stiff_list_mNm_deg]
#    stiffness_settings = [0.15,0.7,0.8,0.8,0.8]
#    dia = np.array(stiffness_settings) * np.array(stiff_list_Nm_rad)
    dia = np.array(rel_stiffness_list) * np.array(stiff_list_Nm_rad)
    k_q = np.matrix(np.diag(dia))
    dia_inv = 1./dia
    k_q_inv = np.matrix(np.diag(dia_inv))

    actual_cart = joint_to_cartesian(q_actual_traj)
    eq_cart = joint_to_cartesian(q_eq_traj)
    force_traj_jacinv = ForceTrajectory()
    force_traj_stiff = ForceTrajectory()
    force_traj_torque = ForceTrajectory()

    k_cart_list = []

    for q_actual,q_dot,q_eq,actual_pos,eq_pos,t,tau_m in zip(q_actual_traj.q_list,q_actual_traj.qdot_list,q_eq_traj.q_list,actual_cart.p_list,eq_cart.p_list,q_actual_traj.time_list,torque_traj.q_list):

        q_eq = firenze.clamp_to_physical_joint_limits('right_arm',q_eq)
        q_delta = np.matrix(q_actual).T - np.matrix(q_eq).T
        tau = k_q * q_delta[0:5,0] - np.matrix(np.array(d_gains_list)*np.array(q_dot)[0:5]).T

        x_delta = np.matrix(actual_pos).T - np.matrix(eq_pos).T

        jac_full = firenze.Jac('right_arm',q_actual)
        jac = jac_full[0:3,0:5]

        jac_full_eq = firenze.Jac('right_arm',q_eq)
        jac_eq = jac_full_eq[0:3,0:5]
        k_cart = np.linalg.inv((jac_eq*k_q_inv*jac_eq.T))    # calculating stiff matrix using Jacobian for eq pt.
        k_cart_list.append(k_cart)

        pseudo_inv_jac = np.linalg.inv(jac_full*jac_full.T)*jac_full

        tau_full = np.row_stack((tau,np.matrix(tau_m[5:7]).T))
        #force = (-1*pseudo_inv_jac*tau_full)[0:3]
        force = -1*pseudo_inv_jac[0:3,0:5]*tau
        force_traj_jacinv.f_list.append(force.A1.tolist())
        force_traj_stiff.f_list.append((k_cart*x_delta).A1.tolist())
        force_traj_torque.f_list.append((pseudo_inv_jac*np.matrix(tau_m).T)[0:3].A1.tolist())

    return force_traj_jacinv,force_traj_stiff,force_traj_torque,k_cart_list

## return two lists containing the radial and tangential components of the forces.
# @param f_list - list of forces. (each force is a list of 2 or 3 floats)
# @param p_list - list of positions. (each position is a list of 2 or 3 floats)
# @param cx - x coord of the center of the circle.
# @param cy - y coord of the center of the circle.
# @return list of magnitude of radial component, list of magnitude tangential component.
def compute_radial_tangential_forces(f_list,p_list,cx,cy):
    f_rad_l,f_tan_l = [],[]
    for f,p in zip(f_list,p_list):
        rad_vec = np.array([p[0]-cx,p[1]-cy])
        rad_vec = rad_vec/np.linalg.norm(rad_vec)
        f_vec = np.array([f[0],f[1]])
        f_rad_mag = np.dot(f_vec,rad_vec)
        f_tan_mag = np.linalg.norm(f_vec-rad_vec*f_rad_mag)
        f_rad_mag = abs(f_rad_mag)
        f_rad_l.append(f_rad_mag)
        f_tan_l.append(f_tan_mag)

    return f_rad_l,f_tan_l
        

def plot_error_forces(measured_list,calc_list):
    err_x, err_y = [],[]
    err_rel_x, err_rel_y = [],[]
    mag_x, mag_y = [],[]
    for m,c in zip(measured_list,calc_list):
        err_x.append(abs(m[0]-c[0]))
        err_rel_x.append(abs(m[0]-c[0])/abs(m[0])*100)
        #err_rel_x.append(ut.bound(abs(m[0]-c[0])/abs(m[0])*100,100,0))
        mag_x.append(abs(m[0]))
        err_y.append(abs(m[1]-c[1]))
        err_rel_y.append(abs(m[1]-c[1])/abs(m[1])*100)
        #err_rel_y.append(ut.bound(abs(m[1]-c[1])/abs(m[1])*100,100,0))
        mag_y.append(abs(m[1]))

    x_idx = range(len(err_x))
    zero = [0 for i in x_idx]

    fig = pl.figure()
    ax1 = fig.add_subplot(111)
    ax2 = ax1.twinx()

    ax1.plot(zero,c='k',linewidth=1,label='_nolegend_')
    l1 = ax1.plot(err_x,c='b',linewidth=1,label='absolute error')
    ax1.scatter(x_idx,err_x,c='b',s=10,label='_nolegend_', linewidths=0)
    l2 = ax1.plot(mag_x,c='g',linewidth=1,label='magnitude')
    ax1.scatter(x_idx,mag_x,c='g',s=10,label='_nolegend_', linewidths=0)
    l3 = ax2.plot(err_rel_x,c='r',linewidth=1,label='relative error %')

    ax1.set_ylim(0,15)
    ax2.set_ylim(0,100)
    ax1.set_xlabel('measurement number')
    ax1.set_ylabel('x component of force (N)')
    ax2.set_ylabel('percentage error')
    ax1.yaxis.set_label_coords(-0.3,0.5)
    ax2.yaxis.set_label_coords(-0.3,0.5)
    leg = pl.legend([l1,l2,l3],['absolute error','magnitude','rel error %'],loc='upper left',
              handletextsep=0.015,handlelen=0.003,labelspacing=0.003)

    fig = pl.figure()
    ax1 = fig.add_subplot(111)
    ax2 = ax1.twinx()

    ax1.plot(zero,c='k',linewidth=1)
    l1 = ax1.plot(err_y,c='b',linewidth=1)
    ax1.scatter(x_idx,err_y,c='b',s=10, linewidths=0)
    l2 = ax1.plot(mag_y,c='g',linewidth=1)
    ax1.scatter(x_idx,mag_y,c='g',s=10,linewidths=0)
    l3 = ax2.plot(err_rel_y,c='r',linewidth=1)

    ax1.set_ylim(0,15)
    ax2.set_ylim(0,100)
    ax1.yaxis.set_label_coords(-0.3,0.5)
    ax2.yaxis.set_label_coords(-0.3,0.5)
    ax1.set_xlabel('measurement number')
    ax1.set_ylabel('y component of force (N)')
    ax2.set_ylabel('percentage error')
    #pl.legend(loc='best')
    leg = pl.legend([l1,l2,l3],['absolute error','magnitude','rel error %'],loc='upper left',
              handletextsep=0.015,handlelen=0.003,labelspacing=0.003)

#    pl.figure()
#    pl.plot(zero,c='k',linewidth=0.5,label='_nolegend_')
#    pl.plot(err_y,c='b',linewidth=1,label='error')
#    pl.plot(err_rel_y,c='r',linewidth=1,label='relative error %')
#    pl.scatter(x_idx,err_y,c='b',s=10,label='_nolegend_', linewidths=0)
#    pl.plot(mag_y,c='g',linewidth=1,label='magnitude')
#    pl.scatter(x_idx,mag_y,c='g',s=10,label='_nolegend_', linewidths=0)
#
#    pl.xlabel('measurement number')
#    pl.ylabel('y component of force (N)')
#    pl.legend(loc='best')
#    pl.axis('equal')


def plot_stiff_ellipses(k_cart_list,pos_traj,skip=10,subplotnum=111):
    import arm_trajectories as at
    if isinstance(pos_traj,at.JointTrajectory):
        pos_traj = joint_to_cartesian(pos_traj)

    pts = np.matrix(pos_traj.p_list).T
    x_l = pts[0,:].A1.tolist()
    y_l = pts[1,:].A1.tolist()
    
    from pylab import figure, show, rand
    from matplotlib.patches import Ellipse

    ells = []
    scale = 25000.
    ratio_list = []
    for k_c,x,y in zip(k_cart_list[::skip],x_l[::skip],y_l[::skip]):
        w,v = np.linalg.eig(k_c[0:2,0:2])
        w_abs = np.abs(w)
        major_axis = np.max(w_abs)
        minor_axis = np.min(w_abs)
        print 'major, minor:',major_axis,minor_axis
#        print 'k_c:', k_c
        ratio_list.append(major_axis/minor_axis)

        ells.append(Ellipse(np.array([x,y]),width=w[0]/scale,height=w[1]/scale,angle=math.degrees(math.atan2(v[1,0],v[0,0]))))
        ells[-1].set_lw(2)
        
    #fig = pl.figure()
    #ax = fig.add_subplot(111, aspect='equal')
    ax = pl.subplot(subplotnum, aspect='equal')

    for e in ells:
        ax.add_artist(e)
        #e.set_clip_box(ax.bbox)
        #e.set_alpha(1.)
        e.set_facecolor(np.array([1,1,1]))

    plot_cartesian(pos_traj,xaxis=0,yaxis=1,color='b',
                   linewidth=0.0,scatter_size=0)
#    plot_cartesian(pos_traj,xaxis=0,yaxis=1,color='b',label='Eq Point',
#                   linewidth=1.5,scatter_size=0)
#    plot_cartesian(d['actual'],xaxis=0,yaxis=1,color='b',label='FK',
#                   linewidth=1.5,scatter_size=0)
#    plot_cartesian(d['eq_pt'], xaxis=0,yaxis=1,color='g',label='Eq Point',
#                   linewidth=1.5,scatter_size=0)

    mean_ratio = np.mean(np.array(ratio_list))
    std_ratio = np.std(np.array(ratio_list))

    return mean_ratio,std_ratio


# plot the force field in the xy plane for the stiffness matrix k_cart.
## @param k_cart: 3x3 cartesian space stiffness matrix.
def plot_stiffness_field(k_cart,plottitle=''):
    n_points = 20
    ang_step = math.radians(360)/n_points
    x_list = []
    y_list = []
    u_list = []
    v_list = []
    k_cart = k_cart[0:2,0:2]

    for i in range(n_points):
        ang = i*ang_step
        for r in [0.5,1.,1.5]:
            dx = r*math.cos(ang)
            dy = r*math.sin(ang)
            dtau = -k_cart*np.matrix([dx,dy]).T
            x_list.append(dx)
            y_list.append(dy)
            u_list.append(dtau[0,0])
            v_list.append(dtau[1,0])
    
    pl.figure()
#    mpu.plot_circle(0,0,1.0,0.,math.radians(360))
    mpu.plot_radii(0,0,1.5,0.,math.radians(360),interval=ang_step,color='r')
    pl.quiver(x_list,y_list,u_list,v_list,width=0.002,color='k',scale=None)
    pl.axis('equal')
    pl.title(plottitle)


def plot_stiff_ellipse_map(stiffness_list,num):
    firenze = hr.M3HrlRobot(connect=False)
    hook_3dprint_angle = math.radians(20-2.54)
    rot_mat = tr.Rz(0.-hook_3dprint_angle)*tr.Ry(math.radians(-90))

    d_gains_list_mN_deg_sec = [-100,-120,-15,-25,-1.25]
    d_gains_list = [180./1000.*s/math.pi for s in d_gains_list_mN_deg_sec]

    stiff_list_mNm_deg = [1800,1300,350,600,60]
    stiff_list_Nm_rad = [180./1000.*s/math.pi for s in stiff_list_mNm_deg]
    dia = np.array(stiffness_list) * np.array(stiff_list_Nm_rad)
    k_q = np.matrix(np.diag(dia))
    dia_inv = 1./dia
    k_q_inv = np.matrix(np.diag(dia_inv))

    s0,s1,s2,s3 = stiffness_list[0],stiffness_list[1],stiffness_list[2],stiffness_list[3]

    i=0
    #for z in np.arange(-0.1,-0.36,-0.05):
    for z in np.arange(-0.23,-0.27,-0.05):
        pl.figure()
        k_cart_list = []
        pos_traj = CartesianTajectory()
        for x in np.arange(0.25,0.56,0.05):
            for y in np.arange(-0.15,-0.56,-0.05):
                if math.sqrt(x**2+y**2)>0.55:
                    continue
                q = firenze.IK('right_arm',np.matrix([x,y,z]).T,rot_mat)
                if q == None:
                    continue
                jac_full = firenze.Jac('right_arm',q)
                jac = jac_full[0:3,0:5]
                k_cart = np.linalg.inv((jac*k_q_inv*jac.T))
                k_cart_list.append(k_cart)
                pos_traj.p_list.append([x,y,z])
                pos_traj.time_list.append(0.1)
        if len(pos_traj.p_list)>0:
            ret = plot_stiff_ellipses(k_cart_list,pos_traj,skip=1)
            pl.axis('equal')
            pl.legend(loc='best')
            title_string = 'z: %.2f stiff:[%.1f,%.1f,%.1f,%.1f]'%(z,s0,s1,s2,s3)
            pl.title(title_string)
            i+=1
            pl.savefig('ellipses_%03d_%03d.png'%(num,i))

        return ret


def compute_workspace(z,plot=False,wrist_roll_angle=math.radians(0),subplotnum=None,title=''):
    firenze = hr.M3HrlRobot(connect=False)
#    hook_3dprint_angle = math.radians(20-2.54)
#    rot_mat = tr.Rz(math.radians(-90.)-hook_3dprint_angle)*tr.Ry(math.radians(-90))
    rot_mat = tr.Rz(wrist_roll_angle)*tr.Ry(math.radians(-90))
    x_list,y_list = [],[]
    for x in np.arange(0.15,0.65,0.02):
        for y in np.arange(-0.05,-0.65,-0.02):
            q = firenze.IK('right_arm',np.matrix([x,y,z]).T,rot_mat)
            if q != None:
                x_list.append(x)
                y_list.append(y)

    if len(x_list) > 2:
        multipoint = sg.Point(x_list[0],y_list[0])
        for x,y in zip(x_list[1:],y_list[1:]):
            multipoint = multipoint.union(sg.Point(x,y))
        hull = multipoint.convex_hull
        if plot:
            coords_seq = hull.boundary.coords
            hull_x_list,hull_y_list = [],[]
            for c in coords_seq:
                hull_x_list.append(c[0])
                hull_y_list.append(c[1])
            mpu.plot_yx(y_list,x_list,linewidth=0,subplotnum=subplotnum,axis='equal',
                        plot_title=title)
            mpu.plot_yx(hull_y_list,hull_x_list,linewidth=2,subplotnum=subplotnum,axis='equal')
        return hull,len(x_list)
    else:
        return None,None

def diff_roll_angles():
    pl.subplot(211,aspect='equal')

    
# search along z coord and make a histogram of the areas
def compute_workspace_z():
    n_points_list,area_list,z_list = [],[],[]
    #for z in np.arange(-0.1,-0.36,-0.02):
    #for z in np.arange(-0.05,-0.35,-0.01):
    for z in np.arange(-0.15,-0.155,-0.01):
        pl.figure()
        hull,n_points = compute_workspace(z,plot=True)
        pl.title('z: %.2f'%(z))
        pl.savefig('z_%.2f.png'%(z))
#        hull,n_points = compute_workspace(z,plot=False)
        if hull != None:
            area_list.append(hull.area)
            z_list.append(z)
            n_points_list.append(n_points)

            coords_seq = hull.boundary.coords
            hull_x_list,hull_y_list = [],[]
            for c in coords_seq:
                hull_x_list.append(c[0])
                hull_y_list.append(c[1])
    
    pl.figure()
    mpu.plot_yx(area_list,z_list,linewidth=2,label='area')
    pl.savefig('hist_area.png')
    pl.figure()
    mpu.plot_yx(n_points_list,z_list,linewidth=2,color='g',label='n_points')
#    pl.legend(loc='best')
    pl.xlabel('Z coordinate (m)')
    pl.ylabel('# points')
    pl.savefig('hist_points.png')




## find the x and y coord of the center of the circle of given radius that
# best matches the data.
# @param rad - radius of the circle (not optimized)
# @param x_guess - guess for x coord of center
# @param y_guess - guess for y coord of center.
# @param pts - 2xN np matrix of points.
# @return x,y  (x and y coord of the center of the circle)
def fit_rotary_joint(rad,x_guess,y_guess,pts):
    def error_function(params):
        center = np.matrix((params[0],params[1])).T
        #print 'pts.shape', pts.shape
        #print 'center.shape', center.shape
        #print 'ut.norm(pts-center).shape',ut.norm(pts-center).shape
        err = ut.norm(pts-center).A1 - rad
        res = np.dot(err,err)
        return res

    params_1 = [x_guess,y_guess]
    r = so.fmin_bfgs(error_function,params_1,full_output=1)
    opt_params_1,f_opt_1 = r[0],r[1]

    params_2 = [x_guess,y_guess+2*rad]
    r = so.fmin_bfgs(error_function,params_2,full_output=1)
    opt_params_2,f_opt_2 = r[0],r[1]

    if f_opt_2<f_opt_1:
        return opt_params_2[0],opt_params_2[1]
    else:
        return opt_params_1[0],opt_params_1[1]

## find the x and y coord of the center of the circle and the radius that
# best matches the data.
# @param rad_guess - guess for the radius of the circle
# @param x_guess - guess for x coord of center
# @param y_guess - guess for y coord of center.
# @param pts - 2xN np matrix of points.
# @param method - optimization method. ('fmin' or 'fmin_bfgs')
# @param verbose - passed onto the scipy optimize functions. whether to print out the convergence info.
# @return r,x,y  (radius, x and y coord of the center of the circle)
def fit_circle(rad_guess,x_guess,y_guess,pts,method,verbose=True):
    def error_function(params):
        center = np.matrix((params[0],params[1])).T
        rad = params[2]
        #print 'pts.shape', pts.shape
        #print 'center.shape', center.shape
        #print 'ut.norm(pts-center).shape',ut.norm(pts-center).shape
        err = ut.norm(pts-center).A1 - rad
        res = np.dot(err,err)
        return res

    params_1 = [x_guess,y_guess,rad_guess]
    if method == 'fmin':
        r = so.fmin(error_function,params_1,xtol=0.0002,ftol=0.000001,full_output=1,disp=verbose)
        opt_params_1,fopt_1 = r[0],r[1]
    elif method == 'fmin_bfgs':
        r = so.fmin_bfgs(error_function,params_1,full_output=1,disp=verbose)
        opt_params_1,fopt_1 = r[0],r[1]
    else:
        raise RuntimeError('unknown method: '+method)

    params_2 = [x_guess,y_guess+2*rad_guess,rad_guess]
    if method == 'fmin':
        r = so.fmin(error_function,params_2,xtol=0.0002,ftol=0.000001,full_output=1,disp=verbose)
        opt_params_2,fopt_2 = r[0],r[1]
    elif method == 'fmin_bfgs':
        r = so.fmin_bfgs(error_function,params_2,full_output=1,disp=verbose)
        opt_params_2,fopt_2 = r[0],r[1]
    else:
        raise RuntimeError('unknown method: '+method)

    if fopt_2<fopt_1:
        return opt_params_2[2],opt_params_2[0],opt_params_2[1]
    else:
        return opt_params_1[2],opt_params_1[0],opt_params_1[1]



if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-f', action='store', type='string', dest='fname',
                 help='pkl file to use.', default='')
    p.add_option('--xy', action='store_true', dest='xy',
                 help='plot the x and y coordinates of the end effector.')
    p.add_option('--yz', action='store_true', dest='yz',
                 help='plot the y and z coordinates of the end effector.')
    p.add_option('--xz', action='store_true', dest='xz',
                 help='plot the x and z coordinates of the end effector.')
    p.add_option('--plot_ellipses', action='store_true', dest='plot_ellipses',
                 help='plot the stiffness ellipse in the x-y plane')
    p.add_option('--pfc', action='store_true', dest='pfc',
                 help='plot the radial and tangential components of the force.')
    p.add_option('--pmf', action='store_true', dest='pmf',
                 help='plot things with the mechanism alinged with the axes.')
    p.add_option('--pff', action='store_true', dest='pff',
                 help='plot the force field corresponding to a stiffness ellipse.')
    p.add_option('--pev', action='store_true', dest='pev',
                 help='plot the stiffness ellipses for different combinations of the rel stiffnesses.')
    p.add_option('--plot_forces', action='store_true', dest='plot_forces',
                 help='plot the force in the x-y plane')
    p.add_option('--plot_forces_error', action='store_true', dest='plot_forces_error',
                 help='plot the error between the computed and measured (ATI) forces in the x-y plane')
    p.add_option('--xyz', action='store_true', dest='xyz',
                 help='plot in 3d the coordinates of the end effector.')
    p.add_option('-r', action='store', type='float', dest='rad',
                 help='radius of the joint.', default=None)
    p.add_option('--rad_fix', action='store_true', dest='rad_fix',
                 help='do not optimize for the radius.')
    p.add_option('--noshow', action='store_true', dest='noshow',
                 help='do not display the figure (use while saving figures to disk)')
    p.add_option('--exptplot', action='store_true', dest='exptplot',
                 help='put all the graphs of an experiment as subplots.')
    p.add_option('--pwf', action='store_true', dest='pwf',
                 help='plot the workspace at some z coord.')

    opt, args = p.parse_args()
    fname = opt.fname
    xy_flag = opt.xy
    yz_flag = opt.yz
    xz_flag = opt.xz
    plot_forces_flag = opt.plot_forces
    plot_ellipses_flag = opt.plot_ellipses
    plot_forces_error_flag = opt.plot_forces_error
    plot_force_components_flag = opt.pfc
    plot_force_field_flag = opt.pff
    plot_mechanism_frame = opt.pmf
    xyz_flag = opt.xyz
    rad = opt.rad
    show_fig = not(opt.noshow)
    plot_ellipses_vary_flag = opt.pev
    expt_plot = opt.exptplot
    rad_fix = opt.rad_fix
    plot_workspace_flag = opt.pwf


    if plot_workspace_flag:
        compute_workspace_z()
#        hull = compute_workspace(z=-0.22,plot=True)
#        pl.show()

    if plot_ellipses_vary_flag:
        show_fig=False
        i = 0
        ratio_list1 = [0.1,0.3,0.5,0.7,0.9] # coarse search
        ratio_list2 = [0.1,0.3,0.5,0.7,0.9] # coarse search
        ratio_list3 = [0.1,0.3,0.5,0.7,0.9] # coarse search
#        ratio_list1 = [0.7,0.8,0.9,1.0]
#        ratio_list2 = [0.7,0.8,0.9,1.0]
#        ratio_list3 = [0.3,0.4,0.5,0.6,0.7]
#        ratio_list1 = [1.0,2.,3.0]
#        ratio_list2 = [1.,2.,3.]
#        ratio_list3 = [0.3,0.4,0.5,0.6,0.7]

        inv_mean_list,std_list = [],[]
        x_l,y_l,z_l = [],[],[]
        s0 = 0.2
        #s0 = 0.4
        for s1 in ratio_list1:
            for s2 in ratio_list2:
                for s3 in ratio_list3:
                    i += 1
                    s_list = [s0,s1,s2,s3,0.8]
                    #s_list = [s1,s2,s3,s0,0.8]
                    print '################## s_list:', s_list
                    m,s = plot_stiff_ellipse_map(s_list,i)
                    inv_mean_list.append(1./m)
                    std_list.append(s)
                    x_l.append(s1)
                    y_l.append(s2)
                    z_l.append(s3)

        ut.save_pickle({'x_l':x_l,'y_l':y_l,'z_l':z_l,'inv_mean_list':inv_mean_list,'std_list':std_list},
                       'stiff_dict_'+ut.formatted_time()+'.pkl')
        d3m.plot_points(np.matrix([x_l,y_l,z_l]),scalar_list=inv_mean_list,mode='sphere')
        mlab.axes()
        d3m.show()

        sys.exit()

    if fname=='':
        print 'please specify a pkl file (-f option)'
        print 'Exiting...'
        sys.exit()

    d = ut.load_pickle(fname)
    actual_cartesian = joint_to_cartesian(d['actual'])
    eq_cartesian = joint_to_cartesian(d['eq_pt'])

    for p in actual_cartesian.p_list:
        print p[0],p[1],p[2]
    

    if rad != None:
        #rad = 0.39 # lab cabinet recessed.
        #rad = 0.42 # kitchen cabinet
        #rad = 0.80 # lab glass door
        pts_list = actual_cartesian.p_list
        ee_start_pos = pts_list[0]
        x_guess = ee_start_pos[0]
        y_guess = ee_start_pos[1] - rad
        print 'before call to fit_rotary_joint'
        pts_2d = (np.matrix(pts_list).T)[0:2,:]
        t0 = time.time()

        if rad_fix:
            rad_guess = 0.9
        else:
            rad_guess = rad
        rad_fmin,cx,cy = fit_circle(rad_guess,x_guess,y_guess,pts_2d[:,0:-4],method='fmin')
        t1 = time.time()
        rad_opt,cx,cy = fit_circle(rad_guess,x_guess,y_guess,pts_2d[:,0:-4],method='fmin_bfgs')
        t2 = time.time()
        print 'after fit_rotary_joint'
        print 'optimized radius:', rad_opt
        print 'optimized radius fmin:', rad_fmin
        print 'time to bfgs:', t2-t1
        print 'time to fmin:', t1-t0
        
        if rad_fix:
            cx,cy = fit_rotary_joint(rad,x_guess,y_guess,pts_2d[:,0:-4])
        else:
            rad = rad_opt
    

    if plot_mechanism_frame:

        if expt_plot:
            pl.subplot(231)

        # transform points so that the mechanism is in a fixed position.
        start_pt = actual_cartesian.p_list[0]
        x_diff = start_pt[0] - cx
        y_diff = start_pt[1] - cy
        angle = math.atan2(y_diff,x_diff) - math.radians(90)
        rot_mat = tr.Rz(angle)[0:2,0:2]
        translation_mat = np.matrix([cx,cy]).T

        robot_width,robot_length = 0.1,0.2
        robot_x_list = [-robot_width/2,-robot_width/2,robot_width/2,robot_width/2,-robot_width/2]
        robot_y_list = [-robot_length/2,robot_length/2,robot_length/2,-robot_length/2,-robot_length/2]
        robot_mat = rot_mat*(np.matrix([robot_x_list,robot_y_list]) - translation_mat)
        mpu.plot_yx(robot_mat[1,:].A1,robot_mat[0,:].A1,linewidth=2,scatter_size=0,
                    color='k',label='torso')

        pts2d_actual = (np.matrix(actual_cartesian.p_list).T)[0:2]
        pts2d_actual_t = rot_mat *(pts2d_actual -  translation_mat)
        mpu.plot_yx(pts2d_actual_t[1,:].A1,pts2d_actual_t[0,:].A1,scatter_size=20,label='FK')

        end_pt = pts2d_actual_t[:,-1]
        end_angle = tr.angle_within_mod180(math.atan2(end_pt[1,0],end_pt[0,0])-math.radians(90))

        mpu.plot_circle(0,0,rad,0.,end_angle,label='Actual_opt',color='r')
        mpu.plot_radii(0,0,rad,0.,end_angle,interval=math.radians(15),color='r')
        pl.legend(loc='best')
        pl.axis('equal')

        if not(expt_plot):
            str_parts = fname.split('.')
            fig_name = str_parts[0]+'_robot_pose.png'
            pl.savefig(fig_name)
            pl.figure()
        else:
            pl.subplot(232)

        pl.text(0.1,0.15,d['info'])
        pl.text(0.1,0.10,'control: '+d['strategy'])
        pl.text(0.1,0.05,'robot angle: %.2f'%math.degrees(angle))
        pl.text(0.1,0,'optimized radius: %.2f'%rad_opt)
        pl.text(0.1,-0.05,'radius used: %.2f'%rad)
        pl.text(0.1,-0.10,'opening angle: %.2f'%math.degrees(end_angle))
        s_list = d['stiffness'].stiffness_list
        s_scale = d['stiffness'].stiffness_scale
        sl = [min(s*s_scale,1.0) for s in s_list]
        pl.text(0.1,-0.15,'stiffness list: %.2f, %.2f, %.2f, %.2f'%(sl[0],sl[1],sl[2],sl[3]))
        pl.text(0.1,-0.20,'stop condition: '+d['result'])
        time_dict = d['time_dict']
        pl.text(0.1,-0.25,'time to hook: %.2f'%(time_dict['before_hook']-time_dict['before_pull']))
        pl.text(0.1,-0.30,'time to pull: %.2f'%(time_dict['before_pull']-time_dict['after_pull']))

        pl.ylim(-0.45,0.25)
        if not(expt_plot):
            pl.figure()


    if xy_flag:
        st_pt = pts_2d[:,0]
        start_angle = tr.angle_within_mod180(math.atan2(st_pt[1,0]-cy,st_pt[0,0]-cx) - math.radians(90))
        end_pt = pts_2d[:,-1]
        end_angle = tr.angle_within_mod180(math.atan2(end_pt[1,0]-cy,end_pt[0,0]-cx) - math.radians(90))
        
        print 'start_angle, end_angle:', math.degrees(start_angle), math.degrees(end_angle)
        print 'angle through which mechanism turned:', math.degrees(end_angle-start_angle)

        if expt_plot:
            pl.subplot(233)

        plot_cartesian(actual_cartesian,xaxis=0,yaxis=1,color='b',label='End Effector Trajectory')
        plot_cartesian(eq_cartesian, xaxis=0,yaxis=1,color='g',label='Eq Point')
        mpu.plot_circle(cx,cy,rad,start_angle,end_angle,label='Estimated Kinematics',color='r')
#        if rad<0.6:
#            mpu.plot_radii(cx,cy,rad,start_angle,end_angle,interval=math.radians(100),color='r')
#        pl.title(d['info'])
        leg = pl.legend(loc='best')#,handletextsep=0.020,handlelen=0.003,labelspacing=0.003)
        leg.draw_frame(False)

        ax = pl.gca()
        ax.set_xlim(ax.get_xlim()[::-1])
        ax.set_ylim(ax.get_ylim()[::-1])

        force_traj = d['force']
        forces = np.matrix(force_traj.f_list).T
        force_mag = ut.norm(forces)
        print 'force_mag:', force_mag.A1

    elif yz_flag:
        plot_cartesian(actual_cartesian,xaxis=1,yaxis=2,color='b',label='FK')
        plot_cartesian(eq_cartesian, xaxis=1,yaxis=2,color='g',label='Eq Point')
    elif xz_flag:
        plot_cartesian(actual_cartesian,xaxis=0,yaxis=2,color='b',label='FK')
        plot_cartesian(eq_cartesian, xaxis=0,yaxis=2,color='g',label='Eq Point')


    if plot_forces_flag or plot_forces_error_flag or plot_ellipses_flag or plot_force_components_flag or plot_force_field_flag:
        arm_stiffness_list = d['stiffness'].stiffness_list
        scale = d['stiffness'].stiffness_scale
        asl = [min(scale*s,1.0) for s in arm_stiffness_list]
        ftraj_jinv,ftraj_stiff,ftraj_torque,k_cart_list=compute_forces(d['actual'],d['eq_pt'],
                                                                       d['torque'],asl)
        if plot_forces_flag:
            plot_forces_quiver(actual_cartesian,d['force'],color='k')
            plot_forces_quiver(actual_cartesian,ftraj_jinv,color='y')
            #plot_forces_quiver(actual_cartesian,ftraj_stiff,color='y')

        if plot_ellipses_flag:
            #plot_stiff_ellipses(k_cart_list,actual_cartesian)
            if expt_plot:
                subplotnum=234
            else:
                pl.figure()
                subplotnum=111
            plot_stiff_ellipses(k_cart_list,eq_cartesian,subplotnum=subplotnum)

        if plot_forces_error_flag:
            plot_error_forces(d['force'].f_list,ftraj_jinv.f_list)
            #plot_error_forces(d['force'].f_list,ftraj_stiff.f_list)

        if plot_force_components_flag:
            p_list = actual_cartesian.p_list
            frad_list,ftan_list = compute_radial_tangential_forces(d['force'].f_list,p_list,cx,cy)
            if expt_plot:
                pl.subplot(235)
            else:
                pl.figure()

            time_list = d['force'].time_list
            time_list = [t-time_list[0] for t in time_list]
            x_coord_list = np.matrix(p_list)[:,0].A1.tolist()
            mpu.plot_yx(frad_list,x_coord_list,scatter_size=50,color=time_list,cb_label='time')
            pl.xlabel('x coord of end effector (m)')
            pl.ylabel('magnitude of radial force (N)')
            pl.title(d['info'])
            if expt_plot:
                pl.subplot(236)
            else:
                pl.figure()
            mpu.plot_yx(ftan_list,x_coord_list,scatter_size=50,color=time_list,cb_label='time')
            pl.xlabel('x coord of end effector (m)')
            pl.ylabel('magnitude of tangential force (N)')
            pl.title(d['info'])

        if plot_force_field_flag:
            plot_stiffness_field(k_cart_list[0],plottitle='start')
            plot_stiffness_field(k_cart_list[-1],plottitle='end')


    if expt_plot:
        f = pl.gcf()
        curr_size = f.get_size_inches()
        f.set_size_inches(curr_size[0]*2,curr_size[1]*2)
        str_parts = fname.split('.')
        if d.has_key('strategy'):
            fig_name = str_parts[0]+'_'+d['strategy']+'.png'
        else:
            fig_name = str_parts[0]+'_res.png'
        f.savefig(fig_name)

    if show_fig:
        pl.show()
    else:
        print '################################'
        print 'show_fig is FALSE'

    if xyz_flag:
        plot_cartesian(traj, xaxis=0,yaxis=1,zaxis=2)
        mlab.show()




