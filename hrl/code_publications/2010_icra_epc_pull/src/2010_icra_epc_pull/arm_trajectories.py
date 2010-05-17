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


import scipy.optimize as so

import math, numpy as np
import pylab as pl
import sys, optparse, time
import copy

from enthought.mayavi import mlab

import mekabot.hrl_robot as hr
import mekabot.coord_frames as mcf
import matplotlib_util.util as mpu

#import util as ut
import roslib; roslib.load_manifest('2010_icra_epc_pull')
import hrl_lib.util as ut, hrl_lib.transforms as tr
import hrl_tilting_hokuyo.display_3d_mayavi as d3m

import segway_motion_calc as smc

class JointTrajectory():
    ''' class to store joint trajectories.
        data only - use for pickling.
    '''
    def __init__(self):
        self.time_list = [] # time in seconds
        self.q_list = [] #each element is a list of 7 joint angles.
        self.qdot_list = [] #each element is a list of 7 joint angles.
        self.qdotdot_list = [] #each element is a list of 7 joint angles.

## class to store trajectory of a coord frame executing planar motion (x,y,a)
#data only - use for pickling
class PlanarTajectory():
    def __init__(self):
        self.time_list = [] # time in seconds
        self.x_list = []
        self.y_list = []
        self.a_list = []

class CartesianTajectory():
    ''' class to store trajectory of cartesian points.
        data only - use for pickling
    '''
    def __init__(self):
        self.time_list = [] # time in seconds
        self.p_list = [] #each element is a list of 3 coordinates
        self.v_list = [] #each element is a list of 3 coordinates (velocity)

class ForceTrajectory():
    ''' class to store time evolution of the force at the end effector.
        data only - use for pickling
    '''
    def __init__(self):
        self.time_list = [] # time in seconds
        self.f_list = [] #each element is a list of 3 coordinates

##
# @param traj - JointTrajectory
# @return CartesianTajectory after performing FK on traj to compute
# cartesian position, velocity
def joint_to_cartesian(traj):
    firenze = hr.M3HrlRobot(connect=False)
    pts = []
    cart_vel = []
    for i in range(len(traj.q_list)):
        q = traj.q_list[i]
        p = firenze.FK('right_arm', q)
        pts.append(p.A1.tolist())

        if traj.qdot_list != []:
            qdot = traj.qdot_list[i]
            jac = firenze.Jac('right_arm', q)
            vel = jac * np.matrix(qdot).T
            cart_vel.append(vel.A1[0:3].tolist())

    ct = CartesianTajectory()
    ct.time_list = copy.copy(traj.time_list)
    ct.p_list = copy.copy(pts)
    ct.v_list = copy.copy(cart_vel)
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

##
# @param xaxis - x axis for the graph (0,1 or 2)
# @param zaxis - for a 3d plot. not implemented.
def plot_cartesian(traj, xaxis=None, yaxis=None, zaxis=None, color='b',label='_nolegend_',
                   linewidth=2, scatter_size=10, plot_velocity=False):

    import arm_trajectories as at
    #if traj.__class__ == at.JointTrajectory:
    if isinstance(traj,at.JointTrajectory):
        traj = joint_to_cartesian(traj)

    pts = np.matrix(traj.p_list).T
    label_list = ['X coord (m)', 'Y coord (m)', 'Z coord (m)']
    x = pts[xaxis,:].A1.tolist()
    y = pts[yaxis,:].A1.tolist()

    if plot_velocity:
        vels = np.matrix(traj.v_list).T
        xvel = vels[xaxis,:].A1.tolist()
        yvel = vels[yaxis,:].A1.tolist()

    if zaxis == None:
        mpu.plot_yx(y, x, color, linewidth, '-', scatter_size, label,
                    axis = 'equal', xlabel = label_list[xaxis],
                    ylabel = label_list[yaxis],)
        if plot_velocity:
            mpu.plot_quiver_yxv(y, x, np.matrix([xvel,yvel]),
                                width = 0.001, scale = 1.)
        mpu.legend()
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
        r = so.fmin_bfgs(error_function, params_1, full_output=1,
                         disp = verbose, gtol=1e-5)
        opt_params_1,fopt_1 = r[0],r[1]
    else:
        raise RuntimeError('unknown method: '+method)

    params_2 = [x_guess,y_guess+2*rad_guess,rad_guess]
    if method == 'fmin':
        r = so.fmin(error_function,params_2,xtol=0.0002,ftol=0.000001,full_output=1,disp=verbose)
        opt_params_2,fopt_2 = r[0],r[1]
    elif method == 'fmin_bfgs':
        r = so.fmin_bfgs(error_function, params_2, full_output=1,
                         disp = verbose, gtol=1e-5)
        opt_params_2,fopt_2 = r[0],r[1]
    else:
        raise RuntimeError('unknown method: '+method)

    if fopt_2<fopt_1:
        return opt_params_2[2],opt_params_2[0],opt_params_2[1]
    else:
        return opt_params_1[2],opt_params_1[0],opt_params_1[1]


## changes the cartesian trajectory to put everything in the same frame.
# NOTE - velocity transformation does not work if the segway is also
# moving. This is because I am not logging the velocity of the segway.
# @param pts - CartesianTajectory
# @param st - object of type PlanarTajectory (segway trajectory)
# @return CartesianTajectory
def account_segway_motion(cart_traj,st):
    ct = CartesianTajectory()
    for i in range(len(cart_traj.p_list)):
        x,y,a = st.x_list[i], st.y_list[i], st.a_list[i]
        p_tl = np.matrix(cart_traj.p_list[i]).T
        p_ts = smc.tsTtl(p_tl, x, y, a)
        p = p_ts
        ct.p_list.append(p.A1.tolist())

        # this is incorrect. I also need to use the velocity of the
        # segway. Unclear whether this is useful right now, so not
        # implementing it. (Advait. Jan 6, 2010.)
        if cart_traj.v_list != []:
            v_tl = np.matrix(cart_traj.v_list[i]).T
            v_ts = smc.tsRtl(v_tl, a)
            ct.v_list.append(v_ts.A1.tolist())

    ct.time_list = copy.copy(cart_traj.time_list)
    return ct

# @param cart_traj - CartesianTajectory
# @param z_l - list of zenither heights
# @return CartesianTajectory
def account_zenithering(cart_traj, z_l):
    ct = CartesianTajectory()
    h_start = z_l[0]
    for i in range(len(cart_traj.p_list)):
        h = z_l[i]
        p = cart_traj.p_list[i]
        p[2] += h - h_start
        ct.p_list.append(p)

        # this is incorrect. I also need to use the velocity of the
        # zenither. Unclear whether this is useful right now, so not
        # implementing it. (Advait. Jan 6, 2010.)
        if cart_traj.v_list != []:
            ct.v_list.append(cart_traj.v_list[i])

    ct.time_list = copy.copy(cart_traj.time_list)
    return ct

##
# remove the parts of the trjectory in which the hook is not moving.
# @param ct - cartesian trajectory of the end effector in the world frame.
# @return 2xN np matrix, reject_idx
def filter_cartesian_trajectory(ct):
    pts_list = ct.p_list
    ee_start_pos = pts_list[0]
    l = [pts_list[0]]

    for i, p in enumerate(pts_list[1:]):
        l.append(p)
        pts_2d = (np.matrix(l).T)[0:2,:]
        st_pt = pts_2d[:,0]
        end_pt = pts_2d[:,-1]
        dist_moved = np.linalg.norm(st_pt-end_pt)
        #if dist_moved < 0.1:
        if dist_moved < 0.03:
            reject_idx = i

    pts_2d = pts_2d[:,reject_idx:]
    return pts_2d, reject_idx

##
# remove the parts of the trjectory in which the hook slipped off
# @param ct - cartesian trajectory of the end effector in the world frame.
# @param ft - force trajectory
# @return cartesian trajectory with the zero force end part removed, force trajectory
def filter_trajectory_force(ct, ft):
    vel_list = copy.copy(ct.v_list)
    pts_list = copy.copy(ct.p_list)
    time_list = copy.copy(ct.time_list)
    ft_list = copy.copy(ft.f_list)
    f_mag_list = ut.norm(np.matrix(ft.f_list).T).A1.tolist()

    if len(pts_list) != len(f_mag_list):
        print 'arm_trajectories.filter_trajectory_force: force and end effector lists are not of the same length.'
        print 'Exiting ...'
        sys.exit()

    n_pts = len(pts_list)
    i = n_pts - 1
    hook_slip_off_threshold = 1.5 # from compliant_trajectories.py
    while i > 0:
        if f_mag_list[i] < hook_slip_off_threshold:
            pts_list.pop()
            time_list.pop()
            ft_list.pop()
            if vel_list != []:
                vel_list.pop()
        else:
            break
        i -= 1

    ct2 = CartesianTajectory()
    ct2.time_list = time_list
    ct2.p_list = pts_list
    ct2.v_list = vel_list

    ft2 = ForceTrajectory()
    ft2.time_list = copy.copy(time_list)
    ft2.f_list = ft_list
    return ct2, ft2


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
    p.add_option('--noshow', action='store_true', dest='noshow',
                 help='do not display the figure (use while saving figures to disk)')
    p.add_option('--exptplot', action='store_true', dest='exptplot',
                 help='put all the graphs of an experiment as subplots.')
    p.add_option('--sturm', action='store_true', dest='sturm',
                 help='make log files to send to sturm')
    p.add_option('--icra_presentation_plot', action='store_true',
                 dest='icra_presentation_plot',
                 help='plot explaining CEP update.')

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
    sturm_output = opt.sturm


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
    actual_cartesian_tl = joint_to_cartesian(d['actual'])
    actual_cartesian = account_segway_motion(actual_cartesian_tl,d['segway'])
    if d.has_key('zenither_list'):
        actual_cartesian = account_zenithering(actual_cartesian,
                                               d['zenither_list'])

    eq_cartesian_tl = joint_to_cartesian(d['eq_pt'])
    eq_cartesian = account_segway_motion(eq_cartesian_tl, d['segway'])
    if d.has_key('zenither_list'):
        eq_cartesian = account_zenithering(eq_cartesian, d['zenither_list'])

    cartesian_force_clean, _ = filter_trajectory_force(actual_cartesian,
                                                       d['force'])
    pts_2d, reject_idx = filter_cartesian_trajectory(cartesian_force_clean)

    if rad != None:
        #rad = 0.39 # lab cabinet recessed.
        #rad = 0.42 # kitchen cabinet
        #rad = 0.80 # lab glass door
        pts_list = actual_cartesian.p_list
        eq_pts_list = eq_cartesian.p_list
        ee_start_pos = pts_list[0]
        x_guess = ee_start_pos[0]
        y_guess = ee_start_pos[1] - rad
        print 'before call to fit_rotary_joint'
        force_list = d['force'].f_list

        if sturm_output:
            str_parts = fname.split('.')
            sturm_file_name = str_parts[0]+'_sturm.log'
            print 'Sturm file name:', sturm_file_name
            sturm_file = open(sturm_file_name,'w')
            sturm_pts = cartesian_force_clean.p_list
            print 'len(sturm_pts):', len(sturm_pts)
            print 'len(pts_list):', len(pts_list)

            for i,p in enumerate(sturm_pts[1:]):
                sturm_file.write(" ".join(map(str,p)))
                sturm_file.write('\n')

            sturm_file.write('\n')
            sturm_file.close()

        rad_guess = rad
        rad, cx, cy = fit_circle(rad_guess,x_guess,y_guess,pts_2d,
                                 method='fmin_bfgs',verbose=False)
        c_ts = np.matrix([cx, cy, 0.]).T
        start_angle = tr.angle_within_mod180(math.atan2(pts_2d[0,1]-cy,
                               pts_2d[0,0]-cx) - math.pi/2)
        end_angle = tr.angle_within_mod180(math.atan2(pts_2d[-1,1]-cy,
                               pts_2d[-1,0]-cx) - math.pi/2)
        mpu.plot_circle(cx, cy, rad, start_angle, end_angle,
                        label='Actual\_opt', color='r')


    if opt.icra_presentation_plot:
        mpu.set_figure_size(30,20)
        rad = 1.0
        x_guess = pts_2d[0,0]
        y_guess = pts_2d[1,0] - rad

        rad_guess = rad
        rad, cx, cy = fit_circle(rad_guess,x_guess,y_guess,pts_2d,
                                 method='fmin_bfgs',verbose=False)
        print 'Estimated rad, cx, cy:', rad, cx, cy

        start_angle = tr.angle_within_mod180(math.atan2(pts_2d[1,0]-cy,
                               pts_2d[0,0]-cx) - math.pi/2)
        end_angle = tr.angle_within_mod180(math.atan2(pts_2d[1,-1]-cy,
                               pts_2d[0,-1]-cx) - math.pi/2)

        subsample_ratio = 1
        pts_2d_s = pts_2d[:,::subsample_ratio]

        cep_force_clean, force_new = filter_trajectory_force(eq_cartesian,
                                                             d['force'])
        cep_2d = np.matrix(cep_force_clean.p_list).T[0:2,reject_idx:]

        # first draw the entire CEP and end effector trajectories
        mpu.figure()
        mpu.plot_yx(pts_2d_s[1,:].A1, pts_2d_s[0,:].A1, color='b',
                    label = 'FK', axis = 'equal', alpha = 1.0,
                    scatter_size=7, linewidth=0, marker='x',
                    marker_edge_width = 1.5)

        cep_2d_s = cep_2d[:,::subsample_ratio]
        mpu.plot_yx(cep_2d_s[1,:].A1, cep_2d_s[0,:].A1, color='g',
                    label = 'CEP', axis = 'equal', alpha = 1.0,
                    scatter_size=10, linewidth=0, marker='+',
                    marker_edge_width = 1.5)

        mpu.plot_circle(cx, cy, rad, start_angle, end_angle,
                        label='Estimated Kinematics', color='r',
                        alpha=0.7)
        mpu.plot_radii(cx, cy, rad, start_angle, end_angle,
                       interval=end_angle-start_angle, color='r',
                       alpha=0.7)
        mpu.legend()
        mpu.savefig('one.png')

        # now zoom in to a small region to show the force
        # decomposition.
        zoom_location = 10
        pts_2d_zoom = pts_2d[:,:zoom_location]
        cep_2d_zoom = cep_2d[:,:zoom_location]

        mpu.figure()
        mpu.plot_yx(pts_2d_zoom[1,:].A1, pts_2d_zoom[0,:].A1, color='b',
                    label = 'FK', axis = 'equal', alpha = 1.0,
                    scatter_size=7, linewidth=0, marker='x',
                    marker_edge_width = 1.5)
        mpu.plot_yx(cep_2d_zoom[1,:].A1, cep_2d_zoom[0,:].A1, color='g',
                    label = 'CEP', axis = 'equal', alpha = 1.0,
                    scatter_size=10, linewidth=0, marker='+',
                    marker_edge_width = 1.5)
        mpu.pl.xlim(0.28, 0.47)
        mpu.legend()
        mpu.savefig('two.png')

        rad, cx, cy = fit_circle(1.0,x_guess,y_guess,pts_2d_zoom,
                                 method='fmin_bfgs',verbose=False)
        print 'Estimated rad, cx, cy:', rad, cx, cy
        start_angle = tr.angle_within_mod180(math.atan2(pts_2d[1,0]-cy,
                               pts_2d[0,0]-cx) - math.pi/2)
        end_angle = tr.angle_within_mod180(math.atan2(pts_2d_zoom[1,-1]-cy,
                               pts_2d_zoom[0,-1]-cx) - math.pi/2)
        mpu.plot_circle(cx, cy, rad, start_angle, end_angle,
                        label='Estimated Kinematics', color='r',
                        alpha=0.7)
        mpu.pl.xlim(0.28, 0.47)
        mpu.legend()
        mpu.savefig('three.png')

        current_pos = pts_2d_zoom[:,-1]
        radial_vec = current_pos - np.matrix([cx,cy]).T
        radial_vec = radial_vec / np.linalg.norm(radial_vec)
        tangential_vec = np.matrix([[0,-1],[1,0]]) * radial_vec
        mpu.plot_quiver_yxv([pts_2d_zoom[1,-1]],
                            [pts_2d_zoom[0,-1]],
                            radial_vec, scale=10., width = 0.002)
        rad_text_loc = pts_2d_zoom[:,-1] + np.matrix([0.001,0.01]).T
        mpu.pl.text(rad_text_loc[0,0], rad_text_loc[1,0], '\huge{$\hat v_{rad}$}')
        mpu.plot_quiver_yxv([pts_2d_zoom[1,-1]],
                            [pts_2d_zoom[0,-1]],
                            tangential_vec, scale=10., width = 0.002)

        tan_text_loc = pts_2d_zoom[:,-1] + np.matrix([-0.012, -0.011]).T
        mpu.pl.text(tan_text_loc[0,0], tan_text_loc[1,0], s = '\huge{$\hat v_{tan}$}')
        mpu.pl.xlim(0.28, 0.47)
        mpu.legend()
        mpu.savefig('four.png')

        wrist_force = -np.matrix(force_new.f_list[zoom_location]).T
        frad = (wrist_force[0:2,:].T * radial_vec)[0,0] * radial_vec
        mpu.plot_quiver_yxv([pts_2d_zoom[1,-1]],
                            [pts_2d_zoom[0,-1]],
                            wrist_force, scale=50., width = 0.002,
                            color='y')
        wf_text = rad_text_loc + np.matrix([-0.05,0.015]).T
        mpu.pl.text(wf_text[0,0], wf_text[1,0], color='y',
                    fontsize = 15, s = 'Wrist Force')

        mpu.plot_quiver_yxv([pts_2d_zoom[1,-1]],
                            [pts_2d_zoom[0,-1]],
                            frad, scale=50., width = 0.002,
                            color='y')
        frad_text = rad_text_loc + np.matrix([0.,0.015]).T
        mpu.pl.text(frad_text[0,0], frad_text[1,0], color='y', s = '\huge{$\hat F_{rad}$}')

        mpu.pl.xlim(0.28, 0.47)
        mpu.legend()
        mpu.savefig('five.png')

        frad = (wrist_force[0:2,:].T * radial_vec)[0,0]
        hook_force_motion = -(frad - 5) * radial_vec * 0.001
        tangential_motion = 0.01 * tangential_vec
        total_cep_motion = hook_force_motion + tangential_motion

        mpu.plot_quiver_yxv([cep_2d_zoom[1,-1]],
                            [cep_2d_zoom[0,-1]],
                            hook_force_motion, scale=0.1, width = 0.002)
        hw_text = cep_2d_zoom[:,-1] + np.matrix([-0.002,-0.012]).T
        mpu.pl.text(hw_text[0,0], hw_text[1,0], color='k', fontsize=14,
                    s = '$h[t]$ = $0.1cm/N \cdot (|\hat{F}_{rad}|-5N) \cdot \hat{v}_{rad}$')
        mpu.pl.xlim(0.28, 0.47)
        mpu.legend()
        mpu.savefig('six.png')

        mpu.plot_quiver_yxv([cep_2d_zoom[1,-1]],
                            [cep_2d_zoom[0,-1]],
                            tangential_motion, scale=0.1, width = 0.002)
        mw_text = cep_2d_zoom[:,-1] + np.matrix([-0.038,0.001]).T
        mpu.pl.text(mw_text[0,0], mw_text[1,0], color='k', fontsize=14,
                    s = '$m[t]$ = $1cm \cdot \hat{v}_{tan}$')
        mpu.pl.xlim(0.28, 0.47)
        mpu.legend()
        mpu.savefig('seven.png')

        mpu.plot_quiver_yxv([cep_2d_zoom[1,-1]],
                            [cep_2d_zoom[0,-1]],
                            total_cep_motion, scale=0.1, width = 0.002)
        cep_text = cep_2d_zoom[:,-1] + np.matrix([-0.058,-0.013]).T
        mpu.pl.text(cep_text[0,0], cep_text[1,0], color='k', fontsize=14,
                    s = '$x_{eq}[t]$ = &x_{eq}[t-1] + m[t] + h[t]$')
        mpu.pl.xlim(0.28, 0.47)
        mpu.legend()
        mpu.savefig('eight.png')

        new_cep = cep_2d_zoom[:,-1] + total_cep_motion
        mpu.plot_yx(new_cep[1,:].A1, new_cep[0,:].A1, color='g',
                    axis = 'equal', alpha = 1.0,
                    scatter_size=10, linewidth=0, marker='+',
                    marker_edge_width = 1.5)
        mpu.pl.xlim(0.28, 0.47)
        mpu.legend()
        mpu.savefig('nine.png')

        #mpu.plot_radii(cx, cy, rad, start_angle, end_angle,
        #               interval=end_angle-start_angle, color='r',
        #               alpha=0.7)




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
                    color='k',label='torso', axis='equal')

        pts2d_actual = (np.matrix(actual_cartesian.p_list).T)[0:2]
        pts2d_actual_t = rot_mat *(pts2d_actual -  translation_mat)
        mpu.plot_yx(pts2d_actual_t[1,:].A1,pts2d_actual_t[0,:].A1,scatter_size=20,label='FK',
                    axis = 'equal')

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
        end_pt = pts_2d[:,-1]

#        if rad != None:
#            start_angle = tr.angle_within_mod180(math.atan2(st_pt[1,0]-cy,st_pt[0,0]-cx) - math.radians(90))
#            end_angle = tr.angle_within_mod180(math.atan2(end_pt[1,0]-cy,end_pt[0,0]-cx) - math.radians(90))
#            
#            print 'start_angle, end_angle:', math.degrees(start_angle), math.degrees(end_angle)
#            print 'angle through which mechanism turned:', math.degrees(end_angle-start_angle)

        if expt_plot:
            pl.subplot(233)

        plot_cartesian(actual_cartesian, xaxis=0, yaxis=1, color='b',
                       label='FK', plot_velocity=False)
        plot_cartesian(eq_cartesian, xaxis=0,yaxis=1,color='g',label='Eq Point')
        #leg = pl.legend(loc='best',handletextsep=0.020,handlelen=0.003,labelspacing=0.003)
        #leg.draw_frame(False)

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
            #plot_forces_quiver(actual_cartesian,ftraj_jinv,color='y')
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
            cx = 45.
            cy = -0.3
            frad_list,ftan_list = compute_radial_tangential_forces(d['force'].f_list,p_list,cx,cy)
            if expt_plot:
                pl.subplot(235)
            else:
                pl.figure()

            time_list = d['force'].time_list
            time_list = [t-time_list[0] for t in time_list]
            x_coord_list = np.matrix(p_list)[:,0].A1.tolist()
            mpu.plot_yx(frad_list,x_coord_list,scatter_size=50,color=time_list,cb_label='time',axis=None)
            pl.xlabel('x coord of end effector (m)')
            pl.ylabel('magnitude of radial force (N)')
            pl.title(d['info'])
            if expt_plot:
                pl.subplot(236)
            else:
                pl.figure()
            mpu.plot_yx(ftan_list,x_coord_list,scatter_size=50,color=time_list,cb_label='time',axis=None)
            pl.xlabel('x coord of end effector (m)')
            pl.ylabel('magnitude of tangential force (N)')
            pl.title(d['info'])

        if plot_force_field_flag:
            plot_stiffness_field(k_cart_list[0],plottitle='start')
            plot_stiffness_field(k_cart_list[-1],plottitle='end')


    str_parts = fname.split('.')
    if d.has_key('strategy'):
        addon = ''
        if opt.xy:
            addon = '_xy'
        if opt.xz:
            addon = '_xz'
        fig_name = str_parts[0]+'_'+d['strategy']+addon+'.png'
    else:
        fig_name = str_parts[0]+'_res.png'

    if expt_plot:
        f = pl.gcf()
        curr_size = f.get_size_inches()
        f.set_size_inches(curr_size[0]*2,curr_size[1]*2)
        f.savefig(fig_name)

    if show_fig:
        pl.show()
    else:
        print '################################'
        print 'show_fig is FALSE'
        if not(expt_plot):
            pl.savefig(fig_name)

    if xyz_flag:
        plot_cartesian(traj, xaxis=0,yaxis=1,zaxis=2)
        mlab.show()







