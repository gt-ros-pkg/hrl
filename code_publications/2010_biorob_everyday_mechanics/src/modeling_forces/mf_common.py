import roslib; roslib.load_manifest('modeling_forces')
import os, sys

#sys.path.append(os.environ['HRLBASEPATH'] + '/src/projects/equilibrium_point_control/src/equilibrium_point_control')
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.data_process as dp
import hrl_lib.data_display as dd
#import equilibrium_point_control.arm_trajectories as at
import numpy as np
import pylab as pb
import math
import copy
import pdb
import smooth
from scipy import interpolate

filter = dp.filter
gradient = dp.gradient
interpolate_1d = dp.interpolate_1d
histogram_get_bin_numb = dp.histogram_get_bin_numb
histogram = dp.histogram
equalize_times = dp.equalize_times
random_color = dd.random_color
#filter = ut.filter
#gradient = ut.gradient
#interpolate_1d = ut.interpolate_1d
#histogram_get_bin_numb = ut.histogram_get_bin_numb
#histogram = ut.histogram
#equalize_times = ut.equalize_times

##
# shortens x by 2. and returns dx/dt
def gradient(t, x):
    dx = x[:, 2:] - x[:, 0:-2]
    dt = t[0, 2:] - t[0, 0:-2]
    dx_dt = np.multiply(dx, 1/dt)

#    dx_dt = np.column_stack((dx_dt[:,0], dx_dt))
#    dx_dt = np.column_stack((dx_dt, dx_dt[:,-1]))
    return dx_dt
##
# Moved to hrl_lib
#def filter(alist, indices):
#    rlist = []
#    for i in indices:
#        rlist.append(alist[i])
#    return rlist
#
#def gradient(t, x):
#    #pdb.set_trace()
#    dx = x[:, 2:] - x[:, 0:-2]
#    dt = t[0, 2:] - t[0, 0:-2]
#    dx_dt = np.multiply(dx, 1/dt)
#    #pdb.set_trace()
#    dx_dt = np.column_stack((dx_dt[:,0], dx_dt))
#    dx_dt = np.column_stack((dx_dt, dx_dt[:,-1]))
#    return dx_dt
##

def velocity(kin_info, smooth_window):
    mech_time_mat = np.matrix(kin_info['mech_time_arr'])
    tstep_size = .03333333333
    num_el = mech_time_mat.shape[1]
    uniform_time2 = np.cumsum(np.round((mech_time_mat[0,1:] - mech_time_mat[0,0:-1]) / tstep_size) * tstep_size)
    uniform_time2 = np.column_stack((np.matrix([0]), uniform_time2))

    mech_intrinsic_poses = kin_info['disp_mech_coord_arr']
    if uniform_time2.shape[1] != mech_intrinsic_poses.shape[0]:
        pdb.set_trace()

    vel = gradient(uniform_time2, np.matrix(mech_intrinsic_poses))
    return smooth.smooth(vel.A1, smooth_window, 'blackman')

##
# computes vel and acc for the mechanism
# returns lists.
def kinematic_params(mech_x_l, time_list, smooth_window):
    mech_time_mat = np.matrix(time_list)
    tstep_size = .03333333333
    num_el = mech_time_mat.shape[1]
    uniform_time2 = np.cumsum(np.round((mech_time_mat[0,1:] - mech_time_mat[0,0:-1]) / tstep_size) * tstep_size)
    uniform_time2 = np.column_stack((np.matrix([0]), uniform_time2))

    mech_x_mat = np.matrix(mech_x_l)
    if uniform_time2.shape[1] != mech_x_mat.shape[1]:
        pdb.set_trace()

    mech_x_mat = np.matrix(smooth.smooth(mech_x_mat.A1, smooth_window,
                                         'blackman'))
    uniform_time2 = uniform_time2[:,smooth_window-1:-smooth_window+1]

    vel = gradient(uniform_time2, mech_x_mat)
    uniform_time2 = uniform_time2[:,1:-1]

    vel = np.matrix(smooth.smooth(vel.A1, smooth_window, 'blackman'))
    mech_x_mat = mech_x_mat[:,smooth_window-1:-smooth_window+1]
    uniform_time2 = uniform_time2[:,smooth_window-1:-smooth_window+1]

    acc = gradient(uniform_time2, vel)
    uniform_time2 = uniform_time2[:,1:-1]
    vel = vel[:,1:-1]
    mech_x_mat = mech_x_mat[:,2:-2]

    acc = np.matrix(smooth.smooth(acc.A1, smooth_window, 'blackman'))
    vel = vel[:,smooth_window-1:-smooth_window+1]
    mech_x_mat = mech_x_mat[:,smooth_window-1:-smooth_window+1]
    uniform_time2 = uniform_time2[:,smooth_window-1:-smooth_window+1]

    return mech_x_mat.A1.tolist(), vel.A1.tolist(), acc.A1.tolist(), uniform_time2.A1.tolist()

def interpolate_1d(x, y, xquery):
    try:
        minx = np.min(x)
        minx_query = np.min(xquery)

        maxx = np.max(x)
        maxx_querry = np.max(xquery)

        if minx_query <= minx:
            x = np.concatenate((np.array([minx_query-.01]), x))
            y = np.concatenate((np.array([y[0]]), y))

        if maxx <= maxx_querry:
            x = np.concatenate((x, np.array([maxx_querry+.01])))
            y = np.concatenate((y, np.array([y[-1]])))

        f = interpolate.interp1d(x, y)
        return f(xquery)
    except ValueError, e:
        pdb.set_trace()
        print e

def calculate_derived_quantities(ler_result, smooth_window):
    exp_records = ler_result['records']
    mech_info = ler_result['mech_info']
    kin_info_list = ler_result['kin_info_list']
    
    #Calculate velocities
    for kin_info in kin_info_list:
        kin_info['velocity'] = velocity(kin_info, smooth_window)
    
    #Interpolate kinematics into force time steps
    for segment_name in exp_records.keys():
        for exp_number, record in enumerate(exp_records[segment_name]):
            record['mech_coord_arr'] = interpolate_1d(kin_info_list[exp_number]['mech_time_arr'], 
                                               kin_info_list[exp_number]['disp_mech_coord_arr'], 
                                               record['ftimes'])
            record['mech_pose_vel_arr'] = interpolate_1d(kin_info_list[exp_number]['mech_time_arr'],
                                               kin_info_list[exp_number]['velocity'],
                                               record['ftimes'])
            record['force_norms'] = ut.norm(np.matrix(record['forces']).T[0:3,:]).A1

#def histogram_get_bin_numb(n, min_index, bin_size, nbins):
#    bin_numb = int(np.floor((n - min_index) / bin_size))
#    if bin_numb == nbins:
#        bin_numb = bin_numb - 1
#    return bin_numb
#
#def histogram(index_list_list, elements_list_list, bin_size, min_index=None, max_index=None):
#    if min_index is None:
#        min_index = np.min(np.concatenate(index_list_list))
#    if max_index is None:
#        max_index = np.max(np.concatenate(index_list_list))
#
#    index_range = (max_index - min_index) 
#    nbins = int(np.ceil(index_range / bin_size))
#    bins = []
#    for i in range(nbins):
#        bins.append([])
#
#    #pdb.set_trace()
#
#    #Each slice contains the data for one trial, idx is the trial number
#    for trial_number, element_list_slice in enumerate(zip(*elements_list_list)):
#        #Iterate by using the length of the first set of data in the given trial
#        for i in range(len(element_list_slice[0])):
#            bin_numb = histogram_get_bin_numb(index_list_list[trial_number][i], min_index, bin_size, nbins)
#            elements = [el_list[i] for el_list in element_list_slice]
#            if bin_numb < 0 or bin_numb > nbins:
#                continue
#            bins[bin_numb].append(elements)
#
#    return bins, np.arange(min_index, max_index, bin_size)

def segment_as_dict(segments):
    a = {}
    for s in segments:
        s, e, name = s
        a[name] = [s,e]
    return a

###
## given a list of 1d time arrays, find the sequence that started first and
## subtract all sequences from its first time recording
#def equalize_times(list_of_time_arrays):
#    start_times = []
#    end_times = []
#    for tarray in list_of_time_arrays:
#        start_times.append(tarray[0])
#        end_times.append(tarray[-1])
#
#    min_start = np.min(start_times)
#    max_end = np.max(end_times)
#
#    adjusted_list_of_time_arrays = []
#    for tarray in list_of_time_arrays:
#        adjusted_list_of_time_arrays.append(tarray - min_start)
#
#    return adjusted_list_of_time_arrays, min_start, max_end

##
#
# @param mechanism_rotations list of 3x3 rotation matrices
def rotary_angles(mechanism_rotations, radius):
    directions_x = (np.row_stack(mechanism_rotations)[:,0]).T.reshape(len(mechanism_rotations), 3).T
    start_normal = directions_x[:,0]
    ra = np.arccos(start_normal.T * directions_x).A1
    if np.any(np.isnan(ra)):
        ra[np.where(np.isnan(ra))] = 0.0
    #.tolist()
    #ra_arr = np.array(ra)
    return ra.tolist()

##
# @param mechanism_positions list of 3x1 vectors
def linear_distances(mechanism_positions):
    pos_mat = np.column_stack(mechanism_positions)
    ld = ut.norm(pos_mat - pos_mat[:,0]).A1.tolist()
    ld_arr = np.array(ld)
    if np.any(np.isnan(ld_arr)):
        pdb.set_trace()
    return ld

##
# Transform a force reading to camera coord frame.
#
# @param hand_rot_matrix - rotation matrix for camera to hand checker.
# @param number - checkerboard number (1, 2, 3 or 4)
def ft_to_camera(force_tool, hand_rot_matrix, number):
    # hc == hand checkerboard
    hc_rot_tool = tr.Rx(math.radians(90)) * tr.Ry(math.radians(180.)) * tr.Rz(math.radians(30.))

    while number != 1:
        hc_rot_tool = tr.Ry(math.radians(90.)) * hc_rot_tool
        number = number-1

    force_hc = hc_rot_tool * force_tool
    return hand_rot_matrix * force_hc

##
# Transform force readings to camera coord frame.
#
def fts_to_camera(combined_dict):
    cd = combined_dict
    number = cd['hook_checker_number']
    hand_rot = cd['hand_rot_list']
    ft_mat = np.matrix(cd['ft_list']).T # 6xN np matrix
    force_mat = ft_mat[0:3, :]

    n_forces = force_mat.shape[1]
    force_cam_list = []
    for i in range(n_forces):
        force_cam_list.append(ft_to_camera(force_mat[:,i],
                                      hand_rot[i], number))
    force_cam = np.column_stack(force_cam_list)
    return force_cam

#def random_color():
#    r = '%02X'%np.random.randint(0, 255)
#    g = '%02X'%np.random.randint(0, 255)
#    b = '%02X'%np.random.randint(0, 255)
#    c = '#' + r + g + b
#    return c

##
# @param d data log dictionary
def decompose_forces_and_recover_mechanism_state(d, type='rotary'):    
    if not d.has_key('force_tan_list'): #old dictionary
        actual_cartesian = end_effector_poses_from_log(d)
        radius, center = find_circle_center_from_log(d, actual_cartesian)
        frad_list, ftan_list = at.compute_radial_tangential_forces(d['force'].f_list, actual_cartesian.p_list, center[0,0], center[1,0])
        if type == 'rotary':   
            poses_list = rotary_mechanism_configurations_from_global_poses((np.matrix(actual_cartesian.p_list).T)[0:2,:] , center)
        elif type == 'prismatic':
            poses_list = linear_mechanism_configurations_from_global_poses((np.matrix(actual_cartesian.p_list).T)[0:2,:] , center)
    else:
        print 'decompose_forces_and_recover_mechanism_state: detected NEW dictionary!'
#        radius = d['r']
        frad_list, ftan_list = d['force_rad_list'], d['force_tan_list']     
        poses_list = d['mechanism_x']
    
    return poses_list, frad_list, ftan_list


def compute_segway_motion_mag(d):
    st = d['segway']
    x_list, y_list = st.x_list, st.y_list
    n1 = ut.norm(np.matrix([x_list, y_list]))
    n1 = n1 - np.min(n1)
    n1 = n1*10
    return n1.A1.tolist()

def linear_mechanism_configurations_from_global_poses(pts_2d, center):
    st_pt = pts_2d[:,0]          
    poses_list = []        
    for pos_idx in range(pts_2d.shape[1]):
        end_pt = pts_2d[:, pos_idx]
        end_displacement = np.linalg.norm(end_pt - st_pt)
        poses_list.append(end_displacement) 
    return poses_list

def rotary_mechanism_configurations_from_global_poses(pts_2d, center):
    st_pt = pts_2d[:,0]          
    cx = center[0,0]
    cy = center[1,0]
    start_angle = tr.angle_within_mod180(math.atan2(st_pt[1,0]-cy, st_pt[0,0]-cx) - math.radians(90)) #-90 for display
    poses_list = []        
    for pos_idx in range(pts_2d.shape[1]):
        end_pt = pts_2d[:, pos_idx]
        end_angle = tr.angle_within_mod180(math.atan2(end_pt[1,0]-cy, end_pt[0,0]-cx) - math.radians(90))                        
        poses_list.append(end_angle)
    return poses_list

##
# Recovers end effector pose in global coordinate frame taking 
# into account the segway's motion and robot's kinematics.
#def account_for_segway_motion_from_log(d):
def end_effector_poses_from_log(d):
    actual_cartesian_tl = at.joint_to_cartesian(d['actual'])
    actual_cartesian = at.account_segway_motion(actual_cartesian_tl, d['segway']) #end effector position -hai
    return actual_cartesian
    
##
# Find the circle's center
# @param actual_cartesian end effector poses in global frame
# @param d a log dictionary
def find_circle_center_from_log(d, actual_cartesian, plot=False):     
    pts_list = actual_cartesian.p_list
    pts_2d = (np.matrix(pts_list).T)[0:2,:]
    
    ee_start_pos = pts_list[0]
    rad_guess = 1.0
    x_guess = ee_start_pos[0]
    y_guess = ee_start_pos[1] - rad_guess
    
    rad_opt, cx, cy = at.fit_circle(rad_guess, x_guess, y_guess, pts_2d, method='fmin_bfgs', verbose=False)
    
    if plot:
        print 'rad_opt', rad_opt
        print 'cx', cx
        print 'cy', cy        
        pb.figure(1)
        pb.plot(pts_2d[0,:].tolist()[0], pts_2d[1,:].tolist()[0], 'b-')
        pb.axis('equal')
        pb.plot([cx], [cy], 'ro')
        pb.show()
        
    return rad_opt, np.matrix([cx, cy]).T

##
# take max force magnitude within a bin size
def bin(poses_list, ftan_list):
    poses_list = copy.copy(poses_list)
    bin_size = math.radians(2.)
    max_dist = max(poses_list)
    poses_array = np.arange(0., max_dist, bin_size)
    ftan_array = np.zeros(len(poses_array))

    for i in range(len(poses_list)):
        p = poses_list[i]
        f = ftan_list[i]
        idx = int(p/bin_size)
        if abs(f) > abs(ftan_array[idx]):
            ftan_array[idx] = f

    plist = poses_array.tolist()

    return poses_array.tolist(), ftan_array.tolist()


if __name__ == '__main__':
    pdb.set_trace()
    a = np.arange(0, 3, .1)
    #bins, ranges = histogram([a, a, a], [[a, a+10, a+20]], .2)
    bins, ranges = histogram([a], [[a]], .2)
    pdb.set_trace()
    print 'done'
    #histogram(index_list_list, elements_list_list, bin_size):
