
import roslib; roslib.load_manifest('modeling_forces')
import rospy

import hrl_lib.util as ut
import hrl_lib.transforms as tr
import matplotlib_util.util as mpu
import hrl_tilting_hokuyo.display_3d_mayavi as d3m

import modeling_forces.smooth as mfs
import kinematics_estimation as ke

import glob
import math, numpy as np
import sys

##
# plot to ensure that the time stamps in the different logs are
# reasonable.
# TODO - check for the rates too.
def check_time_sync(ft_time_list, mechanism_time_list, hand_time_list):
    mpu.plot_yx(np.zeros(len(ft_time_list))+1, ft_time_list,
                color = mpu.random_color(), label='ft\_time\_list',
                axis=None, linewidth=0.5, scatter_size=10)
    mpu.plot_yx(np.zeros(len(mechanism_time_list))+2, mechanism_time_list,
                color = mpu.random_color(), label='mechanism\_time\_list',
                axis=None, linewidth=0.5, scatter_size=10)
    mpu.plot_yx(np.zeros(len(hand_time_list))+3, hand_time_list,
                color = mpu.random_color(), label='hand\_time\_list',
                axis=None, linewidth=0.5, scatter_size=10)
    mpu.legend()
#    mpu.show()


##
#
# @return single dict with ft_list, mech_pose_lists, hand_pose_lists
# and ONE time_list
def synchronize(ft_dict, mechanism_dict, hand_dict):
    ft_time_arr = np.array(ft_dict['time_list'])
    mech_time_arr = np.array(mechanism_dict['time_list'])
    hand_time_arr = np.array(hand_dict['time_list'])
    
    print 'ft_time_arr.shape:', ft_time_arr.shape
    print 'mech_time_arr.shape:', mech_time_arr.shape
    print 'hand_time_arr.shape:', hand_time_arr.shape

    start_time  = max(ft_time_arr[0], mech_time_arr[0],
                      hand_time_arr[0])
    end_time  = min(ft_time_arr[-1], mech_time_arr[-1],
                    hand_time_arr[-1])

    t1_arr = mech_time_arr[np.where(np.logical_and(mech_time_arr >= start_time,
                                          mech_time_arr <= end_time))]
    t2_arr = hand_time_arr[np.where(np.logical_and(hand_time_arr >= start_time,
                                          hand_time_arr <= end_time))]

    #time_arr = np.arange(start_time, end_time, 0.03) # 30ms
    n_times = min(len(t1_arr), len(t2_arr))
    time_arr_list = []
    i, j = 0, 0
    while True:
        if t1_arr[i] == t2_arr[j]:
            time_arr_list.append(t1_arr[i])
            i += 1
            j += 1
        elif t1_arr[i] > t2_arr[j]:
            j += 1
        else:
            i += 1
        if j == n_times or i == n_times:
            break
    time_arr = np.array(time_arr_list)
    tstep_size = .03333333333
    uniform_time = np.cumsum(np.round((time_arr[1:] - time_arr[:-1]) / tstep_size) * tstep_size)
    uniform_time = np.concatenate((np.array([0]), uniform_time))
    uniform_time = uniform_time + time_arr_list[0]
    time_arr = uniform_time

    # adding a 50ms bias. see modeling_forces/image_ft_sync_test
    ft_time_arr = ft_time_arr + 0.05
    raw_ft_arr = np.array(ft_dict['ft_list']).T
    window_len = 3

    sm_ft_l = []
    for i in range(raw_ft_arr.shape[0]):
        s = mfs.smooth(raw_ft_arr[i,:], window_len,'blackman')
        sm_ft_l.append(s.tolist())
    # smooth truncates the array
    if window_len != 1:
        ft_time_arr = ft_time_arr[window_len-1:-window_len+1]
    raw_ft_arr = (np.array(sm_ft_l).T).tolist()

    raw_mech_pos_arr = mechanism_dict['pos_list']
    raw_mech_rot_arr = mechanism_dict['rot_list']

    raw_hand_pos_arr = hand_dict['pos_list']
    raw_hand_rot_arr = hand_dict['rot_list']

    raw_arr_list = [raw_ft_arr, raw_mech_pos_arr, raw_mech_rot_arr,
                    raw_hand_pos_arr, raw_hand_rot_arr]
    time_arr_list = [ft_time_arr, mech_time_arr, mech_time_arr,
                     hand_time_arr, hand_time_arr]
    n_arr = len(raw_arr_list)

    ft_list = []
    mech_pos_list, mech_rot_list = [], []
    hand_pos_list, hand_rot_list = [], []
    acc_list = [ft_list, mech_pos_list, mech_rot_list, hand_pos_list,
                hand_rot_list]
    key_list = ['ft_list', 'mech_pos_list', 'mech_rot_list',
                'hand_pos_list', 'hand_rot_list']

    for i in range(time_arr.shape[0]):
        t = time_arr[i]
        for j in range(n_arr):
            # nearest neighbor interpolation
            min_idx = np.argmin(np.abs(time_arr_list[j] - t))
            acc_list[j].append(raw_arr_list[j][min_idx])

    d = {}
    d['time_list'] = time_arr.tolist()
    for i in range(n_arr):
        d[key_list[i]] = acc_list[i]

    return d


#--------------- functions that operate on combined pkl ----------------------

##
# transform forces to camera coord frame.
# @param hand_rot_matrix - rotation matrix for camera to hand checker.
# @param hand_pos_matrix - position of hand checkerboard in camera coord frame.
# @param mech_pos_matrix - position of mechanism checkerboard in camera coord frame.
# @param number - checkerboard number (1, 2, 3 or 4)
def ft_to_camera(force_tool, hand_rot_matrix, hand_pos_matrix, mech_pos_matrix, number):
    # hc == hand checkerboard
    hc_rot_tool = tr.Rx(math.radians(90)) * tr.Ry(math.radians(180.)) * tr.Rz(math.radians(30.))

    while number != 1:
        hc_rot_tool = tr.Ry(math.radians(90.)) * hc_rot_tool
        number = number-1

    force_hc = hc_rot_tool * force_tool
    p_hc_ft = np.matrix([0.04, 0.01, 0.09]).T # vector from hook checkerboard origin to the base of the FT sensor in hook checker coordinates.

    # vec from FT sensor to mechanism checker origin in camera coordinates.
    p_ft_mech = -hand_pos_matrix + mech_pos_matrix - hand_rot_matrix * p_hc_ft
    
    force_cam = hand_rot_matrix * force_hc # force at hook base in camera coordinates
    moment_cam = hand_rot_matrix * moment_hc
    force_at_mech_origin = force_cam
    moment_at_mech_origin = moment_cam + np.matrix(np.cross(-p_ft_mech.A1, force_cam.A1)).T

    return hand_rot_matrix * force_hc

##
# transform force to camera coord frame.
def ft_to_camera_3(force_tool, moment_tool, hand_rot_matrix, number,
                   return_moment_cam = False):
    # hc == hand checkerboard
    hc_rot_tool = tr.Rx(math.radians(90)) * tr.Ry(math.radians(180.)) * tr.Rz(math.radians(30.))

    while number != 1:
        hc_rot_tool = tr.Ry(math.radians(90.)) * hc_rot_tool
        number = number-1

    force_hc = hc_rot_tool * force_tool
    moment_hc = hc_rot_tool * moment_tool
    p_ft_hooktip = np.matrix([0.0, -0.08, 0.00]).T # vector from FT sensor to the tip of the hook in hook checker coordinates.
#    p_ft_hooktip = np.matrix([0.0, -0.08 - 0.034, 0.00]).T # vector from FT sensor to the tip of the hook in hook checker coordinates.

    p_ft_hooktip = hand_rot_matrix * p_ft_hooktip
    force_cam = hand_rot_matrix * force_hc # force at hook base in camera coordinates
    moment_cam = hand_rot_matrix * moment_hc
    force_at_hook_tip = force_cam
    moment_at_hook_tip = moment_cam + np.matrix(np.cross(-p_ft_hooktip.A1, force_cam.A1)).T

#    if np.linalg.norm(moment_at_hook_tip) > 0.7:
#        import pdb; pdb.set_trace()

    if return_moment_cam:
        return force_at_hook_tip, moment_at_hook_tip, moment_cam
    else:
        return force_at_hook_tip, moment_at_hook_tip

def plot(combined_dict, savefig):
    plot_trajectories(combined_dict)

#    tup = ke.init_ros_node()
#    mech_rot_list = compute_mech_rot_list(combined_dict, tup)
#    combined_dict['mech_rot_list'] = mech_rot_list
#    plot_trajectories(combined_dict)

    cd = combined_dict
    ft_mat = np.matrix(cd['ft_list']).T
    force_mat = ft_mat[0:3, :]
    mpu.plot_yx(ut.norm(force_mat).A1, cd['time_list'])
    mpu.show()

#    plot_forces(combined_dict)
#    if savefig:
#        d3m.savefig(opt.dir+'/trajectory_check.png', size=(900, 800))
#    else:
#        d3m.show()


##
# @param pts - 3xN np matrix
def project_points_plane(pts):
#    import mdp
#    p = mdp.nodes.PCANode(svd=True)
#    p.train((pts-np.mean(pts, 1)).T.A)
#    print 'min mdp:', p.get_projmatrix()
#
#    U, s , _ = np.linalg.svd(np.cov(pts))
#    print 'min svd:', U[:,2]
    eval, evec = np.linalg.eig(np.cov(pts))
    min_idx = np.argmin(eval)
    min_evec = np.matrix(evec[:,min_idx]).T
    if min_evec[1,0] > 0:
        min_evec = min_evec * -1
    print 'min evec:', min_evec
    pts_proj = pts - np.multiply((min_evec.T * pts), min_evec)
    return pts_proj, min_evec

##
# use method 1 to compute the mechanism angle from combined dict.
# method 1 - angle between the x axis of checkerboard coord frame.
# @return list of mechanism angles.
def compute_mech_angle_1(cd, axis_direc=None):
    mech_rot = cd['mech_rot_list']
    directions_x = (np.row_stack(mech_rot)[:,0]).T.reshape(len(mech_rot), 3).T
    if axis_direc != None:
        directions_x = directions_x - np.multiply(axis_direc.T * directions_x,
                                                  axis_direc)
        directions_x = directions_x/ut.norm(directions_x)
    start_normal = directions_x[:,0]
    mech_angle = np.arccos(start_normal.T * directions_x).A1.tolist()
    return mech_angle

##
# use method 2 to compute the mechanism angle from combined dict.
# method 2 - fit a circle to estimate location of axis of rotation and
# radius. Then use that to compute the angle of the mechanism.
# @return list of mechanism angles.
def compute_mech_angle_2(cd, tup, project_plane=False):
    pos_mat = np.column_stack(cd['mech_pos_list'])
    if project_plane:
        pts_proj, min_evec = project_points_plane(pos_arr)
        pos_arr = pts_proj

    kin_dict = ke.fit(pos_mat, tup)
    center = np.matrix((kin_dict['cx'], kin_dict['cy'],
                       kin_dict['cz'])).T
    directions_x = pos_mat - center
    directions_x = directions_x / ut.norm(directions_x)
    start_normal = directions_x[:,0]
    #mech_angle = np.arccos(start_normal.T * directions_x).A1.tolist()

    ct = (start_normal.T * directions_x).A1
    st = ut.norm(np.matrix(np.cross(start_normal.A1, directions_x.T.A)).T).A1
    mech_angle = np.arctan2(st, ct).tolist()
    return mech_angle

def compute_mech_rot_list(cd, tup, project_plane=False):
    pos_arr = np.column_stack(cd['mech_pos_list'])
    rot_list = cd['mech_rot_list']
    directions_y = (np.row_stack(rot_list)[:,1]).T.reshape(len(rot_list), 3).T
    start_y = directions_y[:,0]

    pts_proj, y_mech = project_points_plane(pos_arr)
    if np.dot(y_mech.T, start_y.A1) < 0:
        print 'Negative hai boss'
        y_mech = -1 * y_mech

    if project_plane:
        pos_arr = pts_proj

    kin_dict = ke.fit(np.matrix(pos_arr), tup)
    center = np.array((kin_dict['cx'], kin_dict['cy'],
                       kin_dict['cz'])).T

    print 'pos_arr[:,0]', pos_arr[:,0]
    print 'center:', center

    directions_x = (np.row_stack(rot_list)[:,0]).T.reshape(len(rot_list), 3).T
    start_x = directions_x[:,0]
    directions_x = np.matrix(pos_arr) - np.matrix(center).T.A
    directions_x = directions_x / ut.norm(directions_x)
    if np.dot(directions_x[:,0].A1, start_x.A1) < 0:
        print 'Negative hai boss'
        directions_x = -1 * directions_x

    mech_rot_list = []
    for i in range(len(rot_list)):
        x = -directions_x[:, i]
        y = np.matrix(y_mech)
        z = np.matrix(np.cross(x.A1, y.A1)).T
        rot_mat = np.column_stack((x, y, z))
        mech_rot_list.append(rot_mat)

#    return mech_rot_list
    return rot_list

##
# @param tup - if None then use method 1 else use method 2 to
# compute mech angle.
# @return 1d array (radial force), 1d array (tangential force), list of mechanism angles, type ('rotary' or 'prismatic')
def compute_mechanism_properties(combined_dict, bias_ft = False,
                                 tup = None, cd_pkl_name = None):
    cd = combined_dict
    force_cam, moment_cam, _ = fts_to_camera(combined_dict)
    moment_contact_l = None
    if bias_ft:
        print 'Biasing magnitude:', np.linalg.norm(force_cam[:,0])
        force_cam = force_cam - force_cam[:,0]
        moment_cam = moment_cam - moment_cam[:,0]

    if cd['radius'] != -1:
        if tup == None:
            mech_angle = compute_mech_angle_1(cd)
        else:
            mech_angle = compute_mech_angle_2(cd, tup)
            # compute new mech_rot_list. used for factoring the forces.
            mech_rot_list = compute_mech_rot_list(combined_dict, tup)
            combined_dict['mech_rot_list'] = mech_rot_list

            hook_tip_l = compute_hook_tip_trajectory(cd)
            hand_mat = np.column_stack(hook_tip_l)
            for i,f in enumerate(force_cam.T):
                fmag = np.linalg.norm(f)
                if fmag > 1.0:
                    break
            end_idx = np.argmax(mech_angle)
            hand_mat_short = hand_mat[:,i:end_idx]

            kin_dict = ke.fit(hand_mat_short, tup)
            center_hand = np.matrix((kin_dict['cx'], kin_dict['cy'],
                                     kin_dict['cz'])).T
            radius_hand = kin_dict['r']
            
            center_mech_coord = mech_rot_list[0].T * center_hand
            start_mech_coord = mech_rot_list[0].T * hand_mat_short[:,0]
            opens_right = False
            if start_mech_coord[0,0] < center_mech_coord[0,0]:
                print 'Opens Right'
                opens_right = True

            # radial vectors.
            radial_mat = hand_mat - center_hand
            radial_mat = radial_mat / ut.norm(radial_mat)
            _, nrm_hand = project_points_plane(hand_mat_short)
            if np.dot(nrm_hand.A1, mech_rot_list[0][:,1].A1) < 0:
                nrm_hand = -1 * nrm_hand
            if opens_right == False:
                nrm_hand = -1 * nrm_hand

            frad_l = []
            ftan_l = []
            moment_contact_l = []
            for i, f in enumerate(force_cam.T):
                f = f.T
                m = (moment_cam[:,i].T * nrm_hand)[0,0]
                moment_contact_l.append(m)
                tvec = np.matrix(np.cross(nrm_hand.A1, radial_mat[:,i].A1)).T
                ftan = (f.T * tvec)[0,0]
                ftan_l.append(ftan)

                frad = np.linalg.norm(f - tvec * ftan)
                #frad = (f_cam.T*radial_mat[:,i])[0,0]
                frad_l.append(abs(frad))

        typ = 'rotary'

    else:
        pos_mat = np.column_stack(cd['mech_pos_list'])
        mech_angle = ut.norm(pos_mat-pos_mat[:,0]).A1.tolist()
        #print 'mech_angle:', mech_angle
        typ = 'prismatic'
        moment_axis_list = None

        frad_l = []
        ftan_l = []
        moment_contact_l = []
        rot_list = cd['mech_rot_list']
        directions_z = (np.row_stack(rot_list)[:,2]).T.reshape(len(rot_list), 3).T
        for i, f in enumerate(force_cam.T):
            f = f.T
            tvec = np.matrix(directions_z[:,i])
            ftan = (f.T * tvec)[0,0]
            ftan_l.append(ftan)

            frad = np.linalg.norm(f - tvec * ftan)
            #frad = (f_cam.T*radial_mat[:,i])[0,0]
            frad_l.append(abs(frad))
            radius_hand = 10.

    ut.save_pickle(combined_dict, cd_pkl_name)

    return np.array(frad_l), np.array(ftan_l), mech_angle, typ, \
           np.array(ftan_l)*radius_hand, np.array(moment_contact_l)

def plot_radial_tangential(mech_dict, savefig, fig_name=''):
    radial_mech = mech_dict['force_rad_list']
    tangential_mech = mech_dict['force_tan_list']
    typ = mech_dict['mech_type']
    mech_x = mech_dict['mechanism_x']
    if typ == 'rotary':
        mech_x_degrees = np.degrees(mech_x)
        xlabel = 'angle (degrees)'
    else:
        mech_x_degrees = mech_x
        xlabel = 'distance (meters)'
    mpu.pl.clf()
    mpu.plot_yx(radial_mech, mech_x_degrees, axis=None, label='radial force',
                xlabel=xlabel, ylabel='N', color='r')
    mpu.plot_yx(tangential_mech, mech_x_degrees, axis=None, label='tangential force',
                xlabel=xlabel, ylabel='N', color='g')
    mpu.legend()
    
    if typ == 'rotary':
        mpu.figure()
        rad = mech_dict['radius']
        torques_1 = rad * np.array(tangential_mech)
        torques_3 = np.array(mech_dict['moment_tip_list']) + torques_1
        mpu.plot_yx(torques_1, mech_x_degrees, axis=None,
                    label='torque from tangential',
                    xlabel=xlabel, ylabel='moment', color='r')
        mpu.plot_yx(torques_3, mech_x_degrees, axis=None,
                    label='total torque',
                    xlabel=xlabel, ylabel='moment', color='y')
        mpu.legend()


    if savefig:
        mpu.savefig(opt.dir+'/%s_force_components.png'%fig_name)
    else:
        mpu.show()

##
# returns force and moment at the tip of the hook in camera
# coordinates.
def fts_to_camera(combined_dict):
    cd = combined_dict
    number = cd['hook_checker_number']
    hand_rot = cd['hand_rot_list']
    hand_pos = cd['hand_pos_list']
    ft_mat = np.matrix(cd['ft_list']).T # 6xN np matrix
    force_mat = ft_mat[0:3, :]
    moment_mat = ft_mat[3:6, :]

    n_forces = force_mat.shape[1]
    force_cam_list = []
    moment_cam_list = []
    moment_base_list = []
    for i in range(n_forces):
        f,m,m_base = ft_to_camera_3(force_mat[:,i], moment_mat[:,i], hand_rot[i],
                                    number, return_moment_cam = True)
        force_cam_list.append(f)
        moment_cam_list.append(m)
        moment_base_list.append(m_base)
    force_cam = np.column_stack(force_cam_list)
    moment_cam = np.column_stack(moment_cam_list)
    moment_base = np.column_stack(moment_base_list)
    return force_cam, moment_cam, moment_base

def plot_forces(combined_dict):
    cd = combined_dict
    hand_mat = np.column_stack(cd['hand_pos_list'])
    hand_rot = cd['hand_rot_list']

    mech_mat = np.column_stack(cd['mech_pos_list'])
    mech_rot = cd['mech_rot_list']
    directions_x = (np.row_stack(mech_rot)[:,0]).T.reshape(len(mech_rot), 3).T
    force_cam, moment_cam, _ = fts_to_camera(combined_dict)

    d3m.plot_points(hand_mat, color = (1.,0.,0.), mode='sphere')
    d3m.plot_points(mech_mat, color = (0.,0.,1.), mode='sphere')
    d3m.plot_normals(mech_mat, directions_x, color=(1.,0,0.))
#    d3m.plot_normals(mech_mat, force_mat, color=(0.,1,0.))
    d3m.plot_normals(mech_mat, force_cam, color=(0.,0,1.))

def plot_trajectories(combined_dict):
    cd = combined_dict
    hand_mat = np.column_stack(cd['hand_pos_list'])
    hand_rot = cd['hand_rot_list']
    directions_x = (np.row_stack(hand_rot)[:,0]).T.reshape(len(hand_rot), 3).T
    directions_y = (np.row_stack(hand_rot)[:,1]).T.reshape(len(hand_rot), 3).T
    directions_z = (np.row_stack(hand_rot)[:,2]).T.reshape(len(hand_rot), 3).T

    #d3m.white_bg()

    d3m.plot_points(hand_mat, color = (1.,0.,0.), mode='sphere',
                    scale_factor = 0.005)
    d3m.plot_normals(hand_mat, directions_x, color=(1.,0,0.),
                     scale_factor = 0.02)
    d3m.plot_normals(hand_mat, directions_y, color=(0.,1,0.),
                     scale_factor = 0.02)
    d3m.plot_normals(hand_mat, directions_z, color=(0.,0,1.),
                     scale_factor = 0.02)

    mech_mat = np.column_stack(cd['mech_pos_list'])
    mech_rot = cd['mech_rot_list']
    directions_x = (np.row_stack(mech_rot)[:,0]).T.reshape(len(mech_rot), 3).T
    directions_y = (np.row_stack(mech_rot)[:,1]).T.reshape(len(hand_rot), 3).T
    directions_z = (np.row_stack(mech_rot)[:,2]).T.reshape(len(mech_rot), 3).T

    d3m.plot_points(mech_mat[:,0:1], color = (0.,0.,0.), mode='sphere',
                    scale_factor = 0.01)
    d3m.plot_points(mech_mat, color = (0.,0.,1.), mode='sphere',
                    scale_factor = 0.005)
    d3m.plot_normals(mech_mat, directions_x, color=(1.,0,0.),
                     scale_factor = 0.02)
    d3m.plot_normals(mech_mat, directions_y, color=(0.,1,0.),
                     scale_factor = 0.02)
    d3m.plot_normals(mech_mat, directions_z, color=(0.,0,1.),
                     scale_factor = 0.02)

    m = np.mean(mech_mat, 1)
    d3m.mlab.view(azimuth=-120, elevation=60, distance=1.60,
                  focalpoint=(m[0,0], m[1,0], m[2,0]))

##
# @return list of hook tip coodinates in camera coordinate frame.
def compute_hook_tip_trajectory(combined_dict):
    cd = combined_dict
    hand_mat = np.column_stack(cd['hand_pos_list'])
    hand_rot_l = cd['hand_rot_list']
    directions_x = (np.row_stack(hand_rot_l)[:,0]).T.reshape(len(hand_rot_l), 3).T
    directions_y = (np.row_stack(hand_rot_l)[:,1]).T.reshape(len(hand_rot_l), 3).T
    directions_z = (np.row_stack(hand_rot_l)[:,2]).T.reshape(len(hand_rot_l), 3).T

    hand_pos_list = cd['hand_pos_list']
    hook_tip_l = []
    for i,p in enumerate(hand_pos_list): # p - hook checker origin in camera coordinates.
        hc_P_hc_hooktip = np.matrix([0.035, -0.0864, 0.09]).T # vector from hook checkerboard origin to the tip of the hook in hook checker coordinates.
        cam_P_hc_hooktip = hand_rot_l[i] * hc_P_hc_hooktip
        hook_tip_l.append(p + cam_P_hc_hooktip)
    
    return hook_tip_l

##
# take the open + close trajectory and split it into two separate
# trajectories and save them as pkls.
def split_open_close(rad, tan, ang, typ, mech_radius, time_list,
                     moment_axis, moment_tip):
    ang = np.array(ang)
    incr = ang[1:] - ang[:-1]
    n_pts = ang.shape[0] - 2 #ignoring first and last readings.
    rad_l, tan_l, ang_l = [], [], []
    for i in range(n_pts):
        if typ == 'rotary':
            sgn = incr[i] * incr[i+1]
            mag = abs(incr[i] - incr[i+1])
            if sgn < 0 and mag > math.radians(10):
                continue
            rad_l.append(rad[i+1])
            tan_l.append(tan[i+1])
            ang_l.append(ang[i+1])
        else:
            # no cleanup for prismatic joints, for now
            rad_l.append(rad[i+1])
            tan_l.append(tan[i+1])
            ang_l.append(ang[i+1])

    rad, tan, ang = rad_l, tan_l, ang_l
    max_idx = np.argmax(ang)
    d_open = {'force_rad_list': rad[:max_idx+1],
              'force_tan_list': tan[:max_idx+1],
              'mechanism_x': ang[:max_idx+1], 'mech_type': typ,
              'radius': mech_radius,
              'time_list': time_list[:max_idx+1]}
    if moment_tip != None:
        d_open['moment_tip_list'] = moment_tip[:max_idx+1]
        d_open['moment_list'] = moment_axis[:max_idx+1]
    ut.save_pickle(d_open, opt.dir + '/open_mechanism_trajectories_handhook.pkl')

    d_close = {'force_rad_list': rad[max_idx+1:],
               'force_tan_list': tan[max_idx+1:],
               'mechanism_x': ang[max_idx+1:], 'mech_type': typ,
               'radius': mech_radius,
               'time_list': time_list[max_idx+1:]}
    if moment_tip != None:
        d_open['moment_tip_list'] = moment_tip[max_idx+1:]
        d_open['moment_list'] = moment_axis[max_idx+1:]
    ut.save_pickle(d_close, opt.dir + '/close_mechanism_trajectories_handhook.pkl')


def plot_hooktip_trajectory_and_force(cd):
    hook_tip_l = compute_hook_tip_trajectory(cd)

    # plot trajectory in 3D.
    d3m.white_bg()
    d3m.plot_points(np.column_stack(hook_tip_l), color = (1.,0.,0.), mode='sphere',
                    scale_factor = 0.005)
#    d3m.plot_points(mech_proj[:,0:1], color = (0.,0.,0.), mode='sphere',
#                    scale_factor = 0.01)
#    d3m.plot_points(mech_proj, color = (0.,0.,1.), mode='sphere',
#                    scale_factor = 0.005)
#    d3m.plot(np.column_stack((mech_proj[:,-1],center_mech, mech_proj[:,0])),
#             color = (0.,0.,1.))
#    d3m.plot(np.column_stack((hand_proj[:,-1],center_hand, hand_proj[:,0])),
#             color = (1.,0.,0.))
    d3m.show()

##
# sanity check - fitting circle to mechanism and hook tip
# trajectories, computing the angle between the initial radial
# direction of the mechanism and the radial directions for the hook
# tip. This angle starts out at a slightly positive angle. I'm
# assuming that this corresponds to the fact that the handle sticks
# out from the cabinet door. What makes me nervous is that I am still
# fitting two different circles to the mechanism and hook
# trajectories.
def compare_tip_mechanism_trajectories(mech_mat, hand_mat):
#    hand_proj, nrm_hand = project_points_plane(hand_mat)
#    mech_proj, nrm_mech = project_points_plane(mech_mat)
    hand_proj = hand_mat
    mech_proj = mech_mat

    kin_dict = ke.fit(hand_proj, tup)
    print 'kin_dict from hook tip:', kin_dict
    print 'measured radius:', cd['radius']
    center_hand = np.matrix((kin_dict['cx'], kin_dict['cy'],
                             kin_dict['cz'])).T

    kin_dict = ke.fit(mech_proj, tup)
    print 'kin_dict from mechanism:', kin_dict
    center_mech = np.matrix((kin_dict['cx'], kin_dict['cy'],
                             kin_dict['cz'])).T

    # working with the projected coordinates.
    directions_hand = hand_proj - center_hand
    directions_hand = directions_hand / ut.norm(directions_hand)
    directions_mech = mech_proj - center_mech
    directions_mech = directions_mech / ut.norm(directions_mech)

    start_normal = directions_mech[:,0]
    print 'directions_mech[:,0]', directions_mech[:,0].A1
    print 'directions_hand[:,0]', directions_hand[:,0].A1
    ct = (start_normal.T * directions_hand).A1
    st = ut.norm(np.matrix(np.cross(start_normal.A1, directions_hand.T.A)).T).A1
    
    mech_angle = np.arctan2(st, ct).tolist()
    #mech_angle = np.arccos(start_normal.T * directions_hand).A1.tolist()

    mpu.plot_yx(np.degrees(mech_angle))
    mpu.show()

    # plot trajectory in 3D.
    d3m.white_bg()
    d3m.plot_points(hand_proj, color = (1.,0.,0.), mode='sphere',
                    scale_factor = 0.005)
    d3m.plot_points(mech_proj[:,0:1], color = (0.,0.,0.), mode='sphere',
                    scale_factor = 0.01)
    d3m.plot_points(mech_proj, color = (0.,0.,1.), mode='sphere',
                    scale_factor = 0.005)
    d3m.plot(np.column_stack((mech_proj[:,-1],center_mech, mech_proj[:,0])),
             color = (0.,0.,1.))
    d3m.plot(np.column_stack((hand_proj[:,-1],center_hand, hand_proj[:,0])),
             color = (1.,0.,0.))
    d3m.show()

def angle_between_hooktip_mechanism_radial_vectors(mech_mat, hand_mat):
    kin_dict = ke.fit(hand_mat, tup)
    print 'kin_dict from hook tip:', kin_dict
    print 'measured radius:', cd['radius']
    center_hand = np.matrix((kin_dict['cx'], kin_dict['cy'],
                             kin_dict['cz'])).T

    kin_dict = ke.fit(mech_mat, tup)
    print 'kin_dict from mechanism:', kin_dict
    center_mech = np.matrix((kin_dict['cx'], kin_dict['cy'],
                             kin_dict['cz'])).T

    # working with the projected coordinates.
    directions_hand = hand_mat - center_hand
    directions_hand = directions_hand / ut.norm(directions_hand)
    directions_mech = mech_mat - center_mech
    directions_mech = directions_mech / ut.norm(directions_mech)

    #import pdb; pdb.set_trace()
    ang = np.degrees(np.arccos(np.sum(np.multiply(directions_mech, directions_hand), 0))).A1
    mpu.plot_yx(ang, label = 'angle between hooktip-radial and mechanism radial')
    mpu.legend()
    mpu.show()

def split_forces_hooktip_test(hand_mat):
    kin_dict = ke.fit(hand_mat, tup)
    center_hand = np.matrix((kin_dict['cx'], kin_dict['cy'],
                             kin_dict['cz'])).T

    print 'kin_dict:', kin_dict
    # radial vectors.
    radial_mat = hand_mat - center_hand
    radial_mat = radial_mat / ut.norm(radial_mat)

    # cannot use hook tip to compute mechanism angle because I
    # don't have a way of knowing when the hook starts opening the
    # mechanism. (Think hook makes contact with door, moves in
    # freespace and then makes contact with the handle.)
    #start_rad = radial_mat[:,0]
    #ct = (start_rad.T * radial_mat).A1
    #st = ut.norm(np.matrix(np.cross(start_rad.A1, radial_mat.T.A)).T).A1
    #mech_angle_l = np.arctan2(st, ct).tolist()

    _, nrm_hand = project_points_plane(hand_mat)
    print 'nrm_hand:', nrm_hand.A1

    f_cam_l = []
    m_cam_l = []
    m_base_l = []
    frad_l = []
    ftan_l = []
    hook_num = cd['hook_checker_number']
    print 'hook_num:', hook_num
    for i, f in enumerate(force_mat.T):
        f = f.T
        m = moment_mat[:,i]
        f_cam, m_cam, m_base = ft_to_camera_3(f, m, hook_rot_l[i], hook_num,
                                              return_moment_cam = True)
        f_cam_l.append(f_cam)
        m_cam_l.append(abs((m_cam.T*nrm_hand)[0,0]))
        m_base_l.append(abs((m_base.T*nrm_hand)[0,0]))
        #m_base_l.append(np.linalg.norm(f))

        tangential_vec = np.matrix(np.cross(radial_mat[:,i].A1, nrm_hand.A1)).T
        ftan = (f_cam.T * tangential_vec)[0,0]
        ftan_l.append(ftan)

        #frad = np.linalg.norm(f_cam - tangential_vec * ftan)
        frad = (f_cam.T*radial_mat[:,i])[0,0]
        frad_l.append(abs(frad))


    fig1 = mpu.figure()
    mech_ang_deg = np.degrees(mech_angle_l)
    mpu.plot_yx(ftan_l, mech_ang_deg, label='Tangential Force (hook tip)', color='b')
    mpu.plot_yx(frad_l, mech_ang_deg, label='Radial Force (hook tip)', color='y')

    mech_pkl_name = glob.glob(opt.dir + '/open_mechanism_trajectories_*.pkl')[0]
    md = ut.load_pickle(mech_pkl_name)
    radial_mech = md['force_rad_list']
    tangential_mech = md['force_tan_list']
    mech_x = np.degrees(md['mechanism_x'])
    mpu.plot_yx(tangential_mech, mech_x, label='Tangential Force (mechanism checker)', color='g')
    mpu.plot_yx(radial_mech, mech_x, label='Radial Force (mechanism checker)', color='r')


    mpu.legend()

    fig2 = mpu.figure()
    mpu.plot_yx(m_cam_l, mech_ang_deg, label='\huge{$m_{axis}$}')
    mpu.plot_yx(m_base_l, mech_ang_deg, label='\huge{$m^s$}',
                color = 'r')

    mpu.legend()
    mpu.show()


if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p.add_option('-d', '--dir', action='store', default='',
                 type='string', dest='dir', help='directory with logged data')
    p.add_option('-t', '--time_check', action='store_true', dest='tc',
                 help='plot to check the consistency of time stamps')
    p.add_option('-s', '--sync', action='store_true', dest='sync',
                 help='time synchronize poses, forces etc.')
    p.add_option('--split', action='store_true', dest='split_forces',
                 help='split forces into radial and tangential and save in a pickle')
    p.add_option('--savefig', action='store_true', dest='savefig',
                 help='save the plot instead of showing it.')
    p.add_option('-c', '--cd', action='store_true', dest='cd',
                 help='work with the combined dict')
    p.add_option('-f', '--force', action='store_true', dest='force',
                 help='plot radial and tangential force')
    p.add_option('--mech_prop_ros', action='store_true',
                 dest='mech_prop_ros',
                 help='plot radial and tangential force')
    p.add_option('--moment_test', action='store_true',
                 dest='moment_test',
                 help='trying to compute moment about the joint axis.')
    p.add_option('--hook_tip_test', action='store_true',
                 dest='hook_tip_test',
                 help='plot trajectory of hook tip for debugging etc.')

    opt, args = p.parse_args()

    if opt.dir == '':
        raise RuntimeError('Need a directory to work with (-d or --dir)')

    if opt.force:
        mech_pkl_name = glob.glob(opt.dir + '/open_mechanism_trajectories_*.pkl')[0]
        md = ut.load_pickle(mech_pkl_name)
        plot_radial_tangential(md, opt.savefig, 'open')
#        mech_pkl_name = glob.glob(opt.dir + '/close_mechanism_trajectories_handhook.pkl')[0]
#        md = ut.load_pickle(mech_pkl_name)
#        plot_radial_tangential(md, opt.savefig, 'close')

    ft_pkl = glob.glob(opt.dir + '/ft_log*.pkl')[0]
    poses_pkl = glob.glob(opt.dir + '/poses_dict*.pkl')[0]
    
    ft_dict = ut.load_pickle(ft_pkl)
    poses_dict = ut.load_pickle(poses_pkl)
    mechanism_dict = poses_dict['mechanism']
    hand_dict = poses_dict['hand']


    ft_time_list = ft_dict['time_list']
    mechanism_time_list = mechanism_dict['time_list']
    hand_time_list = hand_dict['time_list']

    if opt.tc:
        check_time_sync(ft_time_list, mechanism_time_list, hand_time_list)
        if opt.savefig:
            mpu.savefig(opt.dir+'/time_check.png')
        else:
            mpu.show()

    if opt.sync:
        print 'Begin synchronize'
        d = synchronize(ft_dict, mechanism_dict, hand_dict)
        print 'End synchronize'
        #ut.save_pickle(d, opt.dir+'/combined_log'+ut.formatted_time()+'.pkl')
        ut.save_pickle(d, opt.dir+'/combined_log.pkl')
        print 'Saved pickle'

    if opt.cd:
        cd = ut.load_pickle(glob.glob(opt.dir + '/combined_log*.pkl')[0])
        plot(cd, opt.savefig)

    if opt.mech_prop_ros:
        import mechanism_analyse as ma

        cd = ut.load_pickle(glob.glob(opt.dir + '/combined_log*.pkl')[0])

        tup = ke.init_ros_node()

        ma2 = compute_mech_angle_2(cd, tup, project_plane=False)
        ma1 = compute_mech_angle_1(cd)
        ma3 = compute_mech_angle_2(cd, tup, project_plane=True)
#        ma4 = compute_mech_angle_1(cd, min_evec)


        lab1 = 'orientation only'
        lab2 = 'checker origin position + circle fit'
        lab3 = 'checker origin position + PCA projection + circle fit'
#        lab4 = 'PCA projection + orientation'

        mpu.figure()
        mpu.plot_yx(np.degrees(ma3), color='r', label=lab3,
                    linewidth = 1, scatter_size = 5)
        mpu.plot_yx(np.degrees(ma2), color='b', label=lab2,
                    linewidth = 1, scatter_size = 5)
        mpu.plot_yx(np.degrees(ma1), color='y', label=lab1,
                    linewidth = 1, scatter_size = 5)
        mpu.legend()

        vel3 = ma.compute_velocity(ma3, cd['time_list'], 1)
        vel2 = ma.compute_velocity(ma2, cd['time_list'], 1)
        vel1 = ma.compute_velocity(ma1, cd['time_list'], 1)
        mpu.figure()
        mpu.plot_yx(np.degrees(vel3), np.degrees(ma3), color='r',
                    label=lab3, linewidth = 1, scatter_size = 5)
        mpu.plot_yx(np.degrees(vel2), np.degrees(ma2), color='b',
                    label=lab2, linewidth = 1, scatter_size = 5)
        mpu.plot_yx(np.degrees(vel1), np.degrees(ma1), color='y',
                    label=lab1, linewidth = 1, scatter_size = 5)
        mpu.legend()

#        acc3 = ma.compute_velocity(vel3, cd['time_list'], 1)
#        mpu.figure()
#        mpu.plot_yx(np.degrees(acc3), np.degrees(ma3), color='r',
#                    label=lab3, linewidth = 1, scatter_size = 5)
#        mpu.legend()

        mpu.show()

    if opt.split_forces:
        tup = ke.init_ros_node()
        pkl_name = glob.glob(opt.dir + '/combined_log*.pkl')[0]
        mech_pkl_name = glob.glob(opt.dir + '/mechanism_info*.pkl')[0]

        md = ut.load_pickle(mech_pkl_name)
        cd = ut.load_pickle(pkl_name)
        cd['hook_checker_number'] = md['checkerboard_number']
        cd['radius'] = md['radius']
        rad, tan, ang, typ, moment_axis, moment_tip = compute_mechanism_properties(cd,
                                                bias_ft=True, tup=tup,
                                                cd_pkl_name = pkl_name)
        split_open_close(rad, tan, ang, typ, md['radius'],
                         cd['time_list'], moment_axis, moment_tip)

    if opt.moment_test:
        tup = ke.init_ros_node()
        pkl_name = glob.glob(opt.dir + '/combined_log*.pkl')[0]
        mech_pkl_name = glob.glob(opt.dir + '/mechanism_info*.pkl')[0]

        md = ut.load_pickle(mech_pkl_name)
        cd = ut.load_pickle(pkl_name)
        cd['hook_checker_number'] = md['checkerboard_number']
        cd['radius'] = md['radius']
        rad, tan, ang, typ, moment_axis, moment_tip = compute_mechanism_properties(cd,
                                                bias_ft=True, tup=tup,
                                                cd_pkl_name = pkl_name)
        ang = np.array(ang)
        incr = ang[1:] - ang[:-1]
        n_pts = ang.shape[0] - 2 #ignoring first and last readings.
        rad_l, tan_l, ang_l = [], [], []
        for i in range(n_pts):
            if typ == 'rotary':
                sgn = incr[i] * incr[i+1]
                mag = abs(incr[i] - incr[i+1])
                if sgn < 0 and mag > math.radians(10):
                    continue
                rad_l.append(rad[i+1])
                tan_l.append(tan[i+1])
                ang_l.append(ang[i+1])
            else:
                # no cleanup for prismatic joints, for now
                rad_l.append(rad[i+1])
                tan_l.append(tan[i+1])
                ang_l.append(ang[i+1])

        rad, tan, ang = rad_l, tan_l, ang_l
        max_idx = np.argmax(ang)
        rad = np.array(rad[:max_idx+1])
        tan = np.array(tan[:max_idx+1])
        ang = np.array(ang[:max_idx+1])
        moment_axis = np.array(moment_axis[:max_idx+1])
        moment_tip = np.array(moment_tip[:max_idx+1])

        fig1 = mpu.figure()
        mpu.plot_yx(tan * cd['radius'], np.degrees(ang), label = 'Moment from Tangential Force',
                    color = 'b')
        mpu.plot_yx(moment_axis, np.degrees(ang), label = 'Computed Moment',
                    color = 'g')
        mpu.plot_yx(moment_tip, np.degrees(ang), label = 'Computed Moment using tip model',
                    color = 'y')
        mpu.legend()
        mpu.show()

    if opt.hook_tip_test:
        tup = ke.init_ros_node()
        pkl_name = glob.glob(opt.dir + '/combined_log*.pkl')[0]
        mech_pkl_name = glob.glob(opt.dir + '/mechanism_info*.pkl')[0]

        md = ut.load_pickle(mech_pkl_name)
        cd = ut.load_pickle(pkl_name)
        cd['hook_checker_number'] = md['checkerboard_number']
        cd['radius'] = md['radius']
        
        hook_tip_l = compute_hook_tip_trajectory(cd)
        hook_rot_l = cd['hand_rot_list']
        
        mech_mat = np.column_stack(cd['mech_pos_list'])
        hand_mat = np.column_stack(hook_tip_l)
        
        ft_mat = np.matrix(cd['ft_list']).T # 6xN np matrix
        force_mat = ft_mat[0:3, :]
#        force_mat = force_mat - force_mat[:,0]

        moment_mat = ft_mat[3:6, :]
#        moment_mat = moment_mat - moment_mat[:,0]
        
        for i,f in enumerate(force_mat.T):
            fmag = np.linalg.norm(f)
            if fmag > 1.0:
                print 'i:', i
                break
        
        mech_angle_l = compute_mech_angle_2(cd, tup, project_plane=False)
        end_idx = np.argmax(mech_angle_l)
        
        hand_mat = hand_mat[:,i:end_idx]
        mech_mat = mech_mat[:,i:end_idx]
        force_mat = force_mat[:,i:end_idx]
        moment_mat = moment_mat[:,i:end_idx]
        hook_rot_l = hook_rot_l[i:end_idx]
        mech_angle_l = mech_angle_l[i:end_idx]

        compare_tip_mechanism_trajectories(mech_mat[:,i:], hand_mat[:,i:])
        #angle_between_hooktip_mechanism_radial_vectors(mech_mat, hand_mat)

        #plot moment at hook ti and base.
        #split_forces_hooktip_test(hand_mat)




