import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import sys
import hrl_lib.util as ut
import hrl_lib.rutils as ru

import tf
import hrl_lib.transforms as htf
import hrl_lib.tf_utils as tfu
import tf.transformations as tr
import pr2_msgs.msg as pm

import scipy.spatial as sp
from multiprocessing import Process
import time
import os
import numpy as np
import math
import cv
import hai_sandbox.features as fea
import hrl_pr2_lib.pr2_kinematics as pr2k
import pdb
import sensor_msgs.msg as sm 
import hrl_pr2_lib.devices as hpr2

#import perception3d.gaussian_curvature as gc

def segment_msgs(time_segments, msgs):
    segs = []
    for segment in time_segments:
        start = segment[0]
        endt  = segment[1]
        sx = 0
        ex = len(msgs)

        if start != 'start':
            for i, m in enumerate(msgs):
                if m.header.stamp.to_time() < start:
                    sx = i
                else:
                    break

        if endt != 'end':
            for i, m in enumerate(msgs[sx:]):
                if m.header.stamp.to_time() > endt:
                    ex = i + sx
                    break

        #pdb.set_trace()
        seg = msgs[sx:ex]
        segs.append(msgs[sx: ex])
    return segs

##
# Find times where contact has been made
#
# @return array of locations where contact has been made, array of times for each location where contact has been made (can be duplicated)
def find_contact_times(left_mat, right_mat, times, thres):
    left_mat = left_mat - left_mat[:, 0] 
    right_mat = right_mat - right_mat[:,0]
    
    #When/where did contact happen? 
    #TODO: we are assuming just one finger of one arm here!
    #pdb.set_trace()
    loc_r, time_c = np.where(np.abs(left_mat) > thres)
    times_contact = times[time_c.A1]
    unique_times = np.array(np.sort(list(set(times_contact.tolist()))))
    #return (loc_r, times_contact)

    return unique_times

def playback_bag(bag_name):
    cmd = 'rosbag play %s --clock' % bag_name
    print cmd
    os.system(cmd)

class SimpleJointStateMsg:
    def __init__(self, header, transforms_dict):
        self.header = header
        self.transforms_dict = transforms_dict

# Find contact points & transform gripper tip to base_frame
class ExtractTFData:
    def __init__(self, listener=None):#, pointcloud_msg):
        rospy.Subscriber('/pressure/l_gripper_motor', pm.PressureState, self.lpress_cb)
        rospy.Subscriber('/joint_states', sm.JointState, self.joint_state_cb)
        self.ftip_frames = ['r_gripper_tool_frame',
                            'l_gripper_tool_frame']

        if listener != None:
            self.tflistener = listener
        else:
            self.tflistener = tf.TransformListener()

        self.lmat0 = None
        self.rmat0 = None
        self.contact_locs = []
        self.base_frame = '/base_footprint'

        self.contact = False
        self.contact_stopped = False
        self.pointcloud_transform = None

        self.jstate_msgs = []
        self.last_jstate_time = time.time() + 999999999.

    def joint_state_cb(self, jmsg):
        tdict = {}

        # good for display purposes (independent of torso link etc)
        tdict['bf_T_rtip'] = tfu.transform('/base_footprint', self.ftip_frames[0], self.tflistener)
        tdict['bf_T_ltip'] = tfu.transform('/base_footprint', self.ftip_frames[1], self.tflistener)

        # want FK from torso
        tdict['torso_T_rtip'] = tfu.transform('/torso_lift_link', self.ftip_frames[0], self.tflistener)
        tdict['torso_T_ltip'] = tfu.transform('/torso_lift_link', self.ftip_frames[1], self.tflistener)

        #self.jstate_msgs.append(SimpleJointStateMsg(jmsg.header, tdict))
        self.last_jstate_time = time.time()

    def lpress_cb(self, pmsg):
        lmat = np.matrix((pmsg.l_finger_tip)).T
        rmat = np.matrix((pmsg.r_finger_tip)).T
        if self.lmat0 == None:
            self.lmat0 = lmat
            self.rmat0 = rmat
            return

        #zero
        lmat = lmat - self.lmat0
        rmat = rmat - self.rmat0
   
        ##
        # extract data during contact events
        #touch detected
        if np.any(np.abs(lmat) > 250) or np.any(np.abs(rmat) > 250): #TODO: replace this with something more sound
            #Contact has been made!! look up gripper tip location
            def frame_loc(from_frame):
                p_base = tfu.transform(self.base_frame, from_frame, self.tflistener) \
                               * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
                return tfu.matrix_as_tf(p_base)
            tip_locs = [frame_loc(n)[0] for n in self.ftip_frames]
            t = pmsg.header.stamp.to_time() 
            rospy.loginfo("contact detected at %.3f" % t)
            self.contact_locs.append([t, tip_locs])
            self.contact = True
        else:
            if self.contact == True:
                rospy.loginfo('contact stopped')
                self.contact_stopped = True
            self.contact = False

            #Only get this transform after we've stopped making contact.
            #self.pointcloud_transform = tfu.transform(self.base_frame, self.pointcloud_msg.header.frame_id, self.tflistener)


class JointMsgConverter:
    def __init__(self):
        self.name_dict = None
        self.joint_groups = ut.load_pickle('link_names.pkl')
        #self.joint_groups['rarm']      = rospy.get_param('/r_arm_controller/joints')
        #self.joint_groups['larm']      = rospy.get_param('/l_arm_controller/joints')
        #self.joint_groups['head_traj'] = rospy.get_param('/head_traj_controller/joints')
        #self.joint_groups['torso']     = rospy.get_param('/torso_controller/joints')

    def msgs_to_dict(self, msgs):
        converted = []
        for i in range(len(msgs)):
            msg = msgs[i]
            if self.name_dict == None:
                self.name_dict = {}
                for i, n in enumerate(msg.name):
                    self.name_dict[n] = i 

                self.joint_idx = {}
                for group in self.joint_groups.keys():
                    self.joint_idx[group] = [self.name_dict[name] for name in self.joint_groups[group]]

            joint_poses = {}
            joint_vel = {}
            joint_eff = {}
            #extract data for pose, vel, eff
            for d, data in zip([joint_poses, joint_vel, joint_eff], [msg.position, msg.velocity, msg.effort]):
                #look up values for each joint group
                dmat = np.matrix(data).T
                for group in self.joint_groups.keys():
                    d[group] = dmat[self.joint_idx[group], 0]
            converted.append({'poses': joint_poses, 'vels': joint_vel, 'efforts': joint_eff, 'time': msg.header.stamp.to_time()})

        #Take out wrapping of forearm & wrist
        for group in ['rarm', 'larm']:
            for i in range(len(self.joint_groups[group])):
                joint = self.joint_groups[group][i]
                if 'forearm_roll' in joint or 'wrist_roll' in joint:
                    delta = msgs[1].position[i] - msgs[0].position[i]
                    realdelta = delta % (2 * math.pi)
                    if realdelta >= math.pi:
                        realdelta -= 2 * math.pi
                    correction = delta - realdelta
                    for j in range(1, len(msgs)):
                        converted[j]['poses'][group][i,0] -= correction
                        #msgs[j].positions[i] -= correction
        return converted

##
# @param surf_locs list of ((x,y), lap, size, dir, hess)
# @param point_cloud_3d 3xn matrix
# @param point_cloud_2d 2xn matrix
def assign_3d_to_surf(surf_locs, point_cloud_3d, point_cloud_2d):
    point_cloud_2d_tree = sp.KDTree(np.array(point_cloud_2d.T))
    #print '>> shape of point_cloud_3d', point_cloud_3d.shape

    surf_loc3d = []
    for s in surf_locs:
        loc = s[0]
        idx = point_cloud_2d_tree.query(np.array(loc))[1]
        surf_loc3d.append(point_cloud_3d[:, idx])
        #print '   %s matched to %s 3d %s' % (str(loc), str(point_cloud_2d[:,idx].T), str(point_cloud_3d[:, idx].T))

    surf_loc3d = np.column_stack(surf_loc3d)
    return surf_loc3d

def find_contacts_and_fk(tflistener, arm):
    find_contact_locs = ExtractTFData(tflistener)
    while not rospy.is_shutdown() \
            and (not find_contact_locs.contact_stopped) \
            and ((time.time() - find_contact_locs.last_jstate_time) < 1.):
        print 'waiting for ExtractTFData to finish.'
        time.sleep(.5)
    print 'got %d joint state messages' % len(find_contact_locs.jstate_msgs)

    contact_locs = find_contact_locs.contact_locs
    if arm == 'right':
        contact_tips = [np.matrix(r[1][0]).T for r in contact_locs]
    else:
        contact_tips = [np.matrix(r[1][1]).T for r in contact_locs]
    contact_tips = np.column_stack(contact_tips)
    return contact_tips[:,0], find_contact_locs.jstate_msgs

def project_2d_bounded(cam_info, point_cloud_cam):
    point_cloud_2d_cam = cam_info.project(point_cloud_cam) 
    # only count points in image bounds (should be in cam info)
    _, in_bounds = np.where(np.invert((point_cloud_2d_cam[0,:] >= (cam_info.w-.6)) + (point_cloud_2d_cam[0,:] < 0) \
                                    + (point_cloud_2d_cam[1,:] >= (cam_info.h-.6)) + (point_cloud_2d_cam[1,:] < 0)))
    point_cloud_2d_cam = point_cloud_2d_cam[:, in_bounds.A1]
    point_cloud_reduced_cam = point_cloud_cam[:, in_bounds.A1]

    return point_cloud_2d_cam, point_cloud_reduced_cam

def find3d_surf(start_conditions):
    ## Project pointcloud into 2d
    point_cloud_bf = ru.pointcloud_to_np(start_conditions['points'])
    # from base_frame to prosilica frame
    point_cloud_pro = tfu.transform_points(start_conditions['pro_T_bf'], point_cloud_bf)
    point_cloud_2d_pro, point_cloud_reduced_pro = project_2d_bounded(start_conditions['camera_info'], point_cloud_pro)
    #point_cloud_2d_pro = .project(point_cloud_pro) 
    ## only count points in image bounds (should be in cam info)
    #cam_info = start_conditions['camera_info']
    #_, in_bounds = np.where(np.invert((point_cloud_2d_pro[0,:] >= (cam_info.w-.6)) + (point_cloud_2d_pro[0,:] < 0) \
    #                                + (point_cloud_2d_pro[1,:] >= (cam_info.h-.6)) + (point_cloud_2d_pro[1,:] < 0)))
    #point_cloud_2d_pro = point_cloud_2d_pro[:, in_bounds.A1]
    #point_cloud_reduced_pro = point_cloud_pro[:, in_bounds.A1]

    ## Find 3D SURF features
    model_file_name = start_conditions['model_image']
    model_surf_loc, model_surf_descriptors = fea.surf_color(cv.LoadImage(model_file_name))
    surf_loc3d_pro = np.matrix(assign_3d_to_surf(model_surf_loc, point_cloud_reduced_pro, point_cloud_2d_pro))
    return model_surf_loc, model_surf_descriptors, surf_loc3d_pro, point_cloud_2d_pro

##########################################################################
# TODO: need some parameters for processing 'model_image', maybe circles
# of different sizes.
def extract_object_localization_features2(start_conditions, tflistener, arm_used, p_base_map):
    mid_contact_bf, jstate_msgs = find_contacts_and_fk(tflistener, arm_used)
    model_surf_loc, model_surf_descriptors, surf_loc3d_pro, point_cloud_2d_pro = find3d_surf(start_conditions)

    #Find frame
    surf_loc3d_bf = (np.linalg.inv(start_conditions['pro_T_bf']) \
            * np.row_stack((surf_loc3d_pro, 1+np.zeros((1,surf_loc3d_pro.shape[1])))))[0:3,:]
    frame_bf = create_frame(surf_loc3d_bf, p=np.matrix([-1, 0, 0.]).T)
    center_bf = np.mean(surf_loc3d_bf, 1)

    #Find out what the SURF features point to in this new frame
    bf_R_pro = (start_conditions['pro_T_bf'][0:3, 0:3]).T
    bf_R_obj = frame_bf
    x_bf     = frame_bf[:,0]
    x_pro    = bf_R_pro.T * x_bf
    x_ang_pro = math.atan2(x_pro[1,0], x_pro[0,0])

    center_pro = tfu.transform_points(start_conditions['pro_T_bf'], center_bf)
    center2d_pro = start_conditions['camera_info'].project(center_pro)

    surf_directions = []
    surf_dir_center = []
    for loc, lap, size, direction, hess in model_surf_loc:
        surf_directions.append(ut.standard_rad(np.radians(direction) - x_ang_pro))
        direction_to_center = center2d_pro - np.matrix(loc).T
        surf_dir_center.append(direction_to_center)

    surf_dir_center = np.column_stack(surf_dir_center)
    return {
            'contact_bf': mid_contact_bf,
            'surf_loc3d_pro': surf_loc3d_pro,
            'surf_loc2d_pro': model_surf_loc,
            'point_cloud_2d_pro': point_cloud_2d_pro,

            'surf_directions': surf_directions, #Orientation
            'surf_pose_dir2d': surf_dir_center,   #Position
            'descriptors': model_surf_descriptors,

            'jtransforms': jstate_msgs,
            'frame_bf': frame_bf,
            'center_bf': center_bf
            }

##########################################################################
# TODO: need some parameters for processing 'model_image', maybe circles
# of different sizes.
def extract_object_localization_features(start_conditions, tflistener):
    ## Find contacts
    find_contact_locs = ExtractTFData(tflistener)
    r = rospy.Rate(10)
    while not rospy.is_shutdown() and not find_contact_locs.contact_stopped:
        r.sleep()
    contact_locs = find_contact_locs.contact_locs

    ## Detect features, get 3d location for each feature
    model_file_name = start_conditions['model_image']
    model_surf_loc, model_surf_descriptors = fea.surf_color(cv.LoadImage(model_file_name))

    ## Assign 3d location to surf features
    point_cloud_bf = ru.pointcloud_to_np(start_conditions['points'])
    point_cloud_pro = start_conditions['pro_T_bf'] * np.row_stack((point_cloud_bf, 1+np.zeros((1, point_cloud_bf.shape[1]))))
    point_cloud_2d_pro = start_conditions['camera_info'].project(point_cloud_pro[0:3,:])
    surf_loc3d_arr_bf = np.array(assign_3d_to_surf(model_surf_loc, point_cloud_bf, point_cloud_2d_pro))
    surf_loc_tree_bf = sp.KDTree(surf_loc3d_arr_bf.T)

    #################################################
    # not needed right now but can be useful later..
    #################################################
    # Get SURF features closest to contact locs
    left_contact, right_contact = zip(*[(np.matrix(r[1][2]).T, np.matrix(r[1][3]).T) for r in contact_locs])
    left_contact = np.column_stack(left_contact)
    right_contact = np.column_stack(right_contact)
    mid_contact_bf = (left_contact[:,0] + right_contact[:,0]) / 2.
    #data_dict['pro_T_bf']  * np.row_stack((mid_contact_bf, np

    surf_closest_idx = surf_loc_tree_bf.query(np.array(mid_contact_bf.T))[1] #Get surf feature at mid point
    surf_closest3d   = surf_loc3d_arr_bf[:, surf_closest_idx]
    surf_closest_fea = model_surf_loc[surf_closest_idx]

    #Create a frame for this group of features
    surf_loc_3d_pro = (start_conditions['pro_T_bf'] * np.row_stack([surf_loc3d_arr_bf, 1 + np.zeros((1, surf_loc3d_arr_bf.shape[1]))]))[0:3,:]
    object_frame_pro = create_frame(np.matrix(surf_loc_3d_pro))

    #Find out what the SURF features point to in this new frame
    surf_directions = []
    for loc, lap, size, direction, hess in model_surf_loc:
        drad = np.radians(direction)
        #project direction into the cannonical object frame
        surf_dir_obj = object_frame_pro * np.matrix([np.cos(drad), np.sin(drad), 0.]).T

        #measure angle between SURF feature and x axis of object frame, store this as delta theta
        delta_theta = math.atan2(surf_dir_obj[1,0], surf_dir_obj[0,0])
        surf_directions.append(delta_theta)

    return {
            'descriptors': model_surf_descriptors, 
            'directions': surf_directions, 
            'contact_bf': mid_contact_bf,
            'closest_feature': surf_closest_fea[0],
            #'object_frame_bf': [np.mean(np.matrix(surf_loc3d_arr_bf), 1), create_frame(surf_loc3d_arr_bf)],
            'object_frame_pro': [np.mean(np.matrix(surf_loc_3d_pro), 1), object_frame_pro, surf_loc_3d_pro]
            }

    #surf_dir_obj => obj_dir
    #project all points ontocz
    #draw this surf feature in image
    #proc_surfed = fea.draw_surf(proc_img, model_surf_loc, (200, 0, 0))
    #proc_surfed = fea.draw_surf(proc_surfed, [surf_closest_fea], (0,0,255))

#########################################################
# Pose estimation
#  # for each SURF direction, subtract delta_theta from it to get a vote for the direction of the object frame's
#    x axis
#  # average all the predictions to get object's x direction

def create_frame(points3d, p=np.matrix([-1,0,0.]).T):
    #pdb.set_trace()
    u, s, vh = np.linalg.svd(np.cov(points3d))
    u = np.matrix(u)

    # Pick normal
    if (u[:,2].T * p)[0,0] < 0:
        normal = -u[:,2]
    else:
        normal = u[:,2]

    # pick the next direction as the one closest to to +z or +x
    z_plus = np.matrix([0, 0, 1.0]).T
    x_plus = np.matrix([1, 0, 0.0]).T

    u0 = u[:,0]
    u1 = u[:,1]

    mags = []
    pos_dirs = []
    for udir in [u0, u1, -u0, -u1]:
        for can_dir in [z_plus, x_plus]:
            mags.append((udir.T * can_dir)[0,0])
            pos_dirs.append(udir)
    x_dir = pos_dirs[np.argmax(mags)]

    # Cross product for the final (is this the same as the final vector?)
    y_dir = np.cross(normal.T, x_dir.T).T
    return np.matrix(np.column_stack([x_dir, y_dir, normal]))

def process_bag(full_bag_name, prosilica_image_file, model_image_file, experiment_start_condition_pkl, arm_used='left'):
    bag_path, bag_name_ext = os.path.split(full_bag_name)
    filename, ext = os.path.splitext(bag_name_ext)
    
    ###############################################################################
    # Playback the bag
    bag_playback = Process(target=playback_bag, args=(full_bag_name,))
    bag_playback.start()

    ###############################################################################
    ## Listen for transforms using rosbag
    rospy.init_node('bag_proceessor')

    tl = tf.TransformListener()

    print 'waiting for transform'
    tl.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(20))
    # Extract the starting location map_T_bf
    p_base = tfu.transform('map', 'base_footprint', tl) \
            * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
    t, r = tfu.matrix_as_tf(p_base)
    pose_base = (t, r)
    print 'done with tf'

    ##########################################################
    ## Find contact locations
    start_conditions = ut.load_pickle(experiment_start_condition_pkl)
    start_conditions['highdef_image'] = prosilica_image_file
    start_conditions['model_image'] = model_image_file
    rospy.loginfo('extracting object localization features')
    start_conditions['pose_parameters'] = extract_object_localization_features2(start_conditions, tl, arm_used, p_base)

    if bag_playback.is_alive():
        rospy.loginfo('Terminating playback process')
        bag_playback.terminate()
        time.sleep(1)
        bag_playback.terminate()
        time.sleep(1)
        rospy.loginfo('Playback process terminated? %s' % str(not bag_playback.is_alive()))


    ###############################################################################
    #Read bag using programmatic API
    pr2_kinematics = pr2k.PR2Kinematics(tl)
    converter = JointMsgConverter()
    rospy.loginfo('opening bag, reading state topics')
    topics_dict = ru.bag_sel(full_bag_name, ['/joint_states', '/l_cart/command_pose', 
                                             '/r_cart/command_pose', '/torso_controller/state',
                                             '/pressure/l_gripper_motor', '/pressure/r_gripper_motor'])

    ## Select the arm that has been moving, segment joint states based on contact states.
    if arm_used == 'left':
        pressures = topics_dict['/pressure/l_gripper_motor']
    elif arm_used == 'right':
        pressures = topics_dict['/pressure/r_gripper_motor']
    else:
        raise RuntimeError('arm_used invalid')

    ## find contact times
    rospy.loginfo('Finding contact times')
    left_f, right_f, ptimes = hpr2.pressure_state_to_mat(pressures['msg'])

    ## create segments based on contacts
    # TODO: make this accept more contact stages
    contact_times = find_contact_times(left_f, right_f, ptimes, 250)
    if len(contact_times) > 2:
        time_segments = [['start', contact_times[0]], [contact_times[0], contact_times[-1]], [contact_times[-1], 'end']]
    else:
        time_segments = [['start', 'end']]

    rospy.loginfo('Splitting messages based on contact times')
    ## split pressure readings based on contact times
    pressure_lseg = segment_msgs(time_segments, topics_dict['/pressure/l_gripper_motor']['msg'])
    pressure_rseg = segment_msgs(time_segments, topics_dict['/pressure/r_gripper_motor']['msg'])

    ## split cartesian commands based on contact times
    lcart_seg = segment_msgs(time_segments, topics_dict['/l_cart/command_pose']['msg'])
    rcart_seg = segment_msgs(time_segments, topics_dict['/r_cart/command_pose']['msg'])

    ## split joint states
    joint_states = topics_dict['/joint_states']['msg']
    print 'there are %d joint state messages in bag' % len(joint_states)

    j_segs     = segment_msgs(time_segments, topics_dict['/joint_states']['msg'])
    jseg_dicts = [converter.msgs_to_dict(seg) for seg in j_segs]
    # find the first set of joint states
    j0_dict    = jseg_dicts[0][0]

    ## perform FK
    rospy.loginfo('Performing FK to find tip locations')
    bf_T_obj = htf.composeHomogeneousTransform(start_conditions['pose_parameters']['frame_bf'], 
                                               start_conditions['pose_parameters']['center_bf'])
    obj_T_bf = np.linalg.inv(bf_T_obj)
    for jseg_dict in jseg_dicts:
        for d in jseg_dict:
            rtip_bf = pr2_kinematics.right.fk('base_footprint',
                    'r_wrist_roll_link', 'r_gripper_tool_frame',
                    d['poses']['rarm'].A1.tolist())
            ltip_bf =  pr2_kinematics.left.fk('base_footprint',
                    'l_wrist_roll_link', 'l_gripper_tool_frame',
                    d['poses']['larm'].A1.tolist())
            rtip_obj = obj_T_bf * rtip_bf
            ltip_obj = obj_T_bf * ltip_bf

            d['rtip_obj'] = tfu.matrix_as_tf(rtip_obj)
            d['ltip_obj'] = tfu.matrix_as_tf(ltip_obj)

            d['rtip_bf'] = tfu.matrix_as_tf(rtip_bf)
            d['ltip_bf'] = tfu.matrix_as_tf(ltip_bf)
    
    ###############################################################################
    # make movement state dictionaries, one for each state
    movement_states = []
    for i, seg in enumerate(time_segments):
        name = "state_%d" % i
        start_times = [lcart_seg[i][0].header.stamp.to_time(), 
                       rcart_seg[i][0].header.stamp.to_time(), 
                       jseg_dicts[i][0]['time'],
                       pressure_lseg[i][0].header.stamp.to_time(), 
                       pressure_rseg[i][0].header.stamp.to_time()]

        sdict = {'name': name,
                 'start_time': np.min(start_times),
                 'cartesian': [[ru.ros_to_dict(ps) for ps in lcart_seg[i]], 
                               [ru.ros_to_dict(ps) for ps in rcart_seg[i]]],
                 'joint_states': jseg_dicts[i]
                 #'pressure': [pressure_lseg[i], pressure_rseg[i]]
                 } 

        movement_states.append(sdict)

    # store in a dict
    data = {'start_conditions': start_conditions, # ['camera_info', 'map_T_bf', 'pro_T_bf', 'points' (in base_frame), 
                                                  # 'highdef_image', 'model_image',
                                                    ## 'pose_parameters'
                                                        ## 'descriptors'
                                                        ## 'directions' (wrt to cannonical orientation)
                                                        ## 'closest_feature'
            'base_pose': pose_base, 
            'robot_pose': j0_dict,
            'arm': arm_used,
            'movement_states': movement_states}

    # save dicts to pickles
    processed_bag_name = '%s_processed.pkl' % os.path.join(bag_path, filename)
    rospy.loginfo('saving to %s' % processed_bag_name)
    ut.save_pickle(data, processed_bag_name)
    bag_playback.join()
    rospy.loginfo('finished!')


#python bag_processor.py 08_06/light/off/_2010-08-06-13-35-04.bag 08_06/light/off/off_start.png light_switch_model.png 08_06/light/off/off_start.pkl
if __name__ == '__main__':
    arm_used = 'left'
    full_bag_name                  = sys.argv[1]
    prosilica_image_file           = sys.argv[2]
    model_image_file               = sys.argv[3]
    experiment_start_condition_pkl = sys.argv[4]

    process_bag(full_bag_name, prosilica_image_file, model_image_file, experiment_start_condition_pkl)















































































   
#def create_frame2(contact_point, points3d, p=np.matrix([1,0,0.]).T):
### contact point is the center, local plane from 3D points close to contact
#    u, s, vh = np.linalg.svd(points3d)
#    u = np.matrix(u)
#    #pdb.set_trace()
#
#    # Pick normal
#    if (u[:,2].T * p)[0,0] < 0:
#        normal = -u[:,2]
#    else:
#        normal = u[:,2]
#
#    # pick the next direction as the one closest to to +z or +x
#    z_plus = np.matrix([0,0,1.0]).T
#    x_plus = np.matrix([1,0,0.0]).T
#
#    u0 = u[:,0]
#    u1 = u[:,1]
#
#    mags = []
#    pos_dirs = []
#    for udir in [u0, u1, -u0, -u1]:
#        for can_dir in [z_plus, x_plus]:
#            mags.append((udir.T * can_dir)[0,0])
#            pos_dirs.append(udir)
#    x_dir = pos_dirs[np.argmax(mags)]
#
#    # Cross product for the final (is this the same as the final vector?)
#    y_dir = np.cross(normal.T, x_dir.T).T
#    return np.column_stack([x_dir, y_dir, normal])










    #for loc, lap, size, direction, hess in model_surf_loc:
    #    drad = np.radians(direction)
    #    #project direction into the cannonical object frame
    #    #surf_dir_obj = object_frame_pro * np.matrix([np.cos(drad), np.sin(drad), 0.]).T
    #    #obj_R_bf = frame_bf.T

    #    bf_R_pro = (start_conditions['pro_T_bf'][0:3,0:3]).T
    #    surf_dir_obj = frame_bf.T * bf_R_pro * np.matrix([np.cos(drad), np.sin(drad), 0.]).T 

    #    #measure angle between SURF feature and x axis of object frame, store this as delta theta
    #    delta_theta = math.atan2(surf_dir_obj[1,0], surf_dir_obj[0,0])
    #    surf_directions.append(delta_theta)

    ##print 'frame_bf.T', frame_bf.T
    ##print 'bf_R_pro', (start_conditions['pro_T_bf'][0:3,0:3]).T










    #r = rospy.Rate(10)
    #while not rospy.is_shutdown() and not find_contact_locs.contact_stopped:
    #    r.sleep()
    #contact_locs = find_contact_locs.contact_locs
    #et = ExtractTFData()
    # ['camera_info', 'map_T_bf', 'pro_T_bf', 'points']







    #print 'got loc %.2f %.2f %.2f'% (t[0], t[1], t[2])
    #loc_fname = '%s_loc.pkl' % os.path.join(bag_path, filename)
    #print 'saving to %s' % loc_fname
    #ut.save_pickle((t,r), loc_fname)


    #print contact_times - contact_times[0]
    #print contact_times[1:] - contact_times[:-1]
    #pb.plot(contact_times-contact_times[0], 'g.')
    #pb.show()



# pose_base = [t, r], t is len3, r is len4
# j0_dict {'poses': joint_poses, 'vels': joint_vel, 'efforts': joint_eff, 'time': msg.header.stamp.to_time()}
#         with each entry having keys: ['rarm'] ['larm'] ['head_traj'] ['torso']    
# arm is a string {'left', 'right'}
# movement_states
#       'name'
#       'start_time'
#       'cartesian'
#       'joint_states'
#       'pressure'


























            #'movement_states': [{'name': #,
            #                     'cartesian':#,
            #                     'joint_states': # j_dicts, j_times
            #                     'pressure': #,
            #                     }]
            #                      #}
    
    ##ut.save_pickle(data, extracted_name)


    #if len(joint_states) <= 1:
    #    raise RuntimeError('Not enough joint state messages.  Got %d messages.' % len(joint_states))
    #joint_states)
