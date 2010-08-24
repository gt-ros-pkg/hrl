import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import sys
import hrl_lib.util as ut
import hrl_lib.rutils as ru

import tf
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
import pdb
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

# Find contact points & transform gripper tip to base_frame
class ListenAndFindContactLocs:
    def __init__(self, listener=None):#, pointcloud_msg):
        #rospy.init_node('contact3d')
        rospy.Subscriber('/pressure/l_gripper_motor', pm.PressureState, self.lpress_cb)
        self.ftip_frames = ['r_gripper_l_finger_tip_link',
                            'r_gripper_r_finger_tip_link',
                            'l_gripper_l_finger_tip_link',
                            'l_gripper_r_finger_tip_link']

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
        #self.pointcloud_msg = pointcloud_msg


    def lpress_cb(self, pmsg):
        #print 'called'
        #conv to mat
        lmat = np.matrix((pmsg.l_finger_tip)).T
        rmat = np.matrix((pmsg.r_finger_tip)).T
        if self.lmat0 == None:
            self.lmat0 = lmat
            self.rmat0 = rmat
            return

        #zero
        lmat = lmat - self.lmat0
        rmat = rmat - self.rmat0
   
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
# @param point_cloud 3xn matrix
# @param point_cloud_2d 2xn matrix
def assign_3d_to_surf(surf_locs, point_cloud, point_cloud_2d):
    point_cloud_2d_tree = sp.KDTree(np.array(point_cloud_2d.T))

    surf_loc3d = []
    for loc, lap, size, d, hess in surf_locs:
        idx = point_cloud_2d_tree.query(np.array(loc))[1]
        surf_loc3d.append(point_cloud[:, idx])
    surf_loc3d = np.column_stack(surf_loc3d)
    return surf_loc3d


##########################################################################
# TODO: need some parameters for processing 'model_image', maybe circles
# of different sizes.
def extract_object_localization_features(start_conditions, tflistener):
    ## Find contacts
    find_contact_locs = ListenAndFindContactLocs(tflistener)
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
    surf_loc3d_arr = np.array(assign_3d_to_surf(model_surf_loc, point_cloud_bf, point_cloud_2d_pro))
    surf_loc_tree_bf = sp.KDTree(surf_loc3d_arr.T)

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
    surf_closest3d   = surf_loc3d_arr[:, surf_closest_idx]
    surf_closest_fea = model_surf_loc[surf_closest_idx]

    #Create a frame for this group of features
    surf_loc_3d_pro = (start_conditions['pro_T_bf'] * np.row_stack([surf_loc3d_arr, 1 + np.zeros((1, surf_loc3d_arr.shape[1]))]))[0:3,:]
    object_frame = create_frame(np.matrix(surf_loc_3d_pro))

    #Find out what the SURF features point to in this new frame
    surf_directions = []
    for loc, lap, size, direction, hess in model_surf_loc:
        drad = np.radians(direction)
        #project direction into the cannonical object frame
        surf_dir_obj = object_frame * np.matrix([np.cos(drad), np.sin(drad), 0.]).T

        #measure angle between SURF feature and x axis of object frame, store this as delta theta
        delta_theta = math.atan2(surf_dir_obj[1,0], surf_dir_obj[0,0])
        surf_directions.append(delta_theta)

    return {'descriptors': model_surf_descriptors, 
            'directions': surf_directions, 
            'closest_feature': surf_closest_fea[0]}

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

def create_frame(points3d, p=np.matrix([0,0,1.]).T):
    u, s, vh = np.linalg.svd(points3d)
    u = np.matrix(u)

    # Pick normal
    if (u[:,2].T * p)[0,0] < 0:
        normal = -u[:,2]
    else:
        normal = u[:,2]

    # pick the next direction as the one closest to to +z or +x
    z_plus = np.matrix([0,0,1.0]).T
    x_plus = np.matrix([1,0,0.0]).T

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
    return np.column_stack([x_dir, y_dir, normal])
    
    


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
    tl.waitForTransform('map', 'base_footprint', rospy.Time.now(), rospy.Duration(20))
    # Extract the starting location
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
    start_conditions['pose_parameters'] = extract_object_localization_features(start_conditions, tl)

    #r = rospy.Rate(10)
    #while not rospy.is_shutdown() and not find_contact_locs.contact_stopped:
    #    r.sleep()
    #contact_locs = find_contact_locs.contact_locs

    #et = ListenAndFindContactLocs()
    # ['camera_info', 'map_T_bf', 'pro_T_bf', 'points']

    if bag_playback.is_alive():
        rospy.loginfo('Terminating playback process')
        bag_playback.terminate()
        time.sleep(1)
        bag_playback.terminate()
        time.sleep(1)
        rospy.loginfo('Playback process terminated? %s' % str(not bag_playback.is_alive()))

    #print 'got loc %.2f %.2f %.2f'% (t[0], t[1], t[2])
    #loc_fname = '%s_loc.pkl' % os.path.join(bag_path, filename)
    #print 'saving to %s' % loc_fname
    #ut.save_pickle((t,r), loc_fname)

    ###############################################################################
    #Read bag using programmatic API
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

    rospy.loginfo('Finding contact times')
    left_f, right_f, ptimes = ru.pressure_state_to_mat(pressures['msg'])
    #TODO: make this accept more contact stages
    contact_times = find_contact_times(left_f, right_f, ptimes, 250)
    if len(contact_times) > 2:
        time_segments = [['start', contact_times[0]], [contact_times[0], contact_times[-1]], [contact_times[-1], 'end']]
    else:
        time_segments = [['start', 'end']]

    pressure_lseg = segment_msgs(time_segments, topics_dict['/pressure/l_gripper_motor']['msg'])
    pressure_rseg = segment_msgs(time_segments, topics_dict['/pressure/r_gripper_motor']['msg'])

    lcart_seg = segment_msgs(time_segments, topics_dict['/l_cart/command_pose']['msg'])
    rcart_seg = segment_msgs(time_segments, topics_dict['/r_cart/command_pose']['msg'])

    #print contact_times - contact_times[0]
    #print contact_times[1:] - contact_times[:-1]
    #pb.plot(contact_times-contact_times[0], 'g.')
    #pb.show()

    #Find the first robot pose
    ## Convert from joint state to dicts
    joint_states = topics_dict['/joint_states']['msg']
    j_segs     = segment_msgs(time_segments, topics_dict['/joint_states']['msg'])
    jseg_dicts = [converter.msgs_to_dict(seg) for seg in j_segs]
    j0_dict    = jseg_dicts[0][0]
    
    #converter.msg_to_dict(j_segs[0][0])
    #jseg_dicts = [[converter.msg_to_dict(j_msg) for j_msg in seg] for seg in j_segs]
    
    ###############################################################################
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

    ###store in a dict
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

    processed_bag_name = '%s_processed.pkl' % os.path.join(bag_path, filename)
    rospy.loginfo('saving to %s' % processed_bag_name)
    ut.save_pickle(data, processed_bag_name)
    bag_playback.join()
    rospy.loginfo('finished!')


#python bag_processor.py 08_06/light/off/_2010-08-06-13-35-04.bag 08_06/light/off/off_start.png light_switch_model.png 08_06/light/off/off_start.pkl
if __name__ == '__main__':
    import pylab as pb

    arm_used = 'left'
    full_bag_name                  = sys.argv[1]
    prosilica_image_file           = sys.argv[2]
    model_image_file               = sys.argv[3]
    experiment_start_condition_pkl = sys.argv[4]

    process_bag(full_bag_name, prosilica_image_file, model_image_file, experiment_start_condition_pkl)

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
