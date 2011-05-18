#! /usr/bin/python
import roslib; roslib.load_manifest('hai_sandbox')
import rospy
#import hrl_pr2_lib.pr2_kinematics as pk

#import pr2_msgs.msg as pm
#import pr2_gripper_reactive_approach.reactive_grasp as rgr
#import pr2_gripper_reactive_approach.controller_manager as con
#from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorGoal
#import object_manipulator.convert_functions as cf
#from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Point 
import tf.transformations as tr
from std_msgs.msg import String
import visualization_msgs.msg as vm
#import sensor_msgs.msg as smsg
#import message_filters
import tf
import cv
import functools as ft

#import copy
import numpy as np
import math
import time
import subprocess as sb
import scipy.spatial as sp
import pdb
import os
import os.path as pt
import shutil
#import math

import hrl_camera.ros_camera as rc
import hrl_pr2_lib.pr2 as pr2
import hrl_pr2_lib.devices as hd
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
#import hrl_lib.prob as pr
import hrl_pr2_lib.linear_move as lm
#import hrl_pr2_lib.devices as de

#import hrl_pr2_lib.collision_monitor as cmon
import hai_sandbox.recognize_3d as r3d
import hrl_lib.image3d as i3d
import hrl_lib.viz as viz
import cProfile
import threading
import dynamic_reconfigure.client as dr
#from hai_sandbox.recognize_3d import InterestPointDataset
#import psyco
#psyco.full()

#import hai_sandbox.interest_point_actions as ipa
#import hai_sandbox.kinect_listener as kl

#from sound_play.msg import SoundRequest

class TaskError(Exception):
    def __init__(self, value):
        self.parameter = value

    def __str__(self):
        return repr(self.parameter)

class ActionType:
    def __init__(self, inputs, outputs):
        self.inputs = inputs
        self.outputs = outputs

class ParamType:
    def __init__(self, name, ptype, options=None):
        self.name = name
        self.ptype = ptype
        self.options = options

class Action:

    def __init__(self, name, params):
        self.name = name
        self.params = params

class BehaviorDescriptor:

    def __init__(self):
        self.descriptors = {
                            'twist':       ActionType([ParamType('angle', 'radian')], [ParamType('success', 'bool')]),
                            'linear_move': ActionType([ParamType('start_loc', 'se3'), 
                                                       ParamType('movement', 'r3'), 
                                                       ParamType('stop', 'discrete', ['pressure', 'pressure_accel'])], 
                                                      [ParamType('success', 'bool')]),
                            }

        start_location = (np.matrix([0.3, 0.15, 0.9]).T, np.matrix([0., 0., 0., 0.1]))
        movement       = np.matrix([.4, 0, 0.]).T
        self.seed = [Action('linear_move', [start_location, movement, 'pressure']),
                     Action('linear_move', [Action('current_location', [])])]
        self.run(self.seed)

    def run(self, seed):
        pass

#TODO move this
class LaserPointerClient:
    def __init__(self, target_frame='/base_link', tf_listener=None, robot=None):
        self.dclick_cbs = []
        #self.dclick_cbs_raw = []
        self.point_cbs = []
        self.target_frame = target_frame
        self.laser_point_base = None
        self.robot = robot
        self.base_sound_path = (sb.Popen(["rospack", "find", "hai_sandbox"], stdout=sb.PIPE).communicate()[0]).strip()

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/beep.wav').play()

    def transform_point(self, point_stamped):
        point_head = point_stamped.point
        #Tranform into base link
        base_T_head = tfu.transform(self.target_frame, point_stamped.header.frame_id, self.tf_listener)
        point_mat_head = tfu.translation_matrix([point_head.x, point_head.y, point_head.z])
        point_mat_base = base_T_head * point_mat_head
        t_base, _ = tfu.matrix_as_tf(point_mat_base)
        return np.matrix(t_base).T
        
    def laser_point_handler(self, point_stamped):
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/blow.wav').play()
        self.laser_point_base = self.transform_point(point_stamped)
        for f in self.point_cbs:
            f(self.laser_point_base)

    def double_click_cb(self, a_str):
        rospy.loginfo('Double CLICKED')
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/beep.wav').play()
        #if self.laser_point_base != None:
        for f in self.dclick_cbs:
            f(self.laser_point_base)
        self.laser_point_base = None
        #for f in self.dclick_cb_raw(

    def add_double_click_cb(self, func):
        self.dclick_cbs.append(func)

    #def add_double_click_cb_raw(self, func):
    #    self.dclick_cbs_raw.append(func)

    def add_point_cb(self, func):
        self.point_cbs.append(func)


def image_diff_val2(before_frame, after_frame):
    br = np.asarray(before_frame)
    ar = np.asarray(after_frame)
    max_sum = br.shape[0] * br.shape[1] * br.shape[2] * 255.
    sdiff = np.abs((np.sum(br) / max_sum) - (np.sum(ar) / max_sum))
    #sdiff = np.sum(np.abs(ar - br)) / max_sum
    return sdiff


class ManipulationBehaviors:

    def __init__(self, arm, pr2_obj, tf_listener=None):
        try:
            rospy.init_node('linear_move', anonymous=True)
        except Exception, e:
            rospy.loginfo('call to init_node failed %s' % str(e))
        self.movement = lm.LinearReactiveMovement(arm, pr2_obj, tf_listener)

    ##
    # reach direction
            #move_back_distance, \
    def reach(self, point, pressure_thres,\
            reach_direction=np.matrix([0,0,0]).T, orientation=None):
        MOVEMENT_TOLERANCE = .1
        #REACH_TOLERANCE = .1

        #self.movement.set_movement_mode_cart()
        #pdb.set_trace()
        self.movement.set_pressure_threshold(pressure_thres)
        loc_bl = self.movement.arm_obj.pose_cartesian_tf()[0]
        front_loc = point.copy()
        front_loc[0,0] = max(loc_bl[0,0], .4)
        #front_loc[0,0] = loc_bl[0,0]
        #pdb.set_trace()

        if orientation == None:
            start_loc = self.movement.arm_obj.pose_cartesian_tf()
            orientation = start_loc[1]
        self.movement.pressure_listener.rezero()
        #pdb.set_trace()
        #for i in range(2):
        r1, residual_error = self.movement.move_absolute((front_loc, orientation), stop='pressure', pressure=pressure_thres)

        #if residual_error > MOVEMENT_TOLERANCE or r1 != None: #if this step fails, we move back then return
        #    #self.move_absolute(start_loc, stop='accel')
        #    pdb.set_trace()
        #    return False, r1, None

        #We expect impact here
        pos_error = None
        try:
            #pdb.set_trace()
            #loc_bl = self.movement.current_location()[0]
            #reach_direction = loc_bl - point 
            #reach_direction = reach_direction / np.linalg.norm(reach_direction)
            point_reach = point + reach_direction
            r2, pos_error = self.movement.move_absolute((point_reach, \
                    self.movement.arm_obj.pose_cartesian_tf()[1]), stop='pressure_accel', pressure=pressure_thres)

        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
            r2 = None

        touch_loc_bl = self.movement.arm_obj.pose_cartesian_tf()
        #if r2 == None or r2 == 'pressure' or r2 == 'accel' or (pos_error < (MOVEMENT_TOLERANCE + np.linalg.norm(reach_direction))):
        if r2 == 'pressure' or r2 == 'accel' or (pos_error != None and (pos_error < (MOVEMENT_TOLERANCE + np.linalg.norm(reach_direction)))):
            self.movement.pressure_listener.rezero()

            # b/c of stiction & low gains, we can't move precisely for small
            # distances, so move back then move forward again
            #reach_dir = reach_direction / np.linalg.norm(reach_direction)
            cur_pose, cur_ang = self.movement.arm_obj.pose_cartesian_tf()

            #p1 = cur_pose - (reach_dir * .05)
            #p2 = cur_pose + move_back_distance
            #rospy.loginfo('moving back')
            #_, pos_error = self.movement.move_absolute((p1, cur_ang), stop='None', pressure=pressure_thres)
            #self.movement.pressure_listener.rezero()

            #rospy.loginfo('moving forward again')
            #_, pos_error = self.movement.move_absolute((p2, cur_ang), stop='None', pressure=pressure_thres)

            #self.movement.move_relative_gripper(4.*move_back_distance, stop='none', pressure=pressure_thres)
            #self.movement.move_relative_gripper(-3.*move_back_distance, stop='none', pressure=pressure_thres)
            #self.movement.pressure_listener.rezero()
            return True, r2, touch_loc_bl
        else:
            #pdb.set_trace()
            #shouldn't get here
            return False, r2, None

    def press(self, direction, press_pressure, contact_pressure):
        #make contact first
        self.movement.set_movement_mode_cart()
        #pdb.set_trace()
        r1, diff_1 = self.movement.move_relative_gripper(direction, stop='pressure', pressure=contact_pressure)
        #now perform press
        if r1 == 'pressure' or r1 == 'accel':
            self.movement.set_movement_mode_cart()
            r2, diff_2 = self.movement.move_relative_gripper(direction, stop='pressure_accel', pressure=press_pressure)
            if r2 == 'pressure' or r2 == 'accel' or r2 == None:
                return True, r2
            else:
                return False, r2
        else:
            return False, r1


    #def twist(self, angle, stop):
    #    pos, rot = self.movement.cman.return_cartesian_pose() # in base_link
    #    ax, ay, az = tr.euler_from_quaternion(rot) 
    #    nrot = tr.quaternion_from_euler(ax + angle, ay, az)
    #    stop_funcs = self.movement._process_stop_option(stop)
    #    #return self._move_cartesian(np.matrix(pos).T, np.matrix(nrot).T, 
    #    #                            stop_funcs, timeout=self.timeout, settling_time=5.0)
    #    return self.movement._move_cartesian(np.matrix(pos).T, np.matrix(nrot).T, \
    #            stop_funcs, timeout=self.timeout, settling_time=5.0)

###
# One large dictionary indexed by unique ids contain location records.
# These ids are related by spatial location which we use to find locations.
# Operations supported
#
#   0)   loading the locations database
#   0.1) saving locations database
#   1)   adding locations
#   2)   querying given point
#   3)   listing all locations
#             (name, ids)
#   4)   listing locations closest to given point
#             (name, ids)
###
def separate_by_labels(points, labels):
    pidx = np.where(labels == r3d.POSITIVE)[1].A1.tolist()
    nidx = np.where(labels == r3d.NEGATIVE)[1].A1.tolist()
    uidx = np.where(labels == r3d.UNLABELED)[1].A1.tolist()
    return points[:, uidx], points[:, pidx], points[:, nidx]


class LocationDisplay(threading.Thread):

    def __init__(self, loc_man): 
        threading.Thread.__init__(self)
        try:
            rospy.init_node('location_display')
        except Exception,e:
            print e

        self.location_label_pub  = rospy.Publisher('location_label', vm.Marker)
        self.location_marker_pub = rospy.Publisher('location_marker', vm.Marker)
        self.loc_man = loc_man

    def run(self):
        text_scale = .1
        text_color = np.matrix([1,0,0,1.]).T

        circle_radii = .9
        circle_scale = .2
        circle_z = .03
        circle_color = np.matrix([1,0,0,1.]).T

        circle_msgs = []
        data = self.loc_man.data
        pdb.set_trace()
        for task_id in data.keys():
            point_map = data[task_id]['center']
            circle_msgs.append(viz.circle_marker(point_map, circle_radii, circle_scale, circle_color, 'map', circle_z))
            
        text_msgs = []
        for task_id in data.keys():
            point_map = data[task_id]['center']
            text_msgs.append(viz.text_marker(task_id, point_map, text_color, text_scale, 'map'))

        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            for c in circle_msgs:
                self.location_marker_pub.publish(c)
            for txt in text_msgs:
                self.location_label_pub.publish(txt)
            r.sleep()



class LocationManager:

    def __init__(self, name, rec_params):
        self.RELIABILITY_RECORD_LIM = 20
        self.RELIABILITY_THRES = .9

        self.rec_params = rec_params
        self.saved_locations_fname = name
        self.LOCATION_ADD_RADIUS = .5
        self.tree = None   #spatial indexing tree
        self.centers = None #3xn array of centers

        self.ids = [] #ids that corresponds with centers that indexes into data
        self.data = {} 

        self.learners = {}
        self._load_database()
        #pdb.set_trace()
        #pdb.set_trace()
        self.image_pubs = {}
        #self.revert()
        #pdb.set_trace()

        #self.save_database()
        #exit()
        for k in self.data.keys():
            self.train(k)
            self.image_pubs[k] = r3d.ImagePublisher(k.replace(':', '_'))
        #self.task_types = ['light_switch', 'light_rocker', 'drawer']
        self.task_types = ['light_switch_down', 'light_switch_up', 
                            'light_rocker_down', 'light_rocker_up', 
                            'push_drawer', 'pull_drawer']
        self.task_pairs = [['light_switch_down', 'light_switch_up'], 
                           ['light_rocker_down', 'light_rocker_up'],
                           ['pull_drawer', 'push_drawer']]
        self.driving_param = {'light_switch_up':   {'coarse': .9, 'fine': .6, 'voi': .4},
                              'light_switch_down': {'coarse': .9, 'fine': .6, 'voi': .4},

                              'light_rocker_down': {'coarse': .9, 'fine': .6, 'voi': .4},
                              'light_rocker_up':   {'coarse': .9, 'fine': .6, 'voi': .4},

                              'pull_drawer':       {'coarse': .9, 'fine': .5, 'voi': .4},
                              'push_drawer':       {'coarse': .9, 'fine': .5, 'voi': .4}}

    def revert(self):
        self.centers = self.centers[:, 0:4]
        self.ids = self.ids[0:4]
        self.data.pop('office_push_drawer')
        self.data.pop('office_pull_drawer')
        #pdb.set_trace()

    def _load_database(self):
        #                           tree         dict
        #indexes of locations in tree => ids list => location data
        if not os.path.isfile(self.saved_locations_fname):
            return
        d = ut.load_pickle(self.saved_locations_fname)
        self.ids = d['ids']
        self.centers = d['centers']
        self.data = d['data']
        self.tree = sp.KDTree(np.array(self.centers).T)

    def save_database(self):
        print 'Saving pickle. DONOT INTERRUPPT!!!'
        d = {'centers': self.centers,
            'ids': self.ids,
            'data': self.data}
        try:
            shutil.copyfile(self.saved_locations_fname, 
                    time.strftime('%m_%d_%Y_%I_%M%p') + '_locations.pkl')
        except Exception, e:
            print e

        ut.save_pickle(d, self.saved_locations_fname)
        print 'SAFE!!!'
        #rospy.loginfo('LocationManager: save_database saved db!')

    def get_complementary_task(self, tasktype):
        for ta, tb in self.task_pairs:
            if ta == tasktype:
                return tb
            if tb == tasktype:
                return ta
        return None

    def update_base_pose(self, taskid, base_pose):
        print 'updating base pose for task', taskid
        self.data[taskid]['base_pose'] = base_pose

    def create_new_location(self, task_type, point_map, base_pose, gather_data=True, name=None):
        if name == None:
            taskid = time.strftime('%A_%m_%d_%Y_%I:%M%p') + ('_%s' % task_type)
        else:
            taskid = name + ('_%s' % task_type)

        try:
            os.mkdir(taskid)
        except OSError, e:
            print e

        if self.centers == None:
            self.centers = point_map
        else:
            self.centers = np.column_stack((self.centers, point_map))
        self.tree = sp.KDTree(np.array(self.centers.T))

        self.ids.append(taskid)
        self.data[taskid] = {'task': task_type,
                             'center': point_map,
                             'base_pose': base_pose,
                             'points': point_map,
                             'dataset': None,
                             'dataset_raw': None,
                             'gather_data': gather_data,
                             'complementary_task_id': None,
                             'pca': None,
                             'execution_record': []}
        self.image_pubs[taskid] = r3d.ImagePublisher(taskid.replace(':', '_'))
        self.save_database()
        return taskid

    def record_time(self, task_id, record_name, value):
        if not self.data[task_id].has_key('times'):
            self.data[task_id]['times'] = {}

        if not self.data[task_id]['times'].has_key(record_name):
            self.data[task_id]['times'][record_name] = []

        self.data[task_id]['times'][record_name].append(value)

    def update_execution_record(self, taskid, value):
        self.data[taskid]['execution_record'].append(value)

    def is_reliable(self, taskid):
        record = self.data[taskid]['execution_record']
        if len(record) < self.RELIABILITY_RECORD_LIM:
            return False

        if np.sum(record) < (self.RELIABILITY_RECORD_LIM * self.RELIABILITY_THRES):
            return False

        return True

    def _id_to_center_idx(self, task_id):
        for i, tid in enumerate(self.ids):
            if tid == task_id:
                return i
        return None

    def add_perceptual_data(self, task_id, fea_dict):
        rospy.loginfo('LocationManager: add_perceptual_data - %s adding %d instance(s)' \
                % (task_id, fea_dict['labels'].shape[1]))
        current_raw_dataset = self.data[task_id]['dataset_raw']
        current_dataset = self.data[task_id]['dataset']

        self.data[task_id]['dataset_raw'] = \
                r3d.InterestPointDataset.add_to_dataset(
                        current_raw_dataset, fea_dict['instances'], 
                        fea_dict['labels'], fea_dict['points2d'], 
                        fea_dict['points3d'], None, None, 
                        sizes=fea_dict['sizes'])

        self.data[task_id]['dataset'] = \
                r3d.InterestPointDataset.add_to_dataset(
                        current_dataset, fea_dict['instances'], 
                        fea_dict['labels'], fea_dict['points2d'], 
                        fea_dict['points3d'], None, None, 
                        sizes=fea_dict['sizes'])

    def get_perceptual_data(self, task_id):
        return self.data[task_id]['dataset']

    def remove_perceptual_data(self, task_id, instance_idx):
        self.data[task_id]['dataset'].remove(instance_idx)
        self.data[task_id]['dataset_raw'].remove(instance_idx)

    def active_learn_add_data(self, task_id, fea_dict):
        #TODO: do something smarter here
        self.add_perceptual_data(task_id, fea_dict)

    def update(self, task_id, point_map):
        #If close by locations found then add to points list and update center
        ldata = self.data[task_id]
        ldata['points'] = np.column_stack((point_map, ldata['points']))
        ldata['center'] = ldata['points'].mean(1)

        center_idx = self._id_to_center_idx(task_id)
        self.centers[:, center_idx] = ldata['center']
        self.tree = sp.KDTree(np.array(self.centers).T)

    def set_center(self, task_id, point_map):
        ldata = self.data[task_id]
        ldata['points'] = point_map
        ldata['center'] = point_map
        center_idx = self._id_to_center_idx(task_id)
        self.centers[:, center_idx] = ldata['center']
        self.tree = sp.KDTree(np.array(self.centers).T)

    def publish_image(self, task_id, image, postfix=''):
        self.image_pubs[task_id].publish(image)
        ffull = pt.join(task_id, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + postfix + '.jpg')
        cv.SaveImage(ffull, image)

    def list_all(self):
        rlist = []
        for k in self.data.keys():
            rlist.append([k, self.data[k]['task']])
        return rlist

    def list_close_by(self, point_map, task=None):
        if self.tree != None:
            indices = self.tree.query_ball_point(np.array(point_map.T), self.LOCATION_ADD_RADIUS)[0]
            print 'list_close_by: indices close by', indices
            #pdb.set_trace()
            ids_selected = []
            for i in indices:
                sid = self.ids[i]
                stask = self.data[sid]['task']
                if task == None:
                    ids_selected.append([sid, stask])
                else:
                    if task == stask:
                        ids_selected.append([sid, stask])
            return ids_selected
        else:
            return []

    def train_all_classifiers(self):
        for k in self.data.keys():
            self.train(k)

    def train(self, task_id, dset_for_pca=None, save_pca_images=True):
        dataset = self.data[task_id]['dataset']
        rec_params = self.rec_params
        #pdb.set_trace()
        if dataset == None:
            return

        #rec_params = self.feature_ex.rec_params
        nneg = np.sum(dataset.outputs == r3d.NEGATIVE) #TODO: this was copied and pasted from r3d
        npos = np.sum(dataset.outputs == r3d.POSITIVE)
        print '================= Training ================='
        print 'NEG examples', nneg
        print 'POS examples', npos
        print 'TOTAL', dataset.outputs.shape[1]
        neg_to_pos_ratio = float(nneg)/float(npos)
        weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
        print 'training'
        #learner = r3d.SVMPCA_ActiveLearner(use_pca=True)
        previous_learner = None
        if self.learners.has_key(task_id):
            previous_learner = self.learners[task_id]
        #pdb.set_trace()
        learner = r3d.SVMPCA_ActiveLearner(use_pca=True, 
                        reconstruction_std_lim=self.rec_params.reconstruction_std_lim, 
                        reconstruction_err_toler=self.rec_params.reconstruction_err_toler,
                        old_learner=previous_learner, pca=self.data[task_id]['pca'])

        #TODO: figure out something scaling inputs field!
        if dset_for_pca != None:
            inputs_for_pca = dset_for_pca['instances']
        else:
            #inputs_for_pca = self.data[task_id]['pca'].pca_data
            inputs_for_pca = dataset.inputs

        learner.train(dataset, 
                      inputs_for_pca,
                      rec_params.svm_params + weight_balance,
                      rec_params.variance_keep)

        self.data[task_id]['pca'] = learner.pca
        self.learners[task_id] = learner
        if save_pca_images:
            #pdb.set_trace()
            basis = learner.pca.projection_basis
            cv.SaveImage('%s_pca.png' % task_id, r3d.instances_to_image(self.rec_params.win_size, basis, np.min(basis), np.max(basis)))

        #self.learners[task_id] = {'learner': learner, 'dataset': dataset}



class ApplicationBehaviors:

    def __init__(self):

        rospy.init_node('linear_move', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.robot = pr2.PR2(self.tf_listener, base=True)
        self.behaviors = ManipulationBehaviors('l', self.robot, tf_listener=self.tf_listener)
        self.laser_scan = hd.LaserScanner('point_cloud_srv')

        self.prosilica = rc.Prosilica('prosilica', 'polled')
        self.prosilica_cal = rc.ROSCameraCalibration('/prosilica/camera_info')

        self.wide_angle_camera_left = rc.ROSCamera('/wide_stereo/left/image_rect_color')
        self.wide_angle_configure = dr.Client('wide_stereo_both')

        self.laser_listener = LaserPointerClient(tf_listener=self.tf_listener, robot=self.robot)
        self.laser_listener.add_double_click_cb(self.click_cb)
        self.rec_params = r3d.Recognize3DParam()
        
        self.OPTICAL_FRAME = 'high_def_optical_frame'
        self.feature_ex = r3d.NarrowTextureFeatureExtractor(self.prosilica, 
                hd.PointCloudReceiver('narrow_stereo_textured/points'),
                self.prosilica_cal, 
                self.robot.projector,
                self.tf_listener, self.rec_params)
        #self.feature_ex = r3d.LaserScannerFeatureExtractor(self.prosilica, self.laser_scan, 
        #        self.prosilica_cal, self.tf_listener, self.rec_params)
        #self.feature_ex = r3d.KinectFeatureExtractor(self.tf_listener, rec_params=self.rec_params)

        self.critical_error = False
        #TODO: define start location in frame attached to torso instead of base_link
        self.start_location_light_switch = (np.matrix([0.35, 0.30, 1.1]).T, np.matrix([0., 0., 0., 0.1]))
        self.start_location_drawer       = (np.matrix([0.20, 0.40, .8]).T,  
                                            np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0)))
        self.folded_pose = np.matrix([ 0.10134791, -0.29295995,  0.41193769]).T
        self.create_arm_poses()
        self.learners = {}

        #self.load_classifier('light_switch', 'friday_730_light_switch2.pkl')
        #self.img_pub = r3d.ImagePublisher('active_learn')
        #self.location_display = LocationDisplay(self.locations_man)
        #self.location_display.start()
        #self.robot.projector.set_prosilica_inhibit(True)
        #pdb.set_trace()
        self.driving_posture('light_switch_down')
        self.robot.projector.set(False)
        self.locations_man = LocationManager('locations_narrow_v11.pkl', rec_params=self.rec_params)

    def draw_dots_nstuff(self, img, points2d, labels, picked_loc):
        pidx = np.where(labels == r3d.POSITIVE)[1].A1.tolist()
        nidx = np.where(labels == r3d.NEGATIVE)[1].A1.tolist()
        uidx = np.where(labels == r3d.UNLABELED)[1].A1.tolist()

        if picked_loc != None:
            r3d.draw_points(img, picked_loc, [255, 0, 0], 4, -1)

        #scale = 1
        if len(uidx) > 0:
            upoints = points2d[:, uidx]
            r3d.draw_points(img, upoints, [255,255,255], 2, -1)

        if len(nidx) > 0:
            npoints = points2d[:, nidx]
            r3d.draw_points(img, npoints, [0,0,255], 2, -1)

        if len(pidx) > 0:
            ppoints = points2d[:, pidx]
            r3d.draw_points(img, ppoints, [0,255,0], 2, -1)

    def create_arm_poses(self):
        self.right_tucked = np.matrix([[-0.02362532,  1.10477102, -1.55669475, \
                -2.12282706, -1.41751231, -1.84175899,  0.21436806]]).T

        self.left_tucked = np.matrix([[ 0.05971848,  1.24980184,  1.79045674, \
                -1.68333801, -1.73430635, -0.09838841, -0.08641928]]).T

        #lift the right arm up a little bit
        self.r0 = np.matrix([[-0.22774141,  0.7735819 , -1.45102092, \
                -2.12152412, -1.14684579, -1.84850287,  0.21397648]]).T

        #left arm rotates
        self.l0 = np.matrix([[ 0.06021592,  1.24844832,  1.78901355, -1.68333801, 1.2, -0.10152105, -0.08641928]]).T

        #left arm moves out
        self.l1 = np.matrix([[0.94524406,  1.24726399,  1.78548574, -1.79148173,  1.20027637, -1.0, -0.08633226]]).T

        #left arm rotates outward a little more
        self.l2 = np.matrix([[ 1.53180837,  1.24362641,  1.78452361, -1.78829678,  1.1996979,-1.00446167, -0.08741998]]).T

    def untuck(self):
        #pdb.set_trace()
        if np.linalg.norm(self.robot.left.pose() - self.left_tucked) < .3:
            rospy.loginfo('untuck: not in tucked position.  Ignoring request')
            return
        #assume we are tucked
        self.behaviors.movement.set_movement_mode_ik()
        self.robot.right.set_pose(self.r0, 1.)
        self.robot.left.set_poses(np.column_stack([self.l0, self.l1, self.l2]), \
                                  np.array([1., 2., 3.]))
        self.robot.right.set_pose(self.right_tucked, 1.)
        self.behaviors.movement.set_movement_mode_cart()

    def tuck(self):
        #pdb.set_trace()
        if np.linalg.norm(self.robot.left.pose() - self.left_tucked) < .5:
            rospy.loginfo('tuck: Already tucked. Ignoring request.')
            return
        #lift the right arm up a little bit
        self.behaviors.movement.set_movement_mode_ik()
        self.robot.right.set_pose(self.r0, 1.)
        self.robot.left.set_poses(np.column_stack([self.l2, self.l1, self.l0, self.left_tucked]), \
                                  np.array([4., 5., 6., 7.]))
        self.robot.right.set_pose(self.right_tucked, 1.)
        self.behaviors.movement.set_movement_mode_cart()

    #def go_to_home_pose(self):
    #    #self.behaviors.movement.set_movement_mode_cart()
    #    return self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
    #    #self.behaviors.movement.set_movement_mode_ik()
    #    #return self.behaviors.movement.move_absolute(self.start_location, stop='pressure')

    def camera_change_detect(self, threshold, f, args):
        config = self.wide_angle_configure.get_configuration()
        config['auto_gain'] = False
        config['auto_exposure'] = False
        self.wide_angle_configure.update_configuration(config)

        #take before sensor snapshot
        start_pose = self.robot.head.pose()
        #pdb.set_trace()
        #self.robot.head.set_pose(np.radians(np.matrix([1.04, -20]).T), 1)
        self.robot.head.set_pose(np.radians(np.matrix([30., -20]).T), 1)
        time.sleep(4)
        for i in range(7):
            before_frame = self.wide_angle_camera_left.get_frame()
        cv.SaveImage('before.png', before_frame)
        f_return = f(*args)
        time.sleep(5)
        for i in range(7):
            after_frame = self.wide_angle_camera_left.get_frame()

        cv.SaveImage('after.png', after_frame)
        sdiff = image_diff_val2(before_frame, after_frame)
        self.robot.head.set_pose(start_pose, 1)
        self.robot.head.set_pose(start_pose, 1)
        time.sleep(3)        
        #take after snapshot
        #threshold = .03
        config['auto_gain'] = True
        config['auto_exposure'] = True
        self.wide_angle_configure.update_configuration(config)

        rospy.loginfo('camera difference %.4f (thres %.3f)' % (sdiff, threshold))
        if sdiff > threshold:
            rospy.loginfo('difference detected!')
            return True, f_return
        else:
            rospy.loginfo('NO differences detected!')
            return False, f_return

    def close_gripper(self):
        GRIPPER_CLOSE = .003
        #self.robot.left_gripper.open(True, position=GRIPPER_CLOSE)
        self.behaviors.movement.gripper_close()

    def open_gripper(self):
        GRIPPER_OPEN = .08
        #self.robot.left_gripper.open(True, position=GRIPPER_OPEN)
        self.behaviors.movement.gripper_open()

    def light_switch1(self, point, 
            point_offset, press_contact_pressure, 
            press_pressure, press_distance, visual_change_thres):

        try:
            #pdb.set_trace()
            #print '===================================================================='
            #point = point + point_offset 
            rospy.loginfo('reaching to ' + str(point.T))
            #pdb.set_trace()
            #self.behaviors.movement.gripper_close()
            self.close_gripper()
            #self.robot.left_gripper.open(True, position=.005)
            time.sleep(1)
            self.behaviors.movement.pressure_listener.rezero()
            #TODO: have go_home check whether it is actually at that location
            #self.behaviors.move_absolute(self.start_location, stop='pressure_accel')

            #start_loc = self.current_location()
            #pdb.set_trace()
            success, reason, touchloc_bl = self.behaviors.reach(point, \
                    press_contact_pressure, \
                    reach_direction=np.matrix([0.1,0,0]).T)
            #r1, pos_error1 = self.behaviors.movement.move_relative_gripper(np.matrix([-.01, 0., 0.]).T, \
            #        stop='none', pressure=press_contact_pressure)

            if touchloc_bl != None:
                dist = np.linalg.norm(point - touchloc_bl[0])
                #print '===================================================================='
                #print '===================================================================='
                #TODO assure that reaching motion did touch the point that we intended to touch.
                rospy.loginfo('Touched point is %.3f m away from observed point' % dist)
                #print '===================================================================='
                #print '===================================================================='

            if not success:
                error_msg = 'Reach failed due to "%s"' % reason
                rospy.loginfo(error_msg)
                rospy.loginfo('Failure recovery: moving back')
                self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='accel', \
                        pressure=press_contact_pressure)
                #raise TaskError(error_msg)
                return False, None, point+point_offset

            rospy.loginfo('pressing')

            #Should not be making contact
            self.behaviors.movement.pressure_listener.rezero()
            change, press_ret = self.camera_change_detect(visual_change_thres, \
                    self.behaviors.press, \
                    (press_distance, press_pressure, press_contact_pressure))
            success, reason = press_ret
            if not success:
                rospy.loginfo('Press failed due to "%s"' % reason)

            #code reward function
            #monitor self collision => collisions with the environment are not self collisions
            rospy.loginfo('moving back')
            #self.behaviors.movement.set_movement_mode_cart()
            r1, pos_error1 = self.behaviors.movement.move_relative_gripper(np.matrix([-.03, 0., 0.]).T, \
                    stop='none', pressure=press_contact_pressure)
            if r1 != None:
                rospy.loginfo('moving back failed due to "%s"' % r1)
                return change, None, point+point_offset

            rospy.loginfo('reseting')
            self.behaviors.movement.pressure_listener.rezero()
            r2, pos_error2 = self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
            if r2 != None and r2 != 'no solution':
                rospy.loginfo('moving back to start location failed due to "%s"' % r2)
                return change, None, point+point_offset
            self.behaviors.movement.pressure_listener.rezero()

            rospy.loginfo('DONE.')
            return change, touchloc_bl, point+point_offset

        except lm.RobotSafetyError, e:
            rospy.loginfo('>>>> ROBOT SAFETY ERROR! RESETTING. %s' % str(e))
            self.behaviors.movement.pressure_listener.rezero()
            r2, pos_error2 = self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
            return change, None, point+point_offset


    def light_rocker_push(self, point, pressure, visual_change_thres, offset):
        rospy.loginfo('Reaching')
        linear_movement = self.behaviors.movement
        #linear_movement.gripper_close()
        self.close_gripper()
        self.behaviors.movement.pressure_listener.rezero()
        #pdb.set_trace()
        #try:
        self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure_accel', pressure=3000)
        #except lm.RobotSafetyError, e:
        #    rospy.loginfo('robot safety error %s' % str(e))

        def reach_with_back_up(point, thres, reach_direction):
            self.behaviors.reach(point, thres, reach_direction)
            try:
                r1, pos_error1 = self.behaviors.movement.move_relative_gripper(np.matrix([-.05, 0., 0.]).T, stop='none')
            except lm.RobotSafetyError, e:
                rospy.loginfo('robot safety error %s' % str(e))
        change, press_ret = self.camera_change_detect(visual_change_thres, \
                                    #self.behaviors.reach, \
                                    reach_with_back_up, \
                                    #(point, pressure, np.matrix([0,0,0.]).T, np.matrix([.1,0,0]).T))
                                    (point, pressure, np.matrix([.1,0,0]).T))
        try:
            linear_movement.move_relative_gripper(np.matrix([-.1,0,0]).T, stop='accel')
            self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure_accel', pressure=3000)
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))

        try:
            self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure_accel', pressure=3000)
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))

        rospy.loginfo('Reseting')
        return change, '', point+offset


    def drawer_push(self, point_bl):
        PUSH_TOLERANCE = .1
        #pdb.set_trace()
        linear_movement = self.behaviors.movement
        #linear_movement.gripper_open()
        #pdb.set_trace()
        self.open_gripper()
        self.behaviors.movement.pressure_listener.rezero()
        #self.robot.left_gripper.open(True, position=.08)
        rospy.loginfo("Moving to start location")
        #linear_movement.move_absolute((self.start_location_drawer[0], 
        #    np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))))
        linear_movement.move_absolute((self.start_location_drawer[0], 
            #np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))), 
            np.matrix(tr.quaternion_from_euler(np.radians(0.), 0, 0))), 
            stop='pressure_accel', pressure=1000)

        #calc front loc
        self.behaviors.movement.set_pressure_threshold(1000)
        loc_bl = self.behaviors.movement.arm_obj.pose_cartesian_tf()[0]
        front_loc = point_bl.copy()
        front_loc[0,0] = max(loc_bl[0,0], .4)

        #pdb.set_trace()
        #move to front
        rospy.loginfo("Moving to front location")
        #orientation = np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))
        orientation = np.matrix(tr.quaternion_from_euler(np.radians(0.), 0, 0))
        self.behaviors.movement.pressure_listener.rezero()
        r1, residual_error = self.behaviors.movement.move_absolute((front_loc, orientation), 
                                stop='pressure', pressure=1500)
        linear_movement.pressure_listener.rezero()

        #move until contact
        rospy.loginfo("Touching surface")
        try:
            linear_movement.move_relative_gripper(np.matrix([.5,0,0]).T, stop='pressure_accel', pressure=100)
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
        contact_loc_bl = linear_movement.arm_obj.pose_cartesian_tf()[0]

        #Push
        rospy.loginfo("PUSH!!!")
        current_position = self.robot.left.pose_cartesian_tf()
        target_position = current_position[0] + np.matrix([.4,0,0.]).T
        try:
            #linear_movement.move_relative_gripper(np.matrix([.2,0,0]).T, stop='pressure_accel', pressure=6000)
            linear_movement.move_absolute((target_position, current_position[1]), stop='pressure_accel', pressure=6000)
            linear_movement.move_absolute((target_position, current_position[1]), stop='pressure_accel', pressure=6000)
            linear_movement.move_absolute((target_position, current_position[1]), stop='pressure_accel', pressure=6000)
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))

        pushed_loc_bl = linear_movement.arm_obj.pose_cartesian_tf()[0]

        rospy.loginfo("Moving away")
        try:
            linear_movement.move_relative_gripper(np.matrix([-.05,0,0]).T, stop='accel')
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
        try:
            linear_movement.move_relative_gripper(np.matrix([-.10,0,0]).T, stop='accel')
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
        try:
            linear_movement.move_relative_gripper(np.matrix([-.1,0,0]).T, stop='accel')
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))

        linear_movement.pressure_listener.rezero()
        #linear_movement.move_relative_base(np.matrix([-.2, .3, 0.1]).T, stop='pressure_accel', pressure=300)
        linear_movement.move_absolute((self.start_location_drawer[0], 
                    #np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))), 
                    np.matrix(tr.quaternion_from_euler(np.radians(0.), 0, 0))), 
                    stop='pressure_accel', pressure=1000)

        move_dist = np.linalg.norm(contact_loc_bl - pushed_loc_bl)
        rospy.loginfo('pushed for distance %.3f' % move_dist)
        success = move_dist > PUSH_TOLERANCE
        return success, 'pushed', pushed_loc_bl


    def drawer(self, point):
        #Prepare
        GRIPPER_OPEN = .08
        GRIPPER_CLOSE = .003
        MAX_HANDLE_SIZE = .03
        linear_movement = self.behaviors.movement
        gripper = self.robot.left_gripper

        #gripper.open(True, position=GRIPPER_OPEN)
        #linear_movement.gripper_open()
        self.open_gripper()
        linear_movement.move_absolute((self.start_location_drawer[0], 
            #np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))), 
            np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))), 
            stop='pressure_accel', pressure=1000)

        #Reach
        success, reason, touchloc_bl = self.behaviors.reach(point, 300, #np.matrix([0.0, 0, 0]).T, 
                             reach_direction=np.matrix([0.1, 0, 0]).T, 
                             orientation=np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0)))

        #Error recovery
        if not success:
            error_msg = 'Reach failed due to "%s"' % reason
            rospy.loginfo(error_msg)
            rospy.loginfo('Failure recovery: moving back')
            try:
                linear_movement.move_relative_gripper(np.matrix([-.25,0,0]).T, stop='pressure_accel', pressure=300)
            except lm.RobotSafetyError, e:
                rospy.loginfo('robot safety error %s' % str(e))
            self.behaviors.movement.move_absolute(self.start_location_drawer, stop='accel', pressure=300)
            return False, 'reach failed', point

        #Grasp
        GRASP_THRES = 100
        try:
            linear_movement.move_relative_gripper(np.matrix([-.01,0,0]).T, stop='none')
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
        #lbf, rbf = linear_movement.pressure_listener.get_pressure_readings()
        #pdb.set_trace()
        self.close_gripper()
        #linear_movement.gripper_close()
        #gripper.open(True, position=GRIPPER_CLOSE)
        #linear_movement.pressure_listener.rezero()
        #laf, raf = linear_movement.pressure_listener.get_pressure_readings()

        #linear_movement.move_relative_gripper(np.matrix([-.05,0,0]).T, stop='none')
        #gripper.open(True, position=.03)
        #linear_movement.pressure_listener.rezero()
        #gripper.open(True, position=GRIPPER_CLOSE)
        #linear_movement.pressure_listener.rezero()
        #bf = np.row_stack((lbf, rbf))
        #af = np.row_stack((laf, raf))
        #pdb.set_trace()
        #grasped_handle = np.any(np.abs(af-bf) > GRASP_THRES) or (gripper.pose()[0,0] > GRIPPER_CLOSE)
        grasped_handle = (gripper.pose()[0,0] > GRIPPER_CLOSE) and (gripper.pose()[0,0] < MAX_HANDLE_SIZE)

        if not grasped_handle:
            rospy.loginfo('Failed to grasp handle :(')
            #linear_movement.gripper_open()
            self.open_gripper()
            #gripper.open(True, position=GRIPPER_OPEN)
            linear_movement.pressure_listener.rezero()
            linear_movement.move_relative_gripper(np.matrix([-.25,0,0]).T, stop='pressure_accel', pressure=300)
            self.behaviors.movement.move_absolute(self.start_location_drawer, stop='accel', pressure=300)
            return False, 'failed to grasp handle', point

        #Pull
        linear_movement.pressure_listener.rezero()
        linear_movement.move_relative_gripper(np.matrix([-.1,0,0]).T, stop='accel', pressure=2500)
        linear_movement.move_absolute(linear_movement.arm_obj.pose_cartesian_tf(), 
                                    stop='pressure_accel', pressure=300)
        #linear_movement.gripper_close()
        #linear_movement.gripper_close()
        self.close_gripper()
        rospy.sleep(1)
        linear_movement.pressure_listener.rezero()
        #lap, rap = linear_movement.pressure_listener.get_pressure_readings()
        #ap = np.row_stack((lap, rap))
        #still_has_handle = np.any(np.abs(ap-af) < GRASP_THRES) or (gripper.pose()[0,0] > GRIPPER_CLOSE)
        still_has_handle = gripper.pose()[0,0] > GRIPPER_CLOSE
        #pdb.set_trace()
        try:
            linear_movement.move_relative_base(np.matrix([-.15,0,0]).T, stop='accel', pressure=2500)
        except lm.RobotSafetyError, e:
            #linear_movement.gripper_open()
            self.open_gripper()
            linear_movement.pressure_listener.rezero()
            rospy.loginfo('robot safety error %s' % str(e))

        #Release & move back 
        #linear_movement.gripper_open()
        location_handle_bl = linear_movement.arm_obj.pose_cartesian_tf()[0]
        #gripper.open(True, position=.08)
        #linear_movement.gripper_open()
        self.open_gripper()
        rospy.sleep(2)
        linear_movement.pressure_listener.rezero()

        #linear_movement.move_relative_gripper(np.matrix([-.15, 0, 0]).T, stop='pressure_accel', pressure=300)
        linear_movement.move_relative_base(np.matrix([-.2, 0, 0.]).T, stop='pressure_accel', pressure=1000)
        linear_movement.move_relative_base(np.matrix([-.1, .2, 0.1]).T, stop='pressure_accel', pressure=1000)
        self.behaviors.movement.move_absolute(self.start_location_drawer, stop='pressure_accel', pressure=1000)

        return still_has_handle, 'pulled', location_handle_bl


    ##
    # Drive using within a dist_far distance of point_bl
    def drive_approach_behavior(self, point_bl, dist_far):
        # navigate close to point
        #pdb.set_trace()
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl)
        t_current_map, r_current_map = self.robot.base.get_pose()
        rospy.loginfo('drive_approach_behavior: point is %.3f m away"' % np.linalg.norm(t_current_map[0:2].T - point_map[0:2,0].T))

        point_dist = np.linalg.norm(point_bl)
        bounded_dist = np.max(point_dist - dist_far, 0)
        point_close_bl = (point_bl / point_dist) * bounded_dist
        point_close_map = tfu.transform_points(map_T_base_link, point_close_bl)
        rvalue = self.robot.base.set_pose(point_close_map.T.A1.tolist(), \
                                          r_current_map, '/map', block=True)
        t_end, r_end = self.robot.base.get_pose()
        rospy.loginfo('drive_approach_behavior: ended up %.3f m away from laser point' % np.linalg.norm(t_end[0:2] - point_map[0:2,0].T))
        rospy.loginfo('drive_approach_behavior: ended up %.3f m away from goal' % np.linalg.norm(t_end[0:2] - point_close_map[0:2,0].T))
        rospy.loginfo('drive_approach_behavior: returned %d' % rvalue)
        return rvalue

    ##
    # Drive so that we are perpendicular to a wall at point_bl (radii voi_radius) 
    # stop at dist_approach
    def approach_perpendicular_to_surface(self, point_bl, voi_radius, dist_approach):
        #return 3
        #TODO: Turn to face point
        #TODO: make this scan around point instead of total scan of env
        #determine normal
        #pdb.set_trace()
        map_T_base_link0 = tfu.transform('map', 'base_link', self.tf_listener)
        point_map0 = tfu.transform_points(map_T_base_link0, point_bl)
        #pdb.set_trace()
        self.turn_to_point(point_bl, block=False)

        point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), \
                                        point_map0)
        point_cloud_bl = self.laser_scan.scan(math.radians(180.), math.radians(-180.), 2.5)
        point_cloud_np_bl = ru.pointcloud_to_np(point_cloud_bl)
        rospy.loginfo('approach_perpendicular_to_surface: pointcloud size %d' \
                % point_cloud_np_bl.shape[1])
        voi_points_bl, limits_bl = i3d.select_rect(point_bl, voi_radius, voi_radius, voi_radius, point_cloud_np_bl)
        #TODO: use closest plane instead of closest points determined with KDTree
        normal_bl = i3d.calc_normal(voi_points_bl)
        point_in_front_mechanism_bl = point_bl + normal_bl * dist_approach
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_in_front_mechanism_map = tfu.transform_points(map_T_base_link, point_in_front_mechanism_bl)

        #Navigate to point (TODO: check for collisions)
        point_map = tfu.transform_points(map_T_base_link, point_bl)
        t_current_map, r_current_map = self.robot.base.get_pose()
        rospy.loginfo('approach_perpendicular_to_surface: driving for %.3f m to front of surface' \
                % np.linalg.norm(t_current_map[0:2] - point_in_front_mechanism_map[0:2,0].T))
        #pdb.set_trace()
        rvalue = self.robot.base.set_pose(point_in_front_mechanism_map.T.A1.tolist(), r_current_map, 'map')
        if rvalue != 3:
            return rvalue

        t1_current_map, r1_current_map = self.robot.base.get_pose()
        rospy.loginfo('approach_perpendicular_to_surface: %.3f m away from from of surface' % np.linalg.norm(t1_current_map[0:2] - point_in_front_mechanism_map[0:2,0].T))

        #Rotate to face point (TODO: check for collisions)
        base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
        point_bl = tfu.transform_points(base_link_T_map, point_map)
        #pdb.set_trace()
        self.turn_to_point(point_bl, block=False)
        time.sleep(2.)

        return rvalue
        #ang = math.atan2(point_bl[1,0], point_bl[0,0])
        #self.robot.base.turn_by(ang, block=True)
        #pdb.set_trace()

    def approach_location(self, point_bl, coarse_stop, fine_stop, voi_radius=.2):
        #return
        point_dist = np.linalg.norm(point_bl[0:2,0])
        rospy.loginfo('approach_location: Point is %.3f away.' % point_dist)
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl)

        dist_theshold = coarse_stop + .1
        if point_dist > dist_theshold:
            rospy.loginfo('approach_location: Point is greater than %.1f m away (%.3f).  Driving closer.' % (dist_theshold, point_dist))
            rospy.loginfo('approach_location: point_bl ' + str(point_bl.T))

            ret = self.drive_approach_behavior(point_bl, dist_far=coarse_stop)
            base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
            point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
            if ret != 3:
                dist_end = np.linalg.norm(point_bl_t1[0:2,0])
                if dist_end > dist_theshold:
                    rospy.logerr('approach_location: drive_approach_behavior failed! %.3f' % dist_end)
                    self.robot.sound.say("I am unable to navigate to that location")
                    return False, 'failed'

            ret = self.approach_perpendicular_to_surface(point_bl_t1, voi_radius=voi_radius, dist_approach=fine_stop)
            if ret != 3:
                rospy.logerr('approach_location: approach_perpendicular_to_surface failed!')
                return False, 'failed'

            self.robot.sound.say('done')
            rospy.loginfo('approach_location: DONE DRIVING!')
            return True, 'done'
        else:
            return False, 'ignored'

    def turn_to_point(self, point_bl, block=True):
        ang = math.atan2(point_bl[1,0], point_bl[0,0])
        rospy.loginfo('turn_to_point: turning by %.2f deg' % math.degrees(ang))
        #pdb.set_trace()
        self.robot.base.turn_by(-ang, block=block, overturn=True)

    #def load_classifier(self, classifier_name, data_file_name):
    #    self.learners[classifier_name] = ipa.InterestPointPerception(classifier_name, 
    #            data_file_name, self.tf_listener)

    def stationary_light_switch_behavior(self, point_bl):
        while True:
            print 'Enter a command u(ntuck), s(tart), l(ight), t(uck), e(x)it.'
            a = raw_input()
            
            if a == 'u': 
                self.untuck()

            if a == 's':
                self.behaviors.movement.pressure_listener.rezero()
                self.behaviors.movement.set_movement_mode_cart()
                self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
                #pdb.set_trace()
                #TODO: set start location to be some set joint angles

            if a == 'l':
                point_offset = np.matrix([0, 0, 0.03]).T
                success, _ = self.light_switch1(point_bl, point_offset=point_offset, \
                        press_contact_pressure=300, \
                        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
                        visual_change_thres=.03)
                        #move_back_distance=np.matrix([-.0075,0,0]).T,

            if a == 't':
                self.tuck()

            if a == 'x':
                break

    #def location_activated_behaviors(self, point_bl, stored_point=False):
    #    driving_param = {'light_switch': {'coarse': .7, 'fine': .5, 'voi': .2},
    #                     'drawer':       {'coarse': .7, 'fine': .7, 'voi': .2}}

    #    map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
    #    point_map = tfu.transform_points(map_T_base_link, point_bl)
    #    matches = self.find_close_by_points(point_map)

    #    if len(matches) > 0:
    #        #pdb.set_trace()
    #        ldata = self.location_data[self.location_labels[matches[0]]]
    #        task = ldata['task']
    #        rospy.loginfo('Found closeby location %s' % str(ldata))
    #    else:
    #        rospy.loginfo( 'No location matches found. Please enter location type:')
    #        for i, k in enumerate(driving_param.keys()):
    #            rospy.loginfo(' %d %s' %(i,k))
    #        task_number = raw_input()
    #        task = driving_param.keys()[int(task_number)]

    #    self.robot.sound.say('task %s' % task.replace('_', ' '))
    #    rospy.loginfo('Task is %s' % task)
    #    if self.approach_location(point_bl, 
    #            coarse_stop=driving_param[task]['coarse'], 
    #            fine_stop=driving_param[task]['fine'], 
    #            voi_radius=driving_param[task]['voi']):
    #        return

    #    else:
    #        ret = self.approach_perpendicular_to_surface(point_bl, 
    #                voi_radius=driving_param[task]['voi'], 
    #                dist_approach=driving_param[task]['fine'])

    #        if ret != 3:
    #            rospy.logerr('location_activated_behaviors: approach_perpendicular_to_surface failed!')
    #            return

    #        base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
    #        point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
    #        try:
    #            self.untuck()
    #            self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
    #            self.behaviors.movement.pressure_listener.rezero()

    #            if task == 'light_switch':
    #                #self.location_add(perturbed_map, task)
    #                # TODO: what happens when we first encounter this location?! experiment n times to create dataset?
    #                self.practice(point_bl_t1, 
    #                        ft.partial(self.light_switch1, 
    #                            point_offset=np.matrix([0,0,.03]).T,
    #                            press_contact_pressure=300,
    #                            move_back_distance=np.matrix([-.0075,0,0]).T,
    #                            press_pressure=3500,
    #                            press_distance=np.matrix([0,0,-.15]).T,
    #                            visual_change_thres=.03), 
    #                        'light_switch')
    #                self.tuck()

    #                if False: #Old branch where we retry blindly
    #                    MAX_RETRIES = 15
    #                    rospy.loginfo('location_activated_behaviors: go_home_pose')
    #                    #self.go_to_home_pose()
    #                    self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
    #                    gaussian = pr.Gaussian(np.matrix([ 0,      0,      0.]).T, \
    #                                           np.matrix([[1.,     0,      0], \
    #                                                      [0, .02**2,      0], \
    #                                                      [0,      0, .02**2]]))
    #                    retry_count = 0
    #                    success = False
    #                    gaussian_noise = np.matrix([0, 0, 0.0]).T
    #                    point_offset = np.matrix([0, 0, 0.03]).T
    #                    while not success:
    #                        perturbation = gaussian_noise
    #                        perturbed_point_bl = point_bl_t1 + perturbation
    #                        success, _ = self.light_switch1(perturbed_point_bl, point_offset=point_offset, \
    #                                press_contact_pressure=300, move_back_distance=np.matrix([-.0075,0,0]).T,\
    #                                press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
    #                                visual_change_thres=.03)
    #                        gaussian_noise = gaussian.sample()
    #                        gaussian_noise[0,0] = 0
    #                        retry_count = retry_count + 1 

    #                        if retry_count > MAX_RETRIES:
    #                            self.robot.sound.say('giving up tried %d times already' % MAX_RETRIES)
    #                            break
    #                        elif not success:
    #                             self.robot.sound.say('retrying')

    #                    if success:
    #                        self.robot.sound.say('successful!')

    #                        if not stored_point or retry_count > 1:
    #                            map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
    #                            perturbed_map = tfu.transform_points(map_T_base_link, perturbed_point_bl)
    #                            self.location_add(perturbed_map, task)
    #                            self.robot.sound.say('recorded point')

    #                        #if retry_count > 1:
    #                        #    if not self.add_perturbation_to_location(point_map, perturbation):
    #                        #        self.robot.sound.say('unable to add perturbation to database! please fix')
    #                    self.tuck()

    #
    #            if task == 'drawer':
    #                self.drawer(point_bl_t1)
    #                self.tuck()
    #                self.robot.sound.say('done')
    #                self.location_add(point_map, task)
    #                
    #                #except lm.RobotSafetyError, e:
    #                #    rospy.loginfo('location_activated_behaviors: Caught a robot safety exception "%s"' % str(e.parameter))
    #                #    #self.behaviors.movement.move_absolute(self.start_location, stop='accel')

    #        except lm.RobotSafetyError, e:
    #            rospy.loginfo('location_activated_behaviors: Caught a robot safety exception "%s"' % str(e.parameter))
    #            self.behaviors.movement.move_absolute(self.start_location, stop='accel')
    #
    #        except TaskError, e:
    #            rospy.loginfo('location_activated_behaviors: TaskError: %s' % str(e.parameter))
    #        rospy.loginfo('location_activated_behaviors: DONE MANIPULATION!')
    #        self.robot.sound.say('done')

    def location_approach_driving(self, task, point_bl):
        #Get closer if point is far away
        ap_result = self.approach_location(point_bl, 
                        coarse_stop=self.locations_man.driving_param[task]['coarse'], 
                        fine_stop=self.locations_man.driving_param[task]['fine'], 
                        voi_radius=self.locations_man.driving_param[task]['voi'])

        if ap_result[1] == 'failed':
            return False, 'approach_location failed'

        if ap_result[1] == 'ignore':
            #reorient with planner
            ret = self.approach_perpendicular_to_surface(point_bl, 
                    voi_radius=self.locations_man.driving_param[task]['voi'], 
                    dist_approach=self.locations_man.driving_param[task]['fine'])
            if ret != 3:
                rospy.logerr('location_approach_driving: approach_perpendicular_to_surface failed!')
                return False, 'approach_perpendicular_to_surface failed'
            else:
                return True, None

        return True, None


        #if self.approach_location(point_bl, 
        #        coarse_stop=self.locations_man.driving_param[task]['coarse'], 
        #        fine_stop=self.locations_man.driving_param[task]['fine'], 
        #        voi_radius=self.locations_man.driving_param[task]['voi']):
        #    #rospy.logerr('location_approach_driving: intial approach failed')
        #    return True, 'initial approach'
        #else:

    def get_behavior_by_task(self, task_type):
        if task_type == 'light_switch_down':
            return ft.partial(self.light_switch1, 
                        #point_offset=np.matrix([0,0,.03]).T,
                        point_offset=np.matrix([0,0, -.08]).T,
                        press_contact_pressure=300,
                        #move_back_distance=np.matrix([-.0075,0,0]).T,
                        press_pressure=6000,
                        press_distance=np.matrix([0.01,0,-.15]).T,
                        visual_change_thres=.025)

        elif task_type == 'light_switch_up':
            return ft.partial(self.light_switch1, 
                        #point_offset=np.matrix([0,0,-.08]).T,
                        point_offset=np.matrix([0,0,.08]).T,
                        press_contact_pressure=300,
                        #move_back_distance=np.matrix([-.0075,0,0]).T,
                        press_pressure=6000,
                        press_distance=np.matrix([0.01,0,.15]).T,
                        visual_change_thres=.025)

        elif task_type == 'light_rocker_up':
            return ft.partial(self.light_rocker_push,
                        pressure=500,
                        visual_change_thres=.025, offset=np.matrix([0,0,-.05]).T)

        elif task_type == 'light_rocker_down':
            return ft.partial(self.light_rocker_push,
                        pressure=500,
                        visual_change_thres=.025, offset=np.matrix([0,0,.05]).T)

        elif task_type == 'pull_drawer':
            return self.drawer

        elif task_type == 'push_drawer':
            return self.drawer_push

        else:
            pdb.set_trace()
            #TODO


    def manipulation_posture(self, task_type):
        self.robot.projector.set(False)
        for i in range(3):
            self.prosilica.get_frame()
        self.robot.projector.set(True)
        #rospy.sleep(1)

        self.robot.left_gripper.open(False, .005)
        #self.robot.right_gripper.open(True, .005)
        self.behaviors.movement.pressure_listener.rezero()

        if task_type == 'light_switch_down' or task_type == 'light_switch_up':
            if np.linalg.norm(self.start_location_light_switch[0] - self.robot.left.pose_cartesian_tf()[0]) < .3:
                return
            self.robot.torso.set_pose(.2, True)
            self.untuck()
            self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
            self.behaviors.movement.pressure_listener.rezero()

        elif task_type == 'light_rocker_up' or task_type == 'light_rocker_down':
            if np.linalg.norm(self.start_location_light_switch[0] - self.robot.left.pose_cartesian_tf()[0]) < .3:
                return
            self.robot.torso.set_pose(.2, True)
            self.untuck()
            self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
            self.behaviors.movement.pressure_listener.rezero()

        elif task_type == 'pull_drawer' or task_type == 'push_drawer':
            if np.linalg.norm(self.start_location_drawer[0] - self.robot.left.pose_cartesian_tf()[0]) < .3:
                return
            self.robot.torso.set_pose(0.01, True)
            self.untuck()
            self.behaviors.movement.move_absolute(self.start_location_drawer, stop='pressure')
            self.behaviors.movement.pressure_listener.rezero()
        else:
            pdb.set_trace()


    def driving_posture(self, task_type):
        self.robot.projector.set(False)
        self.close_gripper()

        if np.linalg.norm(self.folded_pose - self.robot.left.pose_cartesian_tf()[0]) < .1:
            return
        #TODO: specialize this
        self.robot.torso.set_pose(0.03, True)
        self.robot.left_gripper.open(False, .005)
        #self.robot.right_gripper.open(True, .005)
        self.behaviors.movement.pressure_listener.rezero()

        if task_type == 'light_switch_down' or task_type == 'light_switch_up':
            self.tuck()

        elif task_type == 'light_rocker_up' or task_type == 'light_rocker_down':
            self.tuck()

        elif task_type == 'pull_drawer' or task_type == 'push_drawer':
            self.tuck()
        else:
            pdb.set_trace()


    def look_at(self, point_bl, block=True):
        #pdb.set_trace()
        #self.robot.head.look_at(point_bl-np.matrix([0,0,.2]).T, 
        #        pointing_frame=self.OPTICAL_FRAME, 
        #        pointing_axis=np.matrix([1,0,0.]).T, 
        #        wait=block)
        self.robot.head.look_at(point_bl-np.matrix([0,0,.15]).T, pointing_frame=self.OPTICAL_FRAME, 
                pointing_axis=np.matrix([1,0,0.]).T, wait=block)


    ##
    # Initialization phase
    #
    # @param point_bl 3x1 in base_link
    def scenario_user_clicked_at_location(self, point_bl):
        #pdb.set_trace()
        #If that location is new:
        self.look_at(point_bl, False)
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl)
        close_by_locs = self.locations_man.list_close_by(point_map)
        #close_by_locs = self.locations_man.list_all()#(point_map)

        #if False:
        #if True:
        #if len(close_by_locs) <= 0:
        if True:
            #Initialize new location
            rospy.loginfo('Select task type:')
            for i, ttype in enumerate(self.locations_man.task_types):
                print i, ttype
            task_type = self.locations_man.task_types[int(raw_input())]
            rospy.loginfo('Selected task %s' % task_type)

            #pdb.set_trace()
            self.driving_posture(task_type)
            #ret = self.location_approach_driving(task_type, point_bl)
            #if not ret[0]:
            #    return False, ret[1]

            #pdb.set_trace()
            self.manipulation_posture(task_type)
            point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
            self.look_at(point_bl, False)

            #rospy.loginfo('if existing dataset exists enter that dataset\'s name')
            #print 'friday_730_light_switch2.pkl'
            #filename = raw_input()

            #If we have data for perceptual bias
            #if len(filename) > 0:
            #    dataset = ut.load_pickle(filename)
            #    task_id = self.locations_man.create_new_location(task_type, point_map)

            #    #Assume that this is file is 100% reliable
            #    for n in range(LocationManager.RELIABILITY_RECORD_LIM):
            #        self.locations_man.update_execution_record(task_id, 1)
            #    self.locations_man.data[task_id]['dataset'] = dataset
            #    #rec_params = self.feature_ex.rec_params
            #    #def train(self, rec_params, dataset, task_id):
            #    self.locations_man.train(self.feature_ex.rec_params, dataset, task_id)
            #    #TODO: we actually want to find only the successful location
            #    #pdb.set_trace()
            #    self.execute_behavior(self.get_behavior_by_task(task_type), task_id, point_bl)

            ##If we don't have data
            #else:
            def has_pos_and_neg(labels):
                if np.sum(labels == r3d.POSITIVE) > 0 and np.sum(labels == r3d.NEGATIVE) > 0:
                    return True
                else:
                    return False

            def any_pos_sf(labels_mat):
                if np.any(r3d.POSITIVE == labels_mat):
                    return True
                return False

            #Create new tasks
            location_name = raw_input('Enter a name for this location:\n')
            ctask_type = self.locations_man.get_complementary_task(task_type)
            t_current_map, r_current_map = self.robot.base.get_pose()
            task_id = self.locations_man.create_new_location(task_type, 
                    np.matrix([0,0,0.]).T, [t_current_map, r_current_map], name=location_name)
            ctask_id = self.locations_man.create_new_location(ctask_type, 
                    np.matrix([0,0,0.]).T, [t_current_map, r_current_map], name=location_name)

            #Stop when have at least 1 pos and 1 neg
            #pdb.set_trace()
            point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
            self.look_at(point_bl, True)
            ret_dict_action = self.seed_dataset_explore(task_id, ctask_id, point_bl, stop_fun=has_pos_and_neg, should_reset=True)
            dset_action = ret_dict_action['features']
            dset_undo = ret_dict_action['features_undo']
            undo_point_bl = ret_dict_action['undo_point']

            #Lights should be on at this stage!
            #pdb.set_trace()
            #If we don't have enought data for reverse action
            rospy.loginfo('====================================================')
            rospy.loginfo('Don\'t have enough data for reverse action')
            if (self.locations_man.data[ctask_id]['dataset'] == None) or \
                    not has_pos_and_neg(self.locations_man.data[ctask_id]['dataset'].outputs.A1):
                #Turn off the lights 
                point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
                self.look_at(point_bl, True)
                rospy.loginfo('====================================================')
                rospy.loginfo('Running seed_dataset_explore on set %s.' % task_id)
                ret_dict_action = self.seed_dataset_explore(task_id, None, point_bl, stop_fun=any_pos_sf, should_reset=False)
                undo_point_bl = ret_dict_action['undo_point']

                #Practice until stats are met
                self.look_at(point_bl, True)
                #ret_dict_undo = self.seed_dataset_explore(ctask_id, task_id, point_bl, stop_fun=has_pos_and_neg)
                rospy.loginfo('====================================================')
                rospy.loginfo('Running seed_dataset_explore on set %s.' % ctask_id)
                ret_dict_undo = self.seed_dataset_explore(ctask_id, task_id, undo_point_bl, stop_fun=has_pos_and_neg)
                if dset_undo == None:
                    dset_undo = ret_dict_undo['features']

            #Figure out behavior centers in map frame
            tdataset  = self.locations_man.data[task_id]['dataset']
            tpoint_bl = tdataset.pt3d[:, np.where(tdataset.outputs == r3d.POSITIVE)[1].A1[0]]
            self.balance_positives_and_negatives(tdataset)

            cdataset  = self.locations_man.data[ctask_id]['dataset']
            cpoint_bl = cdataset.pt3d[:, np.where(cdataset.outputs == r3d.POSITIVE)[1].A1[0]]
            self.balance_positives_and_negatives(cdataset)

            map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
            point_success_map = tfu.transform_points(map_T_base_link, tpoint_bl)
            cpoint_success_map = tfu.transform_points(map_T_base_link, cpoint_bl)

            #Set newly created task with learned information
            self.locations_man.set_center(task_id, point_success_map)
            self.locations_man.set_center(ctask_id, cpoint_success_map)
            self.locations_man.data[task_id]['complementary_task_id'] = ctask_id
            self.locations_man.data[ctask_id]['complementary_task_id'] = task_id

            self.locations_man.update_execution_record(task_id, 1.)
            self.locations_man.update_execution_record(ctask_id, 1.)

            #param = r3d.Recognize3DParam()
            #param.uncertainty_x = 1.
            #param.n_samples = 2000
            #param.uni_mix = .1
            #kdict, fname = self.read_features_save(task_id, point3d_bl, param)
            self.locations_man.train(task_id, dset_action, save_pca_images=True)
            self.locations_man.train(ctask_id, dset_undo, save_pca_images=True)
            self.locations_man.save_database()
            rospy.loginfo('Done initializing new location!')
            self.driving_posture(task_type)
        #else:
        #    task_id, task_type = close_by_locs[0]
        #    ctask_id = self.locations_man.data[task_id]['complementary_task_id']
        #    rospy.loginfo('Selected task %s' % task_type)

        #    self.driving_posture(task_type)
        #    ret = self.location_approach_driving(task_type, point_bl)
        #    if not ret[0]:
        #        return False, ret[1]
        #    #pdb.set_trace()
        #    self.manipulation_posture(task_type)
        #    point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
        #    self.look_at(point_bl, False)
        #    #close_by_locs = self.locations_man.list_close_by(point_map)

        #    if self.locations_man.is_reliable(task_id):
        #        self.execute_behavior(task_id, point_bl)
        #    else:
        #        self.practice(task_id, ctask_id, point_bl)

        #    self.driving_posture(task_type)

    #def add_to_practice_points_map(self, point_bl):
    #    map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
    #    point_map = tfu.transform_points(map_T_base_link, point_bl)
    #    self.practice_points_map.append(point_map)
    #    rospy.loginfo('Added a new practice point!')


    def move_base_planner(self, trans, rot):
        #pdb.set_trace()
        p_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), np.matrix(trans).T)
        #Do this to clear out any hallucinated obstacles
        self.turn_to_point(p_bl)
        rvalue = self.robot.base.set_pose(trans, rot, '/map', block=True)
        p_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), np.matrix(trans).T)
        #pdb.set_trace()
        self.robot.base.move_to(p_bl[0:2,0], True)
        t_end, r_end = self.robot.base.get_pose()
        return rvalue==3, np.linalg.norm(t_end[0:2] - np.array(trans)[0:2])

    def update_base(self):
        tasks = self.locations_man.data.keys()
        for i, k in enumerate(tasks):
            print i, k
        selected_idx = int(raw_input("Select an action to update base location\n"))

        t_current_map, r_current_map = self.robot.base.get_pose()
        task_id = tasks[selected_idx]
        self.locations_man.update_base_pose(task_id, [t_current_map, r_current_map])
        self.locations_man.save_database()


    ##
    # Practice phase
    def scenario_practice_run_mode(self):
        rospy.loginfo('===================================================')
        rospy.loginfo('= Practice Mode!                                  =')
        rospy.loginfo('===================================================')
        #pdb.set_trace()

        ulocs = self.unreliable_locs()
        rospy.loginfo('%d out of %d locs in database are unreliable' \
                % (len(ulocs), len(self.locations_man.data.keys())))

        #Ask user to select a location
        tasks = self.locations_man.data.keys()
        for i, k in enumerate(tasks):
            print i, k
        #for tidx, tid in enumerate(ulocs):
        #    print tidx, tid
        selected_idx = int(raw_input("Select a location to execute action\n"))

        #tid = ulocs[selected_idx]
        tid = tasks[selected_idx]
        rospy.loginfo('selected %s' % tid)

        #Ask user for practice poses
        #pdb.set_trace()
        if not self.locations_man.data[tid].has_key('practice_locations'):
        #if True:
            #Get robot poses
            map_points = []
            for i in range(4):
                raw_input('%d move robot to desired position\n' % i)
                rospy.sleep(1.)
                #pdb.set_trace()
                t_current_map, r_current_map = self.robot.base.get_pose()
                map_points.append([t_current_map, r_current_map])
            self.locations_man.data[tid]['practice_locations'] = map_points
            self.locations_man.data[tid]['practice_locations_history'] = np.zeros((1, len(map_points)))
            self.locations_man.data[tid]['practice_locations_convergence'] = np.zeros((1, len(map_points)))

            self.locations_man.save_database()

        #Ask user for canonical pose if it does not exist
        if not self.locations_man.data[tid].has_key('base_pose'):
            raw_input('move robot to desired end position\n')
            trans, rot_quat = self.robot.base.get_pose()
            self.locations_man.data[tid]['base_pose'] = [trans, rot_quat]
            self.locations_man.save_database()

        point_map = self.locations_man.data[tid]['center']
        task_type = self.locations_man.data[tid]['task']
        points_added_history = []

        unexplored_locs  = np.where(self.locations_man.data[tid]['practice_locations_history'] == 0)[1]
        unconverged_locs = np.where(self.locations_man.data[tid]['practice_locations_convergence'] == 0)[1]
        rospy.loginfo("Location history: %s" % str(self.locations_man.data[tid]['practice_locations_history']))
        run_loop = True
        #pdb.set_trace()

        #If this is a fresh run, start with current location
        if unexplored_locs.shape[0] == len(self.locations_man.data[tid]['practice_locations']):
            pidx = 3

        #If this is not a fresh run we continue with a location we've never been to before
        elif unexplored_locs.shape[0] > 0:
            pidx = unexplored_locs[0]
            rospy.loginfo("Resuming training from last unexplored location")

        #set the index to an unconverged location
        elif unconverged_locs.shape[0] > 0:
            pidx = unconverged_locs[0]
            rospy.loginfo("Resuming training from unconverged location")

        #if there are no unconverged locations, try to run anyway
        else:
            rospy.loginfo("WARNING: no unexplored or unconverged location")
            pidx = 3
            self.locations_man.data[tid]['practice_locations_convergence'][0, pidx] = 0

        #Commence practice!
        while True: #%not converged:

            if self.locations_man.data[tid]['practice_locations_convergence'][0, pidx] == 0:
                #Drive to location
                self.driving_posture(task_type)

                #Move to setup location
                self.robot.sound.say('Driving to practice location')
                rvalue, dist = self.move_base_planner(*self.locations_man.data[tid]['practice_locations'][pidx])
                rospy.loginfo('move ret value %s dist %f' % (str(rvalue), dist))

                #Move to location where we were first initialized
                self.robot.sound.say('Driving to mechanism location')
                rvalue, dist = self.move_base_planner(*self.locations_man.data[tid]['base_pose'])

                #Reorient base
                bl_T_map = tfu.transform('base_link', 'map', self.tf_listener)
                point_bl = tfu.transform_points(bl_T_map, point_map)
                ret = self.location_approach_driving(task_type, point_bl)
                self.robot.base.set_pose(self.robot.base.get_pose()[0], self.locations_man.data[tid]['base_pose'][1], 'map')
                if not ret[0]:
                    rospy.loginfo('Driving failed!! Resetting.')
                    #pdb.set_trace()
                    #return False, ret[1]
                    continue

                self.manipulation_posture(task_type)
                bl_T_map = tfu.transform('base_link', 'map', self.tf_listener)
                point_bl = tfu.transform_points(bl_T_map, point_map)
                self.look_at(point_bl, False)

                #Practice
                self.robot.sound.say('practicing')
                ctid = self.locations_man.data[tid]['complementary_task_id']

                points_before_t = self.locations_man.data[tid]['dataset'].inputs.shape[1]
                points_before_ct = self.locations_man.data[ctid]['dataset'].inputs.shape[1]

                points_added = self.practice(tid, ctid,  point_bl)

                points_added_history.append(points_added)
                points_after_t = self.locations_man.data[tid]['dataset'].inputs.shape[1]
                points_after_ct = self.locations_man.data[ctid]['dataset'].inputs.shape[1]
                self.locations_man.record_time(tid,  'num_points_added_' + tid, points_after_t - points_before_t)
                self.locations_man.record_time(ctid, 'num_points_added_' + tid, points_after_ct - points_before_ct)

                self.locations_man.data[tid]['practice_locations_history'][0, pidx] += 1
                #If no points added and we have explored all locations
                #if points_added == 0 and np.where(self.locations_man.data[tid]['practice_locations_history'] == 0)[1].shape[0] == 0:
                if points_added == 0:# and np.where(self.locations_man.data[tid]['practice_locations_history'] == 0)[1].shape[0] == 0:
                    self.locations_man.data[tid]['practice_locations_convergence'][0, pidx] = 1
                    rospy.loginfo('===================================================')
                    rospy.loginfo('= LOCATION CONVERGED ')
                    rospy.loginfo('Converged locs: %s' % str(self.locations_man.data[tid]['practice_locations_convergence']))
                    rospy.loginfo('using instances %d points added' % (points_after_t - points_before_t))
                    rospy.loginfo('history %s' % str(points_added_history))
                    rospy.loginfo('number of iterations it took %s' % str(np.sum(points_added_history)))
                    rospy.loginfo('number of datapoints %s' % str(self.locations_man.data[tid]['dataset'].outputs.shape))
                    rospy.loginfo('===================================================')
                    if np.where(self.locations_man.data[tid]['practice_locations_convergence'] == 0)[1].shape[0] <= 0:
                        break
                else:
                    rospy.loginfo('===================================================')
                    rospy.loginfo('= Scan converged!')
                    rospy.loginfo('Converged locs: %s' % str(self.locations_man.data[tid]['practice_locations_convergence']))
                    rospy.loginfo('using instances %d points added' % (points_after_t - points_before_t))
                    rospy.loginfo('history %s' % str(points_added_history))
                    rospy.loginfo('number of iterations so far %s' % str(np.sum(points_added_history)))
                    rospy.loginfo('number of datapoints %s' % str(self.locations_man.data[tid]['dataset'].outputs.shape))
                    rospy.loginfo('===================================================')

            pidx = (pidx + 1) % len(self.locations_man.data[tid]['practice_locations'])
            self.locations_man.save_database()

        self.driving_posture(task_type)
        #ulocs = self.unreliable_locs()
        #rospy.loginfo('%d out of %d locs in database are unreliable' \
        #        % (len(ulocs), len(self.locations_man.data.keys())))

    ##
    # Execution phase
    def scenario_user_select_location(self, save, user_study):
        rospy.loginfo('===================================================')
        rospy.loginfo('= User selection mode!                            =')
        rospy.loginfo('===================================================')
        tasks = self.locations_man.data.keys()
        for i, k in enumerate(tasks):
            print i, k

        tid = tasks[int(raw_input())]
        task_type = self.locations_man.data[tid]['task']
        rospy.loginfo('User selected %s' % tid)
        #self.driving_posture(task_type)
        #self.manipulation_posture(task_type)
        #return

        #record current robot position
        if not self.locations_man.data[tid].has_key('execute_locations'):
            self.locations_man.data[tid]['execute_locations'] = []
        t_current_map, r_current_map = self.robot.base.get_pose()
        self.locations_man.data[tid]['execute_locations'].append([t_current_map, r_current_map])
        if not user_study:
            self.locations_man.save_database()

        point_map = self.locations_man.data[tid]['center']
        #pdb.set_trace()
        self.robot.sound.say('Driving')
        self.driving_posture(task_type)
        rvalue, dist = self.move_base_planner(*self.locations_man.data[tid]['base_pose'])
        if not rvalue:
            self.robot.sound.say('unable to plan to location')
            return False

        bl_T_map = tfu.transform('base_link', 'map', self.tf_listener)
        point_bl = tfu.transform_points(bl_T_map, point_map)
        ret = self.location_approach_driving(task_type, point_bl)
        self.robot.base.set_pose(self.robot.base.get_pose()[0], 
                                 self.locations_man.data[tid]['base_pose'][1], 'map')
        if not ret[0]:
            self.robot.sound.say('unable to get to location!')
            return False

        self.manipulation_posture(task_type)
        bl_T_map = tfu.transform('base_link', 'map', self.tf_listener)
        point_bl = tfu.transform_points(bl_T_map, point_map)
        self.look_at(point_bl, False)

        self.robot.sound.say('Executing behavior')
        self.execute_behavior(tid, point_bl, save, user_study=user_study)
        self.driving_posture(task_type)
        #exit()

    def unreliable_locs(self):
        tasks = self.locations_man.data.keys()
        utid = []
        for tid in tasks:
            if not self.locations_man.is_reliable(tid):
                utid.append(tid)
        return utid



    def click_cb(self, point_bl):
        #self.look_at(point_bl)
        #return
        #point_bl = np.matrix([ 0.68509375,  0.06559023,  1.22422832]).T
        #self.stationary_light_switch_behavior(point_bl)
        #mode = 'autonomous'
        #mode = 'light switch'
        mode = 'location_execute'
        #mode = 'practice'

        if point_bl!= None:
            print 'Got point', point_bl.T
            #1) User points at location and selects a behavior.
            if mode == 'location_execute':
                self.scenario_user_clicked_at_location(point_bl)

            #if mode == 'practice':
            #    self.add_to_practice_points_map(point_bl)


                ##If that location is new:
                #map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
                #point_map = tfu.transform_points(map_T_base_link, point_bl)
                #close_by_locs = self.locations_man.list_close_by(point_map)

                #if len(close_by_locs) <= 0:
                #    #initialize new location
                #    rospy.loginfo('Select task type:')
                #    for i, ttype in enumerate(self.locations_man.task_types):
                #        print i, ttype
                #    task_type = self.locations_man[int(raw_input())]
                #    task_id = self.locations_man.create_new_location(task_type, point_map)

                #    rospy.loginfo('if existing dataset exists enter that dataset\'s name')
                #    print 'friday_730_light_switch2.pkl'
                #    filename = raw_input()
                #    if len(filename) > 0:
                #        dataset = ut.load_pickle(filename)
                #        self.locations_man.data[task_id]['dataset'] = dataset
                #        self.locations_man.train(dataset, task_id)
                #    else:
                #        self.last_ditch_execution(

                #elif len(close_by_locs) == 1:
                #    task_id, task = close_by_locs[0]
                #    rospy.loginfo('Executing task %s with id % s', task, task_id)
                #    self.execute_behavior(task_id, point_bl)

                #elif len(close_by_locs) > 1:
                #    #TODO: implement this case
                #    rospy.logerr('ERROR: unimplemented')
                #    pdb.set_trace()
                #    self.execute_behavior(task_id, point_bl)
                #else:
                #    rospy.logerr('ERROR: we shouldn\'t have reached here')
                #    pdb.set_trace()


            if mode == 'live_label':
                #self.execute_behavior(point_bl, 
                light_switch_beh = ft.partial(self.light_switch1, 
                                        point_offset=np.matrix([0,0,.03]).T,
                                        press_contact_pressure=300,
                                        #move_back_distance=np.matrix([-.0075,0,0]).T,
                                        press_pressure=3500,
                                        press_distance=np.matrix([0,0,-.15]).T,
                                        visual_change_thres=.03)
                point_map = tfu.transform_points(tfu.transform('map', 'base_link', self.tf_listener), point_bl)

                while not rospy.is_shutdown():
                    point_bl_cur = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
                    self.execute_behavior(point_bl_cur, light_switch_beh, 'light_switch')

            if mode == 'capture data':
                self.robot.head.look_at(point_bl, 'base_link', True)
                self.robot.sound.say("taking a scan")
                #self.record_perceptual_data(point_bl)
                #pdb.set_trace()
                #rdict = self.feature_ex.kinect_listener.read()
                rdict = self.feature_ex.read()['rdict']
                #self.record_perceptual_data(point_bl, 'openni_rgb_optical_frame', rdict=rdict)
                self.record_perceptual_data(point_bl, self.OPTICAL_FRAME, rdict=rdict)
                self.robot.sound.say("saved scan")

            if mode == 'light switch':
                point_offset = np.matrix([0, 0, 0.03]).T
                success, _ = self.light_switch1(point_bl, point_offset=point_offset, \
                        press_contact_pressure=300,\
                        # move_back_distance=np.matrix([-.0075,0,0]).T,\
                        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
                        visual_change_thres=.03)

                self.behaviors.movement.pressure_listener.rezero()
                self.behaviors.movement.set_movement_mode_cart()
                self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')


            #if mode == 'autonomous learn':
            #    def light_behavior(point):
            #        point_offset = np.matrix([0, 0, 0.03]).T
            #        success, _ = self.light_switch1(point, point_offset=point_offset, \
            #                        press_contact_pressure=300, move_back_distance=np.matrix([-.0075,0,0]).T,\
            #                        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
            #                        visual_change_thres=.03)
            #        if success:
            #            return 1.0
            #        else:
            #            return 0.0

            #    self.untuck()
            #    self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
            #    self.behaviors.movement.pressure_listener.rezero()
            #    self.autonomous_learn(point_bl, light_behavior, 'light_switch')

            #if mode == 'location activated':
            #    self.location_activated_behaviors(point_bl)

        #elif mode == 'location activated':
        #    all_locs = self.locations_man.list_all()
        #    for i, pair in enumerate(all_locs):
        #        key, task = pair
        #        print i, task, key

        #    rospy.loginfo('Select location to execute action')
        #    selected = int(raw_input())

    def run(self, mode, save, user_study):
        #point = np.matrix([ 0.60956734, -0.00714498,  1.22718197]).T
        #print 'RECORDING'
        #self.record_perceptual_data(point)
        #print 'DONE RECORDING'
        #kdict, fname = self.read_features_save(task_id, point3d_bl, param)
        #point = np.matrix([ 0.60956734, -0.00714498,  1.22718197]).T
        #f = self.feature_ex.read(point, self.rec_params)

        r = rospy.Rate(10)
        rospy.loginfo('Ready.')
        while not rospy.is_shutdown():
            r.sleep()
            if mode == 'practice':
                self.scenario_practice_run_mode()

            if mode == 'execute':
                t = time.time()
                self.scenario_user_select_location(save, user_study)
                t2 = time.time()
                print '>> That took', t2 - t, 'seconds'
                exit()

            if mode == 'update_base':
                self.update_base()


    #def record_perceptual_data_laser_scanner(self, point_touched_bl):
    #    #what position should the robot be in?
    #    #set arms to non-occluding pose

    #    #record region around the finger where you touched
    #    rospy.loginfo('Getting laser scan.')
    #    points = []
    #    for i in range(3):
    #        rospy.loginfo('scan %d' % i)
    #        points.append(self.laser_scan.scan(math.radians(180.), math.radians(-180.), 20./3.))

    #    rospy.loginfo('Getting Prosilica image.')
    #    prosilica_image = self.prosilica.get_frame()
    #    rospy.loginfo('Getting image from left wide angle camera.')
    #    left_image  = self.wide_angle_camera_left.get_frame()
    #    rospy.loginfo('Getting image from right wide angle camera.')
    #    right_image = self.wide_angle_camera_left.get_frame()
    #    rospy.loginfo('Waiting for calibration.')
    #    while self.prosilica_cal.has_msg == False:
    #        time.sleep(.1)

    #    #which frames?
    #    rospy.loginfo('Getting transforms.')
    #    pro_T_bl = tfu.transform('/self.OPTICAL_FRAMEhigh_def_optical_frame', '/base_link', self.tf_listener)
    #    laser_T_bl = tfu.transform('/laser_tilt_link', '/base_link', self.tf_listener)
    #    tstring = time.strftime('%A_%m_%d_%Y_%I:%M%p')
    #    prosilica_name = '%s_highres.png' % tstring
    #    left_name = '%s_left.png' % tstring
    #    right_name = '%s_right.png' % tstring
    #    rospy.loginfo('Saving images (basename %s)' % tstring)
    #    cv.SaveImage(prosilica_name, prosilica_image)
    #    cv.SaveImage(left_name, left_image)
    #    cv.SaveImage(right_name, right_image)

    #    rospy.loginfo('Saving pickles')
    #    pickle_fname = '%s_interest_point_dataset.pkl' % tstring   

    #    data_pkl = {'touch_point': point_touched_bl,
    #                'points_laser': points,
    #                'laser_T_bl': laser_T_bl, 
    #                'pro_T_bl': pro_T_bl,

    #                'high_res': prosilica_name,
    #                'prosilica_cal': self.prosilica_cal, 

    #                'left_image': left_name,
    #                'left_cal': self.left_cal,

    #                'right_image': right_name,
    #                'right_cal': self.right_cal}
    #                #'point_touched': point_touched_bl}
    #                

    #    ut.save_pickle(data_pkl, pickle_fname)
    #    print 'Recorded to', pickle_fname

    def record_perceptual_data(self, point3d_bl, image_frame, rdict=None, folder_name=None):
        rospy.loginfo('saving dataset..')
        #self.feature_ex.read(point3d_bl)
        if rdict == None:
            rospy.loginfo('Getting a kinect reading')
            rdict = self.feature_ex.read()['rdict']
            #rdict = self.kinect_listener.read()
        kimage = rdict['image']
        rospy.loginfo('Waiting for calibration.')
        while not self.feature_ex.cal.has_msg:
            time.sleep(.1)

        #which frames?
        rospy.loginfo('Getting transforms.')
        #k_T_bl = tfu.transform('openni_rgb_optical_frame', '/base_link', self.tf_listener)
        k_T_bl = tfu.transform(image_frame, '/base_link', self.tf_listener)

        tstring = time.strftime('%A_%m_%d_%Y_%I_%M%p')
        kimage_name = '%s_highres.png' % tstring
        pickle_fname = '%s_interest_point_dataset.pkl' % tstring   
        if folder_name != None:
            try:
                os.mkdir(folder_name)
            except OSError, e:
                print e
            kimage_name = pt.join(folder_name, kimage_name)
            pickle_fname = pt.join(folder_name, pickle_fname)

        rospy.loginfo('Saving images (basename %s)' % tstring)
        cv.SaveImage(kimage_name, kimage)
        rospy.loginfo('Saving pickles')

        data_pkl = {'touch_point': point3d_bl,
                    'points3d': rdict['points3d'],
                    'image': kimage_name,
                    'cal': self.feature_ex.cal, 
                    'k_T_bl': k_T_bl}
                    #'point_touched': point3d_bl}

        ut.save_pickle(data_pkl, pickle_fname)
        print 'Recorded to', pickle_fname
        return pickle_fname, kimage_name

    #def gather_interest_point_dataset(self, point):
    #    gaussian = pr.Gaussian(np.matrix([0, 0, 0.]).T, \
    #            np.matrix([[1., 0, 0], \
    #                       [0, .02**2, 0], \
    #                       [0, 0, .02**2]]))

    #    for i in range(100):
    #        # perturb_point
    #        gaussian_noise = gaussian.sample()
    #        gaussian_noise[0,0] = 0
    #        success_off, touchloc_bl = self.light_switch1(point, 
    #                        point_offset=np.matrix([-.15, 0, 0]).T, press_contact_pressure=300, 
    #                        move_back_distance=np.matrix([-.005,0,0]).T, press_pressure=2500, 
    #                        press_distance=np.matrix([0,0,-.15]).T, visual_change_thres=.03)
    #        rospy.loginfo('Lights turned off? %s' % str(success_off))

    #        pdb.set_trace()
    #        self.behaviors.movement.move_absolute((np.matrix([.15, .45, 1.3]).T, self.start_location[1]), stop='pressure_accel')
    #        self.record_perceptual_data(touchloc_bl)
    #        self.behaviors.movement.move_absolute(self.start_location, stop='pressure_accel')
    #        if success_off:
    #            self.behaviors.movement.move_absolute((np.matrix([.15, .45, 1.3]).T, self.start_location[1]), stop='pressure_accel')
    #            self.record_perceptual_data(touchloc_bl)
    #            self.behaviors.movement.move_absolute(self.start_location, stop='pressure_accel')

    #            success_on, touchloc_bl2 = self.light_switch1(point, 
    #                            point_offset=np.matrix([-.15,0,-.10]).T, press_contact_pressure=300, 
    #                            move_back_distance=np.matrix([-0.005, 0, 0]).T, press_pressure=2500, 
    #                            press_distance=np.matrix([0,0,.1]).T, visual_change_thres=.03)
    #            ##1
    #            #if success_on:
    #            #    self.movement.behaviors.move_absolute((np.matrix([.15, .45, 1.3]).T, self.start_location[1]), stop='pressure_accel')
    #            #    self.record_perceptual_data(touchloc_bl)
    #            #    self.movement.behaviors.move_absolute(self.start_location, stop='pressure_accel')
    #            #Turn on lights
    #            #success_on, touchloc_bl = self.light_switch1(npoint, 
    #        else:
    #            return

    ##
    # The behavior can make service calls to a GUI asking users to label
    def practice(self, task_id, ctask_id, point3d_bl, stop_fun=None, params=None, 
            negative_cut_off=.5, resolution=.01, max_samples=5):
        if params == None:
            params = r3d.Recognize3DParam()
            params.uncertainty_x = 1.
            #param.n_samples = 2000
            params.uncertainty_z = .04
            params.uni_mix = .1
        pstart = time.time()

        kdict, image_name = self.read_features_save(task_id, point3d_bl, params)
        #learner = self.locations_man.learners[task_id]
        #pdb.set_trace()
        behavior = self.get_behavior_by_task(self.locations_man.data[task_id]['task'])
        head_pose = self.robot.head.pose()

        kdict['image_T_bl'] = tfu.transform(self.OPTICAL_FRAME, 'base_link', self.tf_listener)
        point3d_img = tfu.transform_points(kdict['image_T_bl'], point3d_bl)
        point2d_img = self.feature_ex.cal.project(point3d_img)

        labels = []
        points3d_tried = []
        points2d_tried = []
        converged = False
        indices_added = []
        reset_times = []

        #while not converged or (stop_fun != None and not stop_fun(np.matrix(labels))):
        while True:# and (stop_fun != None and not stop_fun(np.matrix(labels))):
            if stop_fun != None and stop_fun(np.matrix(labels)):
                rospy.loginfo('Stop satisfied told us to stop loop!')
                break

            if stop_fun == None and len(indices_added) > params.max_points_per_site:
                rospy.loginfo('practice: added enough points from this scan. Limit is %d points.' % params.max_points_per_site)
                break

            #==================================================
            # Pick
            #==================================================
            #Find remaining instances
            iter_start = time.time()
            print 'input to inverse_indices'
            print '>>', indices_added, kdict['instances'].shape[1]
            remaining_pt_indices = r3d.inverse_indices(indices_added, kdict['instances'].shape[1])
            remaining_instances = kdict['instances'][:, remaining_pt_indices]

            #Ask learner to pick an instance
            ridx, selected_dist, converged = self.locations_man.learners[task_id].select_next_instances_no_terminate(remaining_instances)
            #if stop_fun == None and converged:
            #    rospy.loginfo('practice: Converged! Exiting loop.')
            #    break


            selected_idx = remaining_pt_indices[ridx[0]]
            #pdb.set_trace()
            indices_added.append(selected_idx)

            #==================================================
            # DRAW
            #==================================================
            img = cv.CloneMat(kdict['image'])
            #Draw the center
            r3d.draw_points(img, point2d_img, [255, 0, 0], 6, 2)

            #Draw possible points
            r3d.draw_points(img, kdict['points2d']+np.matrix([1,1.]).T, [255, 255, 255], 4, -1)

            if len(points2d_tried) > 0:
                _, pos_exp, neg_exp = separate_by_labels(np.column_stack(points2d_tried), np.matrix(labels))
                r3d.draw_points(img, pos_exp, [50, 255, 0], 8, 1)
                r3d.draw_points(img, neg_exp, [50, 0, 255], 8, 1)

            predictions = np.matrix(self.locations_man.learners[task_id].classify(kdict['instances']))
            _, pos_pred, neg_pred = separate_by_labels(kdict['points2d'], predictions)
            r3d.draw_points(img, pos_pred, [255, 204, 51], 3, -1)
            r3d.draw_points(img, neg_pred, [51, 204, 255], 3, -1)

            #Draw what we're selecting
            r3d.draw_points(img, kdict['points2d'][:, selected_idx], [255, 51, 204], 8, -1)
            self.locations_man.publish_image(task_id, img, postfix='_practice_pick')

            #==================================================
            # Excecute!!
            #==================================================
            self.robot.head.set_pose(head_pose, 1)
            #self.robot.projector.set(False)
            if np.linalg.norm(kdict['points3d'][:, selected_idx] - point3d_bl) > negative_cut_off:
                rospy.loginfo('#########################################')
                rospy.loginfo('Point outside of negative cut off!! Eliminating %s' % (str(kdict['points3d'][:, selected_idx].T)))
                rospy.loginfo('#########################################')
                success = False
            else:
                if len(points3d_tried) > 0:
                    existing_pts_tree = sp.KDTree(np.array(np.column_stack(points3d_tried).T))
                    close_by_indices = existing_pts_tree.query_ball_point(np.array(kdict['points3d'][:, selected_idx]).T, resolution)[0]
                    if len(close_by_indices) > 0:
                        #labelsm = np.matrix(labels)[0, close_by_indices]
                        #ntotal = labelsm.shape[1]
                        rospy.loginfo('#########################################')
                        rospy.loginfo('Point within resolutio of existing point.') #Labeling %s' % (str(kdict['points3d'][:, selected_idx])))
                        rospy.loginfo('#########################################')
                        continue
                        #This can cause the automated mechanism to propagate false labels.
                        #if np.sum(labelsm) > (ntotal/2.0):
                        #    success = True
                        #else:
                        #    success = False
                        #rospy.loginfo('as %s' % str(success))
                    else:
                        success, _, undo_point_bl = behavior(kdict['points3d'][:, selected_idx])
                else:
                    success, _ , undo_point_bl = behavior(kdict['points3d'][:, selected_idx])
            #self.robot.projector.set(True)

            if success:
                color = [0,255,0]
                label = r3d.POSITIVE
                rospy.loginfo('=============================================')
                rospy.loginfo('>> behavior successful')
                rospy.loginfo('=============================================')
                self.robot.sound.say('action succeeded')
            else:
                label = r3d.NEGATIVE
                color = [0,0,255]
                rospy.loginfo('=============================================')
                rospy.loginfo('>> behavior NOT successful')
                rospy.loginfo('=============================================')
                self.robot.sound.say('action failed')

            #==================================================
            # Book keeping
            #==================================================
            labels.append(label)
            points3d_tried.append(kdict['points3d'][:, selected_idx])
            points2d_tried.append(kdict['points2d'][:, selected_idx])

            datapoint = {'instances': kdict['instances'][:, selected_idx],
                         'points2d':  kdict['points2d'][:, selected_idx],
                         'points3d':  kdict['points3d'][:, selected_idx],
                         'sizes':     kdict['sizes'],
                         'labels':    np.matrix([label])
                         }
            self.locations_man.add_perceptual_data(task_id, datapoint)
            self.locations_man.save_database()
            self.locations_man.train(task_id, kdict)
            #pdb.set_trace()

            #==================================================
            # Reset Environment
            #==================================================
            reset_start = time.time()
            if success and ctask_id != None:
                def any_pos_sf(labels_mat):
                    if np.any(r3d.POSITIVE == labels_mat):
                        return True
                    return False
                #self.practice(ctask_id, None, point3d_bl, stop_fun=any_pos_sf)
                undo_point_map = self.locations_man.data[ctask_id]['center']
                undo_point_bl0 = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), undo_point_map)
                #self.practice(ctask_id, None, undo_point_bl, stop_fun=any_pos_sf)
                #pdb.set_trace()
                num_points_added = self.practice(ctask_id, None, undo_point_bl0, stop_fun=any_pos_sf)
                #if num_points_added > params.max_points_per_site:
                #    #pdb.set_trace()
                #    dataset = self.locations_man.get_perceptual_data(ctask_id)
                #    npoints = dataset.inputs.shape[1]
                #    pts_added_idx = range(npoints - num_points_added, npoints)
                #    pts_remove_idx = pts_added_idx[:(num_points_added - params.max_points_per_site)]
                #    pts_remove_idx.reverse()
                #    rospy.loginfo('Got too many data points for %s throwing out %d points' % (ctask_id, len(pts_remove_idx)))
                #    for idx in pts_remove_idx:
                #        self.locations_man.remove_perceptual_data(ctask_id, idx)

                #self.locations_man.

            reset_end = time.time()

            #Classify
            predictions = np.matrix(self.locations_man.learners[task_id].classify(kdict['instances']))

            #==================================================
            # DRAW
            #==================================================
            img = cv.CloneMat(kdict['image'])

            #Draw the center
            r3d.draw_points(img, point2d_img, [255, 0, 0], 6, 2)

            #Draw 'shadows' 
            r3d.draw_points(img, kdict['points2d']+np.matrix([1,1.]).T, [255, 255, 255], 4, -1)

            #Draw points tried
            _, pos_exp, neg_exp = separate_by_labels(np.column_stack(points2d_tried), np.matrix(labels))
            r3d.draw_points(img, pos_exp, [50, 255, 0], 9, 1)
            r3d.draw_points(img, neg_exp, [50, 0, 255], 9, 1)

            _, pos_pred, neg_pred = separate_by_labels(kdict['points2d'], predictions)
            r3d.draw_points(img, pos_pred, [255, 204, 51], 3, -1)
            r3d.draw_points(img, neg_pred, [51, 204, 255], 3, -1)

            #Draw what we're selecting
            r3d.draw_points(img, points2d_tried[-1], color, 8, -1)
            self.locations_man.publish_image(task_id, img, postfix='_practice_result')

            pkname = pt.join(task_id, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + '.pkl')
            #cv.SaveImage(ffull, img)
            ut.save_pickle({'image': image_name,
                            'pos': pos_exp,
                            'neg': neg_exp,
                            'pos_pred': pos_pred,
                            'neg_pred': neg_pred,
                            'tried': (points2d_tried[-1], label),
                            'center': point2d_img}, pkname)

            #ffull = pt.join(task_id, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + '.png')
            #cv.SaveImage(ffull, img)
            #self.img_pub.publish(img)
            reset_time = reset_end - reset_start
            loop_time = (time.time() - iter_start) - (reset_end - reset_start)
            reset_times.append(reset_time)
            print '**********************************************************'
            print 'Loop took %.3f seconds' % loop_time
            print '**********************************************************'
            self.locations_man.record_time(task_id, 'practice_loop_time', loop_time)

        if np.any(r3d.POSITIVE == np.matrix(labels)):
            self.locations_man.update_execution_record(task_id, 1)

        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'returning from', task_id
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        pend = time.time()
        practice_time = pend - pstart - np.sum(reset_times)
        self.locations_man.record_time(task_id, 'practice_func_time', practice_time)
        return len(indices_added)


    def balance_positives_and_negatives(self, dataset):
        #pdb.set_trace() #bug here
        poslist = np.where(dataset.outputs == r3d.POSITIVE)[1].A1.tolist()
        neglist = np.where(dataset.outputs == r3d.NEGATIVE)[1].A1.tolist()
        npoint = min(len(poslist), len(neglist))
        assert(npoint > 0)
        sindices = poslist[:npoint]+neglist[:npoint]
        dataset.pt3d    = dataset.pt3d[:, sindices]
        dataset.pt2d    = dataset.pt2d[:, sindices]
        dataset.outputs = dataset.outputs[:, sindices]
        dataset.inputs  = dataset.inputs[:, sindices]

    def profile_me(self, task_id, point_bl):
        for i in range(2):
            params = r3d.Recognize3DParam()
            params.uncertainty_x = 1.
            params.uncertainty_y = .04
            params.uncertainty_z = .04
            params.n_samples = 5000
            params.uni_mix = 0.1
            print 'PROFILE_ME ITERATION', i
            fea_dict, image_name = self.read_features_save(task_id, point_bl, params)


    ###
    # TODO: GASP!! WE MIGHT NOT NEED THIS AFTER ALL, JUST INITIALIZE RANDOMLY!
    ###
    def seed_dataset_explore(self, task_id, ctask_id, point_bl, stop_fun, 
            max_retries=30, closeness_tolerance=.01, positive_escape=.08, should_reset=False):
        #rospy.loginfo('seed_dataset_explore: %s' % task_id)
        ##case label
        #if self.locations_man.data[task_id]['dataset'] != None:
        #    case_labels = self.locations_man.data[task_id]['dataset'].outputs.A1.tolist()
        #else:
        #    case_labels = []
        #if stop_fun(case_labels):
        #    rospy.loginfo('seed_dataset_explore: stop function satisfied with label set. not executing.')
        #    return

        self.look_at(point_bl, True)
        rospy.sleep(2.)
        params = r3d.Recognize3DParam()
        params.uncertainty_x = 1.
        params.uncertainty_y = .04
        params.uncertainty_z = .04
        params.n_samples = 800
        params.uni_mix = 0.1


        #profile read_reatures_save
        #cProfile.runctx('self.profile_me(task_id, point_bl)', globals(), locals(), filename='read_features_save.prof')
        #pdb.set_trace()
    
        #Scan
        fea_dict, image_name = self.read_features_save(task_id, point_bl, params)

        params2 = r3d.Recognize3DParam()
        params2.n_samples = 5000
        fea_dict2, image_name2 = self.read_features_save(task_id, point_bl, params2)

        behavior = self.get_behavior_by_task(self.locations_man.data[task_id]['task'])
        image_T_bl = tfu.transform(self.OPTICAL_FRAME, 'base_link', self.tf_listener)
        fea_dict['image_T_bl'] = image_T_bl
        
        #Rearrange sampled points by distance
        dists = ut.norm(fea_dict['points3d'] - point_bl)
        ordering = np.argsort(dists).A1
        points3d_sampled = fea_dict['points3d'][:, ordering]
        points2d_sampled = fea_dict['points2d'][:, ordering]
        instances_sampled = fea_dict['instances'][:, ordering]
        start_pose = self.robot.head.pose()

        #Figure out where to draw given point for visualization
        point3d_img = tfu.transform_points(fea_dict['image_T_bl'], point_bl)
        point2d_img = self.feature_ex.cal.project(point3d_img)
   
        #Book keeping for loop
        undo_ret = None
        points3d_tried = []
        points2d_tried = []
        labels = []
        sampled_idx = 0
        iter_count = 0
        need_reset = False
        behavior_end_state = False
        undo_point_bl = point_bl

        #pdb.set_trace()
        while iter_count < max_retries and not stop_fun(np.matrix(labels)) and sampled_idx < points3d_sampled.shape[1]:
            #==================================================
            # Pick
            #==================================================
            if len(points3d_tried) > 0 and \
               np.any(ut.norm(np.column_stack(points3d_tried) - points3d_sampled[:, sampled_idx]) < closeness_tolerance): 
                   sampled_idx = sampled_idx + 1 
                   continue

            if len(labels) > 0 and np.sum(labels) == len(labels) and\
               np.any(ut.norm(np.column_stack(points3d_tried) - points3d_sampled[:, sampled_idx]) < positive_escape): 
                   #pdb.set_trace()
                   sampled_idx = sampled_idx + 1 
                   continue

            #==================================================
            # Excecute!!
            #==================================================
            self.robot.head.set_pose(start_pose, 1)
            #self.robot.projector.set(False)
            success, reason, undo_point_bl = behavior(points3d_sampled[:, sampled_idx])
            rospy.loginfo('======================================')
            rospy.loginfo('%s was %s' % (task_id, str(success)))
            rospy.loginfo('======================================')
            behavior_end_state = success
            #self.robot.projector.set(True)

            #==================================================
            # Book keeping
            #==================================================
            if success:
                label = r3d.POSITIVE
                need_reset = True
            else:
                label = r3d.NEGATIVE
                need_reset = False
                
            points3d_tried.append(points3d_sampled[:, sampled_idx])
            points2d_tried.append(points2d_sampled[:, sampled_idx])
            labels.append(label)
            datapoint = {'instances': instances_sampled[:, sampled_idx],
                         'points3d':  points3d_sampled[:, sampled_idx],
                         'points2d':  points2d_sampled[:, sampled_idx],
                         'sizes':     fea_dict['sizes'],
                         'labels':    np.matrix([label])}
            self.locations_man.add_perceptual_data(task_id, datapoint)
            self.locations_man.save_database()

            iter_count = iter_count + 1
            sampled_idx = sampled_idx + 1

            #==================================================
            # DRAW
            #==================================================
            img = cv.CloneMat(fea_dict['image'])
            r3d.draw_points(img, points2d_sampled+np.matrix([1,1.]).T, [0, 0, 0], 3, -1)
            r3d.draw_points(img, points2d_sampled, [255, 255, 255], 3, -1)
            _, pos_points, neg_points = separate_by_labels(np.column_stack(points2d_tried), np.matrix(labels))
            r3d.draw_points(img, point2d_img, [255, 0, 0], 6, 2)
            r3d.draw_points(img, pos_points, [0, 255, 0], 4, -1)
            r3d.draw_points(img, neg_points, [0, 0, 255], 4, -1)
            r3d.draw_points(img, points2d_tried[-1], [0, 184, 245], 6, -1)
            self.locations_man.publish_image(task_id, img)

            pkname = pt.join(task_id, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + '_seed.pkl')
            ut.save_pickle({'image': image_name,
                            'pos': pos_points,
                            'neg': neg_points,
                            #'pos_pred': pos_pred,
                            #'neg_pred': neg_pred,
                            'tried': [points2d_tried[-1], label],
                            'center': point2d_img}, pkname)

            #ffull = pt.join(task_id, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + '.png')
            #cv.SaveImage(ffull, img)
            #self.img_pub.publish(img)
    
            #==================================================
            # Reset Environment
            #==================================================
            if need_reset and should_reset:
                self.robot.head.set_pose(start_pose, 1)
                if ctask_id != None:
                    #If we were successful, call blind exploration with the undo behavior
                    def any_pos_sf(labels_mat):
                        if np.any(r3d.POSITIVE == labels_mat):
                            return True
                        return False

                    #ctask_point = points3d_tried[-1] #points3d_sampled[:, sampled_idx]
                    ctask_point = undo_point_bl
                    undo_ret = self.seed_dataset_explore(ctask_id, None, ctask_point, stop_fun=any_pos_sf, 
                                        max_retries=max_retries, closeness_tolerance=closeness_tolerance, 
                                        should_reset=False)
                    need_reset = False
                    if undo_ret['end_state']:
                        behavior_end_state = False
                    else:
                        behavior_end_state = True

        rospy.loginfo('Tried %d times' % iter_count)
        fea_dict_undo = None
        if undo_ret != None:
            fea_dict_undo = undo_ret['features']

        return {'end_state': behavior_end_state, 'features': fea_dict2, 
                'features_undo': fea_dict_undo, 'undo_point': undo_point_bl}

    def read_features_save(self, task_id, point3d_bl, params=None):
        self.robot.projector.set(True)
        rospy.sleep(2.)
        f = self.feature_ex.read(point3d_bl, params=params)
        file_name, kimage_name = self.record_perceptual_data(point3d_bl, self.OPTICAL_FRAME, rdict=f['rdict'], folder_name=task_id)
        self.robot.projector.set(False)

        #image_T_bl = tfu.transform(self.OPTICAL_FRAME, 'base_link', self.tf_listener)
        #point3d_img = tfu.transform_points(image_T_bl, point3d_bl)
        #point2d_img = self.feature_ex.cal.project(point3d_img)

        #img = cv.CloneMat(f['image'])
        #self.draw_dots_nstuff(img, f['points2d'], 
        #        np.matrix([r3d.UNLABELED]* f['points2d'].shape[1]), 
        #        point2d_img)
        #self.img_pub.publish(img)

        return f, kimage_name
        #self.save_dataset(point, name, f['rdict'])



    ##
    #TODO: test this
    def execute_behavior(self, task_id, point3d_bl, save, max_retries=15, closeness_tolerance=.01, user_study=False):
        if user_study:
            rospy.loginfo('=====================================================')
            rospy.loginfo('user_study is True.  Trying to fail on first attempt!')
            rospy.loginfo('=====================================================')
        #Extract features
        params = r3d.Recognize3DParam()
        params.uncertainty_x = 1.
        params.uncertainty_z = .03
        params.uni_mix = .1
        params.n_samples = 1500

        kdict, image_name = self.read_features_save(task_id, point3d_bl, params)
        kdict['image_T_bl'] = tfu.transform(self.OPTICAL_FRAME, 'base_link', self.tf_listener)
        point3d_img = tfu.transform_points(kdict['image_T_bl'], point3d_bl)
        point2d_img = self.feature_ex.cal.project(point3d_img)
        head_pose = self.robot.head.pose()

        #Classify
        #pdb.set_trace()
        predictions = np.matrix(self.locations_man.learners[task_id].classify(kdict['instances']))
        pos_indices = np.where(r3d.POSITIVE == predictions)[1].A1
        loc2d_max = None
        try_num = 0
        updated_execution_record = False
        if user_study:
            first_time = True
        else:
            first_time = False

        #If find some positives:
        if len(pos_indices) > 0:
            indices_added = []
            remaining_pt_indices = r3d.inverse_indices(indices_added, kdict['instances'].shape[1])
            remaining_instances = kdict['instances'][:, remaining_pt_indices]
            behavior = self.get_behavior_by_task(self.locations_man.data[task_id]['task'])

            while len(pos_indices) > 0:
                #select from the positive predictions the prediction with the most spatial support
                print 'Num positive points found:', len(pos_indices)

                #figure out max point
                locs2d = None
                if len(pos_indices) > 1:
                    locs2d = kdict['points2d'][:, pos_indices]
                    if np.any(np.isnan(locs2d)) or np.any(np.isinf(locs2d)):
                        pdb.set_trace()
                    locs2d_indices = np.where(False == np.sum(np.isnan(locs2d), 0))[1].A1
                    print locs2d[:, locs2d_indices]
                    loc2d_max, density_image = r3d.find_max_in_density(locs2d[:, locs2d_indices])
                    cv.SaveImage("execute_behavior.png", 255 * (np.rot90(density_image)/np.max(density_image)))
                    dists = ut.norm(kdict['points2d'] - loc2d_max)
                    selected_idx = np.argmin(dists)
                    if not first_time:
                        indices_added.append(selected_idx)
                else:
                    selected_idx = pos_indices[0]
                    loc2d_max = kdict['points2d'][: selected_idx]

                selected_3d = kdict['points3d'][:, selected_idx]

                #Draw
                img = cv.CloneMat(kdict['image'])
                r3d.draw_points(img, point2d_img,       [255, 0, 0],    10, -1)
                r3d.draw_points(img, kdict['points2d'], [51, 204, 255], 3, -1)
                if locs2d != None:
                    r3d.draw_points(img, locs2d,        [255, 204, 51], 3, -1)
                r3d.draw_points(img, loc2d_max,         [255, 204, 51], 10, -1)
                self.locations_man.publish_image(task_id, img, postfix='_execute_pick')

                #Execute
                self.robot.head.set_pose(head_pose, 1)
                print '============================================================'
                print '============================================================'
                print 'Try number', try_num
                print '============================================================'
                print '============================================================'
                if first_time:
                    print 'original point selected is', selected_3d.T
                    selected_3d = selected_3d + np.matrix([0, 0, 0.2]).T
                print 'point selected is', selected_3d.T
                success, _, point_undo_bl = behavior(selected_3d)

                #Save pickle for viz
                pkname = pt.join(task_id, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + '_execute.pkl')
                _, pos_pred, neg_pred = separate_by_labels(kdict['points2d'], predictions)
                if success:
                    label = r3d.POSITIVE
                else:
                    label = r3d.NEGATIVE
                ut.save_pickle({'image': image_name,
                                'pos_pred': pos_pred,
                                'neg_pred': neg_pred,
                                'tried': [kdict['points2d'][:, selected_idx], label],
                                'center': point2d_img}, pkname)

                try_num = try_num + 1
                if not first_time:
                    if success:
                        #if save:
                        if not updated_execution_record:
                            self.locations_man.update_execution_record(task_id, 1.)
                            update_execution_record = True
                        self.robot.sound.say('action succeeded')
                        label = r3d.POSITIVE

                        dataset = self.locations_man.data[task_id]['dataset']
                        nneg = np.sum(dataset.outputs == r3d.NEGATIVE) 
                        npos = np.sum(dataset.outputs == r3d.POSITIVE)
                        if nneg > npos and save:
                            datapoint = {'instances': kdict['instances'][:, selected_idx],
                                         'points2d':  kdict['points2d'][:, selected_idx],
                                         'points3d':  kdict['points3d'][:, selected_idx],
                                         'sizes':     kdict['sizes'],
                                         'labels':    np.matrix([label])}
                            self.locations_man.add_perceptual_data(task_id, datapoint)
                            if save:
                                self.locations_man.save_database()
                            self.locations_man.train(task_id, kdict)
                        break
                    else:
                        self.robot.sound.say('action failed')
                        label = r3d.NEGATIVE
                        if not updated_execution_record:
                            self.locations_man.update_execution_record(task_id, 0.)
                            update_execution_record = True

                        #We were wrong so we add this to our dataset and retrain
                        datapoint = {'instances': kdict['instances'][:, selected_idx],
                                     'points2d':  kdict['points2d'][:, selected_idx],
                                     'points3d':  kdict['points3d'][:, selected_idx],
                                     'sizes':     kdict['sizes'],
                                     'labels':    np.matrix([label])}
                        self.locations_man.add_perceptual_data(task_id, datapoint)
                        if save:
                            self.locations_man.save_database()
                        self.locations_man.train(task_id, kdict)

                        if try_num > max_retries:
                            break

                        #Remove point and predict again!
                        remaining_pt_indices = r3d.inverse_indices(indices_added, kdict['instances'].shape[1])
                        remaining_instances = kdict['instances'][:, remaining_pt_indices]
                        predictions = np.matrix(self.locations_man.learners[task_id].classify(remaining_instances))
                        remaining_pos_indices = np.where(r3d.POSITIVE == predictions)[1].A1
                        pos_indices = remaining_pt_indices[remaining_pos_indices]
                first_time = False

        #If no positives found:
        else:
            img = cv.CloneMat(kdict['image'])
            r3d.draw_points(img, point2d_img,       [255, 0, 0],    10, -1)
            r3d.draw_points(img, kdict['points2d'], [51, 204, 255], 3, -1)
            self.locations_man.publish_image(task_id, img, postfix='_execute_fail')

            if not updated_execution_record:
                self.locations_man.update_execution_record(task_id, 0.)
                update_execution_record = True

            self.robot.sound.say("Perception failure.  Exploring around expected region.")
            #FAIL. Update the location's statistic with this failure. 
            if save:
                self.locations_man.update_execution_record(task_id, 0.)
            def any_pos_sf(labels_mat):
                if np.any(r3d.POSITIVE == labels_mat):
                    return True
                return False
            ctask_id = self.locations_man.data[task_id]['complementary_task_id']
            self.seed_dataset_explore(task_id, ctask_id,
                    point3d_bl, max_retries=max_retries, stop_fun=any_pos_sf, 
                    closeness_tolerance=closeness_tolerance, should_reset=False)

    
    #def autonomous_learn(self, point3d_bl, behavior, object_name): 
    #    # We learn, but must moderate between spatial cues and requirements of
    #    # the learner. Spatial cue is a heuristic that can guide to positive
    #    # examples. Learning heuristic reduces the number of experiments to
    #    # perform given that we know that we are generally *not* successful
    #    # (assume that this procedure launches only during non mission critial circumstances).
    #    # So in the case where we're actively learning we're going to ignore the spatial heuristic.
    #    # Well... can we incorporate distance to the selected 3d point as a feature?
    #    # ah!
    #    learn_manager = self.learners[object_name]
    #    #scan and extract features
    #    self.robot.head.look_at(point3d_bl, 'base_link', True)
    #    learn_manager.scan(point3d_bl)
    #    gaussian = pr.Gaussian(np.matrix([ 0,      0,     0.]).T, \
    #                           np.matrix([[1.,     0,      0], \
    #                                      [0, .02**2,      0], \
    #                                      [0,      0, .02**2]]))

    #    #pdb.set_trace()
    #    gaussian_noise = np.matrix([0,0,0.]).T
    #    while not learn_manager.is_ready():
    #         pi = point3d_bl + gaussian_noise
    #         label = behavior(pi)
    #         #look at point, then try to add again
    #         if not learn_manager.add_example(pi, np.matrix([label])):
    #             rospy.logerr('Unable to extract features from point %s' % str(pi.T))
    #             continue
    #         learn_manager.train()
    #         learn_manager.draw_and_send()
    #         gaussian_noise = gaussian.sample()
    #         gaussian_noise[0,0] = 0

    #    #Acquire data
    #    #Given image, cloud, 3d point ask, extract features.
    #    #while no_interruptions and stopping_criteria_not_reached
    #    #    maximally_informative_point = get maximally informative point
    #    #    label = behavior(maximally_informative_point)
    #    #    retrain!
    #    converged = False
    #    while not converged:
    #        indices, dists = learn_manager.select_next_instances(1)
    #        if idx != None:
    #            pt2d = learn_manager.points2d[:, indices[0]]
    #            pt3d = learn_manager.points3d[:, indices[0]]
    #            label = behavior(pt3d)
    #            #learn_manager.add_example(pt3d, np.matrix([label]), pt2d)
    #            if not learn_manager.add_example(pi, np.matrix([label])):
    #                rospy.logerr('Unable to extract features from point %s' % str(pi.T))
    #                continue
    #            learn_manager.train()
    #            learn_manager.draw_and_send()
    #        else:
    #            converged = True


class TestLearner:

    def __init__(self):
        self.rec_params = r3d.Recognize3DParam()
        self.locations_man = LocationManager('locations_narrow_v11.pkl', rec_params=self.rec_params)

    def test_training_set(self):
        #Pick
        tasks = self.locations_man.data.keys()
        for i, k in enumerate(tasks):
            print i, k
        task_id = tasks[int(raw_input())]

        #Train
        dataset = self.locations_man.data[task_id]['dataset']
        pca = self.locations_man.data[task_id]['pca']

        nneg = np.sum(dataset.outputs == r3d.NEGATIVE) 
        npos = np.sum(dataset.outputs == r3d.POSITIVE)
        print '================= Training ================='
        print 'NEG examples', nneg
        print 'POS examples', npos
        print 'TOTAL', dataset.outputs.shape[1]
        neg_to_pos_ratio = float(nneg)/float(npos)
        #pdb.set_trace()

        #np.random.permutation(range(da
        #separate into positive and negative
        #take 30%
        #train/test
        all_costs_scores = []
        all_ratio_scores = []
        all_bandwidth_scores = []
        for i in range(40):
            percent = .3
            negidx = np.where(dataset.outputs==r3d.NEGATIVE)[1].A1
            posidx = np.where(dataset.outputs==r3d.POSITIVE)[1].A1
            nneg = np.ceil(len(negidx) * percent)
            npos = np.ceil(len(posidx) * percent)
            negperm = np.random.permutation(range(len(negidx)))
            posperm = np.random.permutation(range(len(posidx)))

            test_pos_idx = negperm[0:nneg]
            test_neg_idx = posperm[0:npos]
            test_idx = np.concatenate((test_pos_idx, test_neg_idx))

            train_pos_idx = negperm[nneg:] 
            train_neg_idx = posperm[npos:]
            train_idx = np.concatenate((train_pos_idx, train_neg_idx))

            test_set = dataset.subset(test_idx)
            train_set = dataset.subset(train_idx)

            #pdb.set_trace()
            ratio_score = []
            ratios = [neg_to_pos_ratio * (float(i+1)/10.) for i in range(10)]
            for r in ratios:
                print '######################################################'
                print 'ratio is ', r
                svm_params = '-s 0 -t 2 -g .4 -c 1 '
                learner = self.train(train_set, pca, r, svm_params)
                predictions = np.matrix(learner.classify(test_set.inputs))
                conf_mat = r3d.confusion_matrix(test_set.outputs, predictions)
                combined = conf_mat[0,0] + conf_mat[1,1]
                print '!!!!!!!!!!!'
                print conf_mat
                print combined
                print '!!!!!!!!!!!'
                ratio_score.append(combined)

            bandwidth_score = []
            bandwidths = []
            for i in range(40):
                print '######################################################'
                g = i * .1 #.00625
                bandwidths.append(g)
                print 'g is ', g
                svm_params = '-s 0 -t 2 -g %.2f -c 1 ' % g
                learner = self.train(train_set, pca, neg_to_pos_ratio, svm_params)
                predictions = np.matrix(learner.classify(test_set.inputs))
                conf_mat = r3d.confusion_matrix(test_set.outputs, predictions)
                combined = conf_mat[0,0] + conf_mat[1,1]
                print '!!!!!!!!!!!'
                print conf_mat
                print combined
                print '!!!!!!!!!!!'
                bandwidth_score.append(combined)

            cost_score = []
            costs = []
            for i in range(40):
                print '######################################################'
                cost = i + 1
                costs.append(cost)
                print 'cost is ', cost
                svm_params = '-s 0 -t 2 -g .4 -c %.2f ' % cost
                learner = self.train(train_set, pca, neg_to_pos_ratio, svm_params)
                predictions = np.matrix(learner.classify(test_set.inputs))
                conf_mat = r3d.confusion_matrix(test_set.outputs, predictions)
                combined = conf_mat[0,0] + conf_mat[1,1]
                print '!!!!!!!!!!!'
                print conf_mat
                print combined
                print '!!!!!!!!!!!'
                cost_score.append(combined)

            #print 'Cost score'
            #print costs
            #print cost_score, "\n"

            #print 'Ratio score'
            #print ratios
            #print ratio_score, "\n"

            #print 'Bandwidth score'
            #print bandwidths 
            #print bandwidth_score, "\n"
            all_costs_scores.append(cost_score)
            all_ratio_scores.append(ratio_score)
            all_bandwidth_scores.append(bandwidth_score)

        
        print 'Cost score'
        print costs
        costs_mean = np.sum(np.matrix(all_costs_scores), 0) / float(len(all_costs_scores))
        print costs_mean
        print np.max(costs_mean)
        print costs[np.argmax(costs_mean)]

        print 'Ratio score'
        print ratios
        ratios_mean = np.sum(np.matrix(all_ratio_scores), 0) / float(len(all_ratio_scores))
        print ratios_mean
        print np.max(ratios_mean)
        print ratios[np.argmax(ratios_mean)]

        print 'Bandwidth score'
        print bandwidths 
        bandwidths_mean = np.sum(np.matrix(all_bandwidth_scores), 0) / float(len(all_bandwidth_scores))
        print bandwidths_mean
        print np.max(bandwidths_mean)
        print bandwidths[np.argmax(bandwidths_mean)]




    def train(self, dataset, pca, neg_to_pos_ratio, svm_params):
        weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
        previous_learner = None
        learner = r3d.SVMPCA_ActiveLearner(use_pca=True, 
                        reconstruction_std_lim=self.rec_params.reconstruction_std_lim, 
                        reconstruction_err_toler=self.rec_params.reconstruction_err_toler,
                        old_learner=None, pca=pca)

        rec_params = self.rec_params
        inputs_for_pca = dataset.inputs
        learner.train(dataset, 
                      inputs_for_pca,
                      svm_params + weight_balance,
                      rec_params.variance_keep)
        return learner


def test_display():
    rec_params = r3d.Recognize3DParam()
    locations_man = LocationManager('locations_narrow_v11.pkl', 
            rec_params=rec_params)
    location_display = LocationDisplay(locations_man)
    location_display.run()
    #location_display.start()
    


def launch():
    import optparse
    p = optparse.OptionParser()
    p.add_option("-d", "--display", action="store_true", default=False)
    p.add_option("-p", "--practice", action="store_true", default=False)
    p.add_option("-e", "--execute", action="store_true", default=False)
    p.add_option("-b", "--base", action="store_true", default=False)
    p.add_option("-s", "--static", action="store_true", default=False)
    p.add_option("-i", "--init", action="store_true", default=False)
    p.add_option("-t", "--test", action="store_true", default=False)
    p.add_option("-u", "--user_study", action="store_true", default=False)

    opt, args = p.parse_args()
    if opt.display:
        test_display()
        return

    if opt.practice or opt.execute or opt.init or opt.base:
        l = ApplicationBehaviors()
        mode = None
        if opt.practice:
            mode = 'practice'
        if opt.execute:
            mode = 'execute'
        if opt.init:
            mode = 'init'
        if opt.base:
            mode = 'update_base'
        rospy.loginfo('Using mode %s' % mode)
        l.run(mode, not opt.static, opt.user_study)

    if opt.test:
        tl = TestLearner()
        tl.test_training_set()


if __name__ == '__main__':
    launch()
    #cProfile.run('launch()', 'profile_linear_move.prof')






























        #return {'points3d': np.column_stack(points3d_tried),
        #        'instances': np.column_stack(instances_tried),
        #        'points2d': np.column_stack(points2d_tried),
        #        'labels': np.matrix(labels),
        #        'sizes': fea_dict['sizes']}


    #def blind_exploration3(self, task_id, behavior, undo_behavior, point_bl, stop_fun, 
    #        max_retries=15, closeness_tolerance=.01, fea_dict=None):
    #    params = r3d.Recognize3DParam()
    #    params.uncertainty_x = 1.
    #    params.uncertainty_y = .02
    #    params.uncertainty_z = .02
    #    params.n_samples = 400
    #    params.uni_mix = 0.
    #    #MAX_RETRIES = 20
    #

    #    # instances, locs2d, locs3d, image, rdict, sizes = 
    #    if fea_dict == None:
    #        fea_dict, _ = self.read_features_save(task_id, point_bl, params)
    #        image_T_bl = tfu.transform('openni_rgb_optical_frame', 'base_link', self.tf_listener)
    #        fea_dict['image_T_bl'] = image_T_bl
    #    #fea_dict = self.feature_ex.read(expected_loc_bl=point_bl, params=params)
    #    
    #    dists = ut.norm(fea_dict['points3d'] - point_bl)
    #    ordering = np.argsort(dists).A1
    #    points3d_sampled = fea_dict['points3d'][:, ordering]
    #    points2d_sampled = fea_dict['points2d'][:, ordering]
    #    instances_sampled = fea_dict['instances'][:, ordering]
    #    start_pose = self.robot.head.pose()

    #    point3d_img = tfu.transform_points(fea_dict['image_T_bl'], point_bl)
    #    point2d_img = self.feature_ex.cal.project(point3d_img)
   
    #    sampled_idx = 0
    #    iter_count = 0
    #    labels = []
    #    points_tried = []
    #    tinstances = []
    #    sp2d = []
    #    while iter_count < max_retries and not stop_fun(np.matrix(labels)):
    #        if len(points_tried)> 0 and \
    #           np.any(ut.norm(np.column_stack(points_tried) - points3d_sampled[:, sampled_idx]) < closeness_tolerance):
    #            sampled_idx = sampled_idx + 1
    #            continue

    #        #pdb.set_trace()
    #        #self.robot.sound.say('executing behavior')
    #        self.robot.head.set_pose(start_pose, 1)
    #        success, reason = behavior(points3d_sampled[:, sampled_idx])
    #        iter_count = iter_count + 1
    #        points_tried.append(points3d_sampled[:, sampled_idx])
    #        tinstances.append(instances_sampled[:, sampled_idx])
    #        sp2d.append(points2d_sampled[:, sampled_idx])
    #        sampled_idx = sampled_idx + 1

    #        #tinstances.append(fea_dict['instances'][:,iter_count])
    #        #sp2d.append(fea_dict['points2d'][:,iter_count])
    #        #add point and label to points tried
    #        if success:
    #            labels.append(r3d.POSITIVE)
    #            if undo_behavior != None:
    #                #If we were successful, call blind exploration with the undo behavior
    #                def any_pos_sf(labels_mat):
    #                    if np.any(r3d.POSITIVE == labels_mat):
    #                        return True
    #                    return False
    #                if task_id != None:
    #                    utid = self.locations_man.create_undo_task(task_id)
    #                else:
    #                    utid = None
    #                #TODO: gather instances for undo action
    #                #TODO: figure out why position of point_bl is shifted in second call
    #                self.seed_dataset_explore(utid, undo_behavior, None, point_bl, any_pos_sf, 
    #                        max_retries, fea_dict=fea_dict)
    #                #success, reason = undo_behavior(points3d_sampled[:, 'iter_count'])
    #        else:
    #            labels.append(r3d.NEGATIVE)

    #        #Visualization
    #        img = cv.CloneMat(fea_dict['image'])
    #        r3d.draw_points(img, points2d_sampled, [255, 255, 255], 2, -1)
    #        _, pos_points, neg_points = separate_by_labels(np.column_stack(sp2d), np.matrix(labels))
    #        r3d.draw_points(img, point2d_img, [255, 0, 0], 4, 2)
    #        r3d.draw_points(img, pos_points, [0, 255, 0], 2, -1)
    #        r3d.draw_points(img, neg_points, [0, 0, 255], 2, -1)
    #        r3d.draw_points(img, sp2d[-1], [0, 184, 245], 3, -1)
    #        self.img_pub.publish(img)
    #
    #    rospy.loginfo('tried %d times' % iter_count)
    #    return {'points3d': np.column_stack(points_tried),
    #            'instances': np.column_stack(tinstances),
    #            'points2d': np.column_stack(sp2d),
    #            'labels': np.matrix(labels),
    #            'sizes': fea_dict['sizes']}
    #   #if iter_count > MAX_RETRIES:
    #   #    self.robot.sound.say('giving up tried %d times already' % MAX_RETRIES)
    #   #    break
    #   #elif not success:
    #   #     self.robot.sound.say('retrying')

    #   return points tried record
            #success, _ = self.light_switch1(perturbed_point_bl, point_offset=point_offset, \
            #        press_contact_pressure=300, move_back_distance=np.matrix([-.0075,0,0]).T,\
            #        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
            #        visual_change_thres=.03)

    #   points tried = []
    #   while we have not succeeded and not stop_fun(points tried):
    #       label = behavior(point)
    #       add point and label to points tried
    #       perturb point
    #   return points tried record


    #def load_classifier(self, name, fname):
    #    print 'loading classifier'
    #    dataset = ut.load_pickle(fname)
    #    self.train(dataset, name)

        #self.location_labels = []
        #self.location_data = []
        #if os.path.isfile(self.saved_locations_fname):
        #    location_data = ut.load_pickle(self.saved_locations_fname) #each col is a 3d point, 3xn mat
        #    for idx, rloc in enumerate(location_data):
        #        self.location_centers.append(rloc['center'])
        #        self.location_labels.append(idx)
        #    self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
        #    self.location_data = location_data
        #if os.path.isfile(self.saved_locations_fname):
        #    location_data = ut.load_pickle(self.saved_locations_fname) #each col is a 3d point, 3xn mat
        #    for idx, rloc in enumerate(location_data):
        #        self.location_centers.append(rloc['center'])
        #        self.location_labels.append(idx)
        #    self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
        #    self.location_data = location_data
        #pass

        #location_idx = self.location_labels[close_by_locs[0]]
        #ldata = self.location_data[location_idx]

        #rospy.loginfo('location_add: point close to %d at %s.' % (location_idx, str(ldata['center'].T)))
        #ldata['points'].append(point_map)
        #ldata['center'] = np.column_stack(ldata['points']).mean(1)
        #self.location_centers[location_idx] = ldata['center']
        #self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
#    def update_center(self, center_id, point_map):
#        #If close by locations found then add to points list and update center
#        location_idx = self.location_labels[close_by_locs[0]]
#        ldata = self.location_data[location_idx]
#
#        rospy.loginfo('location_add: point close to %d at %s.' % (location_idx, str(ldata['center'].T)))
#        ldata['points'].append(point_map)
#        ldata['center'] = np.column_stack(ldata['points']).mean(1)
#        self.location_centers[location_idx] = ldata['center']
#        self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
#


    #def location_add(self, point_map, task, data):
    #    close_by_locs = self.find_close_by_points_match_task(point_map, task)
    #    if len(close_by_locs) == 0:
    #        rospy.loginfo('location_add: point not close to any existing location. creating new record.')
    #        self.location_data.append({
    #            'task': task, 
    #            'center': point_map, 
    #            'perceptual_dataset': None,
    #            'points':[point_map]})
    #        self.location_centers.append(point_map)
    #        self.location_labels.append(len(self.location_data) - 1)
    #        self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
    #    else:
    #        #If close by locations found then add to points list and update center
    #        location_idx = self.location_labels[close_by_locs[0]]
    #        ldata = self.location_data[location_idx]

    #        rospy.loginfo('location_add: point close to %d at %s.' % (location_idx, str(ldata['center'].T)))
    #        ldata['points'].append(point_map)
    #        ldata['center'] = np.column_stack(ldata['points']).mean(1)
    #        self.location_centers[location_idx] = ldata['center']
    #        self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))

    #    ut.save_pickle(self.location_data, self.saved_locations_fname)
    #    rospy.loginfo('location_add: saved point in map.')

#    def find_close_by_points(self, point_map):
#        if self.locations_tree != None:
#            close_by_locs = self.locations_tree.query_ball_point(np.array(point_map.T), self.LOCATION_ADD_RADIUS)[0]
#            return close_by_locs
#        else:
#            return []

#   3)   listing all locations
#   4)   listing locations closest to given point, with and without task


    #def find_close_by_points_match_task(self, point_map, task):
    #    matches = self.find_close_by_points(point_map)
    #    task_matches = []
    #    for m in matches:
    #        idx = self.location_labels[m]
    #        ldata = self.location_data[idx]
    #        if ldata['task'] == task:
    #            task_matches.append(m)
    #    return task_matches
    
#class PickPointsCloseToStartLocation:
#
#    def __init__(self, point_bl, closeness_tolerance=.01, max_retries=20):
#        self.params = r3d.Recognize3DParam()
#        self.params.uncertainty_x = 1.
#        self.params.uncertainty_y = .02
#        self.params.uncertainty_z = .02
#        self.params.n_samples = 400
#        self.params.uni_mix = 0.
#
#        self.sampled_idx = 0
#        self.iter_count = 0
#        self.max_retries = max_retries
#        self.closeness_tolerance = closeness_tolerance
#
#        self.points3d_tried = []
#        self.points2d_tried = []
#        self.instances_tried = []
#
#    def process_scan(self, fea_dict):
#        dists = ut.norm(fea_dict['points3d'] - point_bl)
#        ordering = np.argsort(dists).A1
#
#        self.points3d_sampled = fea_dict['points3d'][:, ordering]
#        self.points2d_sampled = fea_dict['points2d'][:, ordering]
#        self.instances_sampled = fea_dict['instances'][:, ordering]
#
#    def get_params(self):
#        return self.params
#
#    def stop(self):
#        return self.iter_count > max_retries
#
#    def pick_next(self):
#        while len(self.points3d_tried) > 0 \
#                and np.any(ut.norm(np.column_stack(self.points3d_tried) - self.points3d_sampled[:, self.sampled_idx]) < self.closeness_tolerance):
#            self.sampled_idx = self.sampled_idx + 1 
#
#        self.points3d_tried.append(self.points3d_sampled[:, self.sampled_idx])
#        self.points2d_tried.append(self.points2d_sampled[:, self.sampled_idx])
#        self.instances_tried.append(self.instances_sampled[:, self.sampled_idx])
#        self.iter_count = iter_count + 1
#
#        return {'points3d':  self.points3d_sampled[:, self.sampled_idx],
#                'points2d':  self.points2d_sampled[:, self.sampled_idx],
#                'instances': self.instances_sampled[:, self.sampled_idx]}
#
#    def get_instances_used(self):
#        if len(self.points3d_sampled) > 0:
#            return {'points3d': np.column_stack(self.points3d_sampled),
#                    'points2d': np.column_stack(self.points2d_sampled),
#                    'instances': np.column_stack(self.instances_sampled)}
#        else:
#            return None
#
#class PickPointsUsingActiveLearning:
#
#    def __init__(self, locations_manager):
#        self.params = r3d.Recognize3DParam()
#        self.params.uncertainty_x = 1.
#        self.params.n_samples = 2000
#        self.params.uni_mix = .1
#
#        self.points3d_tried = []
#        self.points2d_tried = []
#        self.instances_tried = []
#
#    def process_scan(self, fea_dict):
#
#    def get_params(self):
#
#    def pick_next(self):
#
#    def stop(self):
#
#    def get_instances_used(self):


        #self.LOCATION_ADD_RADIUS = .5
        #self.kinect_listener = kl.KinectListener()
        #self.kinect_cal = rc.ROSCameraCalibration('camera/rgb/camera_info')

        #self.kinect_img_sub = message_filters.Subscriber('/camera/rgb/image_color', smsg.Image)
        #self.kinect_depth_sub = message_filters.Subscriber('/camera/depth/points2', smsg.PointCloud2)
        #ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
        #ts.registerCallback(callback)

        #self.load_classifier('light_switch', 'labeled_light_switch_data.pkl')
        #self.start_location = (np.matrix([0.25, 0.30, 1.3]).T, np.matrix([0., 0., 0., 0.1]))

        #loading stored locations
        #self.saved_locations_fname = 'saved_locations.pkl'
        #self.location_centers = []
        #self.location_labels = []
        #self.location_data = []
        #self.locations_tree = None

        #if os.path.isfile(self.saved_locations_fname):
        #    location_data = ut.load_pickle(self.saved_locations_fname) #each col is a 3d point, 3xn mat
        #    for idx, rloc in enumerate(location_data):
        #        self.location_centers.append(rloc['center'])
        #        self.location_labels.append(idx)
        #    self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
        #    self.location_data = location_data

        # joint angles used for tuck
        #pdb.set_trace()
        #self.untuck()
        #self.behaviors.movement.set_movement_mode_ik()
        #self.movement.set_movement_mode_ik()
        #self.tuck()
        #self.r1 = np.matrix([[-0.31006769,  1.2701541 , -2.07800829, -1.45963243, -4.35290489,
        #                 -1.86052221,  5.07369192]]).T
        #self.l0 = np.matrix([[  1.05020383,  -0.34464327,   0.05654   ,  -2.11967694,
        #                 -10.69100221,  -1.95457839,  -3.99544713]]).T
        #self.l1 = np.matrix([[  1.06181076,   0.42026402,   0.78775801,  -2.32394841,
        #                 -11.36144995,  -1.93439025,  -3.14650108]]).T
        #self.l2 = np.matrix([[  0.86275197,   0.93417818,   0.81181124,  -2.33654346,
        #                 -11.36121856,  -2.14040499,  -3.15655164]]).T
        #self.l3 = np.matrix([[ 0.54339568,  1.2537778 ,  1.85395725, -2.27255481, -9.92394984,
        #                 -0.86489749, -3.00261708]]).T



    #def train(self, dataset, name):
    #    rec_params = self.feature_ex.rec_params
    #    nneg = np.sum(dataset.outputs == r3d.NEGATIVE) #TODO: this was copied and pasted from r3d
    #    npos = np.sum(dataset.outputs == r3d.POSITIVE)
    #    print '================= Training ================='
    #    print 'NEG examples', nneg
    #    print 'POS examples', npos
    #    print 'TOTAL', dataset.outputs.shape[1]
    #    neg_to_pos_ratio = float(nneg)/float(npos)
    #    weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
    #    print 'training'
    #    learner = r3d.SVMPCA_ActiveLearner(use_pca=True)
    #    #TODO: figure out something scaling inputs field!
    #    learner.train(dataset, dataset.inputs,
    #                  rec_params.svm_params + weight_balance,
    #                  rec_params.variance_keep)
    #    self.learners[name] = {'learner': learner, 'dataset': dataset}
    #    print 'done loading'



    #def tuck(self):
    #    ldiff = np.linalg.norm(pr2.diff_arm_pose(self.robot.left.pose(), self.l3))
    #            # np.linalg.norm(self.robot.left.pose() - self.l3)
    #    rdiff = np.linalg.norm(pr2.diff_arm_pose(self.robot.right.pose(), self.r1))
    #    #rdiff = np.linalg.norm(self.robot.right.pose() - self.r1)
    #    if ldiff < .3 and rdiff < .3:
    #        rospy.loginfo('tuck: Already tucked. Ignoring request.')
    #        return
    #    self.robot.right.set_pose(self.r1, block=False)
    #    self.robot.left.set_pose(self.l0, block=True)
    #    poses = np.column_stack([self.l0, self.l1, self.l2, self.l3])
    #    #pdb.set_trace()
    #    self.robot.left.set_poses(poses, np.array([0., 1.5, 3, 4.5]))


    #def untuck(self):
    #    if np.linalg.norm(self.robot.left.pose() - self.l0) < .3:
    #        rospy.loginfo('untuck: Already untucked. Ignoring request.')
    #        return
    #    self.robot.right.set_pose(self.r1, 2., block=False)
    #    self.robot.left.set_pose(self.l3, 2.,  block=True)
    #    poses = np.column_stack([self.l3, self.l2, self.l1, self.l0])
    #    self.robot.left.set_poses(poses, np.array([0., 3., 6., 9.])/2.)


            #if len(self.location_centers) < 1:
            #    return
            #rospy.loginfo('click_cb: double clicked but no 3d point given')
            #rospy.loginfo('click_cb: will use the last successful location given')

            #base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
            #point_bl = tfu.transform_points(base_link_T_map, self.location_centers[-1])
            #rospy.loginfo('click_cb: using ' + str(self.location_centers[-1].T))
            #self.location_activated_behaviors(point_bl, stored_point=True)


    #def find_close_by_points(self, point_map):
    #    if self.locations_tree != None:
    #        close_by_locs = self.locations_tree.query_ball_point(np.array(point_map.T), self.LOCATION_ADD_RADIUS)[0]
    #        return close_by_locs
    #    else:
    #        return []

    #def find_close_by_points_match_task(self, point_map, task):
    #    matches = self.find_close_by_points(point_map)
    #    task_matches = []
    #    for m in matches:
    #        idx = self.location_labels[m]
    #        ldata = self.location_data[idx]
    #        if ldata['task'] == task:
    #            task_matches.append(m)
    #    return task_matches

    #def location_add(self, point_map, task, data):
    #    close_by_locs = self.find_close_by_points_match_task(point_map, task)
    #    if len(close_by_locs) == 0:
    #        rospy.loginfo('location_add: point not close to any existing location. creating new record.')
    #        self.location_data.append({
    #            'task': task, 
    #            'center': point_map, 
    #            'perceptual_dataset': None,
    #            'points':[point_map]})
    #        self.location_centers.append(point_map)
    #        self.location_labels.append(len(self.location_data) - 1)
    #        self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
    #    else:
    #        #If close by locations found then add to points list and update center
    #        location_idx = self.location_labels[close_by_locs[0]]
    #        ldata = self.location_data[location_idx]

    #        rospy.loginfo('location_add: point close to %d at %s.' % (location_idx, str(ldata['center'].T)))
    #        ldata['points'].append(point_map)
    #        ldata['center'] = np.column_stack(ldata['points']).mean(1)
    #        self.location_centers[location_idx] = ldata['center']
    #        self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))

    #    ut.save_pickle(self.location_data, self.saved_locations_fname)
    #    rospy.loginfo('location_add: saved point in map.')


    #def location_add(self, point_map, task):
    #    close_by_locs = self.find_close_by_points_match_task(point_map, task)
    #    if len(close_by_locs) == 0:
    #        rospy.loginfo('location_add: point not close to any existing location. creating new record.')
    #        self.location_data.append({
    #            'task': task, 
    #            'center': point_map, 
    #            'points':[point_map]})
    #        self.location_centers.append(point_map)
    #        self.location_labels.append(len(self.location_data) - 1)
    #        self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))

    #    else:
    #        #If close by locations found then add to points list and update center
    #        location_idx = self.location_labels[close_by_locs[0]]
    #        ldata = self.location_data[location_idx]

    #        rospy.loginfo('location_add: point close to %d at %s.' % (location_idx, str(ldata['center'].T)))
    #        ldata['points'].append(point_map)
    #        ldata['center'] = np.column_stack(ldata['points']).mean(1)
    #        self.location_centers[location_idx] = ldata['center']
    #        self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))

    #    ut.save_pickle(self.location_data, self.saved_locations_fname)
    #    rospy.loginfo('location_add: saved point in map.')


    #def record_processed_data_kinect2(self, point3d_bl, kinect_fea):
    #    instances, locs2d_image, locs3d_bl, image = kinect_fea #self.feature_ex.read(point3d_bl)
    #    #rospy.loginfo('Getting a kinect reading')

    #    tstring = time.strftime('%A_%m_%d_%Y_%I:%M%p')
    #    kimage_name = '%s_highres.png' % tstring
    #    cv.SaveImage(kimage_name, kimage)

    #    preprocessed_dict = {'instances': instances,
    #                         'points2d': locs2d_image,
    #                         'points3d': locs3d_bl,
    #                         'image': kimage_name,
    #                         'labels': labels,
    #                         'sizes': feature_extractor.sizes}


        #self.feature_ex.read(point3d_bl)
        #rdict = self.kinect_listener.read()
        #kimage = rdict['image']
        #rospy.loginfo('Waiting for calibration.')
        #while self.kinect_cal.has_msg == False:
        #    time.sleep(.1)

        #which frames?
        #rospy.loginfo('Getting transforms.')
        #k_T_bl = tfu.transform('openni_rgb_optical_frame', '/base_link', self.tf_listener)
        #tstring = time.strftime('%A_%m_%d_%Y_%I:%M%p')
        #kimage_name = '%s_highres.png' % tstring
        #rospy.loginfo('Saving images (basename %s)' % tstring)
        #cv.SaveImage(kimage_name, kimage)
        #rospy.loginfo('Saving pickles')
        #pickle_fname = '%s_interest_point_dataset.pkl' % tstring   

        #data_pkl = {'touch_point': point3d_bl,
        #            'points3d': rdict['points3d'],
        #            'image': kimage_name,
        #            'cal': self.prosilica_cal, 
        #            'k_T_bl': k_T_bl}
                    #'point_touched': point3d_bl}

        #ut.save_pickle(data_pkl, pickle_fname)
        #print 'Recorded to', pickle_fname




            #npoint = point + gaussian_noise
            #success_off, touchloc_bl = self.light_switch1(npoint, 
            #pdb.set_trace()




#    ##
#    # The behavior can make service calls to a GUI asking users to label
#    def repeat_action(self, task_id, ctask_id, point3d_bl, sampling_object, stop_fun, fea_dict=None):
#
#        # instances, locs2d_image, locs3d_bl, image, raw_dict = 
#        #kf_dict = self.feature_ex.read(point3d_bl)
#        params = r3d.Recognize3DParam()
#        params.uncertainty_x = 1.
#        params.n_samples = 2000
#        params.uni_mix = .1
#
#        kdict, fname = self.read_features_save(task_id, point3d_bl, params)
#        learner = self.locations_man.learners[task_id]
#        behavior = self.get_behavior_by_task(self.locations_man.data[task_id]['task'])
#        undo_behavior = self.get_undo_behavior_by_task(self.locations_man.data[task_id]['task'])
#        start_pose = self.robot.head.pose()
#
#        kdict['image_T_bl'] = tfu.transform('openni_rgb_optical_frame', 'base_link', self.tf_listener)
#        point3d_img = tfu.transform_points(kdict['image_T_bl'], point3d_bl)
#        point2d_img = self.feature_ex.cal.project(point3d_img)
#
#        labels = []
#        points3d_tried = []
#        points2d_tried = []
#        converged = False
#        indices_added = []
#        pdb.set_trace()
#
#
#        while not converged and not stop_fun(np.matrix(labels)):
#            #Find remaining instances
#            remaining_pt_indices = r3d.inverse_indices(indices_added, kdict['instances'].shape[1])
#            remaining_instances = kdict['instances'][:, remaining_pt_indices]
#
#            #Ask learner to pick an instance
#            ridx, selected_dist, converged = learner.select_next_instances_no_terminate(remaining_instances)
#            selected_idx = remaining_pt_indices[ridx]
#            indices_added.append(selected_idx)
#
#            #draw
#            img = cv.CloneMat(kdict['image'])
#            #Draw the center
#            r3d.draw_points(img, point2d_img, [255, 0, 0], 4, 2)
#            #Draw possible points
#            r3d.draw_points(img, kdict['points2d'], [255, 255, 255], 2, -1)
#            #Draw what we have so far
#            if len(points2d_tried) > 0:
#                _, pos_exp, neg_exp = separate_by_labels(np.column_stack(points2d_tried), np.matrix(labels))
#                r3d.draw_points(img, pos_exp, [0, 255, 0], 3, 1)
#                r3d.draw_points(img, neg_exp, [0, 0, 255], 3, 1)
#
#            predictions = np.matrix(learner.classify(kdict['instances']))
#            _, pos_pred, neg_pred = separate_by_labels(kdict['points2d'], predictions)
#            r3d.draw_points(img, pos_pred, [0, 255, 0], 2, -1)
#            r3d.draw_points(img, neg_pred, [0, 0, 255], 2, -1)
#
#            #Draw what we're selecting
#            r3d.draw_points(img, kdict['points2d'][:, selected_idx], [0, 184, 245], 3, -1)
#            self.img_pub.publish(img)
#
#            #Get label for instance
#            self.robot.head.set_pose(start_pose, 1)
#
#            #EXCECUTE!!
#            success, reason = behavior(kdict['points3d'][:, selected_idx])
#            if success:
#                color = [0,255,0]
#                label = r3d.POSITIVE
#                def any_pos_sf(labels_mat):
#                    if np.any(r3d.POSITIVE == labels_mat):
#                        return True
#                    return False
#                utid = self.locations_man.create_undo_task(task_id)
#                self.blind_exploration2(utid, undo_behavior, None, point3d_bl, any_pos_sf, 
#                        max_retries=max_undo_retries, fea_dict=kdict)
#
#            else:
#                label = r3d.NEGATIVE
#                color = [0,0,255]
#
#            labels.append(label)
#            points3d_tried.append(kdict['points3d'][:, selected_idx])
#            points2d_tried.append(kdict['points2d'][:, selected_idx])
#
#            datapoint = {'instances': kdict['instances'][:, selected_idx],
#                         'points2d':  kdict['points2d'][:, selected_idx],
#                         'points3d':  kdict['points3d'][:, selected_idx],
#                         'sizes':     kdict['sizes'],
#                         'labels':    np.matrix([label])
#                         }
#            self.locations_man.add_perceptual_data(task_id, datapoint)
#            self.locations_man.save_database()
#            self.locations_man.train(task_id)
#
#            #Classify
#            predictions = np.matrix(learner.classify(kdict['instances']))
#
#            #Draw
#            img = cv.CloneMat(kdict['image'])
#            _, pos_exp, neg_exp = separate_by_labels(np.column_stack(points2d_tried), np.matrix(labels))
#            r3d.draw_points(img, point2d_img, [255, 0, 0], 4, 2)
#            r3d.draw_points(img, kdict['points2d'], [255, 255, 255], 2, -1)
#            r3d.draw_points(img, pos_exp, [0, 255, 0], 3, 1)
#            r3d.draw_points(img, neg_exp, [0, 0, 255], 3, 1)
#
#            _, pos_pred, neg_pred = separate_by_labels(kdict['points2d'], predictions)
#            r3d.draw_points(img, pos_pred, [0, 255, 0], 2, -1)
#            r3d.draw_points(img, neg_pred, [0, 0, 255], 2, -1)
#            r3d.draw_points(img, points2d_tried[-1], color, 3, -1)
#
#            #publish
#            self.img_pub.publish(img)






    #Save dataset in the location's folder
    #def save_dataset(self, task_id, point, rdict):
    #    pt.join(task_id, 
    #    self.locations_man
    #    self.record_perceptual_data(point, rdict)
    #    #TODO...

    #TODO TEST
    #BOOKMARK 3/7 4:03 AM
    #LAST DITCH EXECUTION(point, stop_fun):
    #def blind_exploration(self, behavior, point_bl, stop_fun, max_retries=15):
    #    gaussian = pr.Gaussian(np.matrix([ 0,      0,      0.]).T, \
    #                           np.matrix([[1.,     0,      0], \
    #                                      [0, .02**2,      0], \
    #                                      [0,      0, .02**2]]))

    #    iter_count = 0
    #    gaussian_noise = np.matrix([0, 0, 0.0]).T #We want to try the given point first
    #    labels = []
    #    points_tried = []

    #    #while we have not succeeded and not stop_fun(points tried):
    #    while iter_count < MAX_RETRIES and stop_fun(np.matrix(labels)):
    #        perturbation = gaussian_noise
    #        perturbed_point_bl = point_bl + perturbation

    #        self.robot.sound.say('executing behavior')
    #        success, reason = behavior(perturbed_point_bl)
    #        points_tried.append(perturbed_point_bl)

    #        #add point and label to points tried
    #        if success:
    #            labels.append(r3d.POSITIVE)
    #        else:
    #            labels.append(r3d.NEGATIVE)

    #        #perturb point
    #        gaussian_noise = gaussian.sample()
    #        gaussian_noise[0,0] = 0
    #        iter_count = iter_count + 1 
    #   
    #   self.robot.sound.say('tried %d times' % iter_count)
    #   return np.column_stack(points_tried)

    #def blind_exploration2(self, task_id, behavior, undo_behavior, point_bl, stop_fun, 
    #        max_retries=15, closeness_tolerance=.005, fea_dict=None):
    #    params = r3d.Recognize3DParam()
    #    params.uncertainty_x = 1.
    #    params.uncertainty_y = .02
    #    params.uncertainty_z = .02
    #    params.n_samples = 400
    #    params.uni_mix = 0.
    #    MAX_RETRIES = 20
    #
    #    if fea_dict == None:
    #        fea_dict, _ = self.read_features_save(task_id, point_bl, params)
    #    
    #    dists = ut.norm(fea_dict['points3d'] - point_bl)
    #    ordering = np.argsort(dists).A1
    #    points3d_sampled = fea_dict['points3d'][:, ordering]
    #    points2d_sampled = fea_dict['points2d'][:, ordering]
    #    instances_sampled = fea_dict['instances'][:, ordering]

    #    labels = []
    #    points_tried = []
    #    tinstances = []
    #    sp2d = []

    #    labels.append(r3d.POSITIVE)
    #    points_tried.append(points3d_sampled[:, 0])
    #    tinstances.append(instances_sampled[:, 0])
    #    sp2d.append(points2d_sampled[:, 0])

    #    labels.append(r3d.NEGATIVE)
    #    points_tried.append(points3d_sampled[:, 1])
    #    tinstances.append(instances_sampled[:, 1])
    #    sp2d.append(points2d_sampled[:, 1])

    #    return {'points3d': np.column_stack(points_tried),
    #            'instances': np.column_stack(tinstances),
    #            'points2d': np.column_stack(sp2d),
    #            'labels': np.matrix(labels),
    #            'sizes': fea_dict['sizes']}

        #def __init__(self, object_name, labeled_data_fname, tf_listener):
        #make learner
        #learner = SVMActiveLearnerApp()
        #labeled_light_switch_dataset = ut.load_pickle(data_file_name)
        #learner.train(labeled_light_switch_dataset, 
        #              labeled_light_switch_dataset.sizes['intensity']
        #              self.params.variance_keep)
        #self.learners[classifier_name] = learner


    #def locate_light_switch(self):
    #    #capture data
    #    pointcloud_msg = self.laser_scan.scan(math.radians(180.), math.radians(-180.), 20.)
    #    prosilica_image = self.prosilica.get_frame() #TODO check if this is a cvmat
    #    while self.prosilica_cal.has_msg == False:
    #        time.sleep(.1)

    #    #preprocess 
    #    ic_data = IntensityCloudData(pointcloud_msg, prosilica_image, 
    #                    tfu.transform('/high_def_optical_frame', '/base_link', self.tf_listener), 
    #                    self.prosilica_cal,                                                       
    #                    r3d.Recognize3DParam())
    #    instances = ic_data.extract_vectorized_features()

    #    results = []
    #    for i in range(instances.shape[1]):
    #        nlabel = self.learners['light_switch'].classify(instances[:, i])
    #        results.append(nlabel)

    #    results = np.matrix(results)
    #    positive_indices = np.where(results == r3d.POSITIVE)[1]

    #    #want 3d location of each instance
    #    positive_points_3d = ic_data.sampled_points[:, positive_indices]

    #    #return a random point for now
    #    rindex = np.random.randint(0, len(positive_indices))
    #    return positive_points_3d[:,rindex]


    #def add_perturbation_to_location(self, point_map, perturbation):
    #    locs = self.find_close_by_points(point_map)
    #    if locs != None:
    #        location = self.location_data[self.location_labels(locs[0])]
    #        if not location.has_key('perturbation'):
    #            location['perturbation'] = []
    #        location['perturbation'].append(perturbation)
    #        return True
    #    return False
















                #self.go_to_home_pose()
                #print '>>>> POINT IS', point_bl_t1.T
                #point_bl_t1 = np.matrix([[ 0.73846737,  0.07182931,  0.55951065]]).T
        #DIST_THRESHOLD = .8 for lightswitch
        #DIST_THRESHOLD = .85 #for drawers
        #DIST_APPROACH = .5
        #COARSE_STOP = .7
        #FINE_STOP = .7
        #VOI_RADIUS = .2

        #point_dist = np.linalg.norm(point_bl_t0[0:2,0])
        #rospy.loginfo('run_behaviors: Point is %.3f away.' % point_dist)
        #map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        #point_map = tfu.transform_points(map_T_base_link, point_bl_t0)

        #if point_dist > DIST_THRESHOLD:
        #    rospy.loginfo('run_behaviors: Point is greater than %.1f m away (%.3f).  Driving closer.' % (DIST_THRESHOLD, point_dist))
        #    ##self.turn_to_point(point_bl_t0)
        #    rospy.loginfo( 'run_behaviors: CLICKED on point_bl ' + str(point_bl_t0.T))

        #    ret = self.drive_approach_behavior(point_bl_t0, dist_far=COARSE_STOP)
        #    if ret != 3:
        #        base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
        #        point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
        #        dist_end = np.linalg.norm(point_bl_t1[0:2,0])
        #        if dist_end > DIST_THRESHOLD:
        #            rospy.logerr('run_behaviors: drive_approach_behavior failed! %.3f' % dist_end)
        #            self.robot.sound.say("I am unable to navigate to that location")
        #            return

        #    base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
        #    point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)

        #    ret = self.approach_perpendicular_to_surface(point_bl_t1, voi_radius=VOI_RADIUS, dist_approach=FINE_STOP)
        #    if ret != 3:
        #        rospy.logerr('run_behaviors: approach_perpendicular_to_surface failed!')
        #        return

        #    #map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        #    #point_bl_t2 = tfu.transform_points(base_link_T_map, point_map)
        #    self.robot.sound.say('done')
        #    rospy.loginfo('run_behaviors: DONE DRIVING!')
        #elif False:





        #if tf_listener == None:
        #    self.tf_listener = tf.TransformListener()
        #else:
        #    self.tf_listener = tf_listener

        #self.pr2 = pr2_obj
        #self.cman = con.ControllerManager(arm, self.tf_listener, using_slip_controller=1)
        #self.reactive_gr = rgr.ReactiveGrasper(self.cman)
        #if arm == 'l':
        #    ptopic = '/pressure/l_gripper_motor'
        #    self.arm_obj = self.pr2.left
        #    self.ik_frame = 'l_wrist_roll_link'
        #    self.tool_frame = 'l_gripper_tool_frame'
        #else:
        #    ptopic = '/pressure/r_gripper_motor'
        #    self.arm_obj = self.pr2.right
        #    self.ik_frame = 'r_wrist_roll_link'
        #    self.tool_frame = 'r_gripper_tool_frame'
        #self.movement_mode = 'ik' #or cart

        #rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        #self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)

    #def set_movement_mode_ik(self):
    #    self.movement_mode = 'ik'
    #    self.reactive_gr.cm.switch_to_joint_mode()
    #    self.reactive_gr.cm.freeze_arm()

    #def set_movement_mode_cart(self):
    #    self.movement_mode = 'cart'







                #pdb.set_trace()
                #self.gather_interest_point_dataset(point)
                #point = np.matrix([ 0.60956734, -0.00714498,  1.22718197]).T
                #pressure_parameters = range(1900, 2050, 30)

                #self.record_perceptual_data(point)
                #successes = []
                #parameters = [np.matrix([-.15, 0, 0]).T, 300, np.matrix([-.005, 0, 0]).T, 3500, np.matrix([0,0,-.15]).T, .03]

                #for p in pressure_parameters:
                #    experiment = []
                #    for i in range(4):
                #        #Turn off lights
                #        rospy.loginfo('Experimenting with press_pressure = %d' % p)
                #        success_off = self.light_switch1(point, 
                #                        point_offset=np.matrix([-.15,0,0]).T, press_contact_pressure=300, move_back_distance=np.matrix([-.005,0,0]).T,\
                #                        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, visual_change_thres=.03)
                #        experiment.append(success_off)
                #        rospy.loginfo('Lights turned off? %s' % str(success_off))
                #        return

                #        #Turn on lights
                #        success_on = self.light_switch1(point, 
                #                        point_offset=np.matrix([-.15,0,-.10]).T, press_contact_pressure=300, move_back_distance=np.matrix([-0.005, 0, 0]).T,
                #                        press_pressure=3500, press_distance=np.matrix([0,0,.1]).T, visual_change_thres=.03)
                #        #def light_switch1(self, point, 
                #        #        point_offset, press_contact_pressure, move_back_distance,
                #        #        press_pressure, press_distance, visual_change_thres):

                #        print 'Lights turned on?', success_on
                #    successes.append(experiment)

                #ut.save_pickle({'pressure': pressure_parameters, 
                #                'successes': successes}, 'pressure_variation_results.pkl')










        #return self.pressure_listener.check_threshold() or self.pressure_listener.check_safety_threshold()
        ##stop if you hit a tip, side, back, or palm
        #(left_touching, right_touching, palm_touching) = self.reactive_gr.check_guarded_move_contacts()
        ##saw a contact, freeze the arm
        #if left_touching or right_touching or palm_touching:
        #    rospy.loginfo("CONTACT made!")
        #    return True
        #else:
        #    return False

        #print 'move returning'
        #return whether the left and right fingers were touching
        #return (left_touching, right_touching, palm_touching)




    #def execute_action_list(self):

    #def run(self, seed):
    #    # search for pairs of perception operators and manipulation operators that would work
    #    population = 10
    #    seeds = []
    #    for i in range(population):
    #        aseed = copy.deepcopy(seed)
    #        # 'bool', 'radian', 'se3', 'r3', 'discrete', 
    #        new_seed_actions = []
    #        for action in aseed:

    #            if replace_action:
    #                pass

    #            if delete_action:
    #                pass
    #            
    #            if insert_action:
    #                #pick random action from descriptors list
    #                new_action = 
    #                new_seed_actions += new_action
    #                pass
    #            
    #            if perturb_parameter:
    #                num_params = len(action.params)
    #                rand_param_idx = ...
    #                self.descriptors[action.name].params[rand_param_idx]
    #                rand_param_types[rand_param_types]


    #            #can replace/delete/insert action
    #            #can pick a parameter and perturb it

    #    #pdb.set_trace()
    #    print seed

        #point = np.matrix([0.63125642, -0.02918334, 1.2303758 ]).T
        #print 'move direction', movement.T
        #print 'CORRECTING', point.T
        #print 'NEW', point.T
        #start_location = (np.matrix([0.25, 0.15, 0.7]).T, np.matrix([0., 0., 0., 0.1]))
        #movement = np.matrix([.4, 0., 0.]).T
        #what other behavior would I want?
        # touch then move away..
        # move back but more slowly..
        # want a safe physical
        #   a safe exploration strategy
        #self.behaviors.linear_move(self.behaviors.current_location(), back_alittle, stop='none')
        #loc_before = self.behaviors.current_location()[0]
        #loc_after = self.behaviors.current_location()[0]
        #pdb.set_trace()
        #self.behaviors.linear_move(self.behaviors.current_location(), down, stop='pressure_accel')
        #self.behaviors.linear_move(self.behaviors.current_location(), back, stop='none')
        #pdb.set_trace()
        #b.twist(math.radians(30.))
        #bd = BehaviorDescriptor()
        #movement = point - self.behaviors.current_location()[0]
        #pdb.set_trace()
        #self.behaviors.linear_move(self.behaviors.current_location(), movement, stop='pressure_accel')

        #loc = self.behaviors.current_location()[0]
        #front_loc = point.copy()
        #front_loc[0,0] = loc[0,0]
        #self.behaviors.set_pressure_threshold(150)
        #self.behaviors.move_absolute((front_loc, self.behaviors.current_location()[1]), stop='pressure_accel')
        #self.behaviors.move_absolute((point, self.behaviors.current_location()[1]), stop='pressure_accel')




    #def detect_event(self):
    #    self.behaviors.cman._start_gripper_event_detector(timeout=40.)
    #    stop_func = self.behaviors._tactile_stop_func
    #    while stop_func():

        #pass
        #self.robot = pr2.PR2()
        #self.kin = pk.PR2Kinematics(self.robot.tf_listener)

    #def linear_move(self, start_location, direction, distance, arm):
    #    if arm == 'left':
    #        arm_kin = self.kin.left
    #    else:
    #        arm_kin = self.kin.right

    #    start_pose = arm_kin.ik(start_location)
    #    loc = start_location[0:3, 4]
    #    end_location = loc + distance*direction
    #    end_pose = arm_kin.ik(end_location)

    #    self.robot.left_arm.set_pose(start_pose, 5.)             #!!!
    #    self.robot.left_arm.set_pose(end_pose, 5.)               #!!!

            ##stop if you hit a tip, side, back, or palm
            #(left_touching, right_touching, palm_touching) = rg.check_guarded_move_contacts()
            ##saw a contact, freeze the arm
            #if left_touching or right_touching or palm_touching:
            #    rospy.loginfo("saw contact")
            #    rg.cm.switch_to_joint_mode()
            #    rg.cm.freeze_arm()
            #    break

    #import pdb
    #start_location = [0.34, 0.054, 0.87] + [0.015454981255042808, -0.02674860197736427, -0.012255429236635201, 0.999447577565171]
    #direction = np.matrix([1., 0., 0.]).T

    #self.reactive_l.move_cartesian_step(start_location, blocking = 1)
    #(left_touching, right_touching, palm_touching) = self.reactive_l.guarded_move_cartesian(grasp_pose, 10.0, 5.0)
        #self.cman_r     = con.ControllerManager('r')
        #self.reactive_r = rgr.ReactiveGrasper(self.cman_r)

        #self.cman_r.start_joint_controllers()
        #self.reactive_r.start_gripper_controller()
    
        #(pos, rot) = self.cman.return_cartesian_pose()
        #pdb.set_trace()
        #currentgoal = pos + rot
        #currentgoal[2] -= .05
        #self.reactive_l.move_cartesian_step(currentgoal, blocking = 1)
        #(left_touching, right_touching, palm_touching) = self.reactive_l.guarded_move_cartesian(grasp_pose, 10.0, 5.0)
        #exit()
        #end_loc = start_location + direction * distance
        #self.reactive_l.move_cartesian_step(start_loc, blocking = 1)
        #self.reactive_l.move_cartesian_step(end_loc, blocking = 1)
    #left_pose = b.robot.left.pose()
    #left_cart = ut.load_pickle('start_pose.pkl')
    #pdb.set_trace()
    #kin_sol = b.kin.left.ik(left_cart)
    #b.robot.left.set_pose(kin_sol, 5.)
    ##b.linear_move(left_cart)
    ##left_cart = b.kin.left.fk(left_pose)
    ##pdb.set_trace()
    #print left_cart

    #(pos, rot) = cm.return_cartesian_pose()
    #currentgoal = pos+rot
    #currentgoal[2] -= .05
    #rg.move_cartesian_step(currentgoal, blocking = 1)
    #exit()


#b.linear_move()
#cart_pose = kin.left.fk('torso_lift_link', 'l_wrist_roll_link', joints)
#kin.left.ik(cart_pose, 'torso_lift_link')

    #def light_switch1_on(self, point, press_pressure=3500, press_contact_pressure=150):
    #    point = point + np.matrix([-.15, 0, -0.20]).T

    #    success, reason = self.behaviors.reach(point)
    #    if not success:
    #        rospy.loginfo('Reach failed due to "%s"' % reason)

    #    rospy.loginfo('PRESSING')
    #    success, reason = self.behaviors.press(np.matrix([0, 0, .20]).T, \
    #            press_pressure, press_contact_pressure)
    #    if not success:
    #        rospy.loginfo('Press failed due to "%s"' % reason)
    #        return 

    #    rospy.loginfo('RESETING')
    #    r2 = self.behaviors.move_absolute(self.start_location, stop='pressure_accel')
    #    if r2 != None:
    #        rospy.loginfo('moving back to start location failed due to "%s"' % r2)
    #        return 

    #    print 'DONE.'


    #def _tactile_stop_func(self):
    #    r1 = self.pressure_listener.check_threshold() 
    #    r2 = self.pressure_listener.check_safety_threshold()
    #    if r1:
    #        rospy.loginfo('Pressure exceeded!')
    #    if r2:
    #        rospy.loginfo('Pressure safety limit EXCEEDED!')
    #    return r1 or r2







        #r1 = self.pressure_listener.check_threshold() 
        #r2 = self.pressure_listener.check_safety_threshold()
        #if r1:
        #    rospy.loginfo('Pressure exceeded!')
        #if r2:
        #    rospy.loginfo('Pressure safety limit EXCEEDED!')
        #pressure_state = r1 or r2
        #pressure_state = self.pressure_listener.check_threshold() or self.pressure_listener.check_safety_threshold()
        #action finished (trigger seen)




    #def optimize_parameters(self, x0, x_range, behavior, objective_func, reset_env_func, reset_param):
    #    reset_retries = 3
    #    num_params = len(x0)
    #    x = copy.deepcopy(x0)

    #    # for each parameter
    #    #for i in range(num_params):
    #    while i < num_params:
    #        #search for a good setting
    #        not_converged = True
    #        xmin = x_range[i, 0]
    #        xmax = x_range[i, 1]

    #        while not_converged:
    #            current_val = x[i]
    #            candidates_i = [(x[i] + xmin) / 2., (x[i] + xmax) / 2.]
    #            successes = []
    #            for cand in candidates_i:
    #                x[i] = cand
    #                success = behavior(x)
    #                if success:
    #                    for reset_i in range(reset_retries):
    #                        reset_success = reset_env_func(*reset_param)
    #                        if reset_success:
    #                            break
    #                successes.append(success)

    #            if successes[0] and successes[1]:
    #                raise RuntimeException('What? this isn\'t suppose to happen.')
    #            elif successes[0] and not successes[1]:
    #                next_val = candidates_i[0]
    #            elif successes[1] and not successes[0]:
    #                next_val = candidates_i[1]
    #            else:
    #                raise RuntimeException('What? this isn\'t suppose to happen.')


    #        #if all the trials are bad
    #        if not test(successes):
    #            #go back by 1 parameter
    #            i = i - 1


    #        #if there are more than one good parameter
    #        for p in params
    #            ... = objective_func(p)

    #        i = i + 1

    #    return x


