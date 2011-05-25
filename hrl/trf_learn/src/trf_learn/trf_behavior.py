#! /usr/bin/python
import roslib; roslib.load_manifest('trf_learn')
import rospy

import numpy as np
import math
import time
import scipy.spatial as sp
import pdb
import os
import os.path as pt
import shutil
import threading
import cProfile

from geometry_msgs.msg import PointStamped
import visualization_msgs.msg as vm
import tf.transformations as tr
import functools as ft
import tf
import cv

import hrl_camera.ros_camera as rc
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import hrl_lib.image3d as i3d
import hrl_lib.viz as viz
import hrl_pr2_lib.pr2 as pr2
import hrl_pr2_lib.devices as hd
import hrl_pr2_lib.linear_move as lm

import trf_learn.recognize_3d as r3d
import dynamic_reconfigure.client as dr
import laser_interface.laser_client as lc


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

        self.laser_listener = lc.LaserPointerClient(tf_listener=self.tf_listener)
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
        #mode = 'practice'
        mode = 'location_execute'

        if point_bl!= None:
            print 'Got point', point_bl.T
            #1) User points at location and selects a behavior.
            if mode == 'location_execute':
                self.scenario_user_clicked_at_location(point_bl)

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
                _, pos_exp, neg_exp = r3d.separate_by_labels(np.column_stack(points2d_tried), np.matrix(labels))
                r3d.draw_points(img, pos_exp, [50, 255, 0], 8, 1)
                r3d.draw_points(img, neg_exp, [50, 0, 255], 8, 1)

            predictions = np.matrix(self.locations_man.learners[task_id].classify(kdict['instances']))
            _, pos_pred, neg_pred = r3d.separate_by_labels(kdict['points2d'], predictions)
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
            _, pos_exp, neg_exp = r3d.separate_by_labels(np.column_stack(points2d_tried), np.matrix(labels))
            r3d.draw_points(img, pos_exp, [50, 255, 0], 9, 1)
            r3d.draw_points(img, neg_exp, [50, 0, 255], 9, 1)

            _, pos_pred, neg_pred = r3d.separate_by_labels(kdict['points2d'], predictions)
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
    # TODO: WE MIGHT NOT NEED THIS AFTER ALL, MIGHT BE ABLE TO JUST INITIALIZE RANDOMLY!
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
            _, pos_points, neg_points = r3d.separate_by_labels(np.column_stack(points2d_tried), np.matrix(labels))
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
                _, pos_pred, neg_pred = r3d.separate_by_labels(kdict['points2d'], predictions)
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




