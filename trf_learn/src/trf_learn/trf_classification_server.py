#!/usr/bin/python
import roslib; roslib.load_manifest('trf_learn')
import rospy
import numpy as np
import trf_learn.recognize_3d as r3d
import pdb
import hrl_lib.util as ut
import pickle
import interactive_markers.interactive_marker_server as ims
import cv
import hrl_lib.interactive_marker_helper as imh
import functools as ft
import rcommander_ar_tour.msg as atmsg
import rcommander_ar_tour.srv as atsrv
import scipy.spatial as sp
import move_base_msgs.msg as mm
import rcommander_pr2_gui.msg as rm
import geometry_msgs.msg as geo
import sensor_msgs.msg as sm
import actionlib
import tf
import pointclouds as pc
import pypr2.msg as smb

import hrl_camera.ros_camera as rc
import hrl_pr2_lib.devices as hd
import hrl_lib.tf_utils as tfu
import pypr2.tf_utils as ptfu

import shutil
import time
import copy
import interactive_markers.menu_handler as mh

import os.path as pt
import os


def select_instance_ordered_by_distance(fea_dict, point_bl, ordered_idx=0):
    dists = ut.norm(fea_dict['points3d'] - point_bl)
    ordering = np.argsort(dists).A1
    points3d_sampled = fea_dict['points3d'][:, ordering]
    points2d_sampled = fea_dict['points2d'][:, ordering]
    instances_sampled = fea_dict['instances'][:, ordering]
    return points2d_sampled[:, 0], points3d_sampled[:, 0], instances_sampled[:, 0]

def pose_to_tup(p):
    return [p.position.x, p.position.y, p.position.z], \
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]

def tup_to_pose(t):
    p = geo.Pose()
    p.position.x = t[0][0]
    p.position.y = t[0][1]
    p.position.z = t[0][2]
    p.orientation.x = t[1][0]
    p.orientation.y = t[1][1]
    p.orientation.z = t[1][2]
    p.orientation.w = t[1][3]
    return p
        

class ActiveLearnPointContainer:

    def __init__(self, instances, points2d, points3d, point_frame, sizes):
        self.instances = instances
        self.points2d = points2d
        self.points3d = points3d
        self.point_frame = point_frame
        self.sizes = sizes


class TrainingInformationDatabase:

    NUM_BASE_LOCATIONS = 4

    def __init__(self, name, rec_params):
        self.rec_params = rec_params
        self.saved_locations_fname = name
    
        self.data = {} #This gets saved to disk
        self.learners = {}
        #self.active_learn_sessions = {}
        self.pca_dataset = {}
        self.load_database()

    def load_database(self):
        try:
            rospy.loginfo('Loading pickle %s' % self.saved_locations_fname)
            d = ut.load_pickle(self.saved_locations_fname)
            if d != None:
                self.data = d
        except Exception, e:
            rospy.loginfo('Failed to load.')
            rospy.loginfo('%s %s' % (str(e), str(e.__class__)))

        for actionid in self.data.keys():
            rospy.loginfo('==========================================')
            rospy.loginfo('Loading %s' % actionid)
            rospy.loginfo('==========================================')
            self.train(actionid)

    def save_database(self):
        rospy.loginfo('Saving pickle. DO NOT INTERRUPT!')
        try:
            shutil.copyfile(self.saved_locations_fname, 
                    time.strftime('%m_%d_%Y_%I_%M%p') + '_locations.pkl')
        except Exception, e:
            rospy.loginfo('%s %s' % (str(e), str(e.__class__)))
        ut.save_pickle(self.data, self.saved_locations_fname)
        rospy.loginfo('saved to %s.' % self.saved_locations_fname)

    def get_learner(self, actionid):
        if self.learners.has_key(actionid):
            return self.learners[actionid]

        return None

    def init_data_record(self, actionid):
        if not self.data.has_key(actionid):
            self.data[actionid] = {'dataset': None,
                                   'pca': None,
                                   #'pca_dataset': None,
                                   'practice_locations': None,
                                   'practice_locations_history': None,
                                   'practice_locations_convergence': None,
                                   'execution_record': []}

    #def get_active_learn_session(self, actionid):
    #    return self.active_learn_sessions[actionid]

    #def set_active_learn_session(self, actionid, session):
    #    self.active_learn_sessions[actionid] = session

    #def set_practicing(self, actionid, value):
    #    self.data[actionid]['is_practicing'] = value

    def has_training_locations(self, actionid):
        return not self.data[actionid]['practice_locations'] == None

    def update_training_location(self, actionid, location_idx, value):
        if self.data[actionid]['practice_locations'] == None:
            n = TrainingInformationDatabase.NUM_BASE_LOCATIONS
            self.data[actionid]['practice_locations'] = [None]*n
            self.data[actionid]['practice_locations_history'] = np.zeros((1, n))
            self.data[actionid]['practice_locations_convergence'] = np.zeros((1, n))            

        self.data[actionid]['practice_locations'][location_idx] = value

    def get_training_location(self, actionid, location_idx):
        return self.data[actionid]['practice_locations'][location_idx]

    def get_practice_history(self, actionid):
        return self.data[actionid]['practice_locations_history']

    def add_to_practice_history(self, actionid, idx, numb):
        if not self.data[actionid].has_key('practice_locations_history'):
            self.data[actionid]['practice_locations_history'] = []
        self.data[actionid]['practice_locations_history'][0,idx] += numb

    def get_practice_convergence(self, actionid):
        return self.data[actionid]['practice_locations_convergence']

    def get_dataset(self, actionid):
        return self.data[actionid]['dataset']

    def get_num_data_points(self, actionid):
        if self.data[actionid]['dataset'] == None:
            return 0
        return self.data[actionid]['dataset'].inputs.shape[1]

    def add_pca_dataset(self, actionid, pca_dataset):
        self.pca_dataset[actionid] = pca_dataset
        #self.data[actionid]['pca_dataset'] = pca_dataset

    def get_pca_dataset(self, actionid):
        if self.pca_dataset.has_key(actionid):
            return self.pca_dataset[actionid]
        else:
            return None
        #return self.data[actionid]['pca_dataset']

    def set_converged(self, actionid, location_idx):
        self.data[actionid]['practice_locations_convergence'][0, location_idx] = 1

    def add_data_instance(self, actionid, al_point_container, success):
        #Make sure we have a data record
        self.init_data_record(actionid)

        #Translate true/false to POS/NEG
        if success:
            label = np.matrix([r3d.POSITIVE])
        else:
            label = np.matrix([r3d.NEGATIVE])

        current_dataset = self.get_dataset(actionid)

        #Add to dataset
        self.data[actionid]['dataset'] = \
                r3d.InterestPointDataset.add_to_dataset(
                        current_dataset, al_point_container.instances, #['instances'], 
                        label, al_point_container.points2d, #datapoint_dict['points2d'], 
                        al_point_container.points3d, #datapoint_dict['points3d'], 
                        None, None, sizes=al_point_container.sizes) #datapoint_dict['sizes'])

        #If have no learner, try to train one
        #if self.get_learner(actionid) == None:
        #    self.train(actionid)


    #def train(self, actionid, dset_for_pca=None, save_pca_images=True):
    def train(self, actionid, save_pca_images=True):
        dataset = self.data[actionid]['dataset']
        rec_params = self.rec_params
        if dataset == None:
            return

        #Balance pos/neg
        rospy.loginfo('TrainingInformationDatabase.train: training for %s' % actionid)
        nneg = np.sum(dataset.outputs == r3d.NEGATIVE) 
        npos = np.sum(dataset.outputs == r3d.POSITIVE)
        rospy.loginfo( '================= Training =================')
        rospy.loginfo('NEG examples %d' % nneg)
        rospy.loginfo('POS examples %d' % npos)
        rospy.loginfo('TOTAL %d' % dataset.outputs.shape[1])
        #pdb.set_trace()

        if nneg == 0 or npos == 0:
            rospy.loginfo('Not training as we don\'t have at least one positive and one negative point')
            return

        neg_to_pos_ratio = float(nneg)/float(npos)
        weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio

        previous_learner = None
        if self.learners.has_key(actionid):
            previous_learner = self.learners[actionid]
        learner = r3d.SVMPCA_ActiveLearner(use_pca=True, 
                        reconstruction_std_lim=self.rec_params.reconstruction_std_lim, 
                        reconstruction_err_toler=self.rec_params.reconstruction_err_toler,
                        #Get saved PCA vector from saved data array here
                        old_learner=previous_learner, pca=self.data[actionid]['pca'])

        #TODO: figure out something for scaling inputs field!
        dset_for_pca = self.get_pca_dataset(actionid)
        if dset_for_pca != None:
            inputs_for_pca = dset_for_pca['instances']
        else:
            inputs_for_pca = dataset.inputs

        learner.train(dataset, 
                      inputs_for_pca,
                      rec_params.svm_params + weight_balance,
                      rec_params.variance_keep)

        #Store calculated PCA vector, which gets saved when 'data' is saved
        self.data[actionid]['pca'] = learner.pca 
        self.learners[actionid] = learner
        if save_pca_images:
            #pdb.set_trace()
            basis = learner.pca.projection_basis
            cv.SaveImage('%s_pca.png' % actionid, 
                    r3d.instances_to_image(self.rec_params.win_size, 
                        basis, np.min(basis), np.max(basis)))

class TRFInteractiveMarkerServer:


    def __init__(self, training_db, classification_server):
        self.classification_server = classification_server
        self.marker_server = ims.InteractiveMarkerServer('trf_interactive_markers')
        self.broadcaster = tf.TransformBroadcaster()
        self.training_db = training_db
        self.markers = {}

    def show_base_locations(self, actionid):
        self.markers[actionid] = []
        self.training_db.init_data_record(actionid)
        initialized = self.training_db.has_training_locations(actionid)
        for i in range(TrainingInformationDatabase.NUM_BASE_LOCATIONS):
            if not initialized:
                pose, frame = [[0,0,0.], [0,0,0,1.]], '/map'
                self.training_db.update_training_location(actionid, i, pose)
            else:
                pose, frame = self.training_db.get_training_location(actionid, i)

            cb = ft.partial(self.accept_cb, actionid)
            m = RobotPositionMarker(actionid, i, pose, frame, 
                    self.marker_server, self.broadcaster, self.training_db, cb)
            self.markers[actionid].append(m)
        self.marker_server.applyChanges()

    def accept_cb(self, actionid, feedback):
        rospy.loginfo('accept_cb on %s' % actionid)
        self.training_db.save_database()
        self.classification_server.train_action(actionid)
        self.hide_base_locations(actionid)

    def hide_base_locations(self, actionid):
        for m in self.markers[actionid]:
            m.remove()
        self.marker_server.applyChanges()
        self.markers.pop(actionid)


class RobotPositionMarker:

    def __init__(self, actionid, marker_number, pose, frame, 
            marker_server, broadcaster, training_db, accept_cb, scale=.2):
        self.broadcaster = broadcaster
        self.marker_name = actionid + '_' + str(marker_number)

        int_marker = imh.interactive_marker(self.marker_name, pose, scale)
        int_marker.header.frame_id = frame
        int_marker.scale = scale
        int_marker.description = actionid + '_' + str(marker_number)

        #Make controls
        sph = imh.make_sphere_control(self.marker_name, scale)
        int_marker.controls += [sph]
        int_marker.controls += imh.make_directional_controls(self.marker_name, y=False)
        int_marker.controls += imh.make_orientation_controls(self.marker_name, x=False, y=True, z=False)

        #Make menu control
        menu_control = ims.InteractiveMarkerControl()
        menu_control.interaction_mode = ims.InteractiveMarkerControl.MENU
        menu_control.name = 'menu_' + actionid + '_' + str(marker_number)
        menu_control.markers.append(copy.deepcopy(int_marker.controls[0].markers[0]))
        menu_control.always_visible = True
        int_marker.controls.append(copy.deepcopy(menu_control))

        #make menu handler
        menu_handler = mh.MenuHandler()
        menu_handler.insert('accept', parent=None, callback=accept_cb)

        #add to server
        marker_server.insert(int_marker, self.marker_cb)
        menu_handler.apply(marker_server, int_marker.name)

        self.actionid = actionid
        self.marker_server = marker_server
        self.training_db = training_db
        self.marker_obj = int_marker
        self.menu_handler = menu_handler
        self.marker_number = marker_number
        self.frame = frame

    def marker_cb(self, feedback):
        #print 'robot marker feedback:', imh.feedback_to_string(feedback.event_type)
        if feedback.event_type == ims.InteractiveMarkerFeedback.POSE_UPDATE:
            p_ar = pose_to_tup(feedback.pose)
            #print 'updating training locations'
            self.training_db.update_training_location(self.actionid, self.marker_number, 
                    [p_ar, self.frame])

        if feedback.event_type == ims.InteractiveMarkerFeedback.MOUSE_DOWN:
            p = feedback.pose
            pos = [p.position.x, p.position.y, p.position.z]
            ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            self.broadcaster.sendTransform(pos, ori, rospy.Time.now(), 
                    'training_location', feedback.header.frame_id)

    def remove(self):
        self.marker_server.erase(self.marker_name)



##
# Create a session per scan
class TRFActiveLearnSession:

    TOO_FAR_AWAY = .5
    RESOLUTION = .01

    def __init__(self, classification_server, actionid, point_bl):
        self.cserver = classification_server
        self.actionid = actionid

        self.kdict = classification_server.feature_ex.read(point_bl, params=classification_server.rec_params)
        self.point3d_bl = point_bl

        self.indices_added = []
        self.points3d_tried = []

    def get_features_read(self):
        return self.kdict

    def get_response(self):
        remaining_pt_indices = r3d.inverse_indices(self.indices_added, 
                self.kdict['instances'].shape[1])
        remaining_instances = self.kdict['instances'][:, remaining_pt_indices]

        learner = self.cserver.training_db.get_learner(self.actionid)
        ridx, selected_dist, converged = learner.select_next_instances_no_terminate(remaining_instances)
        selected_idx = remaining_pt_indices[ridx[0]]
        self.indices_added.append(selected_idx)

        if np.linalg.norm(self.kdict['points3d'][:, selected_idx] - self.point3d_bl) > TRFActiveLearnSession.TOO_FAR_AWAY:
            rospy.loginfo('#########################################')
            rospy.loginfo('Point outside of negative cut off!! Eliminating %s' % (str(self.kdict['points3d'][:, selected_idx].T)))
            rospy.loginfo('#########################################')
            success = False

        elif len(self.points3d_tried) > 0:
            existing_pts_tree = sp.KDTree(np.array(np.column_stack(self.points3d_tried).T))
            close_by_indices = existing_pts_tree.query_ball_point(np.array(self.kdict['points3d'][:, selected_idx]).T, 
                    TRFActiveLearnSession.RESOLUTION)[0]

            if len(close_by_indices) > 0:
                rospy.loginfo('#########################################')
                rospy.loginfo('Point within resolution of existing point.') #Labeling %s' % (str(kdict['points3d'][:, selected_idx])))
                rospy.loginfo('#########################################')
                #restart
                return self.get_response()
            else:
                return self._get_datapoint(selected_idx) 
        else:
            return self._get_datapoint(selected_idx)

    def _get_datapoint(self, selected_idx):
        return ActiveLearnPointContainer(self.kdict['instances'][:, selected_idx], 
                self.kdict['points2d'][:, selected_idx], 
                self.kdict['points3d'][:, selected_idx], 
                '/base_link', self.kdict['sizes'])


class TRFClassificationServer:

    def __init__(self, database_file):
        rospy.init_node('trf_classification_server')
        self.visualize_results = True
        self.optical_frame = 'high_def_optical_frame'

        self.get_behavior_property = rospy.ServiceProxy('get_behavior_property', atsrv.ActionProperty)
        self.get_behavior_pose     = rospy.ServiceProxy('get_behavior_pose', atsrv.GetBehaviorPose)
        self.set_behavior_pose     = rospy.ServiceProxy('set_behavior_pose', atsrv.SetBehaviorPose)

        #Handles request to recognize the pose of an object.
        rospy.Service('recognize_pose', atsrv.RecognizePose, self.recognize_pose_srv_cb)

        #Messages that tells this node about the outcome of actions.
        rospy.Service('action_result', atsrv.ActionResult, self.action_result_srv_cb)
        self.last_action_result = None

        #Messages that request an action be trained
        rospy.Subscriber('train_action', atmsg.TrainAction, self.train_action_cb)

        self.run_action_id_client = actionlib.SimpleActionClient('run_actionid_train_mode', atmsg.RunScriptIDAction)

        self.rec_params = r3d.Recognize3DParam()
        self.training_db = TrainingInformationDatabase(database_file, self.rec_params)
        self.marker_server = TRFInteractiveMarkerServer(self.training_db, self)
        self.tf_listener = tf.TransformListener()

        #TODO: change this to kinect
        self.prosilica = rc.Prosilica('prosilica', 'polled')
        self.prosilica_cal = rc.ROSCameraCalibration('/prosilica/camera_info')

        self.feature_ex = r3d.KinectTextureFeatureExtractor('/head_mount_kinect', 
                self.prosilica, self.prosilica_cal, self.tf_listener, self.rec_params)

        #self.robot_base = something
        self.move_base_client = actionlib.SimpleActionClient('move_base', 
                mm.MoveBaseAction)

        self.go_xy_client = actionlib.SimpleActionClient('go_xy',
                smb.GoXYAction)

        self.tuck_arm_client = actionlib.SimpleActionClient('rcommander_tuckarms', 
                rm.RCTuckArmsAction)

        self.optical_frame = 'high_def_optical_frame'

        self.candidate_pub = rospy.Publisher('candidate_points', sm.PointCloud2)

        rospy.loginfo('Ready!')



    def action_result_srv_cb(self, action_result_msg):
        actionid      = action_result_msg.actionid
        result        = action_result_msg.result
        mode          = action_result_msg.mode
        instance_prop = pickle.loads(action_result_msg.info)

        rospy.loginfo('action_result_srv_cb: called actionid %s result %s mode %s' % (actionid, result, mode))
        rospy.loginfo('***********************************************************')
        rospy.loginfo('***********************************************************')
        rospy.loginfo('training mode: Adding data point with label %s to action %s' 
                % (str(result), actionid))
        rospy.loginfo('***********************************************************')
        rospy.loginfo('***********************************************************')

        #Not initialized yet, so we add instance
        if self.training_db.get_learner(actionid) == None or mode == 'train':
            if self.training_db.get_learner(actionid) == None:
                self.initialize_action(actionid, [instance_prop, result])
            else:
                self.training_db.add_data_instance(actionid, instance_prop, result)
                self.training_db.save_database()
                #als = self.training_db.get_active_learn_session()
                self.training_db.train(actionid, True) #als.get_features_read())
            #if mode == 'train':
            #else:

        self.last_action_result = result
        return atsrv.ActionResultResponse()

    ##
    #
    def initialize_action(self, actionid, first_run_info):
        rospy.loginfo('Initializing action %s with a classifier' % actionid)
        active_learn_point_container, success = first_run_info
        self.training_db.add_data_instance(actionid, active_learn_point_container, success)

        fea_dict = self.training_db.get_pca_dataset(actionid)
        posestamped = self.get_behavior_pose(actionid).posestamped
        frame = posestamped.header.frame_id
        point = posestamped.pose.position
        point_mat = np.matrix([point.x, point.y, point.z]).T
        point_bl  = tfu.transform_points(
                        tfu.transform('base_link', frame, self.tf_listener), 
                        point_mat)

        if success:
            #add a failing point
            point2d, point3d, instance = select_instance_ordered_by_distance(fea_dict, point_bl, -1)
            neg_point = ActiveLearnPointContainer(instance, point2d, point3d, frame, fea_dict['sizes'])
            self.training_db.add_data_instance(actionid, neg_point, False)
        else:
            #add guess of a 'success' point as point closest to point stored in map
            point2d, point3d, instance = select_instance_ordered_by_distance(fea_dict, point_bl, 1)
            pos_point = ActiveLearnPointContainer(instance, point2d, point3d, frame, fea_dict['sizes'])
            self.training_db.add_data_instance(actionid, pos_point, True)

        self.training_db.train(actionid)
        self.training_db.save_database()

    def visualize_recognition_results(self, actionid, feature_dict, learner, draw_dict, postfix=''):
        if not self.visualize_results:
            return

        predictions = learner.classify(feature_dict['instances'])
        img = cv.CloneMat(feature_dict['image'])
        
        #Draw 'shadows' 
        r3d.draw_points(img, feature_dict['points2d']+np.matrix([1,1.]).T, [255, 255, 255], 4, -1)
        
        #Draw points tried
        _, pos_pred, neg_pred = r3d.separate_by_labels(feature_dict['points2d'], np.matrix(predictions))
        r3d.draw_points(img, pos_pred, [255, 204, 51], 3, -1)
        r3d.draw_points(img, neg_pred, [51, 204, 255], 3, -1)
        save_dict = {'pos_pred': pos_pred,
                     'neg_pred': neg_pred}
        
        #Draw point selected 
        if draw_dict.has_key('selected'):
            color = [0,0,255]
            #pdb.set_trace()
            r3d.draw_points(img, draw_dict['selected'] , color, 8, -1)
            save_dict['tried'] = draw_dict['selected']

        #Draw the center
        if draw_dict.has_key('center'):
            point3d_img = tfu.transform_points(tfu.transform(self.optical_frame, 'base_link', self.tf_listener), 
                        draw_dict['center'])
            point2d_img = self.feature_ex.cal.project(point3d_img)
            r3d.draw_points(img, point2d_img, [255, 0, 0], 6, 2)
            save_dict['center'] = point2d_img

        try:
            os.mkdir(actionid)
        except OSError, e:
            pass

        #Save visualization
        ffull = pt.join(actionid, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + postfix + '_vis.jpg')
        cv.SaveImage(ffull, img)

        #Save raw image
        raw_image_name = pt.join(actionid, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + postfix + '_raw.jpg')
        cv.SaveImage(raw_image_name, feature_dict['image'])
        save_dict['image'] = raw_image_name

        #Save pickle
        pkname = pt.join(actionid, time.strftime('%A_%m_%d_%Y_%I_%M_%S%p') + postfix + '.pkl')
        ut.save_pickle(save_dict, pkname)




    ##
    # Assume that the robot is looking at what it needs to recognize
    # @param RecognizePoseRequest has field actionid (string), and last_known_pose (PoseStamped)
    # @return RecognizePoseResponse with a PoseStamped
    def recognize_pose_srv_cb(self, recognize_pose_request):
        actionid  = recognize_pose_request.actionid
        mode      = recognize_pose_request.mode
        #frame     = recognize_pose_request.last_known_pose.header.frame_id #should be /map
        #point     = recognize_pose_request.last_known_pose.pose.position
        last_known_pose = self.get_behavior_pose(actionid).posestamped
        frame = last_known_pose.header.frame_id
        point = last_known_pose.pose.position


        point_mat = np.matrix([point.x, point.y, point.z]).T
        point_bl  = tfu.transform_points(
                        tfu.transform('base_link', frame, self.tf_listener), 
                        point_mat)

        rospy.loginfo('recognize_pose_srv_cb: actionid %s mode %s' % (str(actionid), mode))
        rospy.loginfo('recognize_pose_srv_cb: LAST KNOWN POSE %s' % (str(point_bl.T)))

        #No learner!
        if self.training_db.get_learner(actionid) == None:
            rospy.loginfo('Has no learner. Just using closest point to prior.')
            # There is no associated classifier, we use the seed as our
            # positive point, construct the classifier and return.
            params = r3d.Recognize3DParam()
            params.n_samples = 5000
    
            #Capture a scan 
            fea_dict = self.feature_ex.read(point_bl, params=params)
            point2d, point3d, instance = select_instance_ordered_by_distance(fea_dict, point_bl, 0)
            datapoint = ActiveLearnPointContainer(instance, point2d, point3d, frame, fea_dict['sizes'])
                    
            #Add this scan as a PCA dataset
            self.training_db.init_data_record(actionid)
            self.training_db.add_pca_dataset(actionid, fea_dict)
            self.training_db.save_database()

            #return atsrv.RecognizePoseResponse(recognize_pose_request.last_known_pose,
            return atsrv.RecognizePoseResponse(last_known_pose, pickle.dumps(datapoint))

        elif mode == 'train':
            #elif self.training_db.is_practicing(actionid):
            #active_learn = self.training_db.get_active_learn_session(actionid)
            #if active_learn == None:
            active_learn = TRFActiveLearnSession(self, actionid, point_bl)
            #self.training_db.set_active_learn_session(actionid, active_learn)

            al_point_container = active_learn.get_response()

            points3d_bl = active_learn.kdict['points3d']
            pc2 = pc.xyz_array_to_pointcloud2(np.array(points3d_bl.T), frame_id='base_link')
            self.candidate_pub.publish(pc2)
            #pdb.set_trace()

            #Convert from container frame to /map frame
            p_map = tfu.transform_points(tfu.transform('map', al_point_container.point_frame, 
                        self.tf_listener), al_point_container.points3d[:,0])

            self.visualize_recognition_results(actionid, active_learn.kdict, 
                    self.training_db.get_learner(actionid),
                    draw_dict={'selected': al_point_container.points2d[:,0],
                               'center': point_bl}, postfix='_train')
                
            ps = geo.PoseStamped()
            ps.header.frame_id = '/map'
            ps.pose = last_known_pose.pose #sets position/orientation
            #p = al_point_container.points3d[:,0]
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p_map.A1.tolist()

            rospy.loginfo('recognize_pose_srv_cb: TRYING POINT %s' % str(al_point_container.points3d[:,0].T))

            #Don't need to save training database here as we should get an action_result_srv_cb in the case of training.
            return atsrv.RecognizePoseResponse(ps, pickle.dumps(al_point_container))

        #Mode is 'execute' or something else that we'll just assume is execute
        elif mode == 'execute':
            #should have learner so we'll just do classification
            kdict = self.feature_ex.read(point_bl)
            predictions = np.matrix(self.training_db.get_learner(actionid).classify(kdict['instances']))
            #predictions = np.matrix(self.locations_man.learners[actionid].classify(kdict['instances']))
            pos_indices = np.where(r3d.POSITIVE == predictions)[1].A1

            locs2d = None
            if len(pos_indices) > 1:
                locs2d = kdict['points2d'][:, pos_indices]
                if np.any(np.isnan(locs2d)) or np.any(np.isinf(locs2d)):
                    pdb.set_trace()
                locs2d_indices = np.where(False == np.sum(np.isnan(locs2d), 0))[1].A1
                loc2d_max, density_image = r3d.find_max_in_density(locs2d[:, locs2d_indices])
                dists = ut.norm(kdict['points2d'] - loc2d_max)
                selected_idx = np.argmin(dists)
                selected_3d = kdict['points3d'][:, selected_idx]
                selected_2d = kdict['points2d'][:, selected_idx]
                #selected_instance = kdict['instances'][:, sampled_idx]
                selected_instance = kdict['instances'][:, selected_idx]

            else:
                rospy.loginfo('FOUND NO POSITIVE POINTS. JUST USING CLOSEST POINT TO PRIOR.')
                #selected_2d, selected_3d, selected_instance = select_closest_instance(kdict, point_bl)
                selected_2d, selected_3d, selected_instance = select_instance_ordered_by_distance(kdict, point_bl, ordered_idx=0)

            self.visualize_recognition_results(actionid, kdict, 
                    self.training_db.get_learner(actionid),
                    draw_dict={'selected': selected_2d,
                               'center': point_bl}, postfix='_execute')

            rospy.loginfo('recognize_pose_srv_cb: TRYING POINT %s' % str(selected_3d))
            selected_3d_f = tfu.transform_points(
                                tfu.transform(last_known_pose.header.frame_id,
                                    'base_link', self.tf_listener), selected_3d)
            
            ps = geo.PoseStamped()
            ps.header.frame_id = last_known_pose.header.frame_id
            ps.pose = last_known_pose.pose #sets position/orientation
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = selected_3d_f.A1.tolist()
            datapoint = ActiveLearnPointContainer(selected_instance, selected_2d,
                                                    selected_3d_f, ps.header.frame_id,
                                                    kdict['sizes'])

            return atsrv.RecognizePoseResponse(ps, pickle.dumps(datapoint))
        else:
            raise Exception('recognize_pose_srv_cb: Invalid mode! %s' % (mode))
    

    def train_action_cb(self, msg):
        #do this only if the database doesn't have training locations
        #create interactive markers, 4, wait for user to click on confirm on one of them
        rospy.loginfo('train_action_cb: Got a a request for training. Showing base locations.')
        self.marker_server.show_base_locations(msg.actionid)

    def _tuck(self, left, right):
        goal = rm.RCTuckArmsGoal()
        goal.tuck_left = left
        goal.tuck_right = right 
        self.tuck_arm_client.send_goal(goal)
        rospy.loginfo('waiting for tuck arm results')
        self.tuck_arm_client.wait_for_result()

    def move_to_location(self, position, quat, frame):
        rospy.loginfo('move_to_location: **************************')
        rospy.loginfo('move_to_location: %s' % (str(position) + ' ' + str(quat)))
        rospy.loginfo('move_to_location: **************************')
        raw_input('Moving base. Press enter when ready!')
        rospy.loginfo('move_to_location: tucking')
        self._tuck(True, True)

        global_nav = False

        if global_nav:
            g = mm.MoveBaseGoal()
            p = g.target_pose
            p.header.frame_id = frame
            p.header.stamp = rospy.get_rostime()
            p.pose = tup_to_pose((position, quat))
            self.move_base_client.send_goal(g)
            rospy.loginfo('move_to_location: waiting for move base results')
            self.move_base_client.wait_for_result()
            rospy.loginfo('move_to_location: moved!')
        else:
            self.tf_listener.waitForTransform('/base_footprint',\
                    frame,  rospy.Time(0), rospy.Duration(10.))
            bl_T_frame = ptfu.tf_as_matrix(\
                    self.tf_listener.lookupTransform(\
                    '/base_footprint',\
                    frame, rospy.Time(0)))
            h_frame = ptfu.tf_as_matrix((position, quat))
            t, r = ptfu.matrix_as_tf(bl_T_frame*h_frame)
            xy_goal = smb.GoXYGoal(t[0],t[1])
            self.go_xy_client.send_goal(xy_goal)
            rospy.loginfo('move_to_location: waiting for go_xy results')
            self.go_xy_client.wait_for_result()
            rospy.loginfo('move_to_location: moved!')


    def _train_helper(self, actionid, cactionid, stop_fun=None):
        rospy.loginfo('*************************************')
        rospy.loginfo('train_helper: training action %s' % actionid)
        rospy.loginfo('*************************************')
        labels = []

        start_num_points = self.training_db.get_num_data_points(actionid)
        while not rospy.is_shutdown():
            #Run action
            rospy.loginfo('*************************************')
            rospy.loginfo('*************************************')
            rospy.loginfo('_train_helper: sent goal for action %s' % actionid)
            rospy.loginfo('*************************************')
            rospy.loginfo('*************************************')
            self.run_action_id_client.send_goal(atmsg.RunScriptIDGoal(actionid, ''))
            rospy.loginfo('_train_helper: waiting for result')
            self.run_action_id_client.wait_for_result()
            state_machine_end_state = self.run_action_id_client.get_result().result

            #Get success/failure from our service callback
            success = self.last_action_result
            self.last_action_result = None

            if success:
                rospy.loginfo('_train_helper: SUCCEEDED!')
                label = r3d.POSITIVE
            else:
                rospy.loginfo('_train_helper: Failed!')
                label = r3d.NEGATIVE
            labels.append(label)

            #if stop function tells us to stop
            if stop_fun != None and stop_fun(np.matrix(labels)):
                rospy.loginfo('Stop satisfied told us to stop loop!')
                break
            
            #run the reverse if we succeed 
            if success:
                def any_pos_sf(labels_mat):
                    if np.any(r3d.POSITIVE == labels_mat):
                        return True
                    else:
                        return False
                #reverse our action.
                self._train_helper(cactionid, actionid, stop_fun=any_pos_sf)

            #if we have enough data, quit.
            num_points_added = self.training_db.get_num_data_points(actionid) - start_num_points
            #This loop will run forever if we fail to get callbacks for actions
            if stop_fun == None and num_points_added > self.rec_params.max_points_per_site:
                rospy.loginfo('practice: added enough points from this scan. Limit is %d points.' \
                        % self.rec_params.max_points_per_site)
                break

        self.training_db.save_database()


    def train_action(self, actionid):
        rospy.loginfo('train_action: ===================================================')
        rospy.loginfo('train_action: ===================================================')
        rospy.loginfo('train_action: training %s' % actionid)
        rospy.loginfo('train_action: ===================================================')
        rospy.loginfo('train_action: ===================================================')

        #pdb.set_trace()
        self.training_db.init_data_record(actionid)
        cactionid = self.get_behavior_property(actionid, 'complement').value
        self.training_db.init_data_record(cactionid)

        unexplored_locs  = np.where(self.training_db.get_practice_history(actionid) == 0)[1]
        unconverged_locs = np.where(self.training_db.get_practice_convergence(actionid) == 0)[1]
        rospy.loginfo("Location history: %s" % str(self.training_db.get_practice_history(actionid)))

        pidx = 0
        if unexplored_locs.shape[0] > 0:
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
            self.training_db.get_practice_convergence(actionid)[0, pidx] = 0

        # Loop
        while not rospy.is_shutdown(): #%not converged:
            #If this is not a fresh run we continue with a location we've never been to before
            #if haven't converged
            if self.training_db.get_practice_convergence(actionid)[0, pidx] == 0:
                #self.training_db.set_active_learn_session(actionid, None)
                #self.training_db.set_active_learn_session(cactionid, None)
                ptup, frame = self.training_db.get_training_location(actionid, pidx)
                self.move_to_location(ptup[0], ptup[1], frame)
                #def move_to_location(self, position, quat, frame):

                ####################################################################
                ####################################################################
                action_b = self.training_db.get_num_data_points(actionid)
                self._train_helper(actionid, cactionid)
                points_added = self.training_db.get_num_data_points(actionid) - action_b
                ####################################################################
                ####################################################################

                self.training_db.add_to_practice_history(actionid, pidx, 1)
                if points_added == 0:# and np.where(self.training_db.data[actionid]['practice_locations_history'] == 0)[1].shape[0] == 0:
                    self.training_db.set_converged(actionid, pidx)
                    rospy.loginfo('===================================================')
                    rospy.loginfo('= LOCATION CONVERGED ')
                    rospy.loginfo('Converged locs: %s' % str(self.training_db.get_practice_convergence(actionid)))
                    rospy.loginfo('number of datapoints %s' % str(self.training_db.get_num_data_points(actionid)))
                    rospy.loginfo('===================================================')
                    if np.where(self.training_db.get_practice_convergence(actionid) == 0)[1].shape[0] <= 0:
                        break
                else:
                    rospy.loginfo('===================================================')
                    rospy.loginfo('= Scan converged!')
                    rospy.loginfo('Converged locs: %s' % str(self.training_db.data[actionid]['practice_locations_convergence']))
                    rospy.loginfo('number of datapoints %s' % str(self.training_db.get_num_data_points(actionid)))
                    rospy.loginfo('===================================================')

            pidx = (pidx + 1) % TrainingInformationDatabase.NUM_BASE_LOCATIONS 
            self.training_db.save_database()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        pkl_db = sys.argv[1]
    else:
        pkl_db = 'trf_learn_db.pkl'

    server = TRFClassificationServer(pkl_db)
    rospy.spin()

        #   Loop
        #     Set variable for state of environment
        #     Set STATE for the behavior saying that it's in training mode.
        #     Call behavior/behavior complement, which drives to mechanism location.
        #         Calls classify, which runs in training mode, #recycling previous scans if needed
        #             Returns the instance, along with its 3D point
        #         Calls success function with param of actionid, and BEFORE state 
        #                 (can be in state machine side and not on classification server side)
        #             Some return value = actual manipulation behavior executes
        #         Class success function with param of actionid, and AFTER state
        #         Notify the recognition server about instance and the outcome of success classifier.
        #             The server looks up the behavior, see that it's in training mode (STATE)
        #             adds the point and retrain the classifier.
        #
        #     Behavior returns success/failure
        #
        # Set STATE back to not training mode

    #def get_behavior_pose3d(self, actionid):
    #    posestamped = self.get_behavior_pose(actionid)
    #    p = posestamped.pose
    #    return np.matrix([p.x, p.y, p.z]).T

    #def run_behavior(self, actionid, point3d, frame_id):
    #    ps = PoseStamped()
    #    ps.pose = point3d
    #    ps.header.frame_id = frame

    #    self.set_behavior_pose(ps)
    #    goal = rmsg.RunScriptActionIDGoal(actionid) 
    #    return self.run_action_id_client.send_goal_and_wait(goal, 
    #            execute_timeout=rospy.Duration(60.*5))

