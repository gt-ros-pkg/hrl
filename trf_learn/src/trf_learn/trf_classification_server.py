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
import actionlib
import tf

import hrl_camera.ros_camera as rc
import hrl_pr2_lib.devices as hd


def select_closest_instance(fea_dict, point_bl):
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
        self.active_learn_sessions = {}

    def save_database(self):
        rospy.loginfo('Saving pickle. DONOT INTERRUPPT!!!')
        try:
            shutil.copyfile(self.saved_locations_fname, 
                    time.strftime('%m_%d_%Y_%I_%M%p') + '_locations.pkl')
        except Exception, e:
            rospy.loginfo('%s %s' % (str(e), str(e.__class__)))
        ut.save_pickle(self.data, self.saved_locations_fname)
        rospy.loginfo('SAVED!!!')

    def get_learner(self, actionid):
        if self.learners.has_key(actionid):
            return self.learners[actionid]

        return None

    def init_data_record(self, actionid):
        if not self.data.has_key(actionid):
            self.data[actionid] = {'dataset': None,
                                   'pca': None,
                                   'pca_dataset': None,
                                   'practice_locations': None,
                                   'practice_locations_history': None,
                                   'practice_locations_convergence': None,
                                   'execution_record': []}

    def get_active_learn_session(self, actionid):
        return self.active_learn_sessions[actionid]

    def set_active_learn_session(self, actionid, session):
        self.active_learn_sessions[actionid] = session

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
        self.data[actionid]['practice_locations_history'][0,idx] += numb

    def get_practice_convergence(self, actionid):
        return self.data[actionid]['practice_locations_convergence']

    def get_dataset(self, actionid):
        return self.data[actionid]['dataset']

    def get_num_data_points(self, actionid):
        return self.data[actionid]['dataset'].inputs.shape[1]

    def add_pca_dataset(self, actionid, pca_dataset):
        self.data[actionid]['pca_dataset'] = pca_dataset

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

        #Add to dataset
        self.data[actionid]['dataset'] = \
                r3d.InterestPointDataset.add_to_dataset(
                        current_dataset, al_point_container.instances, #['instances'], 
                        label, al_point_container.points2d, #datapoint_dict['points2d'], 
                        al_point_container.points3d, #datapoint_dict['points3d'], 
                        None, None, sizes=al_point_container.sizes) #datapoint_dict['sizes'])

        #If have no learner, try to train one
        if self.get_learner(actionid) == None:
            self.train(actionid)


    #def train(self, actionid, dset_for_pca=None, save_pca_images=True):
    def train(self, actionid, save_pca_images=True):
        dataset = self.data[actionid]['dataset']
        rec_params = self.rec_params
        if dataset == None:
            return

        #Balance pos/neg
        nneg = np.sum(dataset.outputs == r3d.NEGATIVE) 
        npos = np.sum(dataset.outputs == r3d.POSITIVE)
        rospy.loginfo( '================= Training =================')
        rospy.loginfo('NEG examples %d' % nneg)
        rospy.loginfo('POS examples %d' % npos)
        rospy.loginfo('TOTAL %d' % dataset.outputs.shape[1])
        if npos == 0:
            rospy.loginfo('Not training as we don\'t have at least one positive point')
            return

        if nneg == 0 or npos == 0:
            weight_balance = ''
        else:
            neg_to_pos_ratio = float(nneg)/float(npos)
            weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio

        rospy.loginfo('TRAINING for %s' % actionid)
        previous_learner = None
        if self.learners.has_key(actionid):
            previous_learner = self.learners[actionid]
        learner = r3d.SVMPCA_ActiveLearner(use_pca=True, 
                        reconstruction_std_lim=self.rec_params.reconstruction_std_lim, 
                        reconstruction_err_toler=self.rec_params.reconstruction_err_toler,
                        old_learner=previous_learner, pca=self.data[actionid]['pca'])

        #TODO: figure out something for scaling inputs field!
        if dset_for_pca != None:
            inputs_for_pca = dset_for_pca['instances']
        else:
            inputs_for_pca = dataset.inputs

        learner.train(dataset, 
                      inputs_for_pca,
                      rec_params.svm_params + weight_balance,
                      rec_params.variance_keep)

        self.data[actionid]['pca'] = learner.pca
        self.learners[actionid] = learner
        if save_pca_images:
            #pdb.set_trace()
            basis = learner.pca.projection_basis
            cv.SaveImage('%s_pca.png' % actionid, r3d.instances_to_image(self.rec_params.win_size, basis, np.min(basis), np.max(basis)))

class TRFInteractiveMarkerServer:


    def __init__(self, training_db, classification_server):
        self.classification_server = classification_server
        self.marker_server = ims.InteractiveMarkerServer('trf_interactive_markers')
        self.training_db = training_db
        self.markers = {}

    def show_base_locations(self, actionid):
        self.markers[actionid] = []
        initialized = self.training_db.has_training_locations(actionid)
        for i in range(TrainingInformationDatabase.NUM_BASE_LOCATIONS):
            if not initialized:
                pose = [[0,0,0.], [0,0,0,1.], '/map']
                self.training_db.update_training_location(actionid, i, pose)
            else:
                pose = self.training_db.get_training_location(actionid, i)

            cb = ft.partial(self.accept_cb, actiond)
            m = RobotPositionMarker(actionid, i, pose, frame, 
                    self.marker_server, self.training_db, cb)
            self.markers[actionid].append(m)

    def accept_cb(self, actionid, feedback):
        rospy.loginfo('accept_cb on %s' % actionid)
        self.classification_server.train_action(actionid)

class RobotPositionMarker:

    def __init__(self, actionid, marker_number, pose, frame, 
            marker_server, training_db, accept_cb):
        self.marker_name = actionid + '_' + str(marker_number)

        int_marker = imh.interactive_marker(self.marker_name, pose, scale)
        int_marker.header.frame_id = frame
        int_marker.scale = .6
        int_marker.description = actionid + '_' + str(marker_number)

        #Make controls
        sph = imh.make_sphere_control(self.marker_name, scale)
        int_marker.controls += [sph]
        int_marker.controls += imh.make_directional_controls(self.marker_name)
        int_marker.controls += imh.make_orientation_controls(self.marker_name)

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
        if feedback.event_type == ims.InteractiveMarkerFeedback.POSE_UPDATE:
            p_ar = pose_to_tup(feedback.pose)
            self.training_db.update_training_location(self.actionid, self.marker_number, 
                    [p_ar, self.frame])



##
# Create a session per scan
class TRFActiveLearnSession:

    TOO_FAR_AWAY = .5
    RESOLUTION = .01

    def __init__(self, classification_server, actionid, point_bl):
        self.cserver = classification_server
        self.actionid = actionid

        self.kdict = classification_server.feature_ex.read(point_bl, params=self.rec_params)
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

    def __init__(self):
        rospy.init_node('trf_classification_server')
        self.get_behavior_property = rospy.ServiceProxy('get_behavior_property', atsrv.ActionProperty)
        self.get_behavior_pose     = rospy.ServiceProxy('get_behavior_pose', atsrv.GetBehaviorPose)
        self.set_behavior_pose     = rospy.ServiceProxy('set_behavior_pose', atsrv.SetBehaviorPose)

        #Handles request to recognize the pose of an object.
        rospy.Service('recognize_pose', atsrv.RecognizePose, self.recognize_pose_srv_cb)

        #Messages that tells this node about the outcome of actions.
        rospy.Service('action_result', atsrv.ActionResult, self.action_result_srv_cb)

        #Messages that request an action be trained
        rospy.Subscriber('train_action', atmsg.TrainAction, self.train_action_cb)

        #rospy.Service('initialize', InitializeTRF, self.initialize_trf_cb)
        #self.run_action_path_client = actionlib.SimpleActionClient('run_rcommander_action_web', atmsg.RunScriptAction)

        self.run_action_id_client = actionlib.SimpleActionClient('run_actionid', atmsg.RunScriptIDAction)

        #self.last_known_pose = rospy.ServiceProxy('last_known_pose', LastKnownPose)
        #self.locations_man = lcm.LocationsManager('trf_learn_db.pkl', rec_params=self.rec_params) #TODO

        self.rec_params = r3d.Recognize3DParam()
        self.training_db = TrainingInformationDatabase('trf_learn_db.pkl', self.rec_params)
        self.marker_server = TRFInteractiveMarkerServer(self.training_db, self)
        self.tf_listener = tf.TransformListener()


        #TODO: change this to kinect
        self.prosilica = rc.Prosilica('prosilica', 'polled')
        self.prosilica_cal = rc.ROSCameraCalibration('/prosilica/camera_info')
        self.feature_ex = r3d.NarrowTextureFeatureExtractor(self.prosilica, 
                hd.PointCloudReceiver('narrow_stereo_textured/points'),
                self.prosilica_cal, 
                self.tf_listener, self.rec_params)

        #self.robot_base = something
        self.move_base_client = actionlib.SimpleActionClient('move_base', 
                mm.MoveBaseAction)

        self.tuck_arm_client = actionlib.SimpleActionClient('rcommander_tuckarms', 
                rm.RCTuckArmsAction)

        self.optical_frame = 'high_def_optical_frame'
        rospy.loginfo('Ready!')


    def action_result_srv_cb(self, action_result_msg):
        actionid      = action_result_msg.actionid
        result        = action_result_msg.result
        mode          = action_result_msg.mode
        instance_prop = pickle.loads(action_result_msg.info)

        #Not initialized yet, so we add instance
        if training_db.get_learner(actionid) == None or mode == 'train':
            rospy.loginfo('Adding data point with label %s to action %s' 
                    % (str(result), actionid))
            self.training_db.add_data_instance(actionid, instance_prop['instance'], result)
            self.training_db.save_database()
            if mode != 'train':
                self.training_db.train(actionid)
            else:
                als = self.training_db.get_active_learn_session()
                self.training_db.train(actionid, als.get_features_read())

    ##
    # Assume that the robot is looking at what it needs to recognize
    # @param RecognizePoseRequest has field actionid (string), and last_known_pose (PoseStamped)
    # @return RecognizePoseResponse with a PoseStamped
    def recognize_pose_srv_cb(self, recognize_pose_request):
        actionid  = recognize_pose_request.actionid
        mode      = recognize_pose_request.mode
        #frame     = recognize_pose_request.last_known_pose.header.frame_id #should be /map
        #point     = recognize_pose_request.last_known_pose.pose.position
        posestamped = self.get_behavior_pose(actionid).posestamped
        frame = posestamped.header.frame_id
        point = posestamped.pose.position

        point_mat = np.matrix([point.x, point.y, point.z]).T
        point_bl  = tfu.transform_points(
                        tfu.transform('base_link', frame, self.tf_listener), 
                        point_mat)

        #No learner!
        if self.training_db.get_learner(actionid) == None:
            rospy.loginfo('Has no learner. Just using closest point to prior.')
            # There is no associated classifier, we use the seed as our
            # positive point, construct the classifier and return.
            params = r3d.Recognize3DParam()
            params.n_samples = 5000
    
            #Capture a scan 
            fea_dict = self.feature_ex.read(point_bl, params=params)
            point2d, point3d, instance = select_closest_instance(fea_dict, point_bl)
            datapoint = ActiveLearnPointContainer(instance, point2d,
                                                    point3d, frame,
                                                    fea_dict['sizes'])
                    
            #Add this scan as a PCA dataset
            self.training_db.init_data_record(actionid)
            self.training_db.add_pca_dataset(fea_dict)

            return RecognizePoseResponse(recognize_pose_request.last_known_pose,
                                         pickle.dumps(datapoint))

        elif mode == 'train':
        #elif self.training_db.is_practicing(actionid):
            active_learn = self.training_db.get_active_learn_session(actionid)
            if active_learn == None:
                active_learn = TRFActiveLearnSession(self, actionid, point_bl)
                self.training_db.set_active_learn_session(actionid, active_learn)

            al_point_container = active_learn.get_response()

            ps = PoseStamped()
            ps.header.frame_id = 'base_link'
            ps.pose = recognize_pose_request.last_known_pose.pose #sets position/orientation
            p = al_point_container.points3d[:,0]
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p.A1.tolist()

            return RecognizePoseResponse(ps, pickle.dumps(al_point_container))

        else:
            #should have learner so we'll just do classification
            kdict = self.feature_ex.read(point_bl)
            predictions = np.matrix(self.locations_man.learners[actionid].classify(kdict['instances']))
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
                selected_instance = kdict['instances'][:, sampled_idx]
            else:
                rospy.loginfo('FOUND NO POSITIVE POINTS. JUST USING CLOSEST POINT TO PRIOR.')
                selected_2d, selected_3d, selected_instance = select_closest_instance(kdict, point_bl)

            selected_3d_f = tfu.transform_points(
                                tfu.transform(recognize_pose_request.last_known_pose.header.frame_id,
                                    'base_link', self.tf_listener), selected_3d)
            
            ps = PoseStamped()
            ps.header.frame_id = recognize_pose_request.last_known_pose.header
            ps.pose = recognize_pose_request.last_known_pose.pose #sets position/orientation
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = selected_3d_f.A1.tolist()
            datapoint = ActiveLearnPointContainer(selected_instance, selected_2d,
                                                    selected_3d_f, ps.header.frame_id,
                                                    kdict['sizes'])
            return RecognizePoseResponse(ps, pickle.dumps(datapoint))

    def train_action_cb(self, msg):
        #do this only if the database doesn't have training locations
        #create interactive markers, 4, wait for user to click on confirm on one of them
        self.marker_server.show_base_locations(msg.actionid)

    def _tuck(self, left, right):
        goal = rm.RCTuckArmsGoal()
        goal.tuck_left = left
        goal.tuck_right = right 
        self.tuck_arm_client.send_goal(goal)
        rospy.loginfo('waiting for tuck arm results')
        self.tuck_arm_client.wait_for_result()

    def move_to_location(self, position, quat, frame):
        self._tuck(True, True)
        g = mm.MoveBaseGoal()
        p = g.target_pose
        p.header.frame_id = frame
        p.header.stamp = rospy.get_rostime()
        p.pose = tup_to_pose((position, quat))
        self.move_base_client.send_goal(g)
        rospy.loginfo('waiting for move base results')
        self.move_base_client.wait_for_result()

    def _train_helper(self, actionid, cactionid, stop_fun=None):
        labels = []

        while not rospy.is_shutdown():
            active_learn_session = self.training_db.get_active_learn_session(actionid)

            if stop_fun != None and stop_fun(np.matrix(labels)):
                rospy.loginfo('Stop satisfied told us to stop loop!')
                break

            if active_learn_session != None and stop_fun == None \
                    and len(active_learn_session.indices_added) > self.rec_params.max_points_per_site:
                rospy.loginfo('practice: added enough points from this scan. Limit is %d points.' \
                        % self.rec_params.max_points_per_site)
                break

            #Run action
            self.run_action_id_client.send_goal(atmsg.RunScriptIDActionGoal(actionid, pickle.dumps({'mode':'train'})))
            self.run_action_id_client.wait_for_result()
            runaction_result = self.run_action_id_client.get_result()
            success = (runaction_result == 'success')

            if success:
                label = r3d.POSITIVE
                #run the reverse if we succeed 
                def any_pos_sf(labels_mat):
                    if np.any(r3d.POSITIVE == labels_mat):
                        return True
                    return False
                #reverse our action.
                self._train_helper(cactionid, actionid, stop_fun=any_pos_sf)
            else:
                label = r3d.NEGATIVE

            labels.append(label)
        self.training_db.save_database()


    def train_action(self, actionid):
        self.training_db.init_data_record(actionid)
        cactionid = self.get_behavior_property(actionid, 'complement')
        unexplored_locs  = np.where(self.training_db.get_practice_history(actionid) == 0)[1]
        unconverged_locs = np.where(self.training_db.get_practice_convergence(actionid) == 0)[1]
        rospy.loginfo("Location history: %s" % str(self.training_db.get_practice_history('practice_locations_history')))

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
                self.training_db.set_active_learn_session(actionid, None)
                self.training_db.set_active_learn_session(cactionid, None)
                self.move_to_location(self.training_db.get_training_location(actionid, pidx))

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
    server = TRFClassificationServer()
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

