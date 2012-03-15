
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

    #def go_to_home_pose(self):
    #    #self.behaviors.movement.set_movement_mode_cart()
    #    return self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
    #    #self.behaviors.movement.set_movement_mode_ik()
    #    #return self.behaviors.movement.move_absolute(self.start_location, stop='pressure')


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


        #if self.approach_location(point_bl, 
        #        coarse_stop=self.locations_man.driving_param[task]['coarse'], 
        #        fine_stop=self.locations_man.driving_param[task]['fine'], 
        #        voi_radius=self.locations_man.driving_param[task]['voi']):
        #    #rospy.logerr('location_approach_driving: intial approach failed')
        #    return True, 'initial approach'
        #else:


    #def load_classifier(self, classifier_name, data_file_name):
    #    self.learners[classifier_name] = ipa.InterestPointPerception(classifier_name, 
    #            data_file_name, self.tf_listener)































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

