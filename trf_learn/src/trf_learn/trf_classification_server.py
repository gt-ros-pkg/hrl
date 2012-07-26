import roslib; roslib.load_manifest('trf_learn')
import rospy
import numpy as np
import trf_learn.recognize_3d as r3d
import pdb
import hrl_lib.util as ut


class TRFClassificationServer:

    def __init__(self):
        rospy.init_node('trf_classification_server')
        rospy.Service('recognize_pose', RecognizePose, self.recognize_pose_srv_cb)
        self.run_action_client = actionlib.SimpleActionClient('run_rcommander_action_web', rmsg.RunScriptAction)

        #self.last_known_pose = rospy.ServiceProxy('last_known_pose', LastKnownPose)
        self.rec_params = r3d.Recognize3DParam()
        self.locations_man = lcm.LocationsManager('trf_learn_db.pkl', rec_params=self.rec_params) #TODO

        #change this to kinect
        self.feature_ex = r3d.NarrowTextureFeatureExtractor(self.prosilica, 
                hd.PointCloudReceiver('narrow_stereo_textured/points'),
                self.prosilica_cal, 
                #self.robot.projector,
                self.tf_listener, self.rec_params)

        self.robot_base = something

    def say(self, message):
        pass

    def driving_posure(self):
        pass

    def recognize_pose_srv_cb(self, recognize_pose_request):
        task_id = recognize_pose_request.task_id
        point = recognize_pose_request.last_known_pose.pose.position
        point_mat = np.matrix([point.x, point.y, point.z]).T

        kdict = self.feature_ex.read(point_mat, params=None)
        predictions = np.matrix(self.locations_man.learners[task_id].classify(kdict['instances']))
        pos_indices = np.where(r3d.POSITIVE == predictions)[1].A1

        locs2d = None
        if len(pos_indices) > 1:
            locs2d = kdict['points2d'][:, pos_indices]
            if np.any(np.isnan(locs2d)) or np.any(np.isinf(locs2d)):
                pdb.set_trace()
            locs2d_indices = np.where(False == np.sum(np.isnan(locs2d), 0))[1].A1
            print locs2d[:, locs2d_indices]
            loc2d_max, density_image = r3d.find_max_in_density(locs2d[:, locs2d_indices])
            #cv.SaveImage("execute.png", 
            #        cv.fromarray(255 * (np.rot90(density_image)/np.max(density_image))))
            dists = ut.norm(kdict['points2d'] - loc2d_max)
            selected_idx = np.argmin(dists)
        else:
            selected_idx = pos_indices[0]
            loc2d_max = kdict['points2d'][: selected_idx]

        selected_3d = kdict['points3d'][:, selected_idx]
        ps = PoseStamped()
        ps.position.x = selected_3d[0,0]
        ps.position.y = selected_3d[1,0]
        ps.position.z = selected_3d[2,0]

        return RecognizePoseResponse(ps)



    def init_task(self, point_bl):
        #If that location is new:
        #pdb.set_trace()
        self.look_at(point_bl, False)
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl)

        #Initialize new location
        rospy.loginfo('Select task type:')
        for i, ttype in enumerate(self.locations_man.task_types):
            print i, ttype
        task_type = self.locations_man.task_types[int(raw_input())]
        rospy.loginfo('Selected task %s' % task_type)
        self.manipulation_posture(task_type)
        point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
        self.look_at(point_bl, False)

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
        t_current_map, r_current_map = self.robot_base.get_pose()
        task_id = self.locations_man.create_new_location(task_type, 
                np.matrix([0,0,0.]).T, [t_current_map, r_current_map], name=location_name)
        ctask_id = self.locations_man.create_new_location(ctask_type, 
                np.matrix([0,0,0.]).T, [t_current_map, r_current_map], name=location_name)

        #Stop when have at least 1 pos and 1 neg
        point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
        self.look_at(point_bl, True)
        ret_dict_action = self.random_explore_init(task_id, ctask_id, point_bl, stop_fun=has_pos_and_neg, should_reset=True)
        dset_action = ret_dict_action['features']
        dset_undo = ret_dict_action['features_undo']
        undo_point_bl = ret_dict_action['undo_point']

        #Lights should be on at this stage!
        #If we don't have enought data for reverse action
        rospy.loginfo('====================================================')
        rospy.loginfo('Don\'t have enough data for reverse action')
        if (self.locations_man.data[ctask_id]['dataset'] == None) or \
                not has_pos_and_neg(self.locations_man.data[ctask_id]['dataset'].outputs.A1):
            #Turn off the lights 
            point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
            self.look_at(point_bl, True)
            rospy.loginfo('====================================================')
            rospy.loginfo('Running random_explore_init on set %s.' % task_id)
            ret_dict_action = self.random_explore_init(task_id, None, point_bl, stop_fun=any_pos_sf, should_reset=False)
            undo_point_bl = ret_dict_action['undo_point']

            #Practice until stats are met
            self.look_at(point_bl, True)
            #ret_dict_undo = self.random_explore_init(ctask_id, task_id, point_bl, stop_fun=has_pos_and_neg)
            rospy.loginfo('====================================================')
            rospy.loginfo('Running random_explore_init on set %s.' % ctask_id)
            ret_dict_undo = self.random_explore_init(ctask_id, task_id, undo_point_bl, stop_fun=has_pos_and_neg)
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

        self.locations_man.train(task_id, dset_action, save_pca_images=True)
        self.locations_man.train(ctask_id, dset_undo, save_pca_images=True)
        self.locations_man.save_database()
        rospy.loginfo('Done initializing new location!')
        self.driving_posture(task_type)

    ##
    # Practice phase
    def practice_task(self, task_id):
        rospy.loginfo('===================================================')
        rospy.loginfo('= Practice Mode!                                  =')
        rospy.loginfo('===================================================')
        #pdb.set_trace()

        #ulocs = self.unreliable_locs()
        #rospy.loginfo('%d out of %d locs in database are unreliable' \
        #        % (len(ulocs), len(self.locations_man.data.keys())))

        #Ask user to select a location
        #tasks = self.locations_man.data.keys()
        #for i, k in enumerate(tasks):
        #    print i, k
        #selected_idx = int(raw_input("Select a location to execute action\n"))

        #tid = ulocs[selected_idx]
        tid = task_id
        #tid = tasks[selected_idx]
        #rospy.loginfo('selected %s' % tid)

        #Ask user for practice poses
        if not self.locations_man.data[tid].has_key('practice_locations'):
            #Get robot poses
            map_points = []
            for i in range(4):
                raw_input('%d move robot to desired position\n' % i)
                rospy.sleep(1.)
                #pdb.set_trace()
                t_current_map, r_current_map = self.robot_base.get_pose()
                map_points.append([t_current_map, r_current_map])
            self.locations_man.data[tid]['practice_locations'] = map_points
            self.locations_man.data[tid]['practice_locations_history'] = np.zeros((1, len(map_points)))
            self.locations_man.data[tid]['practice_locations_convergence'] = np.zeros((1, len(map_points)))
            self.locations_man.save_database()

        #Ask user for canonical pose if it does not exist
        if not self.locations_man.data[tid].has_key('base_pose'):
            raw_input('move robot to desired end position\n')
            trans, rot_quat = self.robot_base.get_pose()
            self.locations_man.data[tid]['base_pose'] = [trans, rot_quat]
            self.locations_man.save_database()

        point_map = self.locations_man.data[tid]['center']
        task_type = self.locations_man.data[tid]['task']
        points_added_history = []
        #pdb.set_trace()

        unexplored_locs  = np.where(self.locations_man.data[tid]['practice_locations_history'] == 0)[1]
        unconverged_locs = np.where(self.locations_man.data[tid]['practice_locations_convergence'] == 0)[1]
        rospy.loginfo("Location history: %s" % str(self.locations_man.data[tid]['practice_locations_history']))
        run_loop = True

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

        #pdb.set_trace()

        #Commence practice!
        while not rospy.is_shutdown(): #%not converged:

            if self.locations_man.data[tid]['practice_locations_convergence'][0, pidx] == 0:
                #TODO: JUST CALL TUCK
                #Drive to location
                #self.driving_posture(task_type)

                #Move to setup location
                self.say('Driving to practice location')
                rvalue, dist = self.move_base_planner(*self.locations_man.data[tid]['practice_locations'][pidx])
                rospy.loginfo('move ret value %s dist %f' % (str(rvalue), dist))

                #Move to location where we were first initialized
                self.say('Driving to mechanism location')
                #TODO: JUST CALL BEHAVIOR HERE

                #rvalue, dist = self.move_base_planner(*self.locations_man.data[tid]['base_pose'])

                #Reorient base
                #bl_T_map = tfu.transform('base_link', 'map', self.tf_listener)
                #point_bl = tfu.transform_points(bl_T_map, point_map)
                #ret = self.location_approach_driving(task_type, point_bl)
                #self.robot_base.set_pose(self.robot_base.get_pose()[0], self.locations_man.data[tid]['base_pose'][1], 'map')
                #if not ret[0]:
                #    rospy.loginfo('Driving failed!! Resetting.')
                #    #pdb.set_trace()
                #    #return False, ret[1]
                #    continue

                #self.manipulation_posture(task_type)
                #bl_T_map = tfu.transform('base_link', 'map', self.tf_listener)
                #point_bl = tfu.transform_points(bl_T_map, point_map)
                #self.look_at(point_bl, False)

                #Practice
                self.say('practicing')
                ctid = self.locations_man.data[tid]['complementary_task_id']

                points_before_t = self.locations_man.data[tid]['dataset'].inputs.shape[1]
                points_before_ct = self.locations_man.data[ctid]['dataset'].inputs.shape[1]

                points_added = self.execute_one_training_session(tid, ctid,  point_bl)

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
        #TODO: call Tuck
        #self.driving_posture(task_type)

    def execute_one_training_session(self, task_id, ctask_id, point3d_bl, stop_fun=None, params=None, 
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

        kdict['image_T_bl'] = tfu.transform(self.optical_frame, 'base_link', self.tf_listener)
        point3d_img = tfu.transform_points(kdict['image_T_bl'], point3d_bl)
        point2d_img = self.feature_ex.cal.project(point3d_img)

        labels = []
        points3d_tried = []
        points2d_tried = []
        converged = False
        indices_added = []
        reset_times = []
        task_name = task_id.replace('_', ' ')

        #while not converged or (stop_fun != None and not stop_fun(np.matrix(labels))):
        while not rospy.is_shutdown():# and (stop_fun != None and not stop_fun(np.matrix(labels))):
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
                        self.robot.sound.say('executing ' +  task_name)
                        pstart_0 = self.robot.left.pose_cartesian_tf()
                        success, _, undo_point_bl = behavior(kdict['points3d'][:, selected_idx])
                        #self.did_end_effector_move(pstart)
                else:
                    self.robot.sound.say('executing ' +  task_name)
                    pstart_0 = self.robot.left.pose_cartesian_tf()
                    success, _ , undo_point_bl = behavior(kdict['points3d'][:, selected_idx])
                    #self.did_end_effector_move(pstart)
            #self.robot.projector.set(True)

            if success:
                color = [0,255,0]
                label = r3d.POSITIVE
                rospy.loginfo('=============================================')
                rospy.loginfo('>> %s successful' % task_id)
                rospy.loginfo('=============================================')
                self.robot.sound.say('action succeeded')
            else:
                label = r3d.NEGATIVE
                color = [0,0,255]
                rospy.loginfo('=============================================')
                rospy.loginfo('>> %s NOT successful' % task_id)
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

