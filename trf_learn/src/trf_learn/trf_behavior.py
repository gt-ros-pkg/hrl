#! /usr/bin/python

#
# Copyright (c) 2010, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#


import roslib; roslib.load_manifest('trf_learn')
import rospy

import scipy.spatial as sp
import os.path as pt
import numpy as np
import time
import pdb
import os

import cv
import tf
import tf.transformations as tr
import laser_interface.laser_client as lc
import hrl_camera.ros_camera as rc
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import hrl_lib.viz as viz
import hrl_pr2_lib.devices as hd

import trf_learn.recognize_3d as r3d
import trf_learn.application_behaviors as ab
import trf_learn.locations_manager as lcm

class TaskRelevantLearningBehaviors:

    def __init__(self, app_behaviors, tf_listener, optical_frame):
        #Import few fuctions from app_behaviors to keep interface clean
        self.look_at = app_behaviors.look_at
        self.manipulation_posture = app_behaviors.manipulation_posture
        self.driving_posture = app_behaviors.driving_posture
        self.get_behavior_by_task = app_behaviors.get_behavior_by_task
        self.move_base_planner = app_behaviors.move_base_planner
        self.location_approach_driving = app_behaviors.location_approach_driving
        self.optical_frame = optical_frame

        #Init ROS things
        self.tf_listener = tf_listener
        self.robot = app_behaviors.robot

        self.prosilica = app_behaviors.prosilica
        self.prosilica_cal = rc.ROSCameraCalibration('/prosilica/camera_info')
        self.rec_params = r3d.Recognize3DParam()

        self.laser_listener = lc.LaserPointerClient(tf_listener=self.tf_listener)
        self.laser_listener.add_double_click_cb(self.click_cb)

        self.feature_ex = r3d.NarrowTextureFeatureExtractor(self.prosilica, 
                hd.PointCloudReceiver('narrow_stereo_textured/points'),
                self.prosilica_cal, 
                self.robot.projector,
                self.tf_listener, self.rec_params)
        self.locations_man = lcm.LocationsManager('locations_narrow_v11.pkl', rec_params=self.rec_params) #TODO

    def click_cb(self, point_bl):
        if point_bl!= None:
            print 'Got point', point_bl.T
            self.init_task(point_bl)


    def run(self, mode, save, user_study):
        r = rospy.Rate(10)
        rospy.loginfo('Ready.')
        #self.init_task(np.matrix([[ 0.94070742, -0.23445448,  1.14915097]]).T)
        while not rospy.is_shutdown():
            r.sleep()
            if mode == 'practice':
                self.practice_task()

            if mode == 'execute':
                t = time.time()
                self.execute_task(save, user_study)
                t2 = time.time()
                print '>> That took', t2 - t, 'seconds'
                exit()

            if mode == 'update_base':
                self.update_base()

    #######################################################################################
    # Utility function
    #######################################################################################

    def update_base(self):
        tasks = self.locations_man.data.keys()
        for i, k in enumerate(tasks):
            print i, k
        selected_idx = int(raw_input("Select an action to update base location\n"))

        t_current_map, r_current_map = self.robot.base.get_pose()
        task_id = tasks[selected_idx]
        self.locations_man.update_base_pose(task_id, [t_current_map, r_current_map])
        self.locations_man.save_database()

    def unreliable_locs(self):
        tasks = self.locations_man.data.keys()
        utid = []
        for tid in tasks:
            if not self.locations_man.is_reliable(tid):
                utid.append(tid)
        return utid

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

    #######################################################################################
    # Perception Behaviors
    #######################################################################################

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

    def read_features_save(self, task_id, point3d_bl, params=None):
        self.robot.projector.set(True)
        rospy.sleep(2.)
        f = self.feature_ex.read(point3d_bl, params=params)
        file_name, kimage_name = self.record_perceptual_data(point3d_bl, self.optical_frame, rdict=f['rdict'], folder_name=task_id)
        self.robot.projector.set(False)

        #image_T_bl = tfu.transform(self.optical_frame, 'base_link', self.tf_listener)
        #point3d_img = tfu.transform_points(image_T_bl, point3d_bl)
        #point2d_img = self.feature_ex.cal.project(point3d_img)

        #img = cv.CloneMat(f['image'])
        #self.draw_dots_nstuff(img, f['points2d'], 
        #        np.matrix([r3d.UNLABELED]* f['points2d'].shape[1]), 
        #        point2d_img)
        #self.img_pub.publish(img)

        return f, kimage_name
        #self.save_dataset(point, name, f['rdict'])

    #######################################################################################
    # Learning Manipulation Behaviors
    #######################################################################################
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

        kdict['image_T_bl'] = tfu.transform(self.optical_frame, 'base_link', self.tf_listener)
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

    ##
    def execute(self, task_id, point3d_bl, save, max_retries=15, closeness_tolerance=.01, user_study=False):
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
        kdict['image_T_bl'] = tfu.transform(self.optical_frame, 'base_link', self.tf_listener)
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
                    cv.SaveImage("execute.png", 255 * (np.rot90(density_image)/np.max(density_image)))
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
            self.random_explore_init(task_id, ctask_id,
                    point3d_bl, max_retries=max_retries, stop_fun=any_pos_sf, 
                    closeness_tolerance=closeness_tolerance, should_reset=False)

    ###
    # TODO: WE MIGHT NOT NEED THIS AFTER ALL, MIGHT BE ABLE TO JUST INITIALIZE RANDOMLY!
    ###
    def random_explore_init(self, task_id, ctask_id, point_bl, stop_fun, 
            max_retries=30, closeness_tolerance=.01, positive_escape=.08, should_reset=False):
        #rospy.loginfo('random_explore_init: %s' % task_id)
        ##case label
        #if self.locations_man.data[task_id]['dataset'] != None:
        #    case_labels = self.locations_man.data[task_id]['dataset'].outputs.A1.tolist()
        #else:
        #    case_labels = []
        #if stop_fun(case_labels):
        #    rospy.loginfo('random_explore_init: stop function satisfied with label set. not executing.')
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
        image_T_bl = tfu.transform(self.optical_frame, 'base_link', self.tf_listener)
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
                    undo_ret = self.random_explore_init(ctask_id, None, ctask_point, stop_fun=any_pos_sf, 
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

    #######################################################################################
    # Learning Manipulation Behaviors
    #######################################################################################
    # Initialization phase
    #
    # @param point_bl 3x1 in base_link
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
        t_current_map, r_current_map = self.robot.base.get_pose()
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
    def practice_task(self):
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
        selected_idx = int(raw_input("Select a location to execute action\n"))

        #tid = ulocs[selected_idx]
        tid = tasks[selected_idx]
        rospy.loginfo('selected %s' % tid)

        #Ask user for practice poses
        if not self.locations_man.data[tid].has_key('practice_locations'):
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

    ##
    # Execution phase
    def execute_task(self, save, user_study):
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
        self.execute(tid, point_bl, save, user_study=user_study)
        self.driving_posture(task_type)

class TestLearner:

    def __init__(self):
        self.rec_params = r3d.Recognize3DParam()
        self.locations_man = lcm.LocationsManager('locations_narrow_v11.pkl', rec_params=self.rec_params)

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
    locations_man = lcm.LocationsManager('locations_narrow_v11.pkl', 
            rec_params=rec_params)
    location_display = lcm.LocationDisplay(locations_man)
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
        rospy.init_node('trf_learn', anonymous=True)
        optical_frame = 'high_def_optical_frame'
        tf_listener = tf.TransformListener()

        app_behaviors = ab.ApplicationBehaviorsDB(optical_frame, tf_listener)
        learn_behaviors = TaskRelevantLearningBehaviors(app_behaviors, tf_listener, optical_frame)

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
        learn_behaviors.run(mode, not opt.static, opt.user_study)

    if opt.test:
        tl = TestLearner()
        tl.test_training_set()

if __name__ == '__main__':
    launch()
    #cProfile.run('launch()', 'profile_linear_move.prof')




