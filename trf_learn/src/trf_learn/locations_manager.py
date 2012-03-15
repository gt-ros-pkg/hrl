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
import threading
import hrl_lib.util as ut
import shutil
import visualization_msgs.msg as vm
import os.path as pt
import numpy as np
import time
import pdb
import os
import trf_learn.recognize_3d as r3d
import cv


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

class LocationsManager:

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
        self.image_pubs = {}

        for k in self.data.keys():
            self.train(k)
            self.image_pubs[k] = r3d.ImagePublisher(k.replace(':', '_'))

        self.task_types = ['light_switch_down', 'light_switch_up', 
                            'light_rocker_down', 'light_rocker_up', 
                            'push_drawer', 'pull_drawer']

        self.task_pairs = [['light_switch_down', 'light_switch_up'], 
                           ['light_rocker_down', 'light_rocker_up'],
                           ['pull_drawer', 'push_drawer']]


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
        #rospy.loginfo('LocationsManager: save_database saved db!')

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
        rospy.loginfo('LocationsManager: add_perceptual_data - %s adding %d instance(s)' \
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

