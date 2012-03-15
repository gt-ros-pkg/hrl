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
import cv
import rospy
import sensor_msgs.msg as sm
#import visualization_msgs.msg as vm

import numpy as np
import scipy.spatial as sp
import threading
import Queue as qu
import os.path as pt
import glob

#import pdb
#pdb.set_trace()
#import libsvm.svm as svm
import libsvm.svmutil as su

import copy
import os

import ml_lib.dataset as ds
import ml_lib.dimreduce as dr

#import hrl_opencv.blob as blob
import hrl_opencv.image3d as i3d
import hrl_lib.util as ut
#import hrl_lib.viz as viz
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
#import hrl_lib.prob as pr
import trf_learn.intensity_feature as infea


#import message_filters
#import feature_extractor_fpfh.msg as fmsg
import time
import pdb

UNLABELED = 2.0
POSITIVE = 1.0
NEGATIVE = 0.

def separate_by_labels(points, labels):
    pidx = np.where(labels == POSITIVE)[1].A1.tolist()
    nidx = np.where(labels == NEGATIVE)[1].A1.tolist()
    uidx = np.where(labels == UNLABELED)[1].A1.tolist()
    return points[:, uidx], points[:, pidx], points[:, nidx]

def confusion_matrix(true_labels, predicted):
    posidx = np.where(true_labels == POSITIVE)[1].A1
    negidx = np.where(true_labels == NEGATIVE)[1].A1
    m00 = m01 = m10 = m11 = 0
    nm00 = nm01 = nm10 = nm11 = 0

    if len(negidx) > 0:
        nm00 = float(np.sum(NEGATIVE == predicted[:, negidx]))
        nm01 = float(np.sum(POSITIVE == predicted[:, negidx]))
        m00 =  nm00 / len(negidx)
        m01 =  nm01 / len(negidx)

    if len(posidx) > 0:
        nm10 = float(np.sum(NEGATIVE == predicted[:, posidx]))
        nm11 = float(np.sum(POSITIVE == predicted[:, posidx]))
        m10 =  nm10 / len(posidx)
        m11 =  nm11 / len(posidx)

    return np.matrix([[m00, m01], [m10, m11]], 'float')
    #if verbose:
    #    print 'Confusion matrix:'
    #    print '-   %5.2f, %5.2f' % (100.* m00, 100.* m01)
    #    print '+  %5.2f, %5.2f' % (100.* m10, 100.* m11)
    #    print '   Total %5.2f' % (100.* (float(np.sum(true_labels == predicted)) / true_labels.shape[1]))
    #return {'mat': np.matrix([[nm00, nm01], [nm10, nm11]], 'float'), 
    #        'neg': len(negidx), 
    #        'pos': len(posidx)}


def instances_to_image(win_size, instances, min_val, max_val):
    img_arrs = []
    for i in range(instances.shape[1]):
        img_arrs.append(instance_to_image(win_size, instances[:,i], min_val, max_val).copy())
    return cv.fromarray(np.concatenate(img_arrs, 0).copy())

def instance_to_image(win_size, instance, min_val, max_val):
    winside = win_size*2+1
    patch_size = winside*winside*3
    #multipliers = [1,2,4,8,16,32]
    srange = max_val - min_val

    offset = 0
    patches = []
    num_patches = instance.shape[0] / patch_size
    #pdb.set_trace()
    #for i in range(len(multipliers)):
    for i in range(num_patches):
        s = instance[offset:offset+patch_size, 0]
        s = np.array(np.abs(np.round(((s-min_val)/srange) * 255.)), 'uint8')
        patches.append(np.reshape(s, (winside, winside, 3)))
        offset+=patch_size
    #pdb.set_trace()
    return np.concatenate(patches, 1)

def insert_folder_name(apath, folder_name):
    path, filename = pt.split(apath)
    return pt.join(pt.join(path, folder_name), filename)

def load_data_from_file2(fname, rec_param):
    data_pkl = ut.load_pickle(fname)
    image_fname = pt.join(pt.split(fname)[0], data_pkl['image'])
    intensity_image = cv.LoadImageM(image_fname)
    center_point_bl = data_pkl['touch_point']

    print 'Robot touched point cloud at point', center_point_bl.T
    distance_feature_points = None
    syn_locs_fname = pt.splitext(fname)[0] + '_synthetic_locs3d.pkl'
    if pt.isfile(syn_locs_fname):
        print 'found synthetic locations file', syn_locs_fname
        distance_feature_points = ut.load_pickle(syn_locs_fname)
        data_pkl['synthetic_locs3d'] = distance_feature_points
    else:
        print 'synthetic loc file not found', syn_locs_fname

    #return IntensityCloudData(data_pkl['points3d'], intensity_image,
    #        data_pkl['k_T_bl'], data_pkl['cal'], center_point_bl, center_point_bl, 
    #        distance_feature_points, rec_param), data_pkl
    return infea.IntensityCloudFeatureExtractor(data_pkl['points3d'], intensity_image, center_point_bl,
                                            distance_feature_points, data_pkl['k_T_bl'], 
                                            data_pkl['cal'], rec_param), data_pkl

def dataset_to_libsvm(dataset, filename):
    f = open(filename, 'w')

    for i in range(dataset.outputs.shape[1]):
        if dataset.outputs[0,i] == POSITIVE:
            f.write('+1 ')
        else:
            f.write('-1 ')
        #instance
        for j in range(dataset.inputs.shape[0]):
            f.write('%d:%f ' % (j, dataset.inputs[j, i]))
        f.write('\n')

    f.close()

def draw_labeled_points(image, dataset, pos_color=[255,102,55], neg_color=[0,184,245], scale=1.):
    #pt2d = np.column_stack(dataset.pt2d)
    pt2d = dataset.pt2d 
    for l, color in [(POSITIVE, pos_color), (NEGATIVE, neg_color)]:
        cols = np.where(l == dataset.outputs)[1]
        locs2d = np.matrix(np.round(pt2d[:, cols.A1]/scale), 'int')
        draw_points(image, locs2d, color)

def draw_points(img, img_pts, color, size=1, thickness=-1):
    for i in range(img_pts.shape[1]):
        center = tuple(np.matrix(np.round(img_pts[:,i]),'int').T.A1.tolist())
        cv.Circle(img, center, size, color, thickness)

def draw_dataset(dataset, img, scale=1., size=2, scan_id=None):
    npt2d = []
    ppt2d = []
    for i in range(dataset.inputs.shape[1]):
        if dataset.pt2d != None and dataset.pt2d[0,i] != None:
            if scan_id != None and dataset.scan_ids != None:
                if scan_id != dataset.scan_ids[0,i]:
                    continue
            if POSITIVE == dataset.outputs[0,i]:
                ppt2d.append(dataset.pt2d[:,i]/scale)
            if NEGATIVE == dataset.outputs[0,i]:
                npt2d.append(dataset.pt2d[:,i]/scale)
    if len(ppt2d) > 0:
        draw_points(img, np.column_stack(ppt2d), [0,255,0], size)
    if len(npt2d) > 0:
        draw_points(img, np.column_stack(npt2d), [0,0,255], size)

def inverse_indices(indices_exclude, num_elements):
    temp_arr = np.zeros(num_elements)+1
    temp_arr[indices_exclude] = 0
    return np.where(temp_arr)[0]

#'_features_dict.pkl'
def preprocess_scan_extract_features(raw_data_fname, ext):
    rec_params = Recognize3DParam()
    #if not kinect:
    #    feature_extractor, data_pkl = load_data_from_file(raw_data_fname, rec_params)
    #    image_fname = pt.join(pt.split(raw_data_fname)[0], data_pkl['high_res'])
    #else:
    feature_extractor, data_pkl = load_data_from_file2(raw_data_fname, rec_params)
    image_fname = pt.join(pt.split(raw_data_fname)[0], data_pkl['image'])
    print 'Image name is', image_fname
    img = cv.LoadImageM(image_fname)
    feature_cache_fname = pt.splitext(raw_data_fname)[0] + ext
    instances, points2d, points3d = feature_extractor.extract_features()
    labels = np.matrix([UNLABELED] * instances.shape[1])

    #pdb.set_trace()
    #cv.SaveImage('DATASET_IMAGES.png', instances_to_image(rec_params.win_size, instances[38:,:], 0., 1.))
    preprocessed_dict = {'instances': instances,
                         'points2d': points2d,
                         'points3d': points3d,
                         'image': image_fname,
                         'labels': labels,
                         'sizes': feature_extractor.get_sizes()}
    if data_pkl.has_key('synthetic_locs3d'):
        preprocessed_dict['synthetic_locs3d'] = data_pkl['synthetic_locs3d']

    rospy.loginfo('saving results')
    ut.save_pickle(preprocessed_dict, feature_cache_fname)
    return feature_cache_fname

def preprocess_data_in_dir(dirname, ext):
    print 'Preprocessing data in', dirname
    data_files = glob.glob(pt.join(dirname, '*dataset.pkl'))
    print 'Found %d scans' % len(data_files)

    for n in data_files:
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Loading', n
        cn = preprocess_scan_extract_features(n, ext)
        print 'Saved to', cn
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'

def make_point_exclusion_test_set(training_dataset, all_data_dir, ext):
    #pdb.set_trace()
    scan_names = glob.glob(pt.join(all_data_dir, '*' + ext))
    dset = {'inputs': [], 'outputs': [], 'pt2d': [], 'pt3d': []}
    sizes = None
    i = 0
    for sn in scan_names:
        #load scan
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Loading scan', sn
        cn = ut.load_pickle(sn)
        if sizes == None:
            sizes = cn['sizes']
        selected_pts_in_train = np.where(training_dataset.scan_ids == sn)[1]
        print '  There are %d points in scan' % cn['instances'].shape[1]

        #if there are points in training set that are in this scan, unselect them
        if len(selected_pts_in_train) > 0:
            print '  There are %d of these points in the training set.' % len(selected_pts_in_train)
            scan_ind = training_dataset.idx_in_scan[0, selected_pts_in_train].A1
            select_ind = inverse_indices(scan_ind, cn['instances'].shape[1])
            assert((len(scan_ind) + len(select_ind)) == cn['instances'].shape[1])
        else:
            select_ind = np.array(range(cn['instances'].shape[1]))
        print '  Including %d points from this dataset' % len(select_ind)
        
        for datak, dsetk in [('instances', 'inputs'), ('labels', 'outputs'),\
                  ('points2d', 'pt2d'), ('points3d', 'pt3d')]:
            dset[dsetk].append(cn[datak][:, select_ind])
        i = i + 1
        if i > 2:
            break

    return InterestPointDataset(np.column_stack(dset['inputs']),\
                                np.column_stack(dset['outputs']),\
                                None, None, None, sizes = sizes)




class SVM:
    ##
    # zero is on negative side of decision boundary
    # 0  /  1
    # -  /  +
    def __init__(self, dataset, params):
        samples = dataset.inputs.T.tolist()
        labels = dataset.outputs.T.A1.tolist()
        #pdb.set_trace()
        print 'SVM: training with params => ', params
        #pdb.set_trace()
        #dataset_to_libsvm(dataset, 'total_set_light_switch_libsvm.data')
        self.model = su.svm_train(labels, samples, params)
        self.nsamples = len(samples)

    def sv_indices(self):
        return np.where(self.model.get_support_vectors(self.nsamples))[0]

    def predict(self, instances):
        xi = instances.T.tolist()
        return su.svm_predict([0]*instances.shape[1], xi, self.model)[0]

    def distances(self, instances):
        xi = instances.T.tolist()
        dists = su.svm_predict([0]*instances.shape[1], xi, self.model)[2]
        return (np.matrix(dists)).T.A1.tolist()


class DataScale:

    def __init__(self):
        self.maxes = None
        self.mins = None
        self.ranges = None

    def scale(self, data):
        if self.maxes == None:
            self.maxes = np.max(data,1)
            self.mins = np.min(data,1)
            self.ranges = self.maxes - self.mins
            self.ranges[np.where(self.ranges < .001)[0], :] = 1.
            #pdb.set_trace()
        #pdb.set_trace()
        #pdb.set_trace()
        scaled = (((data - self.mins) / self.ranges) * 2.) - 1
        return scaled


class PCAIntensities:

    def __init__(self, intensities_index, reconstruction_std_lim, reconstruction_err_toler):
        self.intensities_mean     = None
        self.intensities_std      = None
        self.projection_basis     = None
        self.intensities_index    = intensities_index
        self.recon_err_raw        = None
        self.reconstruction_error = None
        self.pca_data             = None
        self.reconstruction_std_lim = reconstruction_std_lim
        self.reconstruction_err_toler = reconstruction_err_toler

    def calculate_pca_vectors(self, data, variance_keep):
        data_in = data[self.intensities_index:, :]
        #we already have pca vectors
        #pdb.set_trace()
        print 'PCAIntensities: data.shape', data.shape
        if self.intensities_mean != None:
            normed_data = (data_in - self.intensities_mean) / self.intensities_std
            #if not self.is_dataset_far_from_pca_subspace(self.projection_basis.T * normed_data, normed_data):
            if True:
                print 'PCAIntensities: is_dataset_far_from_pca_subspace no, dataset is not far.'
                return
            else:
                print 'PCAIntensities: is_dataset_far_from_pca_subspace yes, dataset is far. recalculating pca.'
                #pdb.set_trace()
                nsamples = min(data_in.shape[1], 3000) #2000 is the max that we can handle
                loaded_data = ut.load_pickle(self.pca_data)
                data_in  = np.column_stack((data_in, loaded_data[self.intensities_index:, :]))
                ntotal   = data_in.shape[1]
                data_in  = data_in[:, np.random.permutation(np.array(range(ntotal)))[0:nsamples]]

        self.intensities_mean = np.mean(data_in, 1)
        self.intensities_std = np.std(data_in, 1)
        self.intensities_std[np.where(self.intensities_std == 0)[0].A1, :] = 1

        data_in_shifted = data_in - self.intensities_mean
        data_in_normed = data_in_shifted / self.intensities_std
        print 'PCAIntensities.calculate_pca_vectors: Constructing PCA basis'
        #self.projection_basis = dr.pca_vectors(data_in_normed, variance_keep)[:,:50]
        #pdb.set_trace()
        self.projection_basis = dr.pca_vectors(data_in_normed, variance_keep)[:,:50]
        #self.projection_basis = dr.pca_vectors(data_in_normed, variance_keep)
        print 'PCAIntensities.calculate_pca_vectors: PCA basis size -', self.projection_basis.shape

        projected = (self.projection_basis.T * data_in_normed)
        reconstruction = self.projection_basis * projected
        self.recon_err_raw = ut.norm(data_in_normed - reconstruction)
        self.recon_mean = np.mean(self.recon_err_raw)
        self.recon_std = np.std(self.recon_err_raw)
        self.reconstruction_error = self.calc_reconstruction_errors(projected, data_in_normed)

        pca_data_name = time.strftime('%A_%m_%d_%Y_%I:%M%p') + '_pca_data.pkl'
        #pdb.set_trace()
        ut.save_pickle(data, pca_data_name)
        self.pca_data = pca_data_name

    def calc_reconstruction_errors(self, projected_instances, original_instances):
        errors = ut.norm((self.projection_basis * projected_instances) - original_instances)
        mal_dist_errors = (errors - self.recon_mean) / (self.recon_std**2)
        nhigh_errors = np.sum(np.abs(mal_dist_errors).A1 > self.reconstruction_std_lim)
        npoints = original_instances.shape[1]
        percent_error = nhigh_errors / (float(npoints))
        rospy.loginfo('calc_reconstruction_errors: %.2f (%d / %d) with high reconstruction errors' \
                % (percent_error, nhigh_errors, npoints))
        return percent_error

    def is_dataset_far_from_pca_subspace(self, projected_instances, original_instances):
        percent_error = self.calc_reconstruction_errors(projected_instances, original_instances)
        return percent_error > (self.reconstruction_error + self.reconstruction_err_toler)

    def partial_pca_project(self, instances):
        #pdb.set_trace()
        #self.intensities_mean = np.mean(data_in, 1)
        #instances1084G
        instance_mean = np.mean(instances[self.intensities_index:, :], 1) 
        #instances_in = (instances[self.intensities_index:, :] - self.intensities_mean) / self.intensities_std
        instances_in = (instances[self.intensities_index:, :] - instance_mean) / self.intensities_std
        reduced_intensities = self.projection_basis.T * instances_in
        return np.row_stack((instances[:self.intensities_index, :], reduced_intensities))

class SVMPCA_ActiveLearner:
    def __init__(self, use_pca, reconstruction_std_lim=None, 
                 reconstruction_err_toler=None, old_learner=None, pca=None):
        self.classifier = 'svm' #or 'knn'
        self.classifiers = {}
        self.n = 3.

        #self.intensities_mean = None
        #self.projection_basis = None
        self.reduced_dataset = None
        self.dataset = None

        self.use_pca = use_pca
        self.reconstruction_std_lim = reconstruction_std_lim
        self.reconstruction_err_toler = reconstruction_err_toler
        self.pca = None

        if old_learner != None:
            self.pca = old_learner.pca

        if pca != None:
            self.pca = pca

        #pdb.set_trace()
        #if old_learner != None:
        #    self.intensities_mean = old_learner.intensities_mean
        #    self.intensities_std = old_learner.intensities_std
        #    self.projection_basis = old_learner.projection_basis
        #    self.intensities_index = old_learner.intensities_index
        #    self.recon_err_raw = old_learner.recon_err_raw
        #    self.reconstruction_error = old_learner.reconstruction_error
        #    self.pca_data = old_learner.pca_data

    def partial_pca_project(self, instances):
        if self.use_pca:
            return self.pca.partial_pca_project(instances)
        else:
            return instances

    def get_closest_instances(self, instances, n=1):
        #pdb.set_trace()
        if self.use_pca:
            p_instances = np.matrix(self.partial_pca_project(instances))
        else:
            p_instances = instances
            #pdb.set_trace()
        s_instances = self.scale.scale(p_instances)
        distances = np.array(self.classifiers['svm'].distances(s_instances))
        selected_indices = np.argsort(np.abs(distances))[0:n]
        return selected_indices.tolist(), distances[selected_indices]

    def select_next_instances(self, instances, n=1):
        #pdb.set_trace()
        data_idx, data_dists = self.get_closest_instances(instances, n)
        data_dist = abs(data_dists[0])
        sv_dist   = abs(self.sv_dist[0])

        #we've converged if distance to datapoint closest to decision boundary
        #is no closer than distance to the support vectors. 
        #So we return nothing as a proposal.
        print 'SVMPCA_ActiveLearner: support vector dist %f data dist %f' % (sv_dist, data_dist)
        if sv_dist < data_dist:
            return None, None
        else:
            return data_idx, data_dists

    def select_next_instances_no_terminate(self, instances, n=1):
        data_idx, data_dists = self.get_closest_instances(instances, n)
        data_dist = abs(data_dists[0])
        sv_dist   = abs(self.sv_dist[0])
        return data_idx, data_dists, sv_dist < data_dist

    def train(self, dataset, inputs_for_scaling, svm_params, variance_keep=.95):
        #pdb.set_trace()
        #pdb.set_trace()
        #TODO: somehow generate labels for these datasets...
        self.dataset = dataset
        trainingset = dataset.inputs
        responses = dataset.outputs

        #Calculate PCA vectors (also try do detect if our PCA vectors need updating)
        if self.use_pca: #and self.projection_basis == None:
            #self.intensities_index = self.dataset.metadata[-1].extent[0]
            #rebalanced_set = self._balance_classes(trainingset, responses)
            #self._calculate_pca_vectors(rebalanced_set, variance_keep)
            #self._calculate_pca_vectors(inputs_for_scaling, variance_keep)
            if self.pca == None:
                self.pca = PCAIntensities(self.dataset.metadata[-1].extent[0], self.reconstruction_std_lim, self.reconstruction_err_toler)
            self.pca.calculate_pca_vectors(inputs_for_scaling, variance_keep)
            #self.pca.calculate_pca_vectors(rebalanced_set, variance_keep, incremental=True)

        trainingset = self.partial_pca_project(trainingset)
        inputs_for_scaling = self.partial_pca_project(inputs_for_scaling)
    
        print 'SVMPCA_ActiveLearner.train: Training classifier.'
        #train => float32 mat, each row is an example
        #responses => float32 mat, each column is a corresponding response
        trainingset = np.matrix(trainingset, dtype='float32').copy()
        responses = np.matrix(responses, dtype='float32').copy()

        self.reduced_dataset = ds.Dataset(trainingset.copy(), responses.copy())
        self.scale = DataScale()
        if inputs_for_scaling == None:
            inputs_for_scaling = self.reduced_dataset.inputs
        else:
            print 'SVMPCA_ActiveLearner.train: using large input set for setting scaling parameters'
        self.scale.scale(inputs_for_scaling)
        rescaled_inputs = self.scale.scale(self.reduced_dataset.inputs)
        self.rescaled_dataset = ds.Dataset(rescaled_inputs, self.reduced_dataset.outputs)

        #dataset_to_libsvm(self.rescaled_dataset, 'interest_point_data.libsvm')
        self.classifiers['svm'] = SVM(self.rescaled_dataset, svm_params)
        #self.classifiers['knn'] = sp.KDTree(np.array(self.rescaled_dataset.inputs.T))

        self.sv_instances = self.dataset.inputs[:, self.classifiers['svm'].sv_indices()]
        self.sv_idx, self.sv_dist = self.get_closest_instances(self.sv_instances, 1)

    def classify(self, instances):
        #pdb.set_trace()
        projected_inst = np.matrix(self.partial_pca_project(instances), dtype='float32').copy()
        scaled_inst = self.scale.scale(projected_inst)
        if self.classifier == 'knn':
            labels = self.reduced_dataset.outputs[:, \
                    self.classifiers['knn'].query(scaled_inst, self.n)[1].tolist()[0]]
            if np.sum(labels) > (self.n/2.):
                return POSITIVE
            else:
                return NEGATIVE

        elif self.classifier == 'svm':
            #pdb.set_trace()
            r = self.classifiers['svm'].predict(scaled_inst)
            return r

    def _balance_classes(self, data, labels):
        posidx = np.where(labels == POSITIVE)[1].A1
        negidx = np.where(labels == NEGATIVE)[1].A1
        diff = abs(posidx.shape[0] - negidx.shape[0])
        if diff == 0:
            return data

        rospy.loginfo('SVMPCA_ActiveLearner: _balance_classes difference in class sizes is %d' % diff)

        posex = data[:, posidx]
        negex = data[:, negidx]
        if posidx.shape[0] < negidx.shape[0]:
            rospy.loginfo('SVMPCA_ActiveLearner. _balance_classes making data from POSITIVE instances')
            sset = posex
        else:
            rospy.loginfo('SVMPCA_ActiveLearner. _balance_classes making data from NEGATIVE instances')
            sset = negex
        
        rsubset = sset[:, np.random.randint(0, sset.shape[1], diff)]
        rospy.loginfo('SVMPCA_ActiveLearner: _balance_classes created %d instances to make up for it' % rsubset.shape[1])

        return np.column_stack((posex, negex, rsubset))

class InterestPointDataset(ds.Dataset):

    def __init__(self, inputs, outputs, pt2d, pt3d, feature_extractor, scan_ids=None, idx_in_scan=None, sizes=None):
        ds.Dataset.__init__(self, inputs, outputs)
        self.pt2d = pt2d
        self.pt3d = pt3d
        self.scan_ids = scan_ids
        #if scan_ids != None:
        #    print 'scan id class', self.scan_ids.__class__
        self.idx_in_scan = idx_in_scan
        offset = 0

        #Intensity has to be the last feature
        if feature_extractor != None or sizes != None:
            if feature_extractor != None:
                sizes = feature_extractor.get_sizes
            for k in ['expected_loc', 'distance', 'fpfh', 'intensity']:
                if sizes.has_key(k):
                    start_idx = offset 
                    end_idx = offset + sizes[k]
                    self.add_attribute_descriptor(ds.AttributeDescriptor(k, (start_idx, end_idx)))
                    offset = end_idx

    def select_features(self, features):
        fset = set(features)
        selected_fea = []
        meta_data = []
        offset = 0
        for meta in self.metadata:
            m = copy.deepcopy(meta)
            if fset.issuperset([m.name]):
                # TODO FIX THIS
                #if m.name == 'distance':
                #    m.extent = [0,4]
                size_m = m.extent[1] - m.extent[0]
                selected_fea.append(self.inputs[m.extent[0]:m.extent[1], :])
                m.extent = [offset, offset+size_m]
                meta_data.append(m)
                offset = offset + size_m
        self.metadata = meta_data
        self.inputs = np.row_stack(selected_fea)

    def features_named(self, features):
        fset = set(features)
        selected_fea = []
        for meta in self.metadata:
            if fset.issuperset([meta.name]):
                selected_fea.append(self.inputs[meta.extent[0]:meta.extent[1], :])
        return np.row_stack(selected_fea)

    def add(self, features, label, pt2d, pt3d, scan_id=None, idx_in_scan=None):
        print 'added point', pt2d[0,0], pt2d[1,0]
        if np.abs(pt2d[1,0]) > 10000:
            pdb.set_trace()
        if np.abs(pt2d[0,0]) > 10000:
            pdb.set_trace()
        self.inputs = np.column_stack((self.inputs, features))
        self.outputs = np.column_stack((self.outputs, label))
        self.pt2d = np.column_stack((self.pt2d, pt2d))
        self.pt3d = np.column_stack((self.pt3d, pt3d))
        if scan_id != None:
            self.scan_ids = np.column_stack((self.scan_ids, scan_id))
        if idx_in_scan != None:
            self.idx_in_scan = np.column_stack((self.idx_in_scan, idx_in_scan))
        
    def remove(self, instance_idx):
        self.inputs = np.column_stack((self.inputs[:,0:instance_idx], self.inputs[:,instance_idx+1:]))
        self.outputs = np.column_stack((self.outputs[:,0:instance_idx], self.outputs[:,instance_idx+1:]))
        self.pt2d = np.column_stack((self.pt2d[:,0:instance_idx], self.pt2d[:,instance_idx+1:]))
        self.pt3d = np.column_stack((self.pt3d[:,0:instance_idx], self.pt3d[:,instance_idx+1:]))
        if self.scan_ids != None:
            self.scan_ids = np.column_stack((self.scan_ids[:,0:instance_idx], self.scan_ids[:,instance_idx+1:]))
        if self.idx_in_scan != None:
            self.idx_in_scan = np.column_stack((self.idx_in_scan[:,0:instance_idx], self.idx_in_scan[:,instance_idx+1:]))

    def copy(self):
        ipd = InterestPointDataset(self.inputs.copy(), self.outputs.copy(), 
                copy.copy(self.pt2d), copy.copy(self.pt3d), None)
        ipd.metadata = copy.deepcopy(self.metadata)
        return ipd

    def subset(self, indices):
        ipd = InterestPointDataset(self.inputs[:, indices].copy(), self.outputs[:, indices].copy(), 
                copy.copy(self.pt2d[:, indices]), copy.copy(self.pt3d[:, indices]), None)
        ipd.metadata = copy.deepcopy(self.metadata)
        return ipd

    @classmethod
    def add_to_dataset(cls, dataset, features, label, pt2d, pt3d, scan_id, idx_in_scan, sizes):
        if dataset == None:
            dataset = InterestPointDataset(features, label, pt2d, pt3d, None, scan_id, idx_in_scan, sizes)
        else:
            dataset.add(features, label, pt2d, pt3d, scan_id=scan_id, idx_in_scan=idx_in_scan)
        return dataset


class Recognize3DParam:

    def __init__(self):
        #Data extraction parameters
        self.grid_resolution = .01
        self.win_size = 15
        #self.win_multipliers = [1,2,4,8,16,32] for prosilica
        #self.win_multipliers = [1,2,4,8,16,32]
        self.win_multipliers = [2,4,8,16]

        self.win3d_size = .02
        self.voi_bl = [.5, .5, .5]
        self.radius = .5
        self.robot_reach_radius = 2.5
        #self.svm_params = '-s 0 -t 2 -g .0625 -c 4'
        #self.svm_params = '-s 0 -t 2 -g .0625 -c 4'
        #self.svm_params = '-s 0 -t 2 -g .4 -c 4'
        #self.svm_params = '-s 0 -t 2 -g 3.0 -c 4'
        #self.svm_params = '-s 0 -t 2 -g 5. -c 4'
        #self.svm_params = '-s 0 -t 2 -g .01 -c 4'
        #self.svm_params = '-s 0 -t 2 -g .2 -c 4'
        self.svm_params = '-s 0 -t 2 -g .5 -c .5'
        #self.svm_params = '-s 0 -t 2 -g 100 -c .5'

        #sampling parameters
        #self.n_samples = 5000
        #self.n_samples = 5000
        self.n_samples = 1500
        self.uni_mix = .3
        self.max_points_per_site = 5

        self.uncertainty_x = .1
        self.uncertainty_y = .1
        self.uncertainty_z = .1
        #print "Uncertainty is:", self.uncertainty

        #variance
        self.variance_keep = .99
        self.reconstruction_std_lim = 1.
        self.reconstruction_err_toler = .05


class ImagePublisher:

    def __init__(self, channel, cal=None):
        from cv_bridge import CvBridge, CvBridgeError
        from sensor_msgs.msg import Image

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(channel + '/image', Image)
        if cal != None:
            self.image_cal_pub = rospy.Publisher(channel + '/camera_info', sm.CameraInfo)
            self.cal = cal
        else:
            self.image_cal_pub = None

    def publish(self, cv_image): 
        self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
        if self.image_cal_pub != None:
            self.image_cal_pub.publish(self.cal.msg)

def find_max_in_density(locs2d):
    import scipy.stats as sct
    import scipy
    x = locs2d[0,:]
    y = locs2d[1,:]

    max_x = np.max(x)
    max_y = np.max(y)
    min_x = np.min(x)
    min_y = np.min(y)

    X, Y      = scipy.mgrid[min_x:max_x:500j, min_y:max_y:500j]
    positions = scipy.c_[X.ravel(), Y.ravel()]
    values    = scipy.c_[locs2d[0,:].A1, locs2d[1,:].A1]
    kernel    = sct.kde.gaussian_kde(values.T)
    Z         = np.reshape(kernel(positions.T).T, X.T.shape)
    loc_max   = np.argmax(Z)
    loc2d_max = np.matrix([X.flatten()[loc_max], Y.flatten()[loc_max]]).T
    
    #pylab.imshow(np.rot90(Z, 2), cmap=pylab.cm.gist_earth_r, extent=[0,img.cols, 0, img.rows])
    #pylab.imshow(img, cmap=pylab.cm.gist_earth_r, extent=[0,img.cols, 0, img.rows])
    #pylab.plot(locs2d[0,:].A1, locs2d[1,:].A1, 'k.', markersize=2)
    #pylab.plot([loc2d_max[0,0]], [loc2d_max[1,0]], 'gx')
    #cv.ShowImage("Positive Density", 255 * (np.rot90(Z)/np.max(Z)))
    #pdb.set_trace()

    return loc2d_max, Z

class ScanLabeler:

    def __init__(self, dirname, ext, scan_to_train_on, seed_dset, features_to_use):
        self.scan_dir_name = dirname
        self.scan_names = glob.glob(pt.join(dirname, '*' + ext))
        self.scan_names.append(self.scan_names.pop(0))
        self.feature_file_ext = ext
        self.features_to_use = features_to_use

        self.current_scan_pred = None
        self.current_scan = None
        self.cdisp = None

        self.scan_idx = self.search_dataset_id(scan_to_train_on)
        if self.scan_idx == None:
            print 'ERROR: %s not found in dir %s' % (scan_to_train_on, dirname)
            self.scan_idx = 0

        self.scale = 1/3.
        #self.scale = 1.
        self.mode = 'GROUND_TRUTH'
        if seed_dset == None:
            self.training_sname = pt.splitext(self.scan_names[self.scan_idx])[0] + '_seed.pkl'
        else:
            self.training_sname = seed_dset
        print 'Using seed dataset name', self.training_sname
        self.frame_number = 0

        self.rec_params = Recognize3DParam()
        self.learner = None
        self.dataset = None #Training dataset

        cv.NamedWindow('Scan', cv.CV_WINDOW_AUTOSIZE)
        cv.NamedWindow('Positive Density', cv.CV_WINDOW_AUTOSIZE)
        self.load_scan(self.scan_names[self.scan_idx])
        cv.SetMouseCallback('Scan', self.mouse_cb, None)


    def search_dataset_id(self, scan_name):
        scan_idx = None
        for i, n in enumerate(self.scan_names):
            if n == scan_name:
                scan_idx = i
        return scan_idx


    def classify_current_scan(self):
        if self.learner != None:
            sdset = self.select_features(self.current_scan['instances'], 
                    self.features_to_use, self.current_scan['sizes'])
            results = np.matrix(self.learner.classify(sdset))
            self.current_scan_pred = InterestPointDataset(sdset, results,
                                        self.current_scan['points2d'], self.current_scan['points3d'], None)
            if np.any(self.current_scan['labels'] == UNLABELED):
                return
        
    def load_scan(self, fname):
        print 'Loading scan', fname
        self.cdisp = {}
        self.current_scan = ut.load_pickle(fname)
        #image_fname = pt.join(pt.split(fname)[0], self.current_scan['image'])
        img = cv.LoadImageM(self.current_scan['image'])
        self.cdisp['tree'] = sp.KDTree(np.array(self.current_scan['points2d'].T))
        self.cdisp['cv'] = cv.CreateMat(int(round(img.rows*self.scale)), 
                                        int(round(img.cols*self.scale)), cv.CV_8UC3)
        cv.Resize(img, self.cdisp['cv'])
        print 'loaded!'
        self.classify_current_scan()
        self.draw()

    def save_current(self):
        print 'saving current scan\'s data'
        fname = self.scan_names[self.scan_idx]
        print 'WARNING: THIS SHOULD NOT BE CALLED.'
        ut.save_pickle(self.current_scan, fname)
        print 'done saving!'

    def load_next(self):
        self.scan_idx = max(min(self.scan_idx+1, len(self.scan_names)-1), 0)
        self.load_scan(self.scan_names[self.scan_idx])

    def load_prev(self):
        self.scan_idx = min(max(self.scan_idx-1, 0), len(self.scan_names)-1)
        self.load_scan(self.scan_names[self.scan_idx])

    def draw(self, save_postf=""):
        img = cv.CloneMat(self.cdisp['cv'])
        #density_img = cv.fromarray(np.zeros((img.rows, img.cols)))

        #Draw ground truth labels
        #pdb.set_trace()
        pidx = np.where(self.current_scan['labels'] == POSITIVE)[1].A1.tolist()
        nidx = np.where(self.current_scan['labels'] == NEGATIVE)[1].A1.tolist()
        uidx = np.where(self.current_scan['labels'] == UNLABELED)[1].A1.tolist()

        if len(pidx) > 0:
            ppoints = self.current_scan['points2d'][:, pidx]
            draw_points(img, ppoints * self.scale, [0,255,0], 2, 1)

        if len(nidx) > 0:
            npoints = self.current_scan['points2d'][:, nidx]
            draw_points(img, npoints * self.scale, [0,0,255], 2, 1)

        if len(uidx) > 0:
            upoints = self.current_scan['points2d'][:, uidx]
            draw_points(img, upoints * self.scale, [255,255,255], 2, 1)

        #Draw training set
        if self.dataset != None:
            #pdb.set_trace()
            draw_dataset(self.dataset, img, 1./self.scale, 4, scan_id=self.scan_names[self.scan_idx])

        if self.current_scan_pred != None:
            draw_labeled_points(img, self.current_scan_pred, scale=1./self.scale)
            locs2d = self.current_scan_pred.pt2d[:, (np.where(POSITIVE == self.current_scan_pred.outputs)[1]).A1]

        #    loc_max, Z = find_max_in_density(locs2d)
        #    print 'Max location is', loc_max.T
        #    cv.ShowImage("Positive Density", 255 * (np.rot90(Z)/np.max(Z)))

            #X, Y = scipy.mgrid[0:img.cols:500j, 0:img.rows:500j]
            #positions = scipy.c_[X.ravel(), Y.ravel()]
            #values = scipy.c_[locs2d[0,:].A1, locs2d[1,:].A1]
            #kernel = sct.kde.gaussian_kde(values.T)
            #Z = np.reshape(kernel(positions.T).T, X.T.shape)
            #loc_max = np.argmax(Z)
            #loc2d_max = np.matrix([X.flatten()[loc_max], Y.flatten()[loc_max]]).T
            ##pdb.set_trace()

            #pylab.imshow(np.rot90(Z, 2), cmap=pylab.cm.gist_earth_r, extent=[0,img.cols, 0, img.rows])
            ##pylab.imshow(img, cmap=pylab.cm.gist_earth_r, extent=[0,img.cols, 0, img.rows])
            #pylab.plot(locs2d[0,:].A1, locs2d[1,:].A1, 'k.', markersize=2)
            #pylab.plot([loc2d_max[0,0]], [loc2d_max[1,0]], 'gx')
            #pdb.set_trace()
            #pylab.show()
            #draw_labeled_points(density_img, self.current_scan_pred, 
            #        scale=1./self.scale, pos_color=[255, 255, 255], neg_color=[0,0,0])
            #cv.Smooth(density_img, density_img, cv.CV_GAUSSIAN, 151, 151, 2)

        #Display
        #cv.ShowImage('Positive Density', density_img)
        cv.ShowImage('Scan', img)
        path = pt.splitext(insert_folder_name(self.scan_names[self.scan_idx], save_postf))[0]
        img_name = path + ('_active_learn%d.png' % (self.frame_number))
        cv.SaveImage(img_name, img)
        self.frame_number = self.frame_number + 1
        print '- refreshed display - %s' % img_name

    def step(self):
        k = cv.WaitKey(33)
        if k == ord('n'):
            print 'Loading next scan..'
            self.load_next()

        if k == ord('p'):
            print 'loading previous scan..'
            self.load_prev()

        if k == ord('0'):
            self.current_scan['labels'][0, :] = NEGATIVE
            self.draw()
            print 'Labeling everything as negative!'

        if k == ord('`'):
            if self.mode == 'TRAINING_SET':
                self.mode = 'GROUND_TRUTH'
            else:
                self.mode = 'TRAINING_SET'
            print 'ScanLabeler GUI mode changed to', self.mode

        if k == ord('c'):
            self.save_current()
            
        if k == ord('s'):
            if self.dataset != None:
                #pdb.set_trace()
                ut.save_pickle(self.dataset, self.training_sname)
                print 'saved to', self.training_sname

        if k == ord('l'):
            if self.dataset == None and pt.isfile(self.training_sname):
                print 'loading training set from disk'
                self.dataset = ut.load_pickle(self.training_sname)
                print 'loaded', self.training_sname
                self.draw()

        if k == ord('t'):
            if self.dataset != None:
                print 'Making test set.'
                curr_test_set = make_point_exclusion_test_set(self.dataset, self.scan_dir_name)
                print 'Test set has %d instances' % curr_test_set.inputs.shape[1]
                fname = raw_input('Enter test set name to save to: ')
                print 'Saving to', fname
                ut.save_pickle(curr_test_set, fname)
                print 'Saved!'

        if k == ord('r'):
            print "Training"
            #pdb.set_trace()
            self.train(self.select_features(self.current_scan['instances'], 
                self.features_to_use, self.current_scan['sizes']))
            self.classify_current_scan()
            self.draw()

        if k == ord(' '):
            if self.learner != None:
                print "selecting.."
                #pdb.set_trace()
                dset = self.select_features(self.current_scan['instances'], 
                        self.features_to_use, self.current_scan['sizes'])
                #self.current_scan['instances']
                selected_idx, selected_dist = self.learner.select_next_instances(dset)
                if selected_idx != None:
                    self.add_to_training_set(selected_idx)
                    #self.train(self.train(self.current_scan['instances']))
                    self.train(self.select_features(self.current_scan['instances'], 
                        self.features_to_use, self.current_scan['sizes']))
                    self.classify_current_scan()
                    self.draw()
                else:
                    print '======================='
                    print '>> LEARNER CONVERGED <<'
                    print '======================='

        if k == ord('z'):
            #d = self.dataset
            #d1 = InterestPointDataset(self.current_scan['instances'], self.current_scan['labels'],
            #        self.current_scan['points2d'], self.current_scan['points3d'], 
            #        None, sizes=self.current_scan['sizes'])
            # self.load_next()
            # d1.add(self.current_scan['instances'], self.current_scan['labels'],
            #         self.current_scan['points2d'], self.current_scan['points3d'])

            #self.dataset = d1
            #self.train()
            print 'generating libsvm dataset'
            dfname = raw_input('enter dataset filename: ')
            dataset_to_libsvm(self.learner.rescaled_dataset, dfname)
            #self.dataset = d
            print 'done'

    #def instances_to_image(self, instances, min_val, max_val):
    #    img_arrs = []
    #    for i in range(instances.shape[1]):
    #        img_arrs.append(self.instance_to_image(instances[:,i], min_val, max_val).copy())
    #    return cv.fromarray(np.concatenate(img_arrs, 0).copy())

    #def instance_to_image(self, instance, min_val, max_val):
    #    winside = self.rec_params.win_size * 2 + 1
    #    patch_size = winside*winside*3
    #    multipliers = [1,2,4,8,16,32]
    #    srange = max_val - min_val

    #    offset = 0
    #    patches = []
    #    for i in range(len(multipliers)):
    #        s = instance[offset:offset+patch_size, 0]
    #        s = np.array(np.abs(np.round(((s-min_val)/srange) * 255.)), 'uint8')
    #        patches.append(np.reshape(s, (winside, winside, 3)))
    #    return np.concatenate(patches, 1)

    def select_features(self, instances, features_to_use, sizes):
        offset = 0
        fset = set(features_to_use)
        selected_fea = []
        for k in ['expected_loc', 'distance', 'fpfh', 'intensity']:
            if not sizes.has_key(k):
                continue
            start_idx = offset 
            end_idx = offset + sizes[k]

            if fset.issuperset([k]):
                selected_fea.append(instances[start_idx:end_idx, :])
            offset = end_idx

        #pdb.set_trace()
        return np.row_stack(selected_fea)

    def automatic_label(self):
        self.dataset = ut.load_pickle(self.training_sname)
        fea = self.select_features(self.current_scan['instances'], ['distance'], self.current_scan['sizes'])
        #pdb.set_trace()
        self.train(fea)
        #for name in ['4_20_2011/down/Monday_04_18_2011_11_20PM_interest_point_dataset_features_df2_dict.pkl']:
        for idx, name in enumerate(self.scan_names):
            self.scan_idx = idx
            self.load_scan(name)
            self.classify_current_scan()
            self.draw()
            #pdb.set_trace()
            self.current_scan['labels'][0,:] = self.current_scan_pred.outputs[0,:]
            self.draw()
            self.save_current()

        #for k in ['expected_loc', 'distance', 'fpfh', 'intensity']:


    def test_feature_perf(self, scans_to_train_on, features_to_use, exp_name, use_pca=None):
        #Make folder for results
        BASE_FILE_NAME = pt.splitext(insert_folder_name(self.scan_names[self.scan_idx], exp_name))[0] #pt.splitext(self.scan_names[self.scan_idx])[0]
        try:
            os.mkdir(pt.split(BASE_FILE_NAME)[0])
        except OSError, e:
            if e.errno != 17:
                pdb.set_trace()
                raise e

        #Decide whether to use PCA
        if use_pca == None:
            if set(features_to_use).issuperset(['intensity']):
                use_pca = True
            else:
                use_pca = False

        if not pt.isfile(self.training_sname):
            print 'test_feature_perf: Training file', self.training_sname, 'not found! exiting.'
            return

        #Construct new dataset
        feature_sizes = self.current_scan['sizes']
        nsizes = {}
        for f in features_to_use:
            nsizes[f] = feature_sizes[f]

        dset_trained_on = []
        for sname in scans_to_train_on:
            scan_idx = self.search_dataset_id(sname)
            if scan_idx == None:
                print 'ERROR: %s not found in dir given' % (sname)
            dset_trained_on.append(scan_idx)
            self.load_scan(self.scan_names[scan_idx])
            rcurrent_scan = self.select_features(self.current_scan['instances'], features_to_use, feature_sizes)
            scan_ids = np.matrix([sname]*rcurrent_scan.shape[1])
            idx_in_scan = np.matrix(range(rcurrent_scan.shape[1]))
            if self.dataset == None:
                self.dataset = InterestPointDataset(rcurrent_scan, self.current_scan['labels'], 
                                    self.current_scan['points2d'], self.current_scan['points3d'], 
                                    None, scan_ids=scan_ids, idx_in_scan=idx_in_scan, sizes=nsizes)
            else:
                self.dataset.add(rcurrent_scan, self.current_scan['labels'], self.current_scan['points2d'],
                                 self.current_scan['points3d'], scan_id=scan_ids, idx_in_scan=idx_in_scan)

        self.train(self.dataset.inputs, use_pca)
        #pdb.set_trace()
        for d in dset_trained_on:
            self.load_scan(self.scan_names[d])
            self.scan_idx = d
            self.classify_current_scan()
            self.draw(exp_name)

        k = cv.WaitKey(33)

        print '>>>> Training set peformance'
        _, train_conf = self.evaluate_learner(self.dataset.inputs, self.dataset.outputs)
        train_set_statistics = []
        train_set_statistics.append({'conf': train_conf, 
                                     'size': self.dataset.inputs.shape[1]})

        print '>>>> Performance on unseen scans'
        print 'Loading scans we\'ll be testing on'
        #all_scans_except_current = []
        indices_of_other_scans = inverse_indices(dset_trained_on, len(self.scan_names))
        print 'Trained on', dset_trained_on
        print 'Testing with', indices_of_other_scans

        self.dataset=None
        conf_unseen = []
        for i in indices_of_other_scans:
            self.load_scan(self.scan_names[i])
            self.scan_idx = i
            self.classify_current_scan()
            self.draw(exp_name)
            k = cv.WaitKey(33)
            sdset = self.select_features(self.current_scan['instances'], self.features_to_use, self.current_scan['sizes'])
            _, conf = self.evaluate_learner(sdset, self.current_scan['labels'])
            conf_unseen.append({'name': self.scan_names[i], 'conf': conf})
        perf_on_other_scans = []
        perf_on_other_scans.append(conf_unseen)

        #Save training results
        training_results_name = BASE_FILE_NAME + 'active_train_iter_results.pkl'
        ut.save_pickle({'train_set_statistics': train_set_statistics,
                        #'current_scan_statistics': current_scan_statistics,
                        'perf_on_other_scans': perf_on_other_scans}, training_results_name)
        print 'Saved training results to', training_results_name


    def generate_dataset_for_hyperparameter_grid_search(self, features_to_use):
        feature_sizes = self.current_scan['sizes']

        for sname in self.scan_names:
            print 'generate_dataset_for_hyperparameter_grid_search: loading %s' % sname
            self.load_scan(sname)
            rcurrent_scan = self.select_features(self.current_scan['instances'], features_to_use, feature_sizes)
            scan_ids = np.matrix([sname]*rcurrent_scan.shape[1])
            idx_in_scan = np.matrix(range(rcurrent_scan.shape[1]))
            if self.dataset == None:
                self.dataset = InterestPointDataset(rcurrent_scan, self.current_scan['labels'], 
                                    self.current_scan['points2d'], self.current_scan['points3d'], 
                                    None, scan_ids=scan_ids, idx_in_scan=idx_in_scan, sizes=nsizes)
            else:
                self.dataset.add(rcurrent_scan, self.current_scan['labels'], self.current_scan['points2d'],
                                 self.current_scan['points3d'], scan_id=scan_ids, idx_in_scan=idx_in_scan)

        self.train(self.dataset.inputs, True)


    def construct_learner(self, seed_set, prev_pca, g, c, ratio=None):
        features_used = ['intensity']
        learner = SVMPCA_ActiveLearner(use_pca=True, 
                        reconstruction_std_lim = self.rec_params.reconstruction_std_lim, 
                        reconstruction_err_toler = self.rec_params.reconstruction_err_toler,
                        old_learner=self.learner, pca=prev_pca)
        if ratio == None:
            nneg = np.sum(seed_set.outputs == NEGATIVE) #
            npos = np.sum(seed_set.outputs == POSITIVE)
            neg_to_pos_ratio = float(nneg)/float(npos)
        else:
            neg_to_pos_ratio = ratio
        weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
        #pdb.set_trace()
        inputs_for_pca = self.select_features(self.current_scan['instances'], features_used, self.current_scan['sizes'])
        svm_params = '-s 0 -t 2 -g %.6f -c %.2f' % (g, c)
        learner.train(seed_set, 
                      #self.current_scan['instances'],
                      inputs_for_pca,
                      svm_params + weight_balance,
                      self.rec_params.variance_keep)
        return learner

    def evaluate_over_datasets(self, test_idx, learner, exp_name):
        for i in test_idx:
            self.learner = learner
            name = self.scan_names[i]
            self.load_scan(name)
            self.classify_current_scan()
            self.draw(exp_name)

    def active_learn_test3(self):
        #pdb.set_trace()
        #features_used = ['expected_loc', 'fpfh', 'intensity']
        features_used = ['intensity']
        exp_name = 'fixed_intensity_bug_filtered_pca50_window15'
        path = pt.split(insert_folder_name(self.scan_names[self.scan_idx], exp_name))[0]
        g = .5
        c = .5
        #pdb.set_trace()
        try:
            os.mkdir(path)
        except OSError, e:
            pass

        train_idx = range(16)
        test_idx  = range(20,24)
        #test_idx  = range(8,9)

        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Training initial learning from seed dataset'
        #Load seed dataset from the first scan, train initial SVM
        print 'using', self.training_sname
        seed_set = ut.load_pickle(self.training_sname)
        metaintensity =  seed_set.metadata[1]
        sizes = {'intensity': metaintensity.extent[1] - metaintensity.extent[0]}
        seed_set.metadata = [metaintensity]
        metaintensity.extent = [0, sizes['intensity']]

        #self.load_scan(self.scan_names[0])
        #Train learner
        #pca_fname = 'pca_active_learn_test2.pkl'
        #if pt.isfile(pca_fname):
        #    prev_pca = ut.load_pickle(pca_fname)
        #else:
        #    prev_pca = None

        #learner = self.construct_learner(seed_set, prev_pca, g, c)
        #prev_pca = learner.pca
        #ut.save_pickle(prev_pca, pca_fname)
        test_inputs = []
        test_outputs = []
        print 'Loading test sets'
        for i in test_idx:
            name = self.scan_names[i]
            print 'loading', name
            eval_data_set = ut.load_pickle(name)
            test_inputs.append(self.select_features(eval_data_set['instances'], features_used, eval_data_set['sizes']))
            test_outputs.append(eval_data_set['labels'])
            #pdb.set_trace()
        test_inputs  = np.column_stack(test_inputs)
        test_outputs = np.column_stack(test_outputs)

        #Active learning train phase
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Learning phase'
        conf_mats = []
        #current_idx = 0
        #number_of_converged_scans = 0
        #iteration_number = 0
        for i in train_idx[0:5]:
            self.scan_idx = i
            self.load_scan(name)
            cur_instances = self.select_features(self.current_scan['instances'], 
                    features_used, self.current_scan['sizes'])
            seed_set = InterestPointDataset.add_to_dataset(seed_set, 
                            cur_instances[:, :],
                            self.current_scan['labels'][:, :],
                            self.current_scan['points2d'][:, :],
                            self.current_scan['points3d'][:, :],
                            i, None, #np.matrix(range(self.current_scan['instances'].shape[1])),
                            sizes=sizes)
            #learner = self.construct_learner(seed_set, prev_pca, g, c)
            #prev_pca = learner.pca

            #predictions = np.matrix(learner.classify(test_inputs))
            #conf_mat = confusion_matrix(test_outputs, predictions)
            #conf_mats.append(conf_mat)
            #pdb.set_trace()
            #ut.save_pickle(conf_mats, exp_name + '_confusion_mats.pkl')
            #ut.save_pickle(seed_set, exp_name + '_seed_set.pkl')
            #print '---------------------------------'
            ##print iteration_number
            #print 'scan idx', i
            #print 'results'
            #print conf_mat
            #print conf_mat[0,0] + conf_mat[1,1]

        #pdb.set_trace()
        learner = self.construct_learner(seed_set, None, g, c)
        #pdb.set_trace()
        cv.SaveImage('pca_filtered_%s.png' % exp_name,\
                     instances_to_image(\
                        self.rec_params.win_size,\
                        learner.pca.projection_basis,\
                            np.min(learner.pca.projection_basis),\
                            np.max(learner.pca.projection_basis)))
        prev_pca = learner.pca
        self.evaluate_over_datasets(test_idx, learner, exp_name)

        pdb.set_trace()
        print confusion_matrix(test_outputs, np.matrix(self.construct_learner(seed_set, prev_pca, g, c).classify(test_inputs)))
        # self.evaluate_learner(test_idx, self.construct_learner(seed_set, prev_pca, g, c), exp_name)
        #pdb.set_trace()

        #for gv in [.1,.5, 1,2,3,4,5,10,20,30,40,50,100,200]:
        #    learner = self.construct_learner(seed_set, prev_pca, gv, c)
        #    #learner.classify(test_set.inputs)
        #    #conf_mat = r3d.confusion_matrix(test_set.outputs, predictions)
        #    #evaluate performance
        #    predictions = np.matrix(learner.classify(test_inputs))
        #    conf_mat = confusion_matrix(test_outputs, predictions)
        #    conf_mats.append(conf_mat)
        #    print '---------------------------------'
        #    print 'results'
        #    print conf_mat
        #    print conf_mat[0,0] + conf_mat[1,1]

        ut.save_pickle(conf_mats, exp_name + '_confusion_mats.pkl')



    #This time we train the same way that the robot runs
    #train each scan until convergence
    #run until we have seen 4 scans that don't need training
    #evaluate using all unseen scans
    def active_learn_test2(self):
        #pdb.set_trace()
        #features_used = ['expected_loc', 'fpfh', 'intensity']
        features_used = ['intensity']
        exp_name = 'autolearn_g05_c05_max5_pca50_fast_feature_patch15_0'
        path = pt.split(insert_folder_name(self.scan_names[self.scan_idx], exp_name))[0]
        #pdb.set_trace()
        try:
            os.mkdir(path)
        except OSError, e:
            pass

        #train_idx = range(16)
        train_idx = np.random.permutation(range(16)).tolist()
        #test_idx  = range(16,17)
        test_idx  = range(18,24)

        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Training initial learning from seed dataset'
        #Load seed dataset from the first scan, train initial SVM
        print 'using', self.training_sname
        seed_set = ut.load_pickle(self.training_sname)
        #pdb.set_trace()
        metaintensity =  seed_set.metadata[1]
        sizes = {'intensity': metaintensity.extent[1] - metaintensity.extent[0]}
        seed_set.metadata = [metaintensity]
        metaintensity.extent = [0, sizes['intensity']]
        #self.load_scan(self.scan_names[0])

        #Train learner
        pca_fname = 'pca_active_learn_test2.pkl'
        if pt.isfile(pca_fname):
            prev_pca = ut.load_pickle(pca_fname)
        else:
            prev_pca = None
        learner = SVMPCA_ActiveLearner(use_pca=True, 
                        reconstruction_std_lim = self.rec_params.reconstruction_std_lim, 
                        reconstruction_err_toler = self.rec_params.reconstruction_err_toler,
                        old_learner=self.learner, pca=prev_pca)
        nneg = np.sum(seed_set.outputs == NEGATIVE) #
        npos = np.sum(seed_set.outputs == POSITIVE)
        neg_to_pos_ratio = float(nneg)/float(npos)
        weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
        inputs_for_pca = self.select_features(self.current_scan['instances'], 
                features_used, self.current_scan['sizes'])
        learner.train(seed_set, 
                      inputs_for_pca,
                      self.rec_params.svm_params + weight_balance,
                      self.rec_params.variance_keep)
        prev_pca = learner.pca
        ut.save_pickle(prev_pca, pca_fname)

        test_inputs = []
        test_outputs = []
        print 'Loading test sets'
        for i in test_idx:
            name = self.scan_names[i]
            print 'loading', name
            eval_data_set = ut.load_pickle(name)
            test_inputs.append(self.select_features(eval_data_set['instances'], 
                features_used, eval_data_set['sizes']))
            test_outputs.append(eval_data_set['labels'])
            #pdb.set_trace()
        test_inputs  = np.column_stack(test_inputs)
        test_outputs = np.column_stack(test_outputs)

        #Active learning train phase
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Learning phase'
        conf_mats = []
        current_idx = 0
        number_of_converged_scans = 0
        iteration_number = 0

        for i in train_idx:
            #break
            name = self.scan_names[i]
            current_idx = i

            print '!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            print 'loading scan %s' % name
            self.scan_idx = current_idx
            self.load_scan(name)
            self.draw(exp_name)
            cur_instances = self.select_features(self.current_scan['instances'], 
                    features_used, self.current_scan['sizes'])
            converged = False

            indices_added = []
            while True:
                if len(indices_added) > 5:
                    #pdb.set_trace()
                    break

                iteration_number += 1
                remaining_pt_indices = inverse_indices(indices_added, cur_instances.shape[1])
                remaining_instances = cur_instances[:, remaining_pt_indices]
                ridx, selected_dist, converged = learner.select_next_instances_no_terminate(remaining_instances)
                if converged:
                    print 'Converged on scan.'
                    break

                #add to dataset 
                selected_idx = remaining_pt_indices[ridx[0]]
                indices_added.append(selected_idx)
                seed_set = InterestPointDataset.add_to_dataset(seed_set, 
                        cur_instances[:, selected_idx],
                        self.current_scan['labels'][:, selected_idx],
                        self.current_scan['points2d'][:, selected_idx],
                        self.current_scan['points3d'][:, selected_idx],
                        current_idx, selected_idx,
                        sizes=sizes)

                #retrain 
                learner = SVMPCA_ActiveLearner(use_pca=True, 
                                reconstruction_std_lim=self.rec_params.reconstruction_std_lim, 
                                reconstruction_err_toler=self.rec_params.reconstruction_err_toler,
                                old_learner=self.learner, pca=prev_pca)
                nneg = np.sum(seed_set.outputs == NEGATIVE) #
                npos = np.sum(seed_set.outputs == POSITIVE)
                neg_to_pos_ratio = float(nneg)/float(npos)
                weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
                inputs_for_pca = self.select_features(self.current_scan['instances'], features_used, self.current_scan['sizes'])
                learner.train(seed_set, 
                              inputs_for_pca,
                              self.rec_params.svm_params + weight_balance,
                              self.rec_params.variance_keep)
                prev_pca = learner.pca
                ut.save_pickle(prev_pca, pca_fname)

                #evaluate performance
                predictions = np.matrix(learner.classify(test_inputs))
                conf_mat = confusion_matrix(test_outputs, predictions)
                conf_mats.append(conf_mat)
                ut.save_pickle(conf_mats, exp_name + '_confusion_mats.pkl')
                ut.save_pickle(seed_set, exp_name + '_seed_set.pkl')
                print '---------------------------------'
                print iteration_number
                print 'scan idx', i
                print 'results'
                print conf_mat
                print conf_mat[0,0] + conf_mat[1,1]

                self.learner = learner
                self.classify_current_scan()
                self.draw(exp_name)

            if len(indices_added) == 0:
                #pdb.set_trace()
                print 'OUTER LOOP CONVERGED', number_of_converged_scans
                number_of_converged_scans += 1

        for i in test_idx:
            self.learner = learner
            name = self.scan_names[i]
            self.load_scan(name)
            self.classify_current_scan()
            self.draw(exp_name)

        #evaluate performance
        predictions = np.matrix(learner.classify(test_inputs))
        conf_mat = confusion_matrix(test_outputs, predictions)
        conf_mats.append(conf_mat)
        ut.save_pickle(conf_mats, exp_name + '_confusion_mats.pkl')
        print '---------------------------------'
        print 'results'
        print conf_mat
        print conf_mat[0,0] + conf_mat[1,1]

        self.learner = learner
        self.dataset = seed_set
        print 'DO NOT SAVE'
        print 'DO NOT SAVE'
        print 'DO NOT SAVE'
        print 'DO NOT SAVE'
        print 'DO NOT SAVE'
        print 'DO NOT SAVE'
        print 'DO NOT SAVE'
        #self.current_scan['sizes'] = sizes
        self.run_gui()
        pdb.set_trace()


    def active_learn_test(self, features_to_use, exp_name, use_pca=None, run_till_end=False):
        BASE_FILE_NAME = pt.splitext(insert_folder_name(self.scan_names[self.scan_idx], exp_name))[0] #pt.splitext(self.scan_names[self.scan_idx])[0]
        try:
            os.mkdir(pt.split(BASE_FILE_NAME)[0])
        except OSError, e:
            if e.errno != 17:
                pdb.set_trace()
                raise e

        # Run learner until convergence on initial scan
        if use_pca == None:
            if set(features_to_use).issuperset(['intensity']):
                use_pca = True
            else:
                use_pca = False

        if not pt.isfile(self.training_sname):
            print 'active_learn_test: Training file', self.training_sname, 'not found! exiting.'
            return

        feature_sizes = self.current_scan['sizes']
        print 'active_learn_test: Using features', features_to_use
        print 'active_learn_test: feature sizes'
        for k in ['expected_loc', 'distance', 'fpfh', 'intensity']:
            if feature_sizes.has_key(k):
                print k, feature_sizes[k]

        total_dim = 0
        for f in features_to_use:
            total_dim += feature_sizes[f]

        print 'active_learn_test: total feature size should be', total_dim

        #Select only the features we will use
        rcurrent_scan = self.select_features(self.current_scan['instances'], features_to_use, feature_sizes)

        #Load the seed dataset to initialize active learner with
        self.dataset = ut.load_pickle(self.training_sname)
        print 'active_learn_test: loaded seed training file', self.training_sname
        self.dataset.select_features(features_to_use)

        self.train(rcurrent_scan, use_pca)
        self.classify_current_scan()
        self.draw(exp_name)
        k = cv.WaitKey(33)

        #Load all the scans we'll be testing on
        print 'Loading scans we\'ll be testing on'
        print 'TRAINING ON SCAN NAMED', self.scan_names[self.scan_idx]
        all_scans_except_current = []
        indices_of_other_scans = inverse_indices(self.scan_idx, len(self.scan_names))
        for i in indices_of_other_scans:
            sn = self.scan_names[i]
            print '>>', sn
            scan_dict = ut.load_pickle(sn)
            reduced_sd = {}
            reduced_sd['instances'] = self.select_features(scan_dict['instances'], features_to_use, feature_sizes)
            reduced_sd['labels'] = scan_dict['labels']
            reduced_sd['name'] = sn
            all_scans_except_current.append(reduced_sd)

        i = 0
        converged = False
        train_set_statistics = []
        current_scan_statistics  = []
        perf_on_other_scans = []
        converged_at_iter = None
        should_run = True

        #Run active learner until convergence
        while should_run:
            if self.learner.pca.projection_basis != None:
                cv.SaveImage(BASE_FILE_NAME + ('_iter_%d_basis.png' % i),\
                             instances_to_image(\
                                self.rec_params.win_size,\
                                self.learner.pca.projection_basis,\
                                np.min(self.learner.pca.projection_basis),\
                                np.max(self.learner.pca.projection_basis)))
                cv.SaveImage(BASE_FILE_NAME + ('_intensity_set_%d.png' % i),\
                             instances_to_image(self.rec_params.win_size, self.dataset.features_named(['intensity']),\
                                                0., 1.))

            self.classify_current_scan()
            self.draw(exp_name)
            k = cv.WaitKey(33)

            print '>>>> Train set peformance'
            _, train_conf = self.evaluate_learner(self.dataset.inputs, self.dataset.outputs)
            train_set_statistics.append({'conf': train_conf, 
                                         'size': self.dataset.inputs.shape[1]})

            print '>>>> Performance on current scan (correlated test set)'
            remaining_pt_indices = inverse_indices(self.dataset.idx_in_scan, rcurrent_scan.shape[1])
            remaining_instances = rcurrent_scan[:, remaining_pt_indices]
            remaining_outputs = self.current_scan['labels'][:, remaining_pt_indices]
            #remaining_outputs = rcurrent_scan[:, remaining_pt_indices]
            #pdb.set_trace()
            _, rem_conf  = self.evaluate_learner(remaining_instances, remaining_outputs)
            current_scan_statistics.append({'conf': rem_conf})

            print '>>>> Performance on unseen scans (test set)'
            conf_unseen = []
            for scan in all_scans_except_current:
                _, conf = self.evaluate_learner(scan['instances'], scan['labels'], verbose=False)
                conf_unseen.append({'name': scan['name'], 'conf': conf})
            perf_on_other_scans.append(conf_unseen)

            #Only select points that have *not* been added
            #if active_train:
            remaining_pt_indices = inverse_indices(self.dataset.idx_in_scan, rcurrent_scan.shape[1])
            if len(remaining_pt_indices) > 0:
                ridx, selected_dist, converged = self.learner.select_next_instances_no_terminate(rcurrent_scan[:, remaining_pt_indices])
                if not converged or run_till_end:
                    selected_idx = remaining_pt_indices[ridx]
                    self.add_to_training_set(selected_idx)
                    self.train(rcurrent_scan, use_pca)
                    i = i + 1
                else:
                    should_run = False

                if converged and converged_at_iter == None:
                    converged_at_iter = i

                if i > 100 and converged:
                    should_run = False

            else:
                should_run = False

        print '======================='
        print '>> LEARNER CONVERGED <<'
        print '======================='
        print 'saving dataset to', self.training_sname
        ut.save_pickle(self.dataset, BASE_FILE_NAME + '_converged_dset.pkl')
        print 'saved!'

        #Save training results
        training_results_name = BASE_FILE_NAME + 'active_train_iter_results.pkl'
        ut.save_pickle({'train_set_statistics': train_set_statistics,
                        'current_scan_statistics': current_scan_statistics,
                        'perf_on_other_scans': perf_on_other_scans,
                        'converged_at_iter': converged_at_iter}, training_results_name)
        print 'Saved training results to', training_results_name

    def run_gui(self):
        # Make a test set
        while not rospy.is_shutdown():
            self.step()

    def mouse_cb(self, event, x, y, flags, param):
        if not (event == cv.CV_EVENT_LBUTTONDOWN or \
                event == cv.CV_EVENT_RBUTTONDOWN or \
                event == cv.CV_EVENT_MBUTTONDOWN):
            return
        # Grab closest thing, change its label, and redraw
        ck = [self.cdisp['tree'].query(np.array([x,y]) / self.scale, k=1)[1]]
        cr = self.cdisp['tree'].query_ball_point(np.array([x,y]) / self.scale, 5.)
        
        print 'k nearest', len(ck), 'radius', len(cr)

        if len(ck) > len(cr):
            closest_idx = ck
        else:
            closest_idx = cr

        if self.mode == 'GROUND_TRUTH':
            if event == cv.CV_EVENT_LBUTTONDOWN:
                label = POSITIVE
            elif event == cv.CV_EVENT_RBUTTONDOWN:
                label = NEGATIVE
            elif event == cv.CV_EVENT_MBUTTONDOWN:
                label = UNLABELED
            else:
                return

            print 'Current mode is', self.mode
            #print 'old labels', self.current_scan['labels'][0, closest_idx]
            self.current_scan['labels'][0, closest_idx] = label
            #print 'new labels', self.current_scan['labels'][0, closest_idx]

        
        if self.mode == 'TRAINING_SET':
            # Try to add selected point
            if event == cv.CV_EVENT_LBUTTONDOWN:
                self.add_to_training_set(closest_idx)

            # Try to remove the selected point
            elif event == cv.CV_EVENT_RBUTTONDOWN:
                self.remove_from_training_set(closest_idx)
        self.draw()

    def add_to_training_set(self, indices):
        # Don't add examples that have not been labeled (remove examples with the label UNLABELED)
        labeled_idx = []
        for idx in indices:
            #print 'add_to_training_set:', self.current_scan['labels'][0, idx].__class__
            #if np.array([self.current_scan['labels'][0, idx]]).shape[0] > 1:
            #    pdb.set_trace()
            #if len(np.array([self.current_scan['labels'][0, idx]]).shape) > 1:
            #    pdb.set_trace()

            if self.current_scan['labels'][0, idx] != UNLABELED:
                labeled_idx.append(idx)
        
        # Don't add examples that have been added (keep index of examples added from this scan).
        # Search the list of labeled examples.
        filtered_idx = []
        if self.dataset != None:
            for idx in labeled_idx:
                #pdb.set_trace()
                if self.dataset.idx_in_scan == None:
                    matched_idx = []
                else:
                    matched_idx = np.where(self.dataset.idx_in_scan == idx)[1].A1
                if len(matched_idx) > 0:
                    if np.any(self.scan_names[self.scan_idx] == self.dataset.scan_ids[:, matched_idx]):
                        continue
                filtered_idx.append(idx)
        else:
            filtered_idx += labeled_idx
       
        if len(filtered_idx) < 1:
            print 'add_to_training_set: returned point #', labeled_idx, 'as it is already in the dataset'
            return
            #pdb.set_trace()
        filtered_idx = [filtered_idx[0]]
        pinfo = [self.current_scan[k][:, filtered_idx] for k in \
                        ['instances', 'labels', 'points2d', 'points3d']]
        pinfo.append(np.matrix([self.scan_names[self.scan_idx]] * len(filtered_idx)))
        pinfo.append(np.matrix(filtered_idx))

        if self.dataset == None:
            #pdb.set_trace()
            print 'WARNING: THIS BRANCH SHOULD BE REACHED DURING AUTONOMOUS RUN. add_to_training_set'
            #pdb.set_trace()
            self.dataset = InterestPointDataset(self.select_features(pinfo[0], self.features_to_use, self.current_scan['sizes']),
                                                pinfo[1], pinfo[2], pinfo[3], None, pinfo[4], pinfo[5], 
                                                sizes=self.current_scan['sizes'])
        else:
            self.dataset.add(self.select_features(pinfo[0], self.features_to_use, self.current_scan['sizes']), pinfo[1], \
                                                  pinfo[2], pinfo[3], pinfo[4], pinfo[5])

        print '>> Number of examples in dataset:', self.dataset.inputs.shape[1]

    def remove_from_training_set(self, indices):
        if self.dataset != None:
            for idx in indices:
                matched_idx = np.where(self.dataset.idx_in_scan == idx)[1].A1
                if len(matched_idx) > 0:
                    sm = np.where(self.scan_names[self.scan_idx] == self.dataset.scan_ids[:, matched_idx])[1]
                    if len(sm) > 0:
                        to_remove = matched_idx[sm]
                        print 'removing', len(sm), 'points'
                        for ridx in to_remove:
                            self.dataset.remove(ridx)

    def has_enough_data(self):
        pos_ex = np.sum(self.dataset.outputs)
        neg_ex = self.dataset.outputs.shape[1] - pos_ex
        if pos_ex > 0 and neg_ex > 0:
            return True

    def train(self, inputs_for_scaling):
        if self.dataset != None and self.has_enough_data():
            use_pca=False
            for f in self.features_to_use:
                if f == 'intensity':
                    use_pca=True
            nneg = np.sum(self.dataset.outputs == NEGATIVE)
            npos = np.sum(self.dataset.outputs == POSITIVE)
            print '================= Training ================='
            print 'NEG examples', nneg
            print 'POS examples', npos
            print 'TOTAL', self.dataset.outputs.shape[1]
            neg_to_pos_ratio = float(nneg)/float(npos)
            #weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
            #weight_balance = ' -w0 1 -w1 %.2f' % 2.
            #weight_balance = ""
            #weight_balance = ' -w0 1 -w1 5.0'
            weight_balance = ' -w0 1 -w1 %.2f' % (neg_to_pos_ratio)
            #weight_balance = ""

            self.learner = SVMPCA_ActiveLearner(use_pca, 
                    self.rec_params.reconstruction_std_lim, 
                    self.rec_params.reconstruction_err_toler,
                    old_learner=self.learner)

            self.learner.train(self.dataset, 
                               inputs_for_scaling,
                               self.rec_params.svm_params + weight_balance,
                               self.rec_params.variance_keep)

            #correct = np.sum(self.dataset.outputs == np.matrix(self.learner.classify(self.dataset.inputs)))
            #pdb.set_trace()
            #print 'Test set: %.2f' % (100.* (float(correct)/float(self.dataset.outputs.shape[1]))), '% correct'
            #self.evaluate_learner(self.dataset.inputs, self.dataset.outputs)

            if use_pca:
                BASE_FILE_NAME = 'pca_fast'
                i = 111
                cv.SaveImage(BASE_FILE_NAME + ('_iter_%d_basis.png' % i),\
                             instances_to_image(\
                                self.rec_params.win_size,\
                                self.learner.pca.projection_basis,\
                                np.min(self.learner.pca.projection_basis),\
                                np.max(self.learner.pca.projection_basis)))


            print '=================  DONE  =================' 

    def evaluate_learner(self, instances, true_labels, verbose=True):
        predicted = np.matrix(self.learner.classify(instances))

        posidx = np.where(true_labels == POSITIVE)[1].A1
        negidx = np.where(true_labels == NEGATIVE)[1].A1
        m00 = m01 = m10 = m11 = 0
        nm00 = nm01 = nm10 = nm11 = 0
        if len(negidx) > 0:
            nm00 = float(np.sum(NEGATIVE == predicted[:, negidx]))
            nm01 = float(np.sum(POSITIVE == predicted[:, negidx]))
            m00 =  nm00 / len(negidx)
            m01 =  nm01 / len(negidx)

        if len(posidx) > 0:
            nm10 = float(np.sum(NEGATIVE == predicted[:, posidx]))
            nm11 = float(np.sum(POSITIVE == predicted[:, posidx]))
            m10 =  nm10 / len(posidx)
            m11 =  nm11 / len(posidx)

        conf_mat = np.matrix([[m00, m01], [m10, m11]], 'float')
        if verbose:
            print 'Confusion matrix:'
            print '-   %5.2f, %5.2f' % (100.* m00, 100.* m01)
            print '+  %5.2f, %5.2f' % (100.* m10, 100.* m11)
            print '   Total %5.2f' % (100.* (float(np.sum(true_labels == predicted)) / true_labels.shape[1]))

        return predicted, {'mat': np.matrix([[nm00, nm01], [nm10, nm11]], 'float'), 
                           'neg': len(negidx), 
                           'pos': len(posidx)}
        #print 'POS correct %.2f' % 100. * float(pcor) / len(posidx)
        #print 'NEG correct %.2f' % 100. * float(ncor) / len(negidx)

    def add_to_dataset(self, pinfo):
        if self.dataset == None:
            self.dataset = InterestPointDataset(pinfo[0], pinfo[1], pinfo[2], \
                    pinfo[3], None, pinfo[4], pinfo[5], sizes=self.current_scan['sizes'])
        else:
            self.dataset.add(pinfo[0], pinfo[1], \
                    pinfo[2], pinfo[3], pinfo[4], pinfo[5])

class FiducialPicker:

    def __init__(self, fname):
        #load pickle and display image
        self.fname = fname
        self.data_pkl = ut.load_pickle(fname)
        image_fname = pt.join(pt.split(fname)[0], self.data_pkl['image'])
        self.img = cv.LoadImageM(image_fname)
        self.clicked_points = None
        self.clicked_points3d = None

        #make a map in image coordinates
        #bl_pc = ru.pointcloud_to_np(self.data_pkl['points3d'])
        bl_pc = self.data_pkl['points3d']
        self.image_T_laser = self.data_pkl['k_T_bl']
        self.image_arr = np.asarray(self.img)
        self.calibration_obj = self.data_pkl['cal']
        self.points_valid_image, self.colors_valid, self.points2d_valid = \
                i3d.combine_scan_and_image_laser_frame(bl_pc, self.image_T_laser,\
                                            self.image_arr, self.calibration_obj)

        laser_T_image = np.linalg.inv(self.image_T_laser)
        self.points_valid_laser = tfu.transform_points(laser_T_image, self.points_valid_image[0:3,:])
        self.ptree = sp.KDTree(np.array(self.points2d_valid.T))

        cv.NamedWindow('image', cv.CV_WINDOW_AUTOSIZE)
        cv.ShowImage('image', self.img)
        cv.SetMouseCallback('image', self.mouse_cb, None)
        self.exit = False
        

    def mouse_cb(self, event, x, y, flags, param):
        if event != cv.CV_EVENT_LBUTTONDOWN:
            return

        pt = np.array([x,float(y)]).T
        idx = self.ptree.query(pt.T, k=1)[1]
        #pdb.set_trace()
        if self.clicked_points != None:
            self.clicked_points = np.column_stack((self.clicked_points, self.points2d_valid[:, idx]))
            self.clicked_points3d = np.column_stack((self.clicked_points3d, self.points_valid_laser[:,idx]))
        else:
            self.clicked_points = self.points2d_valid[:, idx]
            self.clicked_points3d = self.points_valid_laser[:,idx]

        #self.clicked_points

        #    label = POSITIVE
        #elif event == cv.CV_EVENT_RBUTTONDOWN:
        #    label = NEGATIVE
        #else:
        #    return

    def step(self):
        k = cv.WaitKey(33)
        if k == ord('s'):
            if self.clicked_points.shape[1] > 3:
                fname = pt.splitext(self.fname)[0] + '_synthetic_locs3d.pkl'
                print 'saving data to file:', fname
                #self.data_pkl['synthetic_locs3d'] = self.clicked_points3d
                #ut.save_pickle(self.data_pkl, self.fname)
                ut.save_pickle(self.clicked_points3d, fname)
            else:
                print 'ignored save command, need more points. only has', self.clicked_points3d.shape[1]

        if k == ord('x'):
            self.exit = True

        ibuffer = cv.CloneMat(self.img)
        draw_points(ibuffer, self.points2d_valid, [0,255,0], 1)
        #if self.clicked_points.shape[1] > 0:
        if self.clicked_points != None:
            draw_points(ibuffer, self.clicked_points, [0,0,255], 3)
        cv.ShowImage('image', ibuffer)

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            if self.exit:
                return

class NarrowTextureFeatureExtractor:
    def __init__(self, prosilica, narrow_texture, prosilica_cal, projector, tf_listener, rec_params=None):
        self.cal = prosilica_cal
        self.prosilica = prosilica
        self.narrow_texture = narrow_texture
        self.tf_listener = tf_listener
        self.projector = projector

        if rec_params == None:
            rec_params = Recognize3DParam()
        self.rec_params = rec_params

    def read(self, expected_loc_bl, params=None):
        #self.laser_scan.scan(math.radians(180.), math.radians(-180.), 2.315)
        rospy.loginfo('grabbing point cloud')
        pointcloud_msg = self.narrow_texture.read()
        rospy.loginfo('grabbing prosilica frame')
        self.projector.set(False)
        for i in range(3):
            cvimage_mat = self.prosilica.get_frame()
        self.projector.set(True)
        rospy.loginfo('processing')
        pointcloud_ns = ru.pointcloud_to_np(pointcloud_msg)
        pointcloud_bl = tfu.transform_points(tfu.transform('base_link',\
                                                            'narrow_stereo_optical_frame', self.tf_listener),\
                                            pointcloud_ns)

        image_T_bl = tfu.transform('high_def_optical_frame', 'base_link', self.tf_listener)

        if params == None:
            params = self.rec_params

        #laser_T_bl = tfu.transform('/laser_tilt_link', '/base_link', self.tf_listener)
        #extractor = IntensityCloudData(pointcloud_bl, cvimage_mat, image_T_bl, self.cal, 
        #                     expected_loc_bl, expected_loc_bl, None, params)
        extractor = infea.IntensityCloudFeatureExtractor(pointcloud_bl, cvimage_mat, expected_loc_bl, 
                                                    None, image_T_bl, self.cal, params)
        xs, locs2d, locs3d = extractor.extract_features()

        rdict = {#'histogram': extractor.fpfh_hist,
                 #'hpoints3d': extractor.fpfh_points,
                 'points3d': pointcloud_bl,
                 'image': cvimage_mat}

        return {'instances': xs, 
                'points2d': locs2d, 
                'points3d': locs3d, 
                'image': cvimage_mat, #rdict['image'], 
                'rdict': rdict,
                'sizes': extractor.sizes}


if __name__ == '__main__':
    import sys
    import optparse
    p = optparse.OptionParser()
    p.add_option('-m', '--mode', action='store',
                dest='mode', default='label', 
                help='fiducialpicker, preprocess, or label')
    p.add_option("-f", "--feature", action="append", type="string")
    p.add_option("-t", "--test", action="store_true")
    p.add_option("-r", "--train", action="append", type="string")
    p.add_option("-n", "--expname", action="store", default="")
    p.add_option("-p", "--pca", action="store_true", default=None)
    p.add_option("-s", "--seed", action="store", default=None, help='not strictly neccessary if you use pickles ending with _seed.pkl')
    p.add_option("-e", "--end", action="store_true", default=False)
    p.add_option("-l", "--locations", action="store", default=None)
    p.add_option("-a", "--autolabel", action="store_true", default=None)
    #p.add_option("-k", "--kinect", action="store_true", default=True)
    opt, args = p.parse_args()
    mode = opt.mode

    print '======================================='
    print 'mode:', mode
    print '======================================='
    print args

    if mode == 'fiducialpicker':
        fp = FiducialPicker(args[0])
        fp.run()

    #if mode == 'standalone':
    #    object_name = args[0]
    #    raw_data_fname = args[1]
    #    if len(sys.argv) > 3:
    #        labeled_data_fname = args[2]
    #    else:
    #        labeled_data_fname = None

    #    cvgui = CVGUI4(raw_data_fname, object_name, labeled_data_fname)
    #    cvgui.run()

    if mode == 'preprocess':
        rospy.init_node('detect_fpfh', anonymous=True)
        preprocess_data_in_dir(args[0], ext='_features_df2_dict.pkl')

    if mode == 'locations':
        if opt.locations != None:
            locations = ut.load_pickle(opt.locations)
            keys = locations['data'].keys()
            for i, key in enumerate(keys):
                print i, key
            picked_i = int(raw_input('pick a key to use'))
            seed_dset = keys[i]
            fname = raw_input('pick a file name')
            dset = locations['data'][seed_dset]['dataset']
            dset.idx_in_scan = np.matrix(dset.inputs.shape[1] * [0])
            dset.scan_ids = np.matrix(dset.inputs.shape[1] * [''])
            #dset.pt2d = None
            ut.save_pickle(dset, fname)
            pdb.set_trace()
            print 'saved %s' % fname


    if mode == 'hyper':
        s = ScanLabeler(args[0], ext='_features_df2_dict.pkl', scan_to_train_on=opt.train, 
                seed_dset=opt.seed, features_to_use=opt.feature)
        s.generate_dataset_for_hyperparameter_grid_search(opt.feature)


    if mode == 'label':
        s = ScanLabeler(args[0], ext='_features_df2_dict.pkl', scan_to_train_on=opt.train, 
                seed_dset=opt.seed, features_to_use=opt.feature)
        #s.automatic_label()
        #pdb.set_trace()
        if opt.autolabel:
            s.automatic_label()
        elif opt.test:
            print 'Running automated tests'
            print 'Using features', opt.feature
            s.active_learn_test2()
            #if len(opt.feature) == 1 and opt.feature[0] == 'distance':
            #    s.active_learn_test(['distance'], opt.expname, opt.pca, run_till_end=opt.end)
            #else:
            #    s.active_learn_test(opt.feature, opt.expname, opt.pca, run_till_end=opt.end)
        else:
            print 'launching gui.'
            s.run_gui()

    if mode == 'feature':
        s = ScanLabeler(args[0], ext='_features_df2_dict.pkl', scan_to_train_on=opt.train[0], 
                seed_dset=opt.seed, features_to_use=opt.feature)
        if len(opt.feature) == 1 and opt.feature[0] == 'distance':
            s.test_feature_perf(opt.train, ['distance'], opt.expname, opt.pca)
        else:
            s.test_feature_perf(opt.train, opt.feature, opt.expname, opt.pca)

    if mode == 'call_fpfh':
        rospy.init_node('detect_fpfh', anonymous=True)
        fpfh = rospy.ServiceProxy('fpfh', fsrv.FPFHCalc)

        print 'loading'
        extractor, _ = load_data_from_file(args[0], Recognize3DParam())
        #data_pkl = ut.load_pickle(sys.argv[1])
        req = fsrv.FPFHCalcRequest()
        print 'npoints', extractor.points3d_valid_laser.shape[1]
        req.input = ru.np_to_pointcloud(extractor.points3d_valid_laser, 'base_link')
        #data_pkl['points_laser']
        print 'requesting'
        res = fpfh(req)
        print 'response received'
        #res.hist.histograms
        #res.hist.npoints
        #pdb.set_trace()
        histogram = np.matrix(res.hist.histograms).reshape((res.hist.npoints,33)).T
        points3d = np.matrix(res.hist.points3d).reshape((res.hist.npoints,3)).T
        print 'done'
        print 'done'
        print 'done'
        print 'done'
        print 'done'

    if mode == 'kinect':
        #import roslib; roslib.load_manifest('hai_sandbox')
        #import rospy
        #from sensor_msgs.msg import Image
        rospy.init_node('kinect_features')
        kfe = KinectFeatureExtractor()
        fname = opt.train[0]

        dataset = ut.load_pickle(fname)
        nneg = np.sum(dataset.outputs == NEGATIVE)
        npos = np.sum(dataset.outputs == POSITIVE)
        print '================= Training ================='
        print 'NEG examples', nneg
        print 'POS examples', npos
        print 'TOTAL', dataset.outputs.shape[1]
        neg_to_pos_ratio = float(nneg)/float(npos)
        weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
        learner = SVMPCA_ActiveLearner(use_pca=True)
        trained = False
        ip = ImagePublisher('active_learn')

        while not rospy.is_shutdown():
            #calculate features
            xs, locs2d, locs3d, rdict['image'], rdict = kfe.read()
            if not trained:
                learner.train(dataset, xs, kfe.rec_params.svm_params + weight_balance,
                              kfe.rec_params.variance_keep)
                trained = True
            
            #predict
            results = np.matrix(learner.classify(sdset))
            current_scan_pred = InterestPointDataset(xs, results, locs2d, locs3d, None)

            #Draw
            img = cv.CloneMat(cdisp['cv'])
            draw_labeled_points(img, current_scan_pred, scale=1./self.scale)
            ip.publish(img)

        rospy.spin()

