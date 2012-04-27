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
import trf_learn.recognize_3d as r3d
import trf_learn.locations_manager as lcm
import sys
import optparse
import pdb
import numpy as np
import ml_lib.dataset as ds
import sklearn.metrics as sm 
import hrl_lib.util as ut

def confusion_matrix_in_percent(cmat):
    row_sums = np.sum(np.matrix(cmat), 1)
    pmat = cmat.copy()
    for i in range(cmat.shape[0]):
        pmat[i,:] = cmat[i,:]/float(row_sums[i,0])
    return pmat

##
# Effort to switch to scikit learn
#class LocationManagerDatasetProducer:
#
#    def __init__(self, filename):
#        self.rec_params = r3d.Recognize3DParam()
#        self.locations_man = lcm.LocationsManager(filename, rec_params=self.rec_params, train=False)
#
#    def dataset_names(self):
#        return self.locations_man.data.keys()
#
#    def get_dataset(self, name):
#        dataset = self.locations_man.data[name]
#        return dataset.inputs, dataset.outputs
#
#    def get_pca_vectors_for_dataset(self, name):

class Evaluation:

    def __init__(self, filename, task_id):
        self.rec_params = r3d.Recognize3DParam()
        print 'loading locations manager'
        self.locations_man = lcm.LocationsManager(filename, rec_params=self.rec_params, train=True)
        self.task_id = task_id
        print 'The following datasets are available:', self.locations_man.data.keys()

    def train(self, dataset, rec_params, pca_obj, g=.5, c=.5):
        nneg = np.sum(dataset.outputs == r3d.NEGATIVE) #TODO: this was copied and pasted from r3d
        npos = np.sum(dataset.outputs == r3d.POSITIVE)
        neg_to_pos_ratio = float(nneg)/float(npos)
        weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
        learner = r3d.SVMPCA_ActiveLearner(use_pca=True, 
                        reconstruction_std_lim=rec_params.reconstruction_std_lim, 
                        reconstruction_err_toler=rec_params.reconstruction_err_toler,
                        old_learner=None, pca=pca_obj)
        inputs_for_pca = dataset.inputs #Questionable!
        svm_params = '-s 0 -t 2 -g %f -c %f' % (g, c)
        learner.train(dataset, 
                      inputs_for_pca,
                      svm_params + weight_balance,
                      rec_params.variance_keep)
        return learner

    def leave_one_out(self, g=.5, c=.5, pca=5, subset=20):
        pca_obj = self.locations_man.data[self.task_id]['pca']
        pca_obj.projection_basis = pca_obj.projection_basis[:,:pca]
        dataset = self.locations_man.data[self.task_id]['dataset']
        num_datapoints = dataset.inputs.shape[1]
        predicted_values = []
        perm_subset = np.random.permutation(num_datapoints)[0:subset]
        #for i in range(num_datapoints):
        for i in perm_subset:
            loo_dataset, left_out_input, left_out_output = ds.leave_one_out(dataset, i)
            learner = self.train(loo_dataset, self.rec_params, pca_obj, g, c)
            predicted = learner.classify(left_out_input)
            predicted_values += predicted
        num_correct = np.sum(dataset.outputs.A1[perm_subset] == np.array(predicted_values))
        cmat = sm.confusion_matrix(dataset.outputs.A1[perm_subset], np.array(predicted_values))
        #pdb.set_trace()
        return cmat, num_correct/float(len(perm_subset))

    def searchg(self):
        rangeg = np.arange(0.1, 4, .1)
        cmats = []
        accuracies = []
        for g in rangeg:
            cmat, percentage = self.leave_one_out(g=g, pca=50)
            cmats.append(cmat)
            accuracies.append(percentage)
        return rangeg, cmats, accuracies

    def searchc(self):
        rangeg = np.arange(0.1, 4, .1)
        cmats = []
        accuracies = []
        for c in rangeg:
            cmat, percentage = self.leave_one_out(c=c, pca=50)
            cmats.append(cmat)
            accuracies.append(percentage)
        return rangeg, cmats, accuracies

    def parameter_search(self):
        gvals, gmats, gaccuracies = self.searchg()
        cvals, cmats, caccuracies = self.searchc()

        print '======================================'
        print '======================================'
        print 'G values'
        for v, mat, p in zip(gvals, gmats, gaccuracies):
            print v, p#, mat
            print confusion_matrix_in_percent(mat)

        print '======================================'
        print '======================================'
        print 'C values'
        for v, mat, p in zip(cvals, cmats, caccuracies):
            print v, p#, mat
            print confusion_matrix_in_percent(mat)

        print 'max gamma', gvals[np.argmax(gaccuracies)], 'accuracy', np.max(gaccuracies)
        print 'max C    ', cvals[np.argmax(caccuracies)], 'accuracy', np.max(caccuracies)

    def load_interest_point_dataset(self, name):
        print 'loading features dataset'
        dataset = ut.load_pickle(name)
        print 'classifying'
        predictions = np.matrix(self.locations_man.learners[self.task_id].classify(dataset['instances']))
        pdb.set_trace()
        print 'loaded'

def classify_saved_data():
    kdict, image_name = self.read_features_save(task_id, point3d_bl, params)
    predictions = np.matrix(self.locations_man.learners[task_id].classify(kdict['instances']))


if __name__ == '__main__':
    #p = optparse.OptionParser()
    #p.add_option("-d", "--display", action="store", default='locations_narrow_v11.pkl')

    if len(sys.argv) < 2:
        exit()

    name = sys.argv[1]
    task_id = sys.argv[2]
    features = sys.argv[3]

    loo = Evaluation(name, task_id)
    loo.load_interest_point_dataset(features)
    #loo.parameter_search()
    #load_interest_point_dataset(name)


