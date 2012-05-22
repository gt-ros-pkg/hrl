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

class LeaveOneOut:

    def __init__(self, filename, dataset_name):
        self.rec_params = r3d.Recognize3DParam()
        self.locations_man = lcm.LocationsManager(filename, rec_params=self.rec_params, train=True)
        self.dataset_name = dataset_name
        pdb.set_trace()
        print 'The following datasets are available:', self.locations_man.data.keys()

    def leave_one_out(self):
        #For each data set in locations man, for each data point, remove one data point, train, test on that point
        dataset = self.locations_man.data[self.dataset_name]['dataset']
        num_datapoints = dataset.inputs.shape[1]
        #dataset.inputs = np.row_stack((np.matrix(range(num_datapoints)), dataset.inputs))

        predicted_values = []
        correct = 0
        incorrect = 0
        confusion = np.matrix([[0,0], [0,0.]])
        num_pos = np.sum(dataset.outputs)
        num_neg = num_datapoints-num_pos
        #for i in range(2):
        for i in range(num_datapoints):
            loo_dataset, left_out_input, left_out_output = ds.leave_one_out(dataset, i)
            self.locations_man.data[self.dataset_name]['dataset'] = loo_dataset
            self.locations_man.train(self.dataset_name, save_pca_images=False)
            learner = self.locations_man.learners[self.dataset_name]
            predicted = learner.classify(left_out_input)
            predicted_values += predicted
            if predicted[0] == 0:
                if left_out_output[0,0] == 0:
                    confusion[0,0] += 1
                else:
                    confusion[0,1] += 1
            else:
                #predicted[0] == 1
                if left_out_output[0,0] == 0:
                    confusion[1,0] += 1
                else:
                    confusion[1,1] += 1

            if predicted[0] == left_out_output[0,0]:
                correct += 1
            else:
                incorrect += 1

        print '============================================'
        print 'dataset', self.dataset_name
        print 'confusion matrix\n', confusion
        confusion[:,0] = confusion[:,0] / num_neg
        confusion[:,1] = confusion[:,1] / num_pos
        print 'correct', correct, '\nincorrect', incorrect, '\npercentage', 100.* (correct/float(num_datapoints))
        print predicted_values
        print '============================================'


        #predicted_values += predicted
        #np.matrix(predicted_values)
        #print 'result', predicted[0], predicted.__class__, left_out_output[0,0]

if __name__ == '__main__':
    #p = optparse.OptionParser()
    #p.add_option("-d", "--display", action="store", default='locations_narrow_v11.pkl')

    if len(sys.argv) > 1:
        name = sys.argv[1]
    else:
        name = 'locations_narrow_v11.pkl'

    loo = LeaveOneOut(name, sys.argv[2])
    loo.leave_one_out()
    print 'end!'


