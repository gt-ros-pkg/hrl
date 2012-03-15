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

#  \author Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)

'''
# Location of classifier to load or save is found by: classifier.get_filename(self):
#     Loc = self.processor.config.path+'/classifier_'+self.features+'_'+
#           self.processor.feature_type+'_k'+str(self.processor.feature_neighborhood)+
#           '_r'+str(self.processor.feature_radius)+'.XML'
'''

from classifier import classifier

import opencv as cv
import util as ut
import numpy as np
import os

import processor ###

import ransac
import random


class boosted_tree_classifier(classifier) :
    '''
    classdocs
    '''

    cv_classifier = None

    #def __init__(selfparams):
    #    '''
    #    Constructor
    #    '''
        
        
    def create_train_datastructures(self):
        #loop through all marked datasets
        self.processor.scan_dataset = self.processor.scans_database.get_dataset(0)
          
        training_set_size = 0
        
        data = []
        #get size of training set in total
        while False != self.processor.scan_dataset:
            if self.processor.scan_dataset.is_training_set:
                
                filename = self.processor.get_features_filename(True)
                print 'loading', filename
                dict = ut.load_pickle(filename)

                # make an equal size of points for each class: use object labels more often:
                difference = np.sum(dict['labels'] == processor.LABEL_SURFACE) - np.sum(dict['labels'] == processor.LABEL_CLUTTER)
                #print ut.getTime(), filename
                #print ut.getTime(), 'surface',np.sum(dict['labels'] == LABEL_SURFACE)
                #print ut.getTime(), 'clutter',np.sum(dict['labels'] == LABEL_CLUTTER)
                #print ut.getTime(), difference, "difference = np.sum(dict['labels'] == LABEL_SURFACE) - np.sum(dict['labels'] == LABEL_CLUTTER)"
                #print ut.getTime(), ''
                if difference > 0:
                    clutter_features = (dict['features'])[np.nonzero(dict['labels'] == processor.LABEL_CLUTTER)]
                    if len(clutter_features) > 0: #if there are none, do nothin'
                        dict['set_size'] += difference
                        dict['features'] = np.vstack((dict['features'], clutter_features[np.random.randint(0,len(clutter_features),size=difference)]))
                        dict['labels'] = np.hstack((dict['labels'], np.ones(difference) * processor.LABEL_CLUTTER))
                elif difference < 0: 
                    surface_features = (dict['features'])[np.nonzero(dict['labels'] == processor.LABEL_SURFACE)]
                    if len(surface_features) > 0: #if there are none, do nothin'
                        difference = -difference
                        dict['set_size'] += difference
                        dict['features'] = np.vstack((dict['features'], surface_features[np.random.randint(0,len(surface_features),size=difference)]))
                        dict['labels'] = np.hstack((dict['labels'], np.ones(difference) * processor.LABEL_SURFACE))
                    
                training_set_size += dict['set_size']
                data.append(dict)
            #get next one
            self.processor.scan_dataset = self.processor.scans_database.get_next_dataset()
            #print ut.getTime(),  self.scan_dataset
        
        #create training set:
        self.processor.scan_dataset = self.processor.scans_database.get_dataset(0)
        current_training_set_index = 0
        
       
        feature_vector_length = len(self.processor.features.get_indexvector(self.features))
        print ut.getTime(), feature_vector_length
        #create dataset matrices:
        print ut.getTime(), '#training set size ', training_set_size 
        
        #deactivate for now:
        max_traning_size = 1800000#2040000
        #if training_set_size < max_traning_size:
        #if True:       
        train_data = cv.cvCreateMat(training_set_size,feature_vector_length,cv.CV_32FC1) #CvMat* cvCreateMat(int rows, int cols, int type)
        train_labels = cv.cvCreateMat(training_set_size,1,cv.CV_32FC1)
        
        for dict in data:        
            for index in range(dict['set_size']):
                #only train on surface and clutter
                if dict['labels'][index] == processor.LABEL_SURFACE or dict['labels'][index]== processor.LABEL_CLUTTER:
                
                    #print ut.getTime(), point3d
                    #print ut.getTime(), 'fvindexv',self.get_features_indexvector(features)
                    #print ut.getTime(), 'len', len(self.get_features_indexvector(features))
                    fv = (dict['features'][index])[self.processor.features.get_indexvector(self.features)]

                    #print ut.getTime(), 'fv',fv
                    #print ut.getTime(), np.shape(fv)
                    for fv_index, fv_value in enumerate(fv):
                        train_data[current_training_set_index][fv_index] = fv_value
                    train_labels[current_training_set_index] = dict['labels'][index]
#                    for fv_index, fv_value in enumerate(fv):
#                        print ut.getTime(), train_data[current_training_set_index][fv_index]
#                    print ut.getTime(), '##',train_labels[current_training_set_index],'##'                    
                    #print ut.getTime(), 'fv ', fv
                    #print ut.getTime(), 'tr ',train_data[index]
                    current_training_set_index = current_training_set_index + 1
        
                    #if current_training_set_index % 4096 == 0:
                    #    print ut.getTime(), 'label', dict['labels'][index], 'fv', fv        
                    if current_training_set_index %  16384 == 0:
                        print ut.getTime(), 'reading features:', current_training_set_index, 'of', training_set_size, '(',(float(current_training_set_index)/float(training_set_size)*100.0),'%)'
##subsample from the features, NOT USED/NOT WORKING?
#        else:
#            print ut.getTime(), 'more than',max_traning_size,'features, sample from them...'
#            #select 2040000 features:
#            all_data = []
#            all_labels = []
#            for dict in data:  
#                for index in range(dict['set_size']):
#                    if dict['labels'][index] == processor.LABEL_SURFACE or dict['labels'][index]== processor.LABEL_CLUTTER:
#                        fv = (dict['features'][index])[self.processor.features.get_indexvector(self.features)]
#                        all_data += [fv]
#                        all_labels += [dict['labels'][index]]
#                        
#                        current_training_set_index = current_training_set_index + 1    
#                        if current_training_set_index %  16384 == 0:
#                            print ut.getTime(), 'reading features:', current_training_set_index, 'of', training_set_size, '(',(float(current_training_set_index)/float(training_set_size)*100.0),'%)'
#            
#            del data
#            indices = np.array(random.sample(xrange(len(all_labels)),max_traning_size))
#            all_data = np.asarray(all_data)
#            all_labels = np.asarray(all_labels)
#            
#            all_data = all_data[indices]
#            all_labels = all_labels[indices]
#            
#            train_data = cv.cvCreateMat(max_traning_size,feature_vector_length,cv.CV_32FC1) #CvMat* cvCreateMat(int rows, int cols, int type)
#            train_labels = cv.cvCreateMat(max_traning_size,1,cv.CV_32FC1)
#                        
#            for index in range(max_traning_size):
#                for fv_index, fv_value in enumerate(all_data[index]):
#                    train_data[index][fv_index] = fv_value
#                    train_labels[index] = all_labels[index]
#                if index % 16384 == 0:
#                    print ut.getTime(), 'setting features:', (float(index)/float(max_traning_size))
#          
          
        print ut.getTime(), 'start training Classifier'

        type_mask = cv.cvCreateMat(1, feature_vector_length+1, cv.CV_8UC1)
        cv.cvSet( type_mask, cv.CV_VAR_NUMERICAL, 0)
        type_mask[feature_vector_length] = cv.CV_VAR_CATEGORICAL
        
        return (train_data, train_labels, type_mask)
    

    
    def train(self):
        #cv_boost_params = cv.CvBoostParams()

        #priors = cv.cvCreateMat(1,2,cv.CV_32FC1)
        #priors[0] = 10
        #priors[1] = 1
        
        #cv_boost_params.max_categories = 2
        #cv_boost_params.priors = priors #TODO: activate them
        self.cv_classifier = cv.CvDTree() #cv.CvBoost() #TODO: CHANGE CLASSIFIER HERE
        train_datastructures = self.create_train_datastructures()
            
        (train_data, train_labels, type_mask) = train_datastructures
        print 'WARNING! use CvDTree (single decision trees) for now as load/save works!'#'boost'
        print ut.getTime(), self.cv_classifier.train(train_data, cv.CV_ROW_SAMPLE, train_labels, None, None, type_mask ) #TODO: CHANGE CLASSIFIER HERE
       
        print ut.getTime(), 'traning finished'
       
        #self.release_train_datastructures(train_datastructures)

    #unused - is that necessary in python? how does it work??
    def release_train_datastructures(self, train_datastructures):
        if None != train_datastructures:
            (train_data, train_labels, type_mask) = train_datastructures
            cv.cvReleaseMat(train_data)
            cv.cvReleaseMat(train_labels)
            cv.cvReleaseMat(type_mask)
    
    #test on current scan:
    def test(self, feature_data = None):
        #test on current scan:
        print ut.getTime(), 'test on:', self.processor.scan_dataset.id    
            
        if feature_data == None:
            filename = self.processor.get_features_filename()
            print 'loading', filename
            dict = ut.load_pickle(filename)
        else:
            dict = feature_data
        
        #print ut.getTime(), dict
        current_set_size = dict['set_size']
        feature_vector_length = len(self.processor.features.get_indexvector(self.features))
        print ut.getTime(), feature_vector_length
        labels = np.array(np.zeros(len(self.processor.map_polys)))
        print 'test: length of labels vector:', len(labels)
        test = cv.cvCreateMat(1,feature_vector_length,cv.CV_32FC1)
        
        if current_set_size == 0:
            print ut.getTime(), 'ERROR: test dataset is empty!'
            return labels, 1, 1, 1

        count = 0
        for index in dict['point_indices']:
            fv = (dict['features'][count])[self.processor.features.get_indexvector(self.features)]
            #print ut.getTime(), fv, dict['features'][count]

            for fv_index, fv_value in enumerate(fv):
                test[fv_index] = fv_value
             
            #print 'class',self.cv_classifier
            label = self.cv_classifier.predict(test)
            #print label.value
            labels[index] = label.value
            #print 'tdone'
            if count % 4096 == 0:
                print ut.getTime(), 'testing:', count, 'of', current_set_size, '(',(float(count)/float(current_set_size)*100.0),'%)'
                
            count += 1


        #save for later use for postprocessing:
        self.test_feature_dict = dict
        self.test_labels = labels
        #cv.cvReleaseMat(test)
        return labels, self.test_results(dict, labels)  
        
    
    #test() has to be called before to create intermediate results!
    def test_postprocess(self):
        labels = self.postprocess(self.test_labels)
        return labels, self.test_results(self.test_feature_dict, labels)
    
    def postprocess(self, labels):
        
        debug = False
        model = ransac.PlaneLeastSquaresModel(debug)
        data_idx = np.where(np.asarray(labels) == processor.LABEL_SURFACE)[0]
        data = np.asarray(self.processor.pts3d_bound).T[data_idx]
        n, _ = np.shape(data)
        if n < 5000:
            k = 700
        else:
            k = 2000
        # run RANSAC algorithm
        ransac_fit, ransac_data = ransac.ransac(data,model,
                                         3, k, 0.04, len(data_idx)/2.5, # misc. parameters
                                         debug=debug,return_all=True)
        print 'ransac: model',ransac_fit
        print 'ransac:',ransac_data    
        print 'len inlier',len(ransac_data['inliers']),'shape pts',np.shape(self.processor.pts3d_bound)

        #labels[data_idx[ransac_data['inliers']]] = processor.LABEL_CLUTTER #all non-plane pts
        fancy = np.zeros(len(np.asarray(labels))).astype(bool)
        fancy[data_idx] = True
        fancy[data_idx[ransac_data['inliers']]] = False 
        labels[fancy] = processor.LABEL_CLUTTER #all surface-labeled non-plane pts
        
        return labels
        
    def save(self):
        classifier_filename = self.get_filename()
        
        #if file doesn't exist: create it
        if False == os.path.isfile(classifier_filename):
            open(classifier_filename,'w')
        self.cv_classifier.save(classifier_filename)
        
        
    def load(self):
        self.cv_classifier = cv.CvDTree() #cv.CvBoost() #TODO: CHANGE CLASSIFIER HERE
        print ut.getTime(), 'loading Classifier',self.features
        self.cv_classifier.load(self.get_filename())
        
        
