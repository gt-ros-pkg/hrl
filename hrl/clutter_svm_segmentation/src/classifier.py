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

import util as ut #Local function. Uses only: getTime
import processor #Used for LABEL_CLUTTER, LABEL_SURFACE, config.path, and feature_type


class classifier(object):
    '''
    classdocs
    '''

    processor = None
    features = 'all'

    def __init__(self, processor, features):
        '''
        Constructor
        '''
        self.processor = processor
        self.features = features
        
        
    def train(self):
        return None
    
    #abstract
    def test(self, feature_data = None):
        return None
    
    #dict are the loaded features including the ground truth, labels the algorithm output
    def test_results(self, dict, labels):
        current_set_size = dict['set_size']
        count_correct = 0
        count_clutter_correct = 0
        count_surface_correct = 0
        count_clutter = 0
        count_surface = 0
        count = 0
        for index in dict['point_indices']:
            label = labels[index]
            
            if label == dict['labels'][count]:
                count_correct += 1
                
            if dict['labels'][count] == processor.LABEL_CLUTTER:
                count_clutter += 1
                if label == dict['labels'][count]:
                    count_clutter_correct += 1
            if dict['labels'][count] == processor.LABEL_SURFACE:
                count_surface += 1
                if label == dict['labels'][count]:
                    count_surface_correct += 1                    

            count += 1        
        
        print ut.getTime(), '##########################################'
        print ut.getTime(), '####tested on ', self.features, '###########################'
        print ut.getTime(), '==================================='
        print ut.getTime(), 'percent in total: surface:',(float(count_surface)/float(current_set_size)*100), '%, clutter:',(float(count_clutter)/float(current_set_size)*100),'%'
        print ut.getTime(), '#points surface:',count_surface,'clutter:',count_clutter
        print ut.getTime(), '#points correct: surface:',count_surface_correct,'clutter:',count_clutter_correct
        if count_surface > 0:
            percent_surface_correct = float(count_surface_correct)/float(count_surface) * 100
        else:
            percent_surface_correct = 100
        if count_clutter > 0:
            percent_clutter_correct = float(count_clutter_correct)/float(count_clutter) * 100
        else:
            percent_clutter_correct = 100
        print ut.getTime(), '#percent correct: surface:',percent_surface_correct,'clutter:',percent_clutter_correct
        print ut.getTime(), '==================================='
        print ut.getTime(), '##########################################'
        testresults = (count_surface, count_clutter,count_surface_correct, count_clutter_correct, percent_surface_correct, percent_clutter_correct)
    
        return testresults  
        
        
        
    def get_filename(self):
        return self.processor.config.path+'/classifier_'+self.features+'_'+self.processor.feature_type+'_k'+str(self.processor.feature_neighborhood)+'_r'+str(self.processor.feature_radius)+'.XML'
    
    def save(self):
        return None
            
    
    def load(self):
        return None
        
        
