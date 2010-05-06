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

from labeling import label_object, scan_dataset
import hrl_lib.util as ut  

import shutil #file operations

class scans_database(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        self.datasets = []
        self.current_index = 0
        
        
    def load(self, path, filename):
        self.filename = filename
        self.path = path
        #try:
        dict = ut.load_pickle(self.path+'/'+self.filename)
        #except:
        #    print 'loading of '+self.path+'/'+filename+' failed. WARNING: it will be overwritten on save()!'
        #    return
            
        self.datasets = dict['datasets']
        
    def save(self):
        dict = {'datasets': self.datasets,'version': 0.1} 
        
        #for now: make a backup first:
        database_filename = self.path+'/'+self.filename
        backup_filename = self.path+'/'+self.filename+'_backup_'+ut.formatted_time()
        print 'Backing up old database to ' + backup_filename       
        shutil.copy(database_filename, backup_filename)
        
        print "Saving: "+database_filename
        ut.save_pickle(dict,database_filename)    
           
        
    def get_path(self):
        return self.path
        
    def get_dataset(self, index):
        self.current_index = index
        return self.datasets[index]
    
    def get_dataset_by_id(self, id):
        #TODO: faster lookup, probably using a dictionary instead of a list?
        
        for dataset in self.datasets:
            if dataset.id == id:
                return dataset
        return False
    
    def set_internal_pointer_to_dataset(self, id):
        self.current_index = 0
        for dataset in self.datasets:
            if dataset.id == id:
                return True
            self.current_index += 1
        return False
    
    def get_next_dataset(self):
        if self.current_index < len(self.datasets) - 1:
            self.current_index = self.current_index + 1
            return self.datasets[self.current_index]
        else:
            return False
            
    def get_prev_dataset(self):
        if self.current_index > 0:
            self.current_index = self.current_index - 1
            return self.datasets[self.current_index]
        else:
            return False    
        
    def get_first_dataset(self):
        if len(self.datasets) > 0:
            self.current_index = 0
            return self.datasets[self.current_index]
        else:
            return False    
        
    def get_last_dataset(self):
        if len(self.datasets) > 0:
            self.current_index = len(self.datasets) - 1
            return self.datasets[self.current_index]
        else:
            return False                     
        
        
    def get_count(self):
        return len(self.datasets)
    
    def add_dataset(self, dataset):
        self.datasets.append(dataset)
        
    def delete_current_dataset(self):
        del self.datasets[self.current_index]
        dataset = self.get_prev_dataset()
        if False != dataset:
            return  dataset
        else: 
            dataset = self.get_next_dataset()
        return dataset #TODO: still fails if there is only one dataset!
        
        
    def add_attribute_to_every_dataset(self, name):
        for dataset in self.datasets:
            dataset.dict[name]=''
        
    