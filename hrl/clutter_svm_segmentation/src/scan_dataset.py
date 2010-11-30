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
import label_object

class scan_dataset(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        self.dict = {}
        self.dict['title'] = ''
        self.dict['id'] = ''
        self.dict['polygons'] = [label_object.label_object()]
        self.dict['scan_filename'] = ''
        self.dict['image_filename'] = ''
        self.dict['image_artag_filename'] = ''
        
        self.dict['surface_id'] = ''
        self.dict['surface_height'] = ''
        self.dict['camera_height'] = ''
        self.dict['camera_angle'] = ''
        self.dict['surface_type'] = ''
        
        self.dict['ground_plane_normal'] = ''
        self.dict['ground_plane_three_points'] = ''
        
        self.dict['is_training_set'] = False
        self.dict['is_test_set'] = False
        self.dict['is_labeled'] = False
        self.dict['ground_plane_rotation'] = ''
        #Auto creates: ['table_plane_translation'] = np.matrix([0,0,0]).T
        #    ['ground_plane_translation'] = np.matrix([0,0,1.25]).T
        #    [id] = <unique scan name>

        
    def __setattr__(self, name, value):  
        if not name == 'dict':
            self.dict[name] = value
        else:
            object.__setattr__(self, name, value)    
        
    def __getattr__(self, name):
        if not name == 'dict' and name in self.dict:
            return self.dict[name]
        else:
            return object.__getattribute__(self, name)
        
