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

from classifier import classifier

import hrl_lib.util as ut
import numpy as np

from hrl_lib.util import getTime

import processor

class baseline_classifier(classifier):
    '''
    classdocs
    '''


    #def __init__(selfparams):
    #    '''
    #    Constructor
    #    '''
        
    
    def test(self, feature_data = None):
        #test on current scan:
        print getTime(), 'test on:', self.processor.scan_dataset.id    
            
        if feature_data == None:
            filename = self.processor.get_features_filename()
            dict = ut.load_pickle(filename)
        else:
            dict = feature_data
        
        baseline_labels = self.classify_baseline_code()
    
        return baseline_labels, self.test_results(dict, baseline_labels)  
    
    
    def classify_baseline_code(self):
        import hrl_tilting_hokuyo.processing_3d as p3d
        import hrl_tilting_hokuyo.occupancy_grid_3d as og3d
        import hrl_tilting_hokuyo.display_3d_mayavi as d3m
        pt = np.matrix(self.processor.point_of_interest).T
        #define VOI
        width_half = self.processor.voi_width / 2.0
        brf = pt+np.matrix([-width_half,-width_half,-width_half]).T
        tlb = pt+np.matrix([width_half, width_half, width_half]).T
        resolution = np.matrix([0.1,0.1,0.0025]).T
        max_dist = 15
        min_dist = -15
        gr = og3d.occupancy_grid_3d(brf,tlb,resolution)
        print 'filling grid...'
        gr.fill_grid(self.processor.pts3d_bound)
        print '...filled.'
        gr.to_binary(1)
        l = gr.find_plane_indices(assume_plane=True,hmin=0.3,hmax=2)
        z_min = min(l)*gr.resolution[2,0]+gr.brf[2,0]
        z_max = max(l)*gr.resolution[2,0]+gr.brf[2,0]
        
        pts = np.asarray(self.processor.pts3d_bound)
        conditions_surface = np.multiply(pts[2,:] > z_min, pts[2,:] < z_max)
        print 'cf',conditions_surface
        conditions_clutter = np.invert(conditions_surface)
        conditions_surface = np.multiply(conditions_surface, np.array(self.processor.map_polys) > 0)
        print 'cf',conditions_surface
        idx_surface = np.where(conditions_surface)
        conditions_clutter = np.multiply(conditions_clutter, np.array(self.processor.map_polys) > 0)
        idx_clutter = np.where(conditions_clutter)
        
        n, m = np.shape(self.processor.pts3d_bound)
        print n,m
        labels = np.zeros(m)
        print np.shape(labels), labels
        print np.shape(idx_surface), idx_surface
        labels[idx_surface] = processor.LABEL_SURFACE
        labels[idx_clutter] = processor.LABEL_CLUTTER
        
        print labels
        
        return labels     
         
    
