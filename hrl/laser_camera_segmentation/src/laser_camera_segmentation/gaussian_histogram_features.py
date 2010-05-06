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

from features import features

import laser_camera_segmentation.gaussian_curvature as gaussian_curvature
import scipy.stats as stats
import numpy as np
import opencv as cv
import scipy.spatial.kdtree as kdtree
import hrl_lib.util as ut
from hrl_lib.util import getTime
import os

import laser_camera_segmentation.texture_features as texture_features

import processor
import copy

class gaussian_histogram_features(features):
    '''
    classdocs
    '''
    
    #all_save_load: set to true only if nonzero_indices contain all pts in pt-cloud!
    def prepare(self, features_k_nearest_neighbors, nonzero_indices = None, all_save_load = False, regenerate_neightborhood_indices = False):
        #print np.shape(self.processor.pts3d_bound), 'shape pts3d_bound'

        imgTmp = cv.cvCloneImage(self.processor.img)
        self.imNP = ut.cv2np(imgTmp,format='BGR')
        self.processor.map2d = np.asarray(self.processor.map[0][0:2]) #copied from laser to image mapping
        
        if features_k_nearest_neighbors == None or features_k_nearest_neighbors == False: #use range
            self.kdtree2d = kdtree.KDTree(self.processor.pts3d_bound.T)
            
            #print len(nonzero_indices)
            #print np.shape(np.asarray((self.processor.pts3d_bound.T)[nonzero_indices]))
            
            if nonzero_indices != None:
                print getTime(), 'query ball tree for ', len(nonzero_indices), 'points'
                kdtree_query = kdtree.KDTree((self.processor.pts3d_bound.T)[nonzero_indices])
            else:
                print getTime(), 'query ball tree'
                kdtree_query = kdtree.KDTree(self.processor.pts3d_bound.T)
            
            filename = self.processor.config.path+'/data/'+self.processor.scan_dataset.id+'_sphere_neighborhood_indices_'+str(self.processor.feature_radius)+'.pkl'
            if all_save_load == True and os.path.exists(filename) and regenerate_neightborhood_indices == False:
                #if its already there, load it:
                print getTime(), 'loading',filename
                self.kdtree_queried_indices = ut.load_pickle(filename)    
            else:
                self.kdtree_queried_indices = kdtree_query.query_ball_tree(self.kdtree2d, self.processor.feature_radius, 2.0, 0.2) #approximate
                print getTime(), 'queried kdtree: ',len(self.kdtree_queried_indices),'points, radius:',self.processor.feature_radius
                if all_save_load == True:
                    ut.save_pickle(self.kdtree_queried_indices, filename)
                    
            #make dict out of list for faster operations? (doesn't seem to change speed significantly):
            #self.kdtree_queried_indices = dict(zip(xrange(len(self.kdtree_queried_indices)), self.kdtree_queried_indices))
        
        else: #experiemental: use_20_nearest_neighbors == True
            #TODO: exclude invalid values in get_featurevector (uncomment code there)
           
            self.kdtree2d = kdtree.KDTree(self.processor.pts3d_bound.T)
            self.kdtree_queried_indices = []
            print getTime(), 'kdtree single queries for kNN start, k=', features_k_nearest_neighbors
            count = 0
            for point in ((self.processor.pts3d_bound.T)[nonzero_indices]):
                count = count + 1
                result = self.kdtree2d.query(point, features_k_nearest_neighbors,0.2,2,self.processor.feature_radius)
                #existing = result[0][0] != np.Inf
                #print existing
                #print result[1]
                self.kdtree_queried_indices += [result[1]] #[existing]
                if count % 4096 == 0:
                    print getTime(),count
            print getTime(), 'kdtree singe queries end'
            
            #convert to numpy array -> faster access
            self.kdtree_queried_indices = np.asarray(self.kdtree_queried_indices)
        
        #print self.kdtree_queried_indices
        #takes long to compute:
        #avg_len = 0
        #minlen = 999999
        #maxlen = 0
        #for x in self.kdtree_queried_indices:
        #    avg_len += len(x)
        #    minlen = min(minlen, len(x))
        #    maxlen = max(maxlen, len(x))
        #avg_len = avg_len / len(self.kdtree_queried_indices)
        #print getTime(), "range neighbors: avg_len", avg_len, 'minlen', minlen, 'maxlen', maxlen
        
        
        #create HSV numpy images:
        # compute the hsv version of the image 
        image_size = cv.cvGetSize(self.processor.img)
        img_h = cv.cvCreateImage (image_size, 8, 1)
        img_s = cv.cvCreateImage (image_size, 8, 1)
        img_v = cv.cvCreateImage (image_size, 8, 1)
        img_hsv = cv.cvCreateImage (image_size, 8, 3)
        
        cv.cvCvtColor (self.processor.img, img_hsv, cv.CV_BGR2HSV)
        
        cv.cvSplit (img_hsv, img_h, img_s, img_v, None)
        self.imNP_h = ut.cv2np(img_h)
        self.imNP_s = ut.cv2np(img_s)
        self.imNP_v = ut.cv2np(img_v)
        
        textures = texture_features.eigen_texture(self.processor.img)
        self.imNP_tex1 = textures[:,:,0]
        self.imNP_tex2 = textures[:,:,1]
        
        self.debug_before_first_featurevector = True
        
        self.generate_voi_histogram(self.processor.point_of_interest,self.processor.voi_width)
              


    #has to have at least length 2 because of openCV matrices!!!!
    def get_indexvector(self, type):
        
        var_idx = []
        #start indices
        rh1 = 0 #zhist, normal, eigenvalue1, ev2
        ch1 = rh1 + 6 #hsi zhist, maxheight-diff, tex1, tex2
        ci = ch1 + 25
        end = ci + 4 #
        if type=='range':
            for i in range(rh1, ch1):
                var_idx.append(i) 
        elif type=='color':
            for i in range(ch1, end):
                var_idx.append(i)                                        
        #for plotting:
        elif type=='hsvi':
            for i in range(ci,end):
                var_idx.append(i)                         
        else: #all
            for i in range(rh1, end):
                var_idx.append(i)   

        return np.array(var_idx)    
        
        
        
    #get the feature vector for a specific point
    def get_featurevector(self, index, count, pts = None):
        if pts == None:
            pts = self.processor.pts3d_bound

        #print 'i',index,'c', count
        fv = []
        
        indices = np.asarray(self.kdtree_queried_indices[count])
        invalid_value = np.shape(pts)[1]
        #print indices
        #print 'iv',invalid_value
        indices = indices[indices != invalid_value]
        
        #print getTime(), indices
        #print getTime(), 'number of pts', len(indices)
        a = pts[:,indices]
        view = processor.rotate_to_plane(self.processor.scan_dataset.ground_plane_normal, np.matrix([-1,0,0.]).T)
        normal, eigenvalues = gaussian_curvature.gaussian_curvature(a,view)
        #eigenvalues = eigenvalues / np.square(r)
        #fv += [normal[0,0],0,normal[2,0]]
        #fv += normal.T.A[0].tolist()
        #fv += eigenvalues.tolist()
        #print np.asarray(pts[:,index].T[0])[0]
       # print 'pt',np.asarray(pts[:,index].T[0])
        point = pts[:,index]
        
        ev1, ev2 = self.get_voi_histogram_spread(point)
        #z_max_height_diff = pts[2,index] - self.get_voi_maxcount_height()
        #fv += [self.get_voi_histogram_value(point),z_max_height_diff,normal[0,0],normal[1,0],normal[2,0], ev1, ev2]
        fv += [self.get_voi_histogram_value(point),normal[0,0],normal[1,0],normal[2,0], ev1, ev2]
        
        h = self.imNP_h[self.processor.map2d[1,index],self.processor.map2d[0,index]]
        s = self.imNP_s[self.processor.map2d[1,index],self.processor.map2d[0,index]]
        i = self.processor.intensities_bound[index]
        hsi = self.get_voi_hsi_histogram_values(point,h,s,i)
        fv += [hsi[0],hsi[1],hsi[2]]
        #print np.shape(self.imNP_tex1)
        #print np.shape(self.map2d)
        tex1 = self.imNP_tex1[self.processor.map2d[1,index],self.processor.map2d[0,index]]
        tex2 = self.imNP_tex2[self.processor.map2d[1,index],self.processor.map2d[0,index]]
        fv += [tex1, tex2]
        #print tex1, tex2
        

        #color histograms:
        colors_h = []
        colors_s = []
        colors_v = []
        for idx in indices:
            colors_h.append(float(self.imNP_h[self.processor.map2d[1,idx],self.processor.map2d[0,idx]]))
            colors_s.append(float(self.imNP_s[self.processor.map2d[1,idx],self.processor.map2d[0,idx]]))
            colors_v.append(float(self.imNP_v[self.processor.map2d[1,idx],self.processor.map2d[0,idx]]))
        
        color_hist = stats.histogram2(np.array(colors_h), [0,51,102,153,204])
        color_hist = color_hist / float(np.sum(color_hist))
        color_hist = list(color_hist)
        fv += color_hist
        color_hist = stats.histogram2(np.array(colors_s), [0,51,102,153,204])
        color_hist = color_hist / float(np.sum(color_hist))
        color_hist = list(color_hist)
        fv += color_hist
        color_hist = stats.histogram2(np.array(colors_v), [0,51,102,153,204])
        color_hist = color_hist / float(np.sum(color_hist))
        color_hist = list(color_hist)
        fv += color_hist
        
        #intensities
        intensities = self.processor.intensities_bound[indices]
        intensities = np.asarray(intensities)
        #map to 0-255-range:   TODO: perhaps do some nonlinear transformation here? 
        intensities = intensities / 10000 * 255
        intensity_hist = stats.histogram2(intensities, [0,51,102,153,204])
        intensity_hist = intensity_hist / float(np.sum(intensity_hist))
        intensity_hist = list(intensity_hist)
        fv += intensity_hist    
    
        #current colors:
        fv += [float(self.imNP_h[self.processor.map2d[1,index],self.processor.map2d[0,index]]) / 255.0]
        fv += [float(self.imNP_s[self.processor.map2d[1,index],self.processor.map2d[0,index]]) / 255.0]
        fv += [float(self.imNP_v[self.processor.map2d[1,index],self.processor.map2d[0,index]]) / 255.0]  
        
        #current intensity value (scaled)
        intensity = self.processor.intensities_bound[index]
        #scale:
        intensity = intensity / 15000.0
        intensity = [intensity]
        fv += intensity  

        
        if self.debug_before_first_featurevector == True:
            self.debug_before_first_featurevector = False
            print getTime(), 'feature vector sample(gaussian histograms):', fv
        return fv
    
    

        #cube of interest around point
    def generate_voi_histogram(self, poi, width):
        print 'poi',poi,'width',width
        pts_indices = self.get_voi_pts_indices(poi, width)
        self.voi_pts_indices = pts_indices
        pts = np.asarray(self.processor.pts3d_bound)
        pts = pts[:,pts_indices]
        self.voi_pts = pts
        #mlab.points3d(pts[0,:],pts[1,:],pts[2,:], mode='point')
        #mlab.show() 
        min = 0.
        max = 2.
        self.voi_bincount = 80
        self.voi_interval_size = max - min
        bins = np.asarray(range(self.voi_bincount)) * self.voi_interval_size/float(self.voi_bincount)
        #print 'bins',bins
        hist = stats.histogram2(pts[2],bins) / float(len(pts[2]))
        #print 'zhist',hist
        #print zip(bins, hist)
        self.z_hist = hist
        self.z_hist_bins = bins
        
        slices = self.get_voi_slice_indices()
        self.z_hist_slices_indices = slices
        
        #precalculate spread values:
        self.z_hist_spread = []
        for indices in self.z_hist_slices_indices:
            a = self.processor.pts3d_bound[:,indices]
            u, ev12 = gaussian_curvature.spread(a)
            self.z_hist_spread += [(ev12[0], ev12[1])]
        
        #create h,s histograms:
        pts_h = []
        pts_s = []
        #print self.processor.pts3d_bound
        n,m = np.shape(np.asarray(self.processor.pts3d_bound))
        #print 'm',m,'len(self.processor.pts3d_bound[2,:].A1)',len(self.processor.pts3d_bound[2,:].A1)
        for index in range(m):
            pts_h.append(float(self.imNP_h[self.processor.map2d[1,index],self.processor.map2d[0,index]]))
        for index in range(m):
            pts_s.append(float(self.imNP_s[self.processor.map2d[1,index],self.processor.map2d[0,index]]))
        pts_i = np.asarray(self.processor.intensities_bound)
        #print 'ptsi',pts_i
        if np.max(pts_i) > 0:
            self.intensity_normalization_factor = 1.0 / float(np.max(pts_i)) * 255
        else:
            self.intensity_normalization_factor = 1.
        #print 'self.intensity_normalization_factor', self.intensity_normalization_factor
        #print pts_i
        pts_i *= self.intensity_normalization_factor
        pts_h = np.asarray(pts_h)
        pts_s = np.asarray(pts_s)
        self.z_hist_h_hists = []
        self.z_hist_s_hists = []
        self.z_hist_i_hists = []
        
        #normalize by maximum slice:
        max_count = 0
        max_count_index = 0
        for count_idx, indices in enumerate(slices):
            n = np.shape(indices)
            if n[0] > max_count:
                max_count = n[0]
                max_count_index = count_idx
        slize_height = (self.voi_interval_size / float(self.voi_bincount))
        self.z_hist_height_max =  slize_height * (max_count_index + 0.5)
        #print 'max_count', max_count,'index',max_count_index, 'height in max bin', self.z_hist_height_max

        
        for indices in slices:
            pts_h_slice = pts_h[indices]
            pts_s_slice = pts_s[indices]
            pts_i_slice = pts_i[indices]
            self.hsi_hist_bincount = 5
            bins = np.asarray(range(0,self.hsi_hist_bincount))*float(255.0/float(self.hsi_hist_bincount))
            #print bins
            #todo: smooth with kernel fct
            count = float(len(pts_h_slice))
            if count == 0: 
                count = 1
            hist_h = stats.histogram2(pts_h_slice,bins) / count
            self.z_hist_h_hists.append(hist_h)
            hist_s = stats.histogram2(pts_s_slice,bins) / count
            self.z_hist_s_hists.append(hist_s)
            hist_i = stats.histogram2(pts_i_slice,bins) / count
            #print 'hist_i', hist_i, pts_i_slice, bins, pts_i
            self.z_hist_i_hists.append(hist_i)
        
        #print 'hh',self.z_hist_h_hists
        #print 'sh',self.z_hist_s_hists
        #print 'ih',self.z_hist_i_hists
        
    def get_voi_pts_indices(self, poi, width):
        pts = np.asarray(self.processor.pts3d_bound)
        #region of interest:
        conditions = np.multiply(np.multiply(np.multiply(np.multiply(np.multiply(pts[0] < poi[0]+width/2.0, pts[0] > poi[0]-width/2.0), 
                    pts[1] < poi[1]+width/2.0), pts[1] > poi[1]-width/2.0),
                    pts[2] < poi[2]+width/2.0), pts[2] > poi[2]-width/2.0)
                    
        indices = np.where(conditions)[0]
        return indices
        
    def get_voi_slice_indices(self):
        
        slices = []
        last_z = -999999
        
        for z in self.z_hist_bins:
            indices = copy.copy(self.voi_pts_indices)
            pts = self.voi_pts   
            conditions = np.multiply(pts[2] < z, pts[2] > last_z)
            indices = indices[np.where(conditions)[0]]     
            slices += [indices]
            last_z = z
        return slices
        
    def get_voi_histogram_value(self, point):
        z = point[2]
        z = int(z*self.voi_bincount / float(self.voi_interval_size))
        if z >= 0 and z < self.voi_bincount:
            # print z, self.z_hist[z]
            return self.z_hist[z]
        else:
            #print z,0
            return 0
        
    def get_voi_histogram_spread(self, point):
        z = point[2]
        z = int(z*self.voi_bincount / float(self.voi_interval_size))
        if z >= 0 and z < self.voi_bincount:
#            indices = self.z_hist_slices_indices[z]
#            a = self.processor.pts3d_bound[:,indices]
#            u, ev12 = gaussian_curvature.spread(a)
#            if abs(self.z_hist_spread[z][0] - ev12[0]) > 0.0000000001 or abs(self.z_hist_spread[z][1] - ev12[1]) > 0.0000000001:
#                print 'ERROR', self.z_hist_spread[z], '!=', (ev12[0], ev12[1])
#            return ev12[0], ev12[1]
            return self.z_hist_spread[z]
        else:
            #print z,0
            return 0, 0
        
        
    def get_voi_hsi_histogram_values(self, point,h ,s, i):
        z = point[2]
        z = int(z*self.voi_bincount / float(self.voi_interval_size))
        if z >= 0 and z < self.voi_bincount:
            h_index = int(h * self.hsi_hist_bincount / 255.0)
            s_index = int(s * self.hsi_hist_bincount / 255.0)
            i *= self.intensity_normalization_factor
            i_index = int(i * self.hsi_hist_bincount / 255.0)
            
            h_hist = self.z_hist_h_hists[z][h_index]
            s_hist = self.z_hist_s_hists[z][s_index]
            #print 'z',z,'i_index',i_index, i
            #print self.z_hist_i_hists, np.shape(self.z_hist_i_hists)
            i_hist = self.z_hist_i_hists[z][i_index]
            return h_hist, s_hist, i_hist
        else:
            #print z,0
            return 0, 0, 0
        
    def get_voi_maxcount_height(self):
        return self.z_hist_height_max
    
    
