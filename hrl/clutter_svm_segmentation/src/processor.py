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

#  \author Jason Okerman & Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)

# -----------------------
#Define Global file locations. These must be modified per user.
LOC_ARTAGS_SIMPLE = "/home/martin/artags/ARToolKitPlus_2.1.1/bin/simple"
LOC_ARTAGS_LIB = "/home/martin/artags/ARToolKitPlus_2.1.1/lib:/usr/local/lib:"
# ----------------------

import roslib; roslib.load_manifest('opencv2')
import cv as roscv #Version 2.1: Used for image functions which became difficult in 2.0
import opencv as cv #Version 2.0: Used for everything else, out of convenience
from opencv import highgui

from gaussian_histogram_features import gaussian_histogram_features
from boosted_tree_classifier import boosted_tree_classifier
from baseline_classifier import baseline_classifier

import scans_database
import ransac
import hrl_lib.util as ut  #formerly local 'util.py'
import util as u2
#formerly hrl_lib.util.  Uses: ut.getTime, load_pickle, save_pickle, 
#  also obsolete functions/defs: num2pil, cv2np, cv2np_type_dict, cv2np_type_dict_invertible

import numpy as np
import math, re, copy, subprocess

import tf #ros.tf for transforms
#import hrl_lib.transforms as tr

#These should be optional
import matplotlib.pyplot as plt
import scipy.stats as stats
import scipy.spatial.kdtree as kdtree

###Optional Imports Below: 
#    import roslib; roslib.load_manifest('hrl_tilting_hokuyo')
#    import hrl_tilting_hokuyo.processing_3d, [only to "create_pointcloud()" from laser_scans ]
#    import mekabot.coord_frames as mcf [ for thok0Tglobal(mcf.globalTutmcam0(line, angle)) when device==CodyRobot]
#    import dummyScanner_coord_frames as dcf [for thok0Tglobal(dcf.globalTutmcam0(line, angle)) when device==DummyScanner
#    import roslib; roslib.load_manifest('hrl_hokuyo') [for pickels which have hokuyo objects]

#Below: attempts to use mlab
try: 
    from enthought.mayavi import mlab
    MLAB_LOADED = True
except:
    print "Warning: Failed to import enthought.mayavi.mlab. Ignoring call to 'draw_plane'."
    MLAB_LOADED = False

#class labels:
LABEL_NONE = 0
LABEL_SURFACE = 1
LABEL_CLUTTER = 2
LABEL_ROI = 3



def get_voi_indices_fancy(pts, poi, depth, width, height):
    pts = np.asarray(pts)
    #region of interest:
    conditions = np.multiply(np.multiply(np.multiply(np.multiply(np.multiply(pts[0] < poi[0]+depth/2.0, pts[0] > poi[0]-depth/2.0), 
                pts[1] < poi[1]+width/2.0), pts[1] > poi[1]-width/2.0),
                pts[2] < poi[2]+height/2.0), pts[2] > poi[2]-height/2.0)
    return conditions

#define it here as it's used in the gaussian_histogram_features
def rotate_to_plane(normal, points):
    n = np.copy(normal)
    n = n/np.linalg.norm(n)
    up = np.matrix([0,0,1]).T
    axis = np.matrix(np.cross(up.T, n.T)).T
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos((up.T * n)[0,0])
    rot = tf.transformations.rotation_matrix(angle,axis)
    rot = rot[0:3,0:3].T
    #print np.shape(rot), np.shape(points)
    pts_rotated = np.asarray(rot * np.asmatrix(points))
    return pts_rotated

def apply_transform_matrix(T, p):
    '''This appears multiply a Transform matrix by a 3xN-element point or point set.
       Note that xyzToHomogenous is made to convert 3XN matrix, to 4XN matrix in homogenous coords
    '''
    pn = T * xyzToHomogenous(p)
    #pn = T * np.row_stack((p, np.ones(p.shape[1])))
    return pn[0:3] / pn[3]
    

def xyzToHomogenous(v, floating_vector=False):
    """This is redundantly defined in hrl_lib.transforms.py, as part of gt-ros-pkg
       convert 3XN matrix, to 4XN matrix in homogenous coords
    """
#   v = np.matrix(v)
#   v = v.reshape(3,1)
    if floating_vector == False:
        return np.row_stack((v, np.ones(v.shape[1])))
    else:
        return np.row_stack((v, np.zeros(v.shape[1])))

def draw_plane(table_plane_normal, table_plane_point):
    if not MLAB_LOADED: return #ignore 3D plot commands if unavailable
    
    table_plane_normal = np.copy(table_plane_normal)
    table_plane_normal /= np.linalg.norm(table_plane_normal)
    
    #line originating at [0,0,0].T
    def intersect(line, plane_normal, plane_point):
        gamma = (np.dot(plane_normal.T, plane_point)/np.dot(plane_normal.T, line))[0,0]
        #print 'gamma',gamma
        return gamma * line
    
    p1 = intersect(np.matrix([[0,1,1]]).T,table_plane_normal,table_plane_point)
    p2 = intersect(np.matrix([[0,-1,1]]).T,table_plane_normal,table_plane_point)
    p3 = intersect(np.matrix([[1,1,1]]).T,table_plane_normal,table_plane_point)
    p4 = intersect(np.matrix([[1,-1,1]]).T,table_plane_normal,table_plane_point)
    
    mlab.triangular_mesh([[float(p1[0]),float(p2[0]),float(p3[0]),float(p4[0])]],
              [[float(p1[1]),float(p2[1]),float(p3[1]),float(p4[1])]],
              [[float(p1[2]),float(p2[2]),float(p3[2]),float(p4[2])]],
              [(0,1,2),(1,2,3)], opacity=.2, colormap='gray')

'''
   Key Functions: 
       create_pointcloud --> creates a pts3d cloud in frame of hokuyo from hokuyo laserscans.  
           - Heavily depends on hokuyo library classes to do this.  Preserves intensity field.
   
   
   
   Key Variables: 
       map = camPts_bound, camPts, idx_list, pts3d_bound, scan_indices_bound, intensities_bound
       map2d = first two parts of map = camPts_bound = (x,y) list of length Nbound
            * originally map[0] == camPts_bound had a '1' in the 3rd dimention.
       map_polys == poly_labels_bounded
       pts3d
       laserscans
       intensities
       
       camPts_bound
       idx_list, #relative to full-length lists: intensity or pts3d 
       pts3d_bound, # 3xNBound
       scan_indices_bound, #length Nbound
       intensities_bound #length Nbound

'''

class processor(object):
    '''
    classdocs
    '''
    pts = None
    
    classifiers = None

    features = None

    feature_type = 'gaussian_histograms'
    feature_neighborhood = 20 #unused! just appears in filenames. TODO: remove and replace by features_k_nearest_neighbors
    feature_radius = 0.03

    classifier_training_size = 900000000 #use all == a very large number
    
    #define volume of interest
    point_of_interest = np.array([0.8,0.0,1.0])
    voi_width = 1.4
    
    #do not label points that are too close to the ground:
    ground_exclude_threshold = 0.3 # 30 cm above 'ground'
    
    #use k nearest neighbors for features or all points in a 3cm sphere in NN = None
    #50 seems to be a good value, 20 seems to be too small, None gives better results (about 5percent crossvalidation in total) but is way slower
    features_k_nearest_neighbors = 50 #None
    

    def __init__(self, configuration):
        '''
        Constructor
        '''
        self.features = gaussian_histogram_features(self)
        self.classifiers = {'range' : boosted_tree_classifier(self, 'range'),
                            'color' : boosted_tree_classifier(self, 'color'),
                            'all' : boosted_tree_classifier(self, 'all'),
                            'baseline' : baseline_classifier(self, 'baseline')}
        
        self.config = configuration
        
        self.artag_transformation = np.array([])
        
        self.img_artag = False
        self.map = None
        
        try:
            self.scans_database = scans_database.scans_database()
            self.scans_database.load(self.config.path,'database.pkl')
        except IOError:
            print 'WARNING: processor::init: no database found none loaded'
        

    '''
        This function creates a pointcloud from laser-scan line (distance) and angle data.  
        The conversion relies on method definition in hrl_tilting_hokuyo.py.  This is NOT
        included by default because the PR2 does not require this conversion (has its own).
        This function also checks the self.config.device name in order to decide whether to 
        flip the y-axis as a last step.
        
        @Param reject_zero_ten: Set reject_zero_ten to False to not remove any points and
         preserve intensity->3d mapping.  Be aware that points filtered by remove-graze-effect
         have range-values of +/-1000 mapped into 3d-space
    '''
    def create_pointcloud(self,laserscans, reject_zero_ten=True, get_intensities=True):
        import roslib; roslib.load_manifest('hrl_tilting_hokuyo')
        import hrl_tilting_hokuyo.processing_3d as hokuyo_p3d
    
        pts = np.array([[],[],[]])
        intensities = np.array([[]])
        scan_indices = np.array([])
        for i, laserscan in enumerate(laserscans):
            pos_list,scan_list = laserscan
            
            if get_intensities:
                new_pts, new_intensities = hokuyo_p3d.generate_pointcloud(pos_list, scan_list,
                          math.radians(-180), math.radians(180),
                          l1=self.config.thok_l1, l2=self.config.thok_l2, save_scan=False,
                          max_dist=np.Inf, min_dist=-np.Inf,get_intensities=True, reject_zero_ten=reject_zero_ten)
                intensities = np.hstack((intensities,new_intensities))
            else:
                new_pts = hokuyo_p3d.generate_pointcloud(pos_list, scan_list,
                          math.radians(-180), math.radians(180),
                          l1=self.config.thok_l1, l2=self.config.thok_l2, save_scan=False,
                          max_dist=np.Inf, min_dist=-np.Inf,get_intensities=False, reject_zero_ten=reject_zero_ten)
            
            pts = np.hstack((pts,new_pts))
            
            
            #save index of scan with points:
            n, m = new_pts[0,:].shape
            scan_indices = np.hstack((scan_indices,np.ones(m)*i))
            
        if self.config.device == 'codyRobot' or self.config.device == 'dummyScanner':
            pts[1,:] = -pts[1,:] #flip Y axis
            
        return (pts, scan_indices, np.asarray(intensities)[0])     
    
    '''  TODO: Consistency - stop using 'map'
        Internal: Modifies length of: pts3d_bound, intensities_bound, map2d == camPts_bound, 
            Modifies idx_list (still will have length N, but now only Nbound_new 'True' entries.
            idx_list - length N
            idx_voi - only length Nbound
            idx_list = way to get from <...> to <...>_bound. 

        Truncates the full pts3d (generally much larger then table in span and shape 3xN)
        to tighter cubic region called pts3d_bound.  Saved as a numpy array of shape 3xNbound_new
    '''
    def truncate_pointcloud_to_voi(self,poi, depth, width, height):
        #Get True/False list (indices) of volume of interest:
        pts = np.asarray(self.pts3d_bound)
        _, Nbound = pts.shape
        (x,y,z) = (0,1,2)
        idx_voi = (pts[0] < poi[0]+depth/2.0) & (pts[0] > poi[0]-depth/2.0) & (pts[1] < poi[1]+width/2.0) & (pts[1] > poi[1]-width/2.0) & (pts[2] < poi[2]+height/2.0) & (pts[2] > poi[2]-height/2.0)

        self.idx_voi = idx_voi #Save the idx with respect to original point cloud.
        self.pts3d_bound = self.pts3d_bound[:,idx_voi]
        _, Nbound_new = np.shape(self.pts3d_bound)
        print 'Truncated pts3d_bound by VOI from',Nbound,'to',Nbound_new,'points'
        if Nbound_new == 0:
            print "WARNING: all 3D points removed by VOI cropping.  Exiting."
            import sys; sys.exit()

        self.intensities_bound = self.intensities_bound[idx_voi]
        self.camPts_bound = self.camPts_bound[:,idx_voi]
        self.map2d = self.camPts_bound #also will be shortened.
        # self.map_polys = self.map_polys[idx_voi] #probably not defined yet
        #Cut a view and then set to False entries outside VOI
        self.idx_list[self.idx_list] = self.idx_list[self.idx_list] & idx_voi
        
        ###self.map[0] = np.asarray(self.map[0])[:,idx_voi] #TODO: remove 'map', Hacked
        #--------------------------------------------

    def get_ransac_table_plane_from_labels(self):
        #ransac settings:
        #    n - the minimum number of data values required to fit the model
        #    k - the maximum number of iterations allowed in the algorithm
        #    t - a threshold value for determining when a data point fits a model
        #    d - the number of close data values required to assert that a model fits well to data
        labels = self.map_polys
        model = ransac.PlaneLeastSquaresModel(False)
        data_idx = np.where(np.asarray(labels) == LABEL_SURFACE)[0]
        data = np.asarray(self.pts3d_bound).T[data_idx]
        print 'About to run RANSAC plane with %i labeled points' %(len(data)) 
        # run RANSAC algorithm
        try:
            model = ransac.ransac(data,model,
                                        #n,  k,   t,  d
                                        #3, 1000, 0.01, len(data_idx)/1.5, # misc. parameters
                                        #3, 1000, 0.04, len(data_idx)/2.5, #works for CODY
                                        #3, 1000, 0.08, len(data_idx)/4 #Generous.  Works on PR2
                                        3, 1000, 0.08, len(data_idx)/4, #Try for PR2
                                        debug=False,return_all=False)
        except ValueError:
            #did not meet fit acceptance criteria
            print '[processor] Best ransac plane did not fit threshold criteria listed in processor.py'
            return False
        print 'ransac: model',model
        return model 
    
    def calculate_and_save_ground_and_table_transformations_for_all_scans(self, use_RANSAC_table_plane = True):
        #set all planes to zero:
        self.scans_database.add_attribute_to_every_dataset('ground_plane_rotation')
        self.scans_database.add_attribute_to_every_dataset('ground_plane_translation')
        self.scans_database.add_attribute_to_every_dataset('table_plane_translation')
        self.scans_database.save()
        self.scan_dataset = self.scans_database.get_dataset(0)
        while False != self.scan_dataset:
            if self.scan_dataset.is_labeled:
                
                self.load_data(self.scan_dataset.id, False)
                self.process_raw_data(False) #do not translate
                
                rotation = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
                rotation = rotate_to_plane(self.scan_dataset.ground_plane_normal, rotation)
                ransac_plane = False
                if use_RANSAC_table_plane:
                    ransac_plane = self.get_ransac_table_plane_from_labels()
                if False != ransac_plane:
                    table_plane_point = np.matrix(ransac_plane[0]).T
                    table_plane_normal = np.matrix(ransac_plane[1]).T
                    rotation = rotate_to_plane(table_plane_normal, rotation)
                    table_plane_point = rotate_to_plane(table_plane_normal,table_plane_point)
                elif False == use_RANSAC_table_plane:
                    print "WARNING: Do NOT USE RANSAC FOR TABLE PLANE ESTIMATION!!!!"
                    print "using 3pts-groundplane and manual labeled surface height instead"
                else:
                    print "!!!!!!!!"
                    print "ERROR: NO RANSAC PLANE FOUND!!!!"
                    print "using 3pts-groundplane and manual labeled surface height instead"
                    print "!!!!!!!!"
                #hack: reload and save to prevent pickling of edge and polygon images (=> error)
                self.load_data(self.scan_dataset.id, True)
                self.scans_database.set_internal_pointer_to_dataset(self.scan_dataset.id)
                #set values:
                self.scan_dataset.ground_plane_rotation = rotation
                self.scan_dataset.ground_plane_translation = self.get_groundplane_translation() #call after rotation has been set!
                if False != ransac_plane:
                    table_plane_point += self.scan_dataset.ground_plane_translation 
                    self.scan_dataset.table_plane_translation = np.matrix([0.0,0.0,table_plane_point[2]]).T
                else:
                    #no ransac was possible -> use dummy:
                    self.scan_dataset.table_plane_translation = np.matrix([0.0,0.0,float(self.scan_dataset.surface_height) / 100.0]).T
                print 'rot', self.scan_dataset.ground_plane_rotation
                print 'gp transl',self.scan_dataset.ground_plane_translation
                print 'table transl', self.scan_dataset.table_plane_translation
                self.scans_database.save()
            self.scan_dataset = self.scans_database.get_next_dataset()
    
    def get_mean_table_plane_height_from_labels(self):
        idx = np.where(np.asarray(self.map_polys) == LABEL_SURFACE)
        return np.mean(self.pts3d_bound[2,idx])


    #needs np.matrix as input
    def map_image_point_on_3d_plane(self, point, ground_plane_rotation, ground_plane_translation, table_plane_translation):
        #just to be sure: copy everything for now (may not be necessary for all variables!)
        point = np.matrix(np.copy(point))
        ground_plane_rotation = np.matrix(np.copy(ground_plane_rotation))
        ground_plane_translation = np.matrix(np.copy(ground_plane_translation))
        table_plane_translation = np.matrix(np.copy(table_plane_translation))
        
        #translate 2d point w.r.t. camera center
        #print self.config.cam_centers
        #print point
        point[0][0] -= self.config.cam_centers[0] 
        point[1][0] -= self.config.cam_centers[1] 

        #create camera->3d projection matrix
        cam_proj_inv = np.copy(self.config.cam_proj_mat)
        cam_proj_inv[0:3,0:3] = np.linalg.inv(cam_proj_inv[0:3,0:3])

        #project 2d point to line from origin in 3d
        line = cam_proj_inv * xyzToHomogenous(point)
        
        #create projection matrix from camera COS to laser COS
        
        if self.config.device == 'codyRobot':
            #transform line into laser COS
            import mekabot.coord_frames as mcf
            line = mcf.thok0Tglobal(mcf.globalTutmcam0(line,self.image_angle))
        elif self.config.device == 'dummyScanner':
            import dummyScanner_coord_frames as dcf
            line = dcf.thok0Tglobal(dcf.globalTutmcam0(line,self.image_angle))
        else: #Desktop Scanner -- Now PR2
            laser_to_camera = self.config.camTlaser
            camera_to_laser = np.copy(laser_to_camera)
            camera_to_laser = np.linalg.inv(camera_to_laser)
            #transform line into laser COS
            line = apply_transform_matrix( camera_to_laser, line)
        
        #rotate line around ground plane
        line = ground_plane_rotation * line
        
        #translate table plane w.r.t. groundplane
        table_plane_point = table_plane_translation - ground_plane_translation
        
        #assume everything is rotated so that resulting table plane is horizontal
        table_normal_up = np.matrix([[0,0,1]]).T
        
        gamma = (np.dot(table_normal_up.T, table_plane_point)/np.dot(table_normal_up.T, line))[0,0]
        point3d = gamma * line
        
        point3d += ground_plane_translation
        return point3d

    '''
        This function has  different behavior based on the robot platform specified.
        This is because it needs the camera properties info, as well as static transforms.
        * Transforms 3D points to the coord fram of camera (camTlaser)
        * Sets camera points from 3D points  
        
        crops 3d points to "bounds" on camera.  All "bounded" lists are cropped in this way.
        
        Inputs: pts3d, img, scan_indices, intensities
        Outputs: camPts_bound, #list of (u,v) coords of each 3d points (2xNbound numpy matrix)
               - camPts,       #2xN numpy matrix
               - idx_in_cam    #relative to full pts3d,
               - pts3d_bound, 
                               #Cropped to a 3xNbound array where Nbound is number
                                of 3d points within frame of camera.
               - scan_indices_bound,
               - intensities_bound
        *Note: some material borrowed from Travis-scanr:
    ''' #TODO: Add cropping to this function.  Most efficient place.

    def map_laser_into_cam_2D(self, pts3d, img, scan_indices, intensities):
        # Project self.pts into image, and display it
        
        ###imgTmp = cv.cvCloneImage(img)
        ###imNP = cv.adaptors.Ipl2NumPy(imgTmp)
        w = img.width
        h = img.height

        if self.config.device == 'PR2':
            print 'Warning, still working on best way to do this'
            #somehow do this transformation.  Assume camTlaser is available.
            XformPts = apply_transform_matrix( self.config.camTlaser, pts3d)
        elif self.config.device == 'codyRobot':
            import mekabot.coord_frames as mcf
            XformPts = mcf.utmcam0Tglobal(mcf.globalTthok0(pts3d),self.image_angle)
        elif self.config.device == 'dummyScanner':
            import dummyScanner_coord_frames as dcf
            XformPts = dcf.utmcam0Tglobal(dcf.globalTthok0(pts3d),self.image_angle)
        else: #Desktop Scanner -- Now PR2
            XformPts = apply_transform_matrix( self.config.camTlaser, pts3d)
        
        (x,y) = (0,1) #separate 2D dimentions
        camPts = self.config.cam_proj_mat * xyzToHomogenous(XformPts)
        camPts = camPts / camPts[2]
        #camPts = np.row_stack((camPts[0:2]/camPts[2],camPts[2]))
        camPts[x] = camPts[x] + self.config.cam_centers[x]
        camPts[y] = camPts[y] + self.config.cam_centers[y]
        camPts = np.matrix( np.round(camPts), 'int')
        
        #Crop to points that fall within camera bounds.  Call this True/False array 'idx_list'
        #  * Cropping is appropriate here as well.
        conditions = (camPts[x] >=0) | (camPts[x] < w) | (camPts[y] >= 0) | (camPts[y] < h)
        idx_list = conditions
        
        self.camPts = camPts[0:2] # Eliminate off 3rd normalized dimention.
        self.camPts_bound  = camPts[:, idx_list] # Shorter 3xNbound array
        self.pts3d_bound = pts3d[:, idx_list] # Point cloud bounded by camera frame of view
        self.scan_indices_bound = scan_indices[idx_list]
        self.intensities_bound = intensities[idx_list]
        self.idx_list = idx_list
        self.map2d = self.camPts_bound  #Formerly, self.map2d = np.asarray(map[0][0:2])
        
        return self.camPts_bound, self.camPts, idx_list, self.pts3d_bound, self.scan_indices_bound, self.intensities_bound

    #TODO: change 'map' to be only self.map2d or self.camPts_bound
    def draw_mapped_laser_into_image(self,map,pts3d,img):
        '''colormap points in image
           Note: map[2] == self.idx_list #array of indices inside bounds
        '''
        imgTmp = cv.cvCloneImage(img)
        imNP = u2.cv2np(imgTmp,format='BGR')
    
        ###self.map2d = np.asarray(self.camPts_bound)
        n,m = self.map2d.shape
        color = (0,255,0)
        for i in range(0,m):
            imNP[self.map2d[1,i],self.map2d[0,i],:] = color
#            imNP[map2d[1,i],map2d[0,i],0] = min(255,x[i])
#            imNP[map2d[1,i],map2d[0,i],1] = max(0,255-x[i])
#            imNP[map2d[1,i],map2d[0,i],2] = max(0,512-x[i])
    
        img_mapped = u2.np2cv(imNP)
        return img_mapped
    ###
    
    
    def draw_mapped_masks_into_image(self, feature_type, show_clutter=False):
        '''@author: Jason Okerman
           prints either clutter or surface points on a black backdrop for use
        '''
        #labels_groundtruth = self.map_polys
        if feature_type in ('range','color','all','all_post','baseline'):
            labels = self.load_Classifier_labels(feature_type)
        elif feature_type == 'labels':
            labels = self.map_polys
        else:
            print ut.getTime(), 'ERROR: draw_mapped_masks_into_image(): unsupported feature type:', feature_type
            return cv.cvCloneImage(self.img)
        
        #img_mapped = cv.cvCreateImage(cv.cvSize (self.img.width, self.img.height),8,1)
        try:
            img_mapped = highgui.cvLoadImage(self.config.path+'/data/'+'blank.png')
        except: 
            print "blank.png does not exist in data directory"
            img_mapped = cv.cvCloneImage(self.img)
            
        ###self.map2d = np.asarray(self.camPts_bound)
        n,m = self.map2d.shape
        red=100;
        green=0
        blue=0
        for i in range(0,m):
            if labels[i] == LABEL_SURFACE and not show_clutter:
                if red < 255: 
                    red+=1
                else:
                    red = 100
                    if green <250:
                        green+=25
                    else:
                        green = 100
                        if blue <250:
                            blue+=50
                cv.cvCircle(img_mapped, cv.cvPoint(self.map2d[0,i],self.map2d[1,i]),2, cv.cvScalar(blue,red,green,0),3)
            elif labels[i] == LABEL_CLUTTER and show_clutter:
                if red < 255: 
                    red+=1
                else:
                    red = 0
                    if green <255:
                        green+=20
                cv.cvCircle(img_mapped, cv.cvPoint(self.map2d[0,i],self.map2d[1,i]),2, cv.cvScalar(0, red, green, 0),3)
                
        return img_mapped 
    ###
    
    ''' DRAW MAPPED LABELS INTO IMAGE
        Inputs:
           (passed) type = 'range', 'color', 'all', 'all_post', 'baseline'
           (internal) self.img
           (internal) self.map
            - However only uses the camPts_bound portion of self.map (called map2d)
        Outputs: 
            image with color circle indicators. 
        Relies on the ability to load saved classifier information.
    '''
    def draw_mapped_labels_into_image(self, type):
       
        #labels_groundtruth = self.map_polys
        img_mapped = cv.cvCloneImage(self.img)
        #TODO: If classifier labels are already in ram, use that.        
        if type == 'labels':
            labels = self.map_polys
        elif type == 'range':
            labels = self.load_Classifier_labels('range')
        elif type == 'color':
            labels = self.load_Classifier_labels('color')
        elif type == 'all':
            labels = self.load_Classifier_labels('all')
        elif type == 'all_post':
            labels = self.load_Classifier_labels('all_post')
        elif type == 'baseline':
            labels = self.load_Classifier_labels('baseline')
        else:
            print ut.getTime(), 'WARNING: no method for drawing selected feature type:', type
            print 'Ignoring call draw labels and returning original image.'
            return img_mapped
            
        ###self.map2d = np.asarray(self.map[0][0:2])
        _, m = self.map2d.shape #A 3xNbound vector
        for i in range(0,m):
            if labels[i] == LABEL_SURFACE:
                #if labels[i] == labels_groundtruth[i]:
                try:
                    #cv.cvCircle(img_mapped, cv.cvPoint(self.map2d[0,i],self.map2d[1,i]),3, cv.cvScalar(0, 255, 0))
                    # Note: Python supports drawing shapes and passing arrays instead of
                    #       cvPoint and cvScalar objects.
                    cv.cvCircle(img_mapped, (self.map2d[0,i],self.map2d[1,i]), 3, (0, 255, 0))
                except:
                    print 'having a hard time printing circles'
                    temp = np.array(img_mapped)
                    roscv.Circle(temp, (self.map2d[0,i],self.map2d[1,i]),2, (0, 255, 0))
                    img_mapped = cv.cvCloneImage(temp)
            elif labels[i] == LABEL_CLUTTER:
                #if labels[i] == labels_groundtruth[i]:
                try:
                    cv.cvCircle(img_mapped, (self.map2d[0,i],self.self.map2d[1,i]), 3, (0, 0, 255))
                except:
                    print 'having a hard time printing circles'
                    temp = np.array(img_mapped)
                    roscv.Circle(temp, (self.map2d[0,i],self.map2d[1,i]),2, (0, 0, 255))
                    img_mapped = cv.cvCloneImage(temp)
        return img_mapped 
    

    
    def draw_mapped_laser_polygons_into_image(self,map,pts3d,img):
        #colormap points in image
        #print ut.getTime(), map[2]#==idx_list #array of indices inside bounds
        color_list = [(255,255,0),(255,0,0),(0,255,255),(0,255,0),(0,0,255),(0,100,100),(100,100,0),
                  (100,0,100),(100,200,100),(200,100,100),(100,100,200),(100,0,200),(0,200,100),
                  (0,100,200),(200,0,100),(100,0,100),(255,152,7) ]
    
        imgTmp = cv.cvCloneImage(img)
        imNP = u2.cv2np(imgTmp,format='BGR')
    
        ###self.map2d = np.asarray(map[0][0:2])
        n,m = self.map2d.shape
        
        for i in range(0,m):

            ximg = self.map2d[1,i]
            yimg = self.map2d[0,i]
            
            imNP[ximg,yimg,0] = 255
            imNP[ximg,yimg,1] = 255
            imNP[ximg,yimg,2] = 255            
            
            for index, polygon in enumerate(self.scan_dataset.polygons):
                if len(polygon.get_points()) and polygon.get_type() == 'polygon':
                       
                    color_index = min(index, len(color_list)-1)
                    if 255 == polygon.cvImage[ximg][yimg]:
                        imNP[ximg,yimg,0] = color_list[color_index][0]
                        imNP[ximg,yimg,1] = color_list[color_index][1]
                        imNP[ximg,yimg,2] = color_list[color_index][2]
    
        img_mapped = u2.np2cv(imNP)
        return img_mapped   
    
     
    #return array that can be used by mayavi for coloring:
    # TODO-note: This has nothing to do with laser   
    def map_polygons_into_laser(self, map=None):
        n, m = self.pts3d_bound[0,:].shape
        polygon_mapping = list(0 for x in range(m))
        
        ###map2d = np.asarray(self.camPts_bound)
        Ndim, N = self.map2d.shape
    
        for i in range(0,m):
            ximg = self.map2d[1,i]
            yimg = self.map2d[0,i]
            
            poly_count = 1
            for polygon in self.scan_dataset.polygons:
                if len(polygon.get_points()) and polygon.get_type() == 'polygon':
                    poly_count = poly_count + 1
                       
                    if 255 == polygon.cvImage[ximg][yimg]:
                        polygon_mapping[i]  = poly_count
            
        return polygon_mapping
    
    '''
        Internal. Uses: self.map2d, self.scan_dataset.is_labeled, self.
        
        polygon_labels_bound # formerly called 'polygon_mapping' or 'map_polys'
            
            (?) If Exclude_edges is True, then boundaries of photo and table surface will be 
            tightened so that this does not introduce any strange artifacts (hopefully).
        Returns an array of length N, where 0 = unlabeled, 1 = surface, 2 = clutter
        
        This function should be used for hand-labeled data which is
          specified as Polygons (list of (x,y) pairs).  Used as ground truth when training/verifying.
        These polygons most likely are NOT closed so need to be auto-closed.
        They are given identifiers: 'surface', 'object' and 'rio'.  
        Formerly called: map_polygon_labels_into_laser
    ''' #TODO-note: This could all be done in one swoop with a 'multi-colored image' and a 'single-colored mask'
    
    def create_polygon_labels(self, map=None, exclude_edges = False):
        _, Nbound = np.shape(self.pts3d_bound)
        polygon_labels_bound = np.zeros(Nbound) #list(0 for i in range(Nbound)) 
            # 0 == background/floor/unlabeled/other
        
        #Catch to distinguish new scans and hand-labeled sets.
        if self.scan_dataset.is_labeled == False:
            return polygon_labels_bound      
              
        ### Asume map2d = np.asarray(self.camPts_bound)
        _, Nbound = self.map2d.shape
        
        roi_polygons = []
        surface_polygons = []
        object_polygons = []
        for p in self.scan_dataset.polygons: #scroll through list of polygons
                if len(p.get_points()) and p.get_type() == 'polygon': #else ignore, or is empty []
                    if p.get_label() == 'roi': roi_polygons += [p]
                    if p.get_label() == 'surface': surface_polygons += [p]
                    if p.get_label() == 'object': object_polygons += [p]
        #---
        for i in range(Nbound):
            ximg = self.map2d[1,i]
            yimg = self.map2d[0,i]
            
            #first ROI
            for p in roi_polygons:
                if 255 == p.cvImage[ximg][yimg] and (exclude_edges == False or 255 != self.scan_dataset.cvEdgeImage[ximg][yimg]):
                    polygon_labels_bound[i] = LABEL_ROI #region of interest for testing 

            #surface second
            #always exclude surface edges
            for p in surface_polygons:
                bool1 = (255 == p.cvImage[ximg][yimg])
                bool2 = (255 != self.scan_dataset.cvEdgeImage[ximg][yimg])
                if bool1 and (exclude_edges == False or bool2):
                    #and 255 != self.scan_dataset.cvSurfaceEdgeImage[ximg][yimg]:
                    polygon_labels_bound[i] = LABEL_SURFACE #1 == surface        
                                       
            #clutter object layer has highest priority (on top)
            for p in object_polygons:
                if 255 == p.cvImage[ximg][yimg] and (exclude_edges == False or 255 != self.scan_dataset.cvEdgeImage[ximg][yimg]):
                    polygon_labels_bound[i] = LABEL_CLUTTER #2 == clutter
            
        return polygon_labels_bound
    #-------------------------
      
    def remove_labels_from_low_points(self):
        #do not label points that are too close to the ground:
        (z) = (2)
        idx = np.where(self.pts3d_bound[z,:] < self.ground_exclude_threshold)
        self.map_polys = np.asarray(self.map_polys)
        self.map_polys[idx] = LABEL_NONE
      
    #works
    def create_intensity_image(self, laserscan):

        pos_list, scan_list = laserscan

        for i,s in enumerate(scan_list):
            if 0 == i:
                imgNP = s.intensities
            else:
                imgNP = np.vstack((imgNP,s.intensities))
        
        imgNP = imgNP / np.max(imgNP) * 255
        imgNP = np.asarray(imgNP)
        imgNP = imgNP[::-1] #reverse = hflip as scanner starts at bottom
        
        return u2.np2cv(imgNP)
    
        
    def do_all_laser_image_mapping(self, translate = True):
        ### Evaluate camPts, map2d, etc. #Formerly:self.map = ...
        self.map_laser_into_cam_2D(self.pts3d, self.img, self.scan_indices, self.intensities)
        ###self.pts3d_bound = np.asarray(self.map[3])
        self.rotate_pointcloud_match_groundplane()
        if translate == True:
            self.z_translate_pointcloud_groundplane()
        ''' Sets the following:
            self.scan_indices_bound
            self.intensities_bound
        '''
        self.img_mapped = self.draw_mapped_laser_into_image(None, self.pts3d, self.img)  
        self.create_polygon_images()
        self.img_mapped_polygons = self.draw_mapped_laser_polygons_into_image(None, self.pts3d, self.img)    
        #Ground truth, hand-labels for classification
        self.map_polys = self.create_polygon_labels(self.map)
        
        if translate == True:
            self.remove_labels_from_low_points()
        
        self.img_intensities = self.create_intensity_image(self.laserscans[0])        
        
    #don't create any images but all information needed for the pointcloud
    def do_all_point_cloud_mapping(self, exclude_edges = False):
        self.do_all_point_cloud()
        self.do_polygon_mapping(exclude_edges)

        
    def do_polygon_mapping(self, exclude_edges = False):
        self.create_polygon_images()    
        self.map_polys = self.create_polygon_labels(None, exclude_edges)
        self.remove_labels_from_low_points()  
    
    '''
        Creates a map using map_laser_into_cam_2D
            - this is now internal (formerly standalone) 
              takes pts3d, img, scan_indices, and intensities.
        Sets pts3d_bound based on points that could project in image 
            - (returned as part of map)
        Does a z_translation and rotation (to groundplane) on pts3d_bound
            - intenal shift of pts3d and pts3d_bound
            - Amount to shift defined as var: scan_dataset.ground_plane_translation
            - If defined:
              Rotates by matrix in var: scan_dataset.ground_plane_rotation
            - Else if defined: 
              Rotates to match plane in var: scan_dataset.ground_plane_normal
        Sets intensities_bound and scan_indices_bound based on returned map
    ''' #TODO: There is no reason to return all as 'map', description is not intuative.
    
    def do_all_point_cloud(self, map_already_exists=False):
        #hack, allow this same function to NOT call map_laser_into_cam_2D
        if not map_already_exists:
            self.map_laser_into_cam_2D(self.pts3d, self.img, self.scan_indices, self.intensities)
            ''' Sets the following: 
                self.camPts_bound, 
                self.camPts, idx_list
                self.pts3d_bound, 
                self.scan_indices_bound
                self.intensities_bound
            '''
        self.rotate_pointcloud_match_groundplane()
        self.z_translate_pointcloud_groundplane()
    
    
    def load_data(self, name, reload_database = True):
        print ut.getTime(), 'processor: loading ', name
        self.load_metadata(name, reload_database)
        self.load_raw_data(name)                 
    '''
        In order to run on the PR2, we need to import data in slightly different form.
        This is the easiest way to accomodate that, create a separate loading function.
    ''' 
    def load_raw_PR2_data(self, unique_name=''):
        self.img = ''
        self.pts3d = [] # fix later if needed
        self.laserscans = []
        self.intensities = []
        self.scan_indices = []
        self.image_angle = 0
    '''
        This loads a laserscan from a pickel.  ***The pickel relys on the classes found in
        hrl_hokuyo/hokuyo_scan.py.  So we need to let python know where to find these.
        I am NOT including in manifest.xml because for PR2, you do NOT need this function
    '''
    def load_raw_data(self,name):
        import roslib; roslib.load_manifest('hrl_hokuyo') #***
        self.img = highgui.cvLoadImage(self.config.path+'/data/'+name+'_image.png')
        dict = ut.load_pickle(self.config.path+'/data/'+name+'_laserscans.pkl')
        self.laserscans = dict['laserscans']
        if 'image_angle' in dict:
            self.image_angle = dict['image_angle']
        else:
            self.image_angle = 0
       
    
    def display_all_data(self):
        #window_name = "Image"
        #highgui.cvNamedWindow (window_name, highgui.CV_WINDOW_AUTOSIZE)
        #highgui.cvShowImage (window_name, self.img)
        
        #window_name = "Image Mapped"
        #highgui.cvNamedWindow (window_name, highgui.CV_WINDOW_AUTOSIZE)
        #highgui.cvShowImage (window_name, self.img_mapped) 
        
        window_name = "Image Poly Mapped"
        highgui.cvNamedWindow (window_name, highgui.CV_WINDOW_AUTOSIZE)
        highgui.cvShowImage (window_name, self.img_mapped_polygons)     
        
        #window_name = "Image Intensities"
        #highgui.cvNamedWindow (window_name, highgui.CV_WINDOW_AUTOSIZE)
        #highgui.cvShowImage (window_name, self.img_intensities)                    
        
        #self.display_3d()
        #highgui.cvWaitKey(0)
        
        
    def display_segmentation_image(self, feature_type='all_post'):
        self.img_labels = self.draw_mapped_labels_into_image(feature_type)   
        window_name = "Image Labels for classifier "+feature_type
        highgui.cvNamedWindow (window_name, highgui.CV_WINDOW_AUTOSIZE)
        highgui.cvShowImage (window_name, self.img_labels)  
        #highgui.cvWaitKey(0)   
        
    def save_segmentation_image(self, feature_type='all_post', suffix=None):
        self.img_labels = self.draw_mapped_labels_into_image(feature_type) #'all_post'  
        if not suffix: suffix = '_image_segmentation_'+feature_type  
        filename = self.config.path+'/data/'+self.scan_dataset.id + suffix + '.png'
        print ut.getTime(), "Saving: "+filename
        highgui.cvSaveImage(filename,self.img_labels)
        
    def get_intensity_image(self):
        return self.img_intensities
        
    def display_intensities(self):
        window_name = "Image Intensities"
        highgui.cvNamedWindow (window_name, highgui.CV_WINDOW_AUTOSIZE)
        highgui.cvShowImage (window_name, self.img_intensities)                  
         
         
    def display_featurevector(self, featurevector):       
        fv_range = np.asarray(featurevector)[self.features.get_indexvector('range')]

        idx = np.asarray(range(len(self.features.get_indexvector('range'))))
        plot1 = plt.bar(idx,fv_range,color='b')  
        print 'fv_range',fv_range
        plt.show()  
            
         
    def display_stats(self, global_stats = False): 
        if not MLAB_LOADED: return #ignore 3D plot commands if unavailable
    
        import colorsys
        
        h_values_plot = []
        s_values_plot = []
        v_values_plot = []
        i_values_plot = []
        colors_plot = []   
        rgb_plot = []     
        
        range_hist_surface_plot = np.zeros(len(self.features.get_indexvector('range')))
        range_hist_clutter_plot = np.zeros(len(self.features.get_indexvector('range')))
        
        count_surface = 0
        count_clutter = 0
        
        
        normals_x = []
        normals_y = []
        normals_z = []
        normals_label = []
        
        measured_table_heights = []

        surface_heights = []
        clutter_heights = []
        
        
        filename = self.get_features_filename()
        dict = ut.load_pickle(filename)
        loop_condition = True
        first_loop_run = True
        
        dataset_count = 0
        last_surface_id = -1
        while loop_condition == True:
            count = 0
            if global_stats == True:
                if first_loop_run == True:
                    self.scan_dataset = self.scans_database.get_dataset(0)
                    first_loop_run = False
                else:
                    print 'next'
                    self.scan_dataset = self.scans_database.get_next_dataset()
                    
                if False == self.scan_dataset:
                    break
                
                if True != self.scan_dataset.is_labeled:
                    continue
                
                print 'load',filename
                filename = self.get_features_filename()
                dict = ut.load_pickle(filename)
                dataset_count += 1
                #assume that scans from one surface are in order
                if self.scan_dataset.surface_id != last_surface_id:
                    measured_table_heights += [float(self.scan_dataset.surface_height)]
                    last_surface_id = self.scan_dataset.surface_id
            else: 
                loop_condition = False
                
        
            for index in dict['point_indices']:
                #for plot
                fv_hs = (dict['features'][count])[self.features.get_indexvector('hsvi')]
                if dict['labels'][count] == LABEL_SURFACE or dict['labels'][count] == LABEL_CLUTTER:
                    h_values_plot.append(fv_hs[0])
                    s_values_plot.append(fv_hs[1])
                    v_values_plot.append(fv_hs[2])
                    i_values_plot.append(fv_hs[3])
                    
                   
                    rvalue,gvalue,bvalue = colorsys.hsv_to_rgb(fv_hs[0], fv_hs[1], fv_hs[2])

                    rgb_plot.append([rvalue,gvalue,bvalue])
              
                fv_range = (dict['features'][count])[self.features.get_indexvector('range')]
                #if dict['labels'][count] == LABEL_CLUTTER:
                #    print 'label',dict['labels'][count], 'fvrange', fv_range
                if dict['labels'][count] == LABEL_SURFACE:
                    #colors_plot.append([0,1,0])
                    colors_plot.append(1)
                    range_hist_surface_plot += np.asarray(fv_range)
                    count_surface += 1
                elif dict['labels'][count] == LABEL_CLUTTER:
                    #colors_plot.append([1,0,0])   
                    colors_plot.append(2)  
                    range_hist_clutter_plot += np.asarray(fv_range)
                    count_clutter += 1
                    
                
                    
                if len(fv_range) > 2:
                    normals_x += [fv_range[1]]
                    normals_y += [fv_range[2]]
                    normals_z += [fv_range[3]]
                    normals_label += [dict['labels'][count]]
                    
                count +=1
            
            
            
            #just needed for clutter height stats!!
            print 'load data for ',self.scan_dataset.id
            self.load_data(self.scan_dataset.id, False)
            (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
            self.do_all_point_cloud_mapping()


            surface_height_sum_this_table = 0
            surface_count_this_table = 0 
            for index, label in enumerate(self.map_polys):
                if label == LABEL_SURFACE:
                    surface_height_sum_this_table += np.asarray(self.pts3d_bound)[2,index]
                    surface_count_this_table += 1 
                    
            surface_height_mean_this_table = surface_height_sum_this_table / float(surface_count_this_table)

            for index, label in enumerate(self.map_polys):
                height = np.asarray(self.pts3d_bound)[2,index]
                height -= surface_height_mean_this_table
                if label == LABEL_CLUTTER:
                    clutter_heights += [height]
                elif label == LABEL_SURFACE:
                    surface_heights += [height]
                   
        surface_heights = np.asarray(surface_heights)
        clutter_heights = np.asarray(clutter_heights) 
        
        surface_heights = surface_heights * 100
        clutter_heights = clutter_heights * 100 #in cm
       
        #clutter_heights = clutter_heights[np.where(clutter_heights > -table_height)]
        
        print clutter_heights
        fig_clutterheights = plt.figure()
        plt.title('clutter and surface heights above measured height of table')
        # the histogram of the data
        plt.xlabel('height in cm')
        plt.ylabel('number of range measurements (points)')
        width = 0.35
        bins = np.asarray(range(-10,60))
        print 'len clutterheights',len(clutter_heights)
        surface_heights = surface_heights[np.where(surface_heights < 60)]
        surface_heights = surface_heights[np.where(surface_heights > -10)]
        clutter_heights = clutter_heights[np.where(clutter_heights < 60)]
        clutter_heights = clutter_heights[np.where(clutter_heights > -10)]
        print 'len clutterheights',len(clutter_heights)
        hist_surface = stats.histogram2(surface_heights,bins)
        hist_clutter = stats.histogram2(clutter_heights,bins)
        print bins
        print hist_surface
        print hist_clutter
        #hist_table = copy.copy(hist)
        #hist_table[2*table_height:] = 0
        #hist_clutter = copy.copy(hist)
        #hist_clutter[0:table_height*2] = 0
        plot1 = plt.bar(bins,hist_surface, color='g', width=width,linewidth=0.1)  
        plot1 = plt.bar(bins+width,hist_clutter, color='r', width=width,linewidth=0.1)  
        #plt.xticks(np.array(range(12))*5-5)        
        plt_filename = self.config.path+'/clutter_heights.png'
        print 'saving',plt_filename        
        plt.savefig(plt_filename)
        plt.savefig(self.config.path+'/clutter_heights.pdf')
        plt.savefig(self.config.path+'/clutter_heights.eps')        
        #plt.show()
        #return


#        h_values_plot += [0]
#        s_values_plot += [0]
#        v_values_plot += [0]
#       i_values_plot += [0]
#       colors_plot += [0]
 ##       rgb_plot += [[0,0,0]]
 #       mlab.figure(fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
 #       print 'lens',len(h_values_plot),len(s_values_plot),len(v_values_plot),len(i_values_plot),len(colors_plot)
 #       plot = mlab.points3d(np.asarray(h_values_plot),np.asarray(s_values_plot), np.asarray(v_values_plot), np.asarray(colors_plot),mode='point',resolution=4,scale_mode='none',scale_factor=0.01)
 #       mlab.outline(plot, color=(.7, .7, .7))
 #       mlab.xlabel('h')
 #       mlab.ylabel('s')
 #       mlab.zlabel('v')
 #       mlab.colorbar()
        
        rgb_plot = np.array(rgb_plot)     
        
        v_values_plot = np.asarray(v_values_plot)
        s_values_plot = np.asarray(s_values_plot)
        h_values_plot = np.asarray(h_values_plot)
        colors_plot = np.asarray(colors_plot)
        rgb_plot = np.asarray(rgb_plot)
        
        
        #for s,h plot probabilities:
        #self.plot_s_h_probabilities(s_values_plot[colors_plot == 1], h_values_plot[colors_plot == 1], s_values_plot[colors_plot == 2], h_values_plot[colors_plot == 2])
        dict = {'s_surface':s_values_plot[colors_plot == 1], 
                'h_surface':h_values_plot[colors_plot == 1],
                's_clutter':s_values_plot[colors_plot == 2], 
                'h_clutter':h_values_plot[colors_plot == 2]
                }
        #ut.save_pickle(dict, "s_h_probabilites_values.pkl")

        
        x = s_values_plot
        y = h_values_plot
        xlabel = 's' 
        ylabel = 'h'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,colors_plot)
        xlabel = 's_in_rgb' 
        ylabel = 'h_in_rgb'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb_plot,0.1)  
        x = s_values_plot[colors_plot == 1]
        y = h_values_plot[colors_plot == 1]
        rgb = rgb_plot[colors_plot == 1]
        xlabel = 's, surface' 
        ylabel = 'h, surface'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb,0.1)  
        x = s_values_plot[colors_plot == 2]
        y = h_values_plot[colors_plot == 2]
        rgb = rgb_plot[colors_plot == 2]
        xlabel = 's, clutter' 
        ylabel = 'h, clutter'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb,0.1)  
        
        
        x = v_values_plot
        y = h_values_plot
        xlabel = 'v' 
        ylabel = 'h'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,colors_plot)
        xlabel = 'v_in_rgb' 
        ylabel = 'h_in_rgb'
       # self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb_plot,0.1) 
        x = v_values_plot[colors_plot == 1]
        y = h_values_plot[colors_plot == 1]
        rgb = rgb_plot[colors_plot == 1]
        xlabel = 'v_in_rgb_surface' 
        ylabel = 'h_in_rgb_surface'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb,0.1)  
        x = v_values_plot[colors_plot == 2]
        y = h_values_plot[colors_plot == 2]
        rgb = rgb_plot[colors_plot == 2]
        xlabel = 'v_in_rgb_clutter' 
        ylabel = 'h_in_rgb_clutter'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb,0.1) 
                
        x = i_values_plot
        y = h_values_plot
        xlabel = 'i' 
        ylabel = 'h'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,colors_plot) 
           
        x = v_values_plot
        y = s_values_plot
        xlabel = 'v' 
        ylabel = 's'
       # self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,colors_plot) 
        xlabel = 'v_in_rgb' 
        ylabel = 's_in_rgb'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb_plot,0.1)  
        x = v_values_plot[colors_plot == 1]
        y = s_values_plot[colors_plot == 1]
        rgb = rgb_plot[colors_plot == 1]
        xlabel = 'v_in_rgb_surface' 
        ylabel = 's_in_rgb_surface'
       # self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb,0.1)  
        x = v_values_plot[colors_plot == 2]
        y = s_values_plot[colors_plot == 2]
        rgb = rgb_plot[colors_plot == 2]
        xlabel = 'v_in_rgb_clutter' 
        ylabel = 's_in_rgb_clutter'
       # self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,rgb,0.1)       
              
        x = i_values_plot
        y = s_values_plot
        xlabel = 'i' 
        ylabel = 's'
        #self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,colors_plot)   
            
        x = i_values_plot
        y = v_values_plot
        xlabel = 'i' 
        ylabel = 'v'
       # self.polt_color_scatterplots_2d(x,y,xlabel,ylabel,colors_plot)          
    
        if len(measured_table_heights):
            fig_heights = plt.figure()
            plt.title('table heights')
            # the histogram of the data
            plt.xlabel('height in cm')
            plt.ylabel('number of tables')
            print 'table heights', measured_table_heights
            hist = np.histogram(measured_table_heights,10)
            plot1 = plt.bar(hist[1][0:-1],hist[0],width=3)  
                      
            plt_filename = self.config.path+'/table_heights.png'
            print 'saving',plt_filename
            plt.savefig(plt_filename)
            plt.savefig(self.config.path+'/table_heights.pdf')
            plt.savefig(self.config.path+'/table_heights.eps')
            #plt.show()

        
        
        #normals_x += [0]
        #normals_y += [0]
        #normals_z += [0]
        #normals_label += [0]
        print 'number of points: ', len(normals_x)
        
        normals_x = np.asarray(normals_x)
        normals_y = np.asarray(normals_y)
        normals_z = np.asarray(normals_z)
        normals_label = np.asarray(normals_label)
        import random
        sampel_size = min(len(normals_label),500000)
        rand_idx = np.array(random.sample(xrange(len(normals_label)),sampel_size))
#        rand_idx = np.random.randint(0,len(normals_x),size=100000)
        normals_x = normals_x[rand_idx]
        normals_y = normals_y[rand_idx]
        normals_z = normals_z[rand_idx]
        normals_label = normals_label[rand_idx]
        
        normals_x = np.hstack((normals_x,0))
        normals_y = np.hstack((normals_y,0))
        normals_z = np.hstack((normals_z,0))
        normals_label = np.hstack((normals_label,0))
#        print 'nx',normals_x
#        print 'ny',normals_y
#        print 'nz',normals_z
#        print 'nlabel',normals_label

 
##plot normals sphere:##       
        print 'number of normals:',len(normals_x),len(normals_x),len(normals_x),len(normals_label)
        mlab.figure(fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))
        mlab.points3d(normals_x,normals_y,normals_z,normals_label,resolution=4,scale_factor=0.005,mode='point',scale_mode='none',colormap='jet')
        self.plot_axis(0,0,0, np.eye(3),0.5)
        
        
        
        #idx = np.where(normals_label == LABEL_SURFACE)
        #nx = normals_x[idx]
        #ny = normals_y[idx]
        #nz = normals_z[idx]
        
        #mlab.quiver3d(nx,ny,nz,0.2*nx,0.2*ny,0.2*nz,colormap='jet',scale_factor=0.005, color=(0,1,0))
        #idx = np.where(normals_label == LABEL_CLUTTER)
        #nx = normals_x[idx]
        #ny = normals_y[idx]
        #nz = normals_z[idx]
        #mlab.quiver3d(nx,ny,nz,-0.2*nx,-0.2*ny,-0.2*nz,colormap='jet',scale_factor=0.005, color=(1,0,0))
        #mlab.colorbar()
        
       

        range_hist_surface_plot /= count_surface
        range_hist_clutter_plot /= count_clutter
    
        #calculate standard deviation:
        range_hist_surface_std = 0
        range_hist_clutter_std = 0
        

        
        loop_condition = True
        first_loop_run = True
        while loop_condition == True:
            count = 0
            if global_stats == True:
                if first_loop_run == True:
                    self.scan_dataset = self.scans_database.get_dataset(0)
                    first_loop_run = False
                else:
                    self.scan_dataset = self.scans_database.get_next_dataset()
                    
                if False == self.scan_dataset:
                    break                    
                    
                if True != self.scan_dataset.is_labeled:
                    continue
                
                print 'load',filename
                filename = self.get_features_filename()
                dict = ut.load_pickle(filename)
            else: 
                loop_condition = False
                  
            for index in dict['point_indices']:
                fv_range = (dict['features'][count])[self.features.get_indexvector('range')]
                if dict['labels'][count] == LABEL_SURFACE:
                    range_hist_surface_std += np.square(np.asarray(fv_range) - range_hist_surface_plot)
                    
                elif dict['labels'][count] == LABEL_CLUTTER:
                    range_hist_clutter_std += np.square(np.asarray(fv_range) - range_hist_clutter_plot)
                count +=1
            
        range_hist_surface_std /= count_surface
        range_hist_clutter_std /= count_clutter
        range_hist_surface_std = np.sqrt(range_hist_surface_std)
        range_hist_clutter_std = np.sqrt(range_hist_clutter_std)
    
    


        width = 0.35       # the width of the bars   
        idx = np.asarray(range(len(self.features.get_indexvector('range'))))
        print 'surface',range_hist_surface_plot
        print 'surface std',range_hist_surface_std
        print 'clutter',range_hist_clutter_plot
        print 'clutter std',range_hist_clutter_std        
        
        fig_range = plt.figure()
        plt.title('range features (normalized): mean and standard deviation')
        #print range_hist_surface_plot
        #print range_hist_clutter_plot
        max = np.maximum(np.maximum(np.maximum(range_hist_surface_plot,range_hist_clutter_plot),range_hist_surface_std),range_hist_clutter_std)
        factor = np.divide(np.ones(len(range_hist_surface_plot)),max)
       # print factor
        range_hist_surface_plot = np.multiply(range_hist_surface_plot,factor)
        #print 'factor',range_hist_surface_plot
        #print range_hist_clutter_plot
        range_hist_clutter_plot = np.multiply(range_hist_clutter_plot,factor)
        range_hist_surface_std = np.multiply(range_hist_surface_std,factor)
        range_hist_clutter_std = np.multiply(range_hist_clutter_std,factor)

        plot1 = plt.bar(idx,range_hist_surface_plot,width,yerr=range_hist_surface_std,color='g')  
        plot2 = plt.bar(idx+width,range_hist_clutter_plot,width,yerr=range_hist_clutter_std,color='r') 
        plt.xticks(idx+width,('zhist','nx','ny','nz','ev1','ev2'))
        plt_filename = self.config.path+'/range_features.png'
        print 'saving',plt_filename        
        plt.savefig(plt_filename)
        plt.savefig(self.config.path+'/range_features.pdf')
        plt.savefig(self.config.path+'/range_features.eps')  
        #plt.show()  
         
      
        print 'creating plots done.'
        
        
    def polt_color_scatterplots_2d(self,x,y,xlabel,ylabel, colors_plot, alpha=0.02):
        fig = plt.figure()
        plt.title(xlabel+', '+ylabel)
        plt.xlabel(xlabel+' values')
        plt.ylabel(ylabel+' values')
        
        #TODO: dpi=...
        plot = plt.scatter(x,y,5,c=colors_plot,linewidths=0, alpha=alpha)
        plt_filename = self.config.path+'/colors_'+xlabel+'_'+ylabel
        print 'saving',plt_filename,'...',       
        plt.savefig(plt_filename+'.png')
        #plt.savefig(plt_filename+'.pdf')
        #plt.savefig(plt_filename+'.eps')      
        print 'done'
        
    def plot_s_h_probabilities(self,s_surface, h_surface, s_clutter, h_clutter):
        s_surface = np.hstack((s_surface,0))
        h_surface = np.hstack((h_surface,0))
        s_clutter = np.hstack((s_clutter,0))
        h_clutter = np.hstack((h_clutter,0))
        s_surface = np.hstack((s_surface,1))
        h_surface = np.hstack((h_surface,1))
        s_clutter = np.hstack((s_clutter,1))
        h_clutter = np.hstack((h_clutter,1))         
                              
        bins = 256
        hist2d_surface = np.histogram2d(s_surface,h_surface,bins=bins)[0]
        hist2d_clutter = np.histogram2d(s_clutter,h_clutter,bins=bins)[0]
        
        hist2d_surface = hist2d_surface[:,0:200]
        hist2d_clutter = hist2d_clutter[:,0:200]   
        
        fig = plt.figure()
        plt.xlabel('s values')
        plt.ylabel('h values')
        
        #hist2d_surface[hist2d_surface == 0.0] = 1
        #hist2d_clutter[hist2d_clutter == 0.0] = 1
        #hist2d = np.divide(hist2d_surface / len(s_surface) , hist2d_clutter / len(s_clutter))
        #print hist2d
        #hist2d = hist2d / np.max(hist2d)
        #print hist2d
        
        hist2d_sum = hist2d_clutter + hist2d_surface
        hist2d_sum[hist2d_sum == 0] = 1
        

        
        hist_prob_surface = np.divide(hist2d_surface,hist2d_sum)
        hist_prob_clutter = np.divide(hist2d_clutter,hist2d_sum)
        #hist_prob_clutter = 1.0 - hist_prob_surface     
        
        prob_threshold = 0.80
        hist_prob_surface[hist_prob_surface < prob_threshold] = 0 
        hist_prob_clutter[hist_prob_clutter < prob_threshold] = 0
        
        meas_threshold = 5
        hist_prob_surface[hist2d_sum < meas_threshold] = 0
        hist_prob_clutter[hist2d_sum < meas_threshold] = 0
        #prob. too low or too few measurements? -> white
        blue = np.zeros(np.shape(hist_prob_surface)) 
        idx_white = np.multiply(hist_prob_surface == 0, hist_prob_clutter == 0)
        blue[idx_white] = 1
        hist_prob_surface[idx_white] = 1
        hist_prob_clutter[idx_white] = 1
        
        print hist_prob_surface
        print hist_prob_clutter
        
        histcolored = np.array([hist_prob_clutter, hist_prob_surface, blue]).T
        plt.imshow(histcolored, interpolation='nearest', origin='lower')
        plt_filename = self.config.path+'/sh_probabilities'
        print 'saving',plt_filename,'...',       
        plt.savefig(plt_filename+'.png')
        plt.savefig(plt_filename+'.pdf')
        plt.savefig(plt_filename+'.eps')  

        import sys
        print 'exit'
        plt.show()
        sys.exit()
                    
        
    def plot_axis(self,x,y, z, directions, scale =  1):
        if not MLAB_LOADED: return #ignore 3D plot commands if unavailable
        scale *= 2.
        mlab.quiver3d([x-directions[0,0]/2.0], [y-directions[1,0]/2.0], [z-directions[1,0]/2.0], [directions[0,0]], [directions[1,0]], [directions[2,0]], scale_factor=scale, color=(1,0,0))
        if directions.shape[1] > 1:
            mlab.quiver3d([x-directions[0,1]/2.0], [y-directions[1,1]/2.0], [z-directions[2,0]/2.0], [directions[0,1]], [directions[1,1]], [directions[2,1]], scale_factor=scale, color=(0,1,0))
            mlab.quiver3d([x-directions[0,2]/2.0], [y-directions[1,2]/2.0], [z-directions[2,2]/2.0], [directions[0,2]], [directions[1,2]], [directions[2,2]], scale_factor=scale, color=(0,0,1))        
                
            
    def draw_3d(self,type,spheres=False, new_figure = True):
        if not MLAB_LOADED: return #ignore 3D plot commands if unavailable
        #display bounded 3d-points
        #mlab.points3d(self.pts3d_bound[0,:].A1,self.pts3d_bound[1,:].A1,self.pts3d_bound[2,:].A1,self.pts3d_bound[0,:].A1,mode='point',scale_factor=0.01,colormap='jet')#,colormap='winter'
        
        ##mlab.points3d(self.pts3d_bound[0,:].A1,self.pts3d_bound[1,:].A1,self.pts3d_bound[2,:].A1,self.map_polys,mode='point',scale_factor=0.01,colormap='jet')#,colormap='winter'
       
        #test:
        #self.pts3d_bound = self.pts3d
       
        #test artag
        #self.pts3d_bound = apply_transform_matrix( self.config.camTlaser, self.pts3d_bound)
        
        #don't display ROI
        #map_polys = np.array(self.map_polys)[a==LABEL_ROI]=0
        
        mode = 'point'
        if spheres:
            mode = 'sphere'
        if new_figure:
            mlab.figure() #mlab.figure(fgcolor=(0, 0, 0), bgcolor=(1, 1, 1))        
                
        print ut.getTime(), type
        if type == 'height':
            labels = self.pts3d_bound[2,:]
        elif type == 'intensities':
            labels = self.intensities_bound
        elif type == 'labels':
            labels = self.map_polys
        elif type == 'range':
            labels = self.load_Classifier_labels('range')
        elif type == 'color':
            labels = self.load_Classifier_labels('color')
        elif type == 'all':
            labels = self.load_Classifier_labels('all')
        elif type == 'all_post':
            labels = self.load_Classifier_labels('all_post')
            #labels = self.classifiers['all'].postprocess(labels)
        elif type == 'baseline':
            labels = self.load_Classifier_labels('baseline') #self.classify_baseline_code()
        elif type=='h' or type == 's' or type == 'v':
            ###self.map2d = np.asarray(self.camPts_bound)
            #create HSV numpy images:
            # compute the hsv version of the image 
            image_size = cv.cvGetSize(self.img)
            img_h = cv.cvCreateImage (image_size, 8, 1)
            img_s = cv.cvCreateImage (image_size, 8, 1)
            img_v = cv.cvCreateImage (image_size, 8, 1)
            img_hsv = cv.cvCreateImage (image_size, 8, 3)
            cv.cvCvtColor (self.img, img_hsv, cv.CV_BGR2HSV)
            cv.cvSplit (img_hsv, img_h, img_s, img_v, None)

            labels = []
            
            if type=='h':
                imNP = u2.cv2np(img_h)
            elif type == 's':
                imNP = u2.cv2np(img_s)
            elif type == 'v':
                imNP = u2.cv2np(img_v)
                
            
            for index in range(len(self.pts3d_bound[2,:].A1)):
                labels.append(float(imNP[self.map2d[1,index],self.map2d[0,index]]))

                
        print 'size X', np.size(self.pts3d_bound[0,:]), 'len labels:', len(labels)
                
        #draw groundplane:
        mlab.imshow([[1,1],[1,1]], extent=[-1, 1,  
                                       -1, 1, 
                                       0, 0], opacity=.5, colormap='jet')      
        #draw table plane:
        #print 'tbp',self.scan_dataset.table_plane_translation
        #draw_plane(np.matrix([[0,0,1]]).T, self.scan_dataset.table_plane_translation.T)   
        if self.scan_dataset.table_plane_translation != '':
            print 'self.scan_dataset.table_plane_translation', self.scan_dataset.table_plane_translation
            table_pts = self.pts3d_bound[:,self.map_polys == LABEL_SURFACE]
            table_height = float(self.scan_dataset.table_plane_translation[2])
            if table_pts != []:
                mlab.imshow([[1,1],[1,1]], extent=[np.min(table_pts[0,:]), np.max(table_pts[0,:]),  
                                           np.min(table_pts[1,:]), np.max(table_pts[1,:]), 
                                           table_height, table_height], opacity=.5, colormap='jet')  
                    
            #draw camera:
            #print 'gpt',self.scan_dataset.ground_plane_translation
            mlab.points3d([0],[0],[float(self.scan_dataset.ground_plane_translation[2])],[0],mode='sphere',resolution=8,scale_mode='none',scale_factor=0.1,colormap='winter')#,colormap='winter'
        #FORMERLY, this line below set scale_factor=0.0015 AND =0.01.
        mlab.points3d(self.pts3d_bound[0,:],self.pts3d_bound[1,:],self.pts3d_bound[2,:],labels,mode=mode,resolution=8,scale_mode='none',scale_factor=0.01,colormap='jet')#,colormap='winter'
        #mlab.orientationaxes()
        mean_x = np.mean(self.pts3d_bound[0,:])
        mean_y = np.mean(self.pts3d_bound[1,:])
        #set camera:
        mlab.view(azimuth=0, elevation=-48, distance=3.75, focalpoint=(mean_x,mean_y,0.85))
        
    def save_3d(self,type,spheres=False):
        if not MLAB_LOADED: return #ignore 3D plot commands if unavailable
        mlab.clf()
        self.draw_3d(type, spheres)
        size=(960,800)
        mlab.savefig(self.config.path+'/results/'+self.scan_dataset.id+'_'+type+'.png',size=size)
        #mlab.close() #AttributeError: 'module' object has no attribute 'close' ?????
        
        
    def display_3d(self,type,spheres=False, new_figure = True):
        if not MLAB_LOADED: return #ignore 3D plot commands if unavailable
        
        self.draw_3d(type, spheres, new_figure)
        
        #draw this and that..:
        #mlab.points3d(self.pts3d_bound[0,:].A1,self.pts3d_bound[1,:].A1,self.pts3d_bound[2,:].A1,self.intensities_bound,mode=mode,resolution=4,scale_factor=0.005,scale_mode='none',colormap='winter')#,colormap='winter'
        #mlab.points3d(self.pts3d_bound[0,:].A1,self.pts3d_bound[1,:].A1,self.pts3d_bound[2,:].A1,self.pts3d_bound[2,:].A1,mode=mode,resolution=4,scale_factor=0.005,scale_mode='none',colormap='jet')#,colormap='winter'
        #mlab.points3d(self.pts3d_bound[0,:].A1,self.pts3d_bound[1,:].A1,self.pts3d_bound[2,:].A1,self.map_polys,mode='sphere',resolution=4,scale_factor=0.0015,scale_mode='none',colormap='jet')#,colormap='winter'
        #draw scan indices (coloring)
        #mlab.points3d(self.pts3d_bound[0,:].A1,self.pts3d_bound[1,:].A1,self.pts3d_bound[2,:].A1,self.scan_indices_bound,mode='point',scale_factor=0.01,colormap='autumn')#,colormap='winter'
        #mlab.points3d(self.pts3d_bound[0,:].A1,self.pts3d_bound[1,:].A1,self.pts3d_bound[2,:].A1,self.scan_indices_bound,mode='sphere',resolution=4,scale_factor=0.0015,scale_mode='none',colormap='autumn')#,colormap='winter'
       
        print ut.getTime(), 'points:'
        print ut.getTime(), self.pts3d_bound[0,:].shape
         
        
        R = self.artag_transformation
        if np.any(R):
            print ut.getTime(), 'display ARTag detection coordinates...'
            scale = 0.003
            self.draw_3d_axis(scale)

            R[:,3] = R[:,3] * 0.0005
            R = np.vstack((R,np.array([0,0,0,1.0])))
            #R[:,0:3] = np.linalg.inv(R[:,0:3])
            #draw axis
            v1 = np.array([0,0,0,1])
            v2 = np.array([80*scale,0,0,1])
            v1 = np.dot(R,v1)
            v2 = np.dot(R,v2)
            print ut.getTime(), v2
            self.draw_vector(v1[0:3],v2[0:3],scale, (1,0,0))
            
            v1 = np.array([0,0,0,1])
            v2 = np.array([0,80*scale,0,1])
            v1 = np.dot(R,v1)
            v2 = np.dot(R,v2)
            print ut.getTime(), v2
            self.draw_vector(v1[0:3],v2[0:3],scale,(0,1,0))
            
            v1 = np.array([0,0,0,1])
            v2 = np.array([0,0,80*scale,1])
            v1 = np.dot(R,v1)
            v2 = np.dot(R,v2)
            print ut.getTime(), v2
            self.draw_vector(v1[0:3],v2[0:3],scale,(0,0,1))
            

        mlab.colorbar()
        mlab.show() 
        

    #old, better use quivers instead...
    def draw_vector(self, v1, v2, scale, color=(1,1,1)):
        if not MLAB_LOADED: return #ignore 3D plot commands if unavailable

        mlab.triangular_mesh([[v1[0]-1*scale,v1[0]+1*scale,v2[0]-1*scale,v2[0]+1*scale]], [[v1[1]-1*scale,v1[1]+1*scale,v2[1]-1*scale,v2[1]+1*scale]], [[v1[2]-1*scale,v1[2]+1*scale,v2[2]-1*scale,v2[2]+1*scale]], [(0,1,2),(1,2,3)],color=color)          
    def draw_3d_axis(self, scale):
            #axis
        mlab.triangular_mesh([np.array([0.0,0.0,100.0])*scale], [np.array([-0.3,0.3,0.0])*scale], [np.array([0.0,0.0,0.0])*scale], [(0,1,2)], color=(1,1,1))
        mlab.triangular_mesh([np.array([-0.3,0.3,0.0])*scale], [np.array([0.0,0.0,100.0])*scale], [np.array([0.0,0.0,0.0])*scale], [(0,1,2)], color=(1,1,1))
        mlab.triangular_mesh([np.array([-0.3,0.3,0.0])*scale], [np.array([0.0,0.0,0.0])*scale], [np.array([0.0,0.0,100.0])*scale], [(0,1,2)], color=(1,1,1))

    #needs the ARToolKitPlus with a modified simple example code that reads the image and output the transformation
    def read_artag(self, img):
        
        grey = cv.cvCreateImage((img.width, img.height), 8,1)
        highgui.cvConvertImage(img, grey)
    
        file = open('test.raw', 'wb')       
            
        for i in range(grey.height):
            for j in range(grey.width):
                file.write(chr(grey[i][j]))
                
        file.close()

        output = subprocess.Popen([LOC_ARTAGS_SIMPLE, ""], stdout=subprocess.PIPE, env={"LD_LIBRARY_PATH": LOC_ARTAGS_LIB}).communicate()[0]
        print ut.getTime(), output
        
        try:
            m = re.search('getARMatrix.*\n([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*).*\n([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*).*\n([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*)', output)
        except: 
            #sys.exc_info()[0]
            print ut.getTime(), "ERROR parsing ARToolKitPlus output"
            return np.array([])
            
        if None == m:
            print ut.getTime(), "ERROR parsing ARToolKitPlus output"
            return np.array([])
        
        R = np.array(m.groups(),dtype=np.float)
        R = R.reshape(3,4)
        
        if False == np.any(R): #all elements are 0
            print ut.getTime(), "ERROR: failed to detect AR tag"
            return np.array([])
        
        
        print ut.getTime(), m.groups()
        print ut.getTime(), R
        # success, 3x4 transformation matrix
        print ut.getTime(), 'success'
        return R
           
    #create pointcloud, map laser into image, read artag if present
    def process_raw_data(self, translate = True):
        (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
        self.do_all_laser_image_mapping(translate)
            
    def process_intensities(self):
        #set reject_zero_ten to False to not remove any points and preserve intensity->3d mapping
        # be aware that points filtered by remove-graze-effect have range-values of +/-1000 mapped into 3d-space
        (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans, False)
        self.img_intensities = self.create_intensity_image(self.laserscans[0])
        
    def process_labels(self, features):
        (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
        self.do_all_point_cloud_mapping()
        self.img_labels = self.draw_mapped_labels_into_image(features)  
    ###    
    def process_masks(self, features, show_clutter=False):
        (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
        self.do_all_point_cloud_mapping()
        self.img_labels = self.draw_mapped_masks_into_image(features, show_clutter)  
    ###    
    def save_mapped_image(self,name):
        prefix = self.config.path+'/data/'+name
        print ut.getTime(), "Saving: "+prefix+'_image_mapped.png'
        highgui.cvSaveImage(prefix+'_image_mapped.png',self.img_mapped)

    def save_intensity_image(self, name):
        prefix = self.config.path+'/data/'+name
        print ut.getTime(), "Saving: "+prefix+'_image_intensities.png'
        highgui.cvSaveImage(prefix+'_image_intensities.png',self.img_intensities)
        return prefix+'_image_intensities.png'
    
    ''' SAVE LABELS IMAGE (in same directory as config.path)
        Note it is possible to fail at this function since the
        path contains the keyword 'results'
    '''
    def save_labels_image(self, features):
        filename = self.config.path+'/results/'+self.scan_dataset.id+'_image_labels_'+features+'_'+self.feature_type+'_k'+str(self.feature_neighborhood)+'_r'+str(self.feature_radius)+'.png'
        print ut.getTime(), "Saving: "+filename
        highgui.cvSaveImage(filename,self.img_labels)
        return filename    


    def save_masks_image(self, features, show_clutter=False):
        if not show_clutter:
            filename = self.config.path+'/results/'+self.scan_dataset.id+'_image_mask_surface'+'.png'
        else:
            filename = self.config.path+'/results/'+self.scan_dataset.id+'_image_mask_clutter'+'.png'
        print ut.getTime(), "Saving: "+filename
        try:
            highgui.cvSaveImage(filename,self.img_labels)
        except:
            print'processor.py::save_masks_image(): Unable to save file "'+filename+\
              '. Please check that folder exists and that file permissions allow this location to overwrite.'
        return filename    
    ###     
    def load_metadata(self, id, reload_database = True):

        if reload_database:
            self.scans_database = scans_database.scans_database()
            self.scans_database.load(self.config.path,'database.pkl')
        self.scan_dataset = self.scans_database.get_dataset_by_id(id)
        
    '''
    Purpose: Exclude all edges when training.
    
    This function creates superfluous images perhaps can be excluded.  The images represent the ROI, which can specified by
    boundary points to a square (currently uses a square NOT a polygon) rather than complex images.  This could be nice
    in the future if a map of the table were generated with respect to the surface.  A boundary of 30 (or 30/2) px is used
    on the borders of the image to prevent placement there.  A boundary of 10 (or 10/2) px is used for the borders of the table.
    Given the current setup, these boundaries are identical and awkward.
    '''

    def create_polygon_images(self):
        self.scan_dataset.cvEdgeImage = cv.cvCreateImage(cv.cvSize (self.img.width, self.img.height),8,1)
        cv.cvSet(self.scan_dataset.cvEdgeImage, 0)
        self.scan_dataset.cvSurfaceEdgeImage = cv.cvCreateImage(cv.cvSize (self.img.width, self.img.height),8,1)
        cv.cvSet(self.scan_dataset.cvSurfaceEdgeImage, 0)
        ''' modified code '''
        for polygon in self.scan_dataset.polygons:
            print "Found hand-labeled polygon with length:", len(polygon.get_points()), 'label=', polygon.get_label()
        i=0
        for polygon in self.scan_dataset.polygons:
            i=i+1;
            if len(polygon.get_points()):
                polygon.cvImage = cv.cvCreateImage(cv.cvSize (self.img.width, self.img.height),8,1)
                cv.cvSet(polygon.cvImage, 0)
                try:
                    #formerly used cv.FillPoly(polygon.cvImage,[polygon.get_points()], 255) 
                    temp = np.array(polygon.cvImage)
                    roscv.FillPoly(temp,(polygon.get_points(),), 255) 
                    polygon.cvImage = temp
                except: 
                    cv.cvSet(polygon.cvImage, 255) #If in doubt, fill everything.    
                    print 'Ignored command cvFillPoly() 1'
                #exclude all edges for training
                if len(polygon.get_points()) and polygon.get_type() == 'polygon':
                    try:
                        #formerly used cv.cvPolyLine
                        temp = np.array(self.scan_dataset.cvEdgeImage)
                        roscv.PolyLine(temp,[polygon.get_points()], True, 255,30)
                        self.scan_dataset.cvEdgeImage = temp
                    except: print 'Ignored command cvPolyLine() 2'
                    
                #exclude surfaces edges for testing
                if polygon.get_label() == 'surface' and len(polygon.get_points()) and polygon.get_type() == 'polygon':
                    try:
                        #formerly used cv.cvPolyLine
                        temp = np.array(self.scan_dataset.cvSurfaceEdgeImage)
                        roscv.PolyLine(temp,[polygon.get_points()], True, 255,10)
                        self.scan_dataset.cvSurfaceEdgeImage = temp
                    except: print 'Ignored command cvPolyLine() 3'
        
    '''/author = Jokerman.  Returns a grayscale masked image from polygons'''
    def create_combined_polygon_image(self):
        (w, h) = (self.img.width, self.img.height)
        poly_image = cv.cvCreateImage(cv.cvSize (w, h),8,1)
        cv.cvSet(poly_image, 0) #fill with blank

        print 'draw poly combo image'
        for label_to_draw, color in [('surface',120), ('object', 255)]:
            for polygon in self.scan_dataset.polygons:
                pts = polygon.get_points()
                if len(pts) > 0 and polygon.get_type() == 'polygon':
                    if polygon.get_label() == label_to_draw:
                        #formerly used cv.FillPoly(label_image,[pts()], 255) 
                        temp = np.array(poly_image);
                        roscv.FillPoly(temp,(pts,), color);
                        poly_image = temp;
                    #else: pass
        self.combined_polygon_image = poly_image
        return poly_image
    #-----------------

    #from 2d point in intensity image to 3d:
    def get_3d_point_index(self, image_point):
        width = self.img_intensities.width
        height = self.img_intensities.height
        index = (height - image_point[1]) * width + image_point[0]
        return index
    
    #from 2d point in intensity image to 3d:
    def get_3d_point(self, point2d):
        index = self.get_3d_point_index(point2d)
        return np.asarray(self.pts3d)[:,index]
    
#    #external function, not used internally. 
#    def get_3d_point_index_in_unrotated(self, point3d):
#        print 'contents of point3d',point3d
#        (pts3d, scan_indices, intensities) = self.create_pointcloud(self.laserscans)
#        img = cv.cvClone(self.img)
#        self.map_laser_into_cam_2D(pts3d, img, scan_indices, intensities)
#        self.kdtree2d = kdtree.KDTree(self.pts3d_bound.T)
#        indices = self.kdtree2d.query(point3d.T, 1, eps=0, p=2)
#        print 'indices',indices, 'index',indices[1]
#        return indices[1]
            
    def get_3d_plane_normal(self, image_points):
        points3d = []
        #print ut.getTime(), 'pixelcount'
        print ut.getTime(), self.img_intensities.width * self.img_intensities.height
        for i,point2d in enumerate(image_points):
            #print ut.getTime(), 'point2d: ', point2d
            #point2d = np.round(point2d) #also transforms it into numpy array
            #print ut.getTime(), point2d
            width = self.img_intensities.width
            height = self.img_intensities.height
            index = (height - point2d[1]) * width + point2d[0]
            print ut.getTime(), 'index: ' + str(index)
            print ut.getTime(), np.shape(np.asarray(self.pts3d))
            points3d.append(np.asarray(self.pts3d)[:,index])
            
        #plot all scene points    
        print ut.getTime(), 'image points: ' + str(image_points) + ' points3d: ' + str(points3d)

        a = points3d[1] - points3d[0]
        b = points3d[2] - points3d[0]
        n = np.matrix(np.cross(a,b)).T
        n = n / np.linalg.norm(n)

        #z component should always point upwards
        if n[2] < 0:
            n = -n
        print ut.getTime(), 'plane normal: ' + str(n)
        return n, points3d


    #check if it is a valid point or got set to range=+/-1000 by remove_graze_effect()
    def check_3d_plane_point(self, point2d):
        #point2d = np.round(point2d)
        width = self.img_intensities.width
        height = self.img_intensities.height
        index = (height - point2d[1]) * width + point2d[0]
        point3d = np.asarray(self.pts3d)[:,index]
        print ut.getTime(), 'check 3d point: ', point3d 
        
        if np.max(np.abs(point3d)) > 100:
            return False #invalid
        print ut.getTime(), point2d, point3d
        return True #valid point
                
    def rotate_pointcloud_match_groundplane(self):
        
        if self.scan_dataset.ground_plane_rotation == '':
            #fall back to groundplane
            print 'WARNING: no RANSAC groundplane calculated for',self.scan_dataset.id
            print 'WARNING: Falling back to 3point-estimation!'
        
            n = self.scan_dataset.ground_plane_normal
    
            if n == '':
                print ut.getTime(), 'WARNING: no groundplane defined!'
                return False
            print ut.getTime(), 'normal: ', n.T
                   
            #rotate:     
            print 'shapes pts3d, pts3d_bound' ,np.shape(self.pts3d), np.shape(self.pts3d_bound)
            self.pts3d = rotate_to_plane(n, self.pts3d)
            self.pts3d_bound = rotate_to_plane(n, self.pts3d_bound)   
        else:
            #rotate:     
            self.pts3d = self.scan_dataset.ground_plane_rotation * self.pts3d #rotate_to_plane(n, self.pts3d)
            print self.scan_dataset.ground_plane_rotation, self.pts3d_bound
            self.pts3d_bound = np.asarray(self.scan_dataset.ground_plane_rotation * np.asmatrix(self.pts3d_bound)) #rotate_to_plane(n, self.pts3d_bound)   
        
        return True

    
    def z_translate_pointcloud_groundplane(self):
        if self.scan_dataset.ground_plane_translation == '':
            print ut.getTime(), 'WARNING: no groundplane translation defined! Ignoring. Translation'
            return False
        self.pts3d += self.scan_dataset.ground_plane_translation
        self.pts3d_bound += self.scan_dataset.ground_plane_translation
        
    def get_groundplane_translation(self):
        gpp = self.scan_dataset.ground_plane_three_points
        if gpp == '':
            print ut.getTime(), 'WARNING: no groundplane defined! Ignoring Translation'
            return False
        if self.scan_dataset.ground_plane_rotation == '':
            print ut.getTime(), 'WARNING: no groundplane rotation defined! Ignoring Rotation'
            return False
        
        gpp = np.asarray(gpp).T
        gpp = self.scan_dataset.ground_plane_rotation * np.matrix(gpp)
        z_mean = np.mean(gpp[2,:])
        
        return np.matrix([0.0,0.0, -z_mean]).T
        
    
    #(re)generate only for current test and training set:
    def generate_save_features(self, generate_and_save_all_neightborhood_indices = False, regenerate_neightborhood_indices = False):
        
        #get number of training sets:
        training_set_count = 0
        self.scan_dataset = self.scans_database.get_dataset(0)
        while False != self.scan_dataset:
            if self.scan_dataset.is_training_set:
                training_set_count += 1
            self.scan_dataset = self.scans_database.get_next_dataset()
                 
        training_size_per_scan = self.classifier_training_size / training_set_count       
                 
        #loop through all marked datasets
        self.scan_dataset = self.scans_database.get_dataset(0)
        
        state_exclude_edges = False #just 2 alternating states to do both for each scan
        
        #get size of training set in total
        while False != self.scan_dataset:
            if self.scan_dataset.is_labeled:

                self.load_raw_data(self.scan_dataset.id)
                (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
                if state_exclude_edges == False:
                    self.do_all_point_cloud_mapping(False)
                    print ut.getTime(),  'generating features with edges', self.scan_dataset.id
                else:
                    self.do_all_point_cloud_mapping(True) 
                    print ut.getTime(),  'generating features without edges', self.scan_dataset.id
                    
                dict = self.generate_features(training_size_per_scan,generate_and_save_all_neightborhood_indices,regenerate_neightborhood_indices)
                
                filename = self.get_features_filename(state_exclude_edges)
                print ut.getTime(), "Saving: "+filename
                ut.save_pickle(dict,filename)
                del dict
                #del train_data
                #del train_labels
                #del nonzero_indices
                #del labels
                
            if state_exclude_edges == False:
                state_exclude_edges = True
            else:
                state_exclude_edges = False
                #get next one
                self.scan_dataset = self.scans_database.get_next_dataset()
        print ut.getTime(), 'generating features done.'
        
        
    #generate and return features for the current dataset, assume it is loaded and mapped
    def generate_features(self,training_size_per_scan = 999999999999,generate_and_save_all_neightborhood_indices = False, regenerate_neightborhood_indices = False):
        
        feature_vector_length = len(self.features.get_indexvector('all'))
        nonzero_indices = np.nonzero(self.map_polys)[0] #just surface or clutter for now (index > 0)
        #print ut.getTime(), nonzero_indices
        #print ut.getTime(), self.map_polys
        labels = np.array(self.map_polys)[nonzero_indices]
        current_set_size_total = len(nonzero_indices)
        all_nonzero_indices = copy.copy(nonzero_indices)
        
        #just select a couple of points only for training:
        if False == self.scan_dataset.is_test_set and training_size_per_scan < current_set_size_total:
            training_size_this_scan = min(training_size_per_scan, current_set_size_total)
            t_idx = np.random.randint(0,current_set_size_total,size=training_size_this_scan)
            nonzero_indices = nonzero_indices[t_idx]
            labels = labels[t_idx]
            current_set_size = training_size_this_scan
        else:
            print 'generate on all points in this scan...'
            current_set_size = current_set_size_total #use all for testing
            
        print 'generating',current_set_size,'features for', self.scan_dataset.id, ';fvlength',feature_vector_length
        
        if False == generate_and_save_all_neightborhood_indices:
            self.features.prepare(self.features_k_nearest_neighbors, nonzero_indices) #only for nonzero indices
        else:
            self.features.prepare(self.features_k_nearest_neighbors, all_nonzero_indices, True, regenerate_neightborhood_indices)


        train_data = np.array(np.zeros((current_set_size,feature_vector_length)))
        train_labels = np.array(np.zeros(current_set_size))
        
        count = 0
        for index, label in zip(nonzero_indices,labels):

            #print ut.getTime(), point3d
            fv = self.features.get_featurevector(index, count)
            for fv_index, fv_value in enumerate(fv):
                train_data[count][fv_index] = fv_value
            train_labels[count] = label
            #print ut.getTime(), 'fv ', fv
            #print ut.getTime(), 'tr ',train_data[index], 'label',train_labels[index]

            #if train_labels[count] == LABEL_CLUTTER:
            #    print 'label',train_labels[count], 'fvrange', fv

            if count % 1024 == 0:
                print ut.getTime(), 'label', label, 'fv', fv
            if count % 1024 == 0:
                print ut.getTime(), 'generate features:', count, 'of', current_set_size, '(',(float(count)/float(current_set_size)*100.0),'%)'
                
            count += 1
        #save data:
        dict = {'features' : train_data, 'labels' : train_labels,
                'feature_vector_length' : feature_vector_length,
                'set_size': current_set_size,
                'point_indices': nonzero_indices} 
        return dict
        
    #todo: refactor
    def get_features_filename(self,state_exclude_edges = False):
        if state_exclude_edges == True:
            return self.config.path+'/data/'+self.scan_dataset.id+'_features_without_edges_'+self.feature_type+'_k'+str(self.feature_neighborhood)+'_r'+str(self.feature_radius)+'.pkl'
        else:
            return self.config.path+'/data/'+self.scan_dataset.id+'_features_'+self.feature_type+'_k'+str(self.feature_neighborhood)+'_r'+str(self.feature_radius)+'.pkl'
    
    def train_and_save_Classifiers(self):
        for features, classifier in self.classifiers.iteritems():
            print ut.getTime(), 'training:', features
            
            classifier.train()  
            
            try:
                classifier.save()
            except SystemError:
                print 'ERROR: saving classifier for',features,'failed!'
            #else:
            #    print 'ERROR: saving classifier for',features,'failed!'

            
    def load_Classifiers(self):
        for features, classifier in self.classifiers.iteritems():
            self.classifiers[features].load()            
        
        
    #if fold != None: just process this specific fold of the crossvalidation and save the results
    #if update_postprocess==True: just do postprocessing on 'all', and load the labels for all others
    def test_crossvalidation_on_valid_scans(self, fold=None, update_postprocess=False):
        print 'start crossvalidation'
        #TODO
        #group by tables
        #should I only use tables with 4 valid scans??
        
        tables_per_fold = 1
        
        testresults_crossvalidation = []
        last_fold_first_test_table_id = None
        
        last_table_id = None

        last_fold_last_test_table_id = None
        
        current_fold = 0
        
        debug_fold_count = 0
        while last_fold_last_test_table_id != last_table_id or last_fold_last_test_table_id == None:
            self.scan_dataset = self.scans_database.get_dataset(0)
            skip_test_table_count=0
            current_fold_test_table_count = 0
            state = 'before_test_tables'
            print '======================================'
            while False != self.scan_dataset:
                #print 'id',self.scan_dataset.id ,'state',state
                if self.scan_dataset.is_labeled: #only valid labelings
                    if state == 'before_test_tables':
                        if (last_fold_first_test_table_id == self.scan_dataset.surface_id or skip_test_table_count > 0) and last_table_id != self.scan_dataset.surface_id:
                                skip_test_table_count += 1
                        if last_fold_first_test_table_id != None and skip_test_table_count <= tables_per_fold:
                            self.scan_dataset.is_test_set = False
                            self.scan_dataset.is_training_set = True
                        else:
                            state = 'test_tables'
                            last_fold_first_test_table_id = self.scan_dataset.surface_id
                            continue #with same dataset
                            
                    elif state == 'test_tables':
                        
                        if last_table_id != self.scan_dataset.surface_id:
                            current_fold_test_table_count += 1
                             
                        if current_fold_test_table_count <= tables_per_fold:
                            self.scan_dataset.is_test_set = True
                            self.scan_dataset.is_training_set = False
                            last_fold_last_test_table_id = self.scan_dataset.surface_id#only valid after the whole run of the inner loop
                        else:
                            state = 'after_test_tables'
                            continue #with same dataset
    
                    elif state == 'after_test_tables':
                        self.scan_dataset.is_test_set = False
                        self.scan_dataset.is_training_set = True
                                
                    last_table_id = self.scan_dataset.surface_id    
                #get next one
                self.scan_dataset = self.scans_database.get_next_dataset()
            #self.scans_database.save()
            
            if fold == None or current_fold == fold:
                print ut.getTime(), 'start training, fold', fold

                if False==update_postprocess:
                    #do training:
                    self.train_and_save_Classifiers()
                    #self.load_Classifiers()
                    #do testing:
                    testresults_all = self.test_classifiers_on_testset()
                else: #just do postprocessing and load the rest:
                    testresults_all = self.update_test_postprocessing_on_testset()
                    
                testresults_crossvalidation += [testresults_all]
                if fold != None:
                    self.save_testresults_all_crossvalidation_fold(testresults_all, fold)
                
            current_fold = current_fold + 1
            
            #debug output:
#            debug_fold_count += 1
#            print 'fold', debug_fold_count
#            self.scan_dataset = self.scans_database.get_dataset(0)
#            while False != self.scan_dataset:
#                if self.scan_dataset.is_labeled: #only valid labelings
#                    print 'table id', self.scan_dataset.surface_id, 'train', self.scan_dataset.is_training_set, 'test', self.scan_dataset.is_test_set
#                self.scan_dataset = self.scans_database.get_next_dataset()
        if fold == None:
            self.save_testresults_crossvalidation(testresults_crossvalidation)
        
        return testresults_crossvalidation
    
    def collect_and_save_testresults_crossvalidation(self, folds):
        testresults_crossvalidation = []
        for i in range(folds):
            try:
                testresults_all = self.load_testresults_all_crossvalidation_fold(i)
                testresults_crossvalidation += [testresults_all]
            except IOError:
                print 'ERROR: no data for fold',i,'found!'
            #else:
            #    print 'ERROR: no data for fold',i,'found!'
                
        self.save_testresults_crossvalidation(testresults_crossvalidation)
    
    def save_testresults_all_crossvalidation_fold(self, testresults_all, fold):
        #save results for one fold of the crossvalidation process
        filename = self.config.path+'/'+'testresults_all_crossvalidation_fold_'+str(fold)+'.pkl'
        print ut.getTime(), "Saving: "+filename
        ut.save_pickle(testresults_all,filename)   
        
    def load_testresults_all_crossvalidation_fold(self, fold):
        filename = self.config.path+'/'+'testresults_all_crossvalidation_fold_'+str(fold)+'.pkl'
        print ut.getTime(), "loading: "+filename
        return ut.load_pickle(filename)
                
    
    def save_testresults_crossvalidation(self, testresults_crossvalidation):
        #save data:
        filename = self.config.path+'/'+'testresults_crossvalidation.pkl'
        print ut.getTime(), "Saving: "+filename
        ut.save_pickle(testresults_crossvalidation,filename)       
        
    def load_testresults_crossvalidation(self):
        filename = self.config.path+'/'+'testresults_crossvalidation.pkl'
        #print ut.getTime(), "loading: "+filename
        return ut.load_pickle(filename)
    
    
    #assume all data is already present/set/loaded
    def load_classifier_and_test_on_dataset(self, features, feature_data):
        #needed for display and to get the size of the labels-vector, 
        #do before assigning the correct labels!
        #    * (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
        #    * self.do_all_point_cloud_mapping()  
        if features == 'all_post':
            self.classifiers['all'].load()
            self.classifiers['all'].test(feature_data)
            # Postprocess uses ransac to fit a plane.  
            # This fails if zero 'surface' points are produced from previous two steps.
            labels, testresults = self.classifiers['all'].test_postprocess()
            self.save_classifier_labels(labels, testresults, 'all_post')
        else:
            self.classifiers[features].load()
            labels, testresults = self.classifiers[features].test(feature_data)
            self.save_classifier_labels(labels, testresults, features)
        return labels, testresults
    
                
    def test_classifiers_on_testset(self):
        
        testresults_all = {}        #loop through all marked datasets
    
#            testerrors_surface = {}
#            testerrors_clutter = {}
#            testerrors_sum_surface_clutter = {}
        self.scan_dataset = self.scans_database.get_dataset(0)
        #get size of training set in total
        for features, classifier in self.classifiers.iteritems():
            testresults_all[features] = {}
        testresults_all['all_post'] = {}
            
        while False != self.scan_dataset:
            if self.scan_dataset.is_test_set:
                
                self.load_data(self.scan_dataset.id, False)
                #needed for display and to get the size of the labels-vector, do before assigning the correct labels!
                (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
                self.do_all_point_cloud_mapping()  
                
                for features, classifier in self.classifiers.iteritems():
                    print ut.getTime(), 'start testing on ',features,'id',self.scan_dataset.id

                    labels, testresults = classifier.test()
                    #count_surface, count_clutter, count_surface_correct, count_clutter_correct, percent_surface_correct, percent_clutter_correct = testresults
                    self.save_classifier_labels(labels, testresults, features)
                    testresults_all[features][self.scan_dataset.id] = testresults
                            
                #test all+postprocessing:
                features = 'all_post'
                self.classifiers['all'].features = 'all_post' #just for the print-statements
                labels, testresults = self.classifiers['all'].test_postprocess()
                self.save_classifier_labels(labels, testresults, features)
                self.classifiers['all'].features = 'all' 
                testresults_all[features][self.scan_dataset.id] = testresults

            #get next one
            self.scan_dataset = self.scans_database.get_next_dataset()
                
        self.save_testresults_all(testresults_all)
        return testresults_all
    
    def update_test_postprocessing_on_testset(self):
        
        testresults_all = {}        #loop through all marked datasets

        self.scan_dataset = self.scans_database.get_dataset(0)
        #get size of training set in total
        for features, classifier in self.classifiers.iteritems():
            testresults_all[features] = {}
        testresults_all['all_post'] = {}
        
        while False != self.scan_dataset:
            if self.scan_dataset.is_test_set:
                print '0'
                self.load_data(self.scan_dataset.id, False)
                #needed for display and to get the size of the labels-vector, do before assigning the correct labels!
                (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
                self.do_all_point_cloud_mapping()  
                print '1'
                
                #load labels and features
                for features, classifier in self.classifiers.iteritems():
                    print ut.getTime(), 'start testing on ',features,'id',self.scan_dataset.id
                    self.classifiers[features].test_labels = self.load_Classifier_labels(features)
                    self.classifiers[features].test_feature_dict = ut.load_pickle(self.get_features_filename())
                    testresults = classifier.test_results(self.classifiers[features].test_feature_dict,self.classifiers[features].test_labels)
                    testresults_all[features][self.scan_dataset.id] = testresults
                                            
                #test all+postprocessing:
                features = 'all_post'
                print 'perform postprocessing on ',self.scan_dataset.id
                self.classifiers['all'].features = 'all_post' #just for the print-statements
                labels, testresults = self.classifiers['all'].test_postprocess()
                self.save_classifier_labels(labels, testresults, features)
                self.classifiers['all'].features = 'all' 
                testresults_all[features][self.scan_dataset.id] = testresults

            #get next one
            self.scan_dataset = self.scans_database.get_next_dataset()
                
        self.save_testresults_all(testresults_all)
        return testresults_all

        
    def save_testresults_all(self, testresults_all):
        #save data:
        filename = self.config.path+'/'+'testresults_all.pkl'
        print ut.getTime(), "Saving: "+filename
        ut.save_pickle(testresults_all,filename)       
        
    def load_testresults_all(self):
        filename = self.config.path+'/'+'testresults_all.pkl'
        print ut.getTime(), "loading: "+filename
        return ut.load_pickle(filename)
    
    def print_testresults_all_latex(self, testresults_all):
        
        testresults_total = {}
        for features, results in testresults_all.iteritems():
            testresults_total[features] = {'count_surface' : 0, 'count_clutter' : 0, 'count_surface_correct'  : 0, 'count_clutter_correct' : 0}
        
        testresults_all_reordered = {}
        for features, results in testresults_all.iteritems():
            for id, testresults in results.iteritems():
                testresults_all_reordered[id] = {}
            break
        
        #print testresults_all_reordered
        
        for features, results in testresults_all.iteritems():
            for id, testresults in results.iteritems():
                testresults_all_reordered[id][features] = testresults
                
        #print testresults_all_reordered
       
        ordered_features = ['range','color','all','all_post','baseline']
        #ordered_features = ['range','baseline']
        
        ids = testresults_all_reordered.keys()
        ids.sort()
        for id in ids:
            results = testresults_all_reordered[id]
        #for id, results in testresults_all_reordered.iteritems():
            first = True
            for features in ordered_features:
                testresults = results[features]  
                if first == True: 
                    print '\\multirow{5}{*}{',id.replace('_','-'),'}'
                first = False
                count_surface, count_clutter, count_surface_correct, count_clutter_correct, percent_surface_correct, percent_clutter_correct = testresults 
                
                testresults_total[features]['count_surface'] += count_surface
                testresults_total[features]['count_clutter'] += count_clutter
                testresults_total[features]['count_surface_correct'] += count_surface_correct
                testresults_total[features]['count_clutter_correct'] += count_clutter_correct
                if features != 'baseline':
                    name = features
                else:
                    name = 'baseline algo'
                if features == 'all_post':
                    print '&\\bf{',name.replace('_','-'),'}&\\bf{',count_surface,'}&\\bf{',count_surface_correct,'}&\\bf{',count_clutter, \
                        '}&\\bf{',count_clutter_correct,'}&\\bf{%(ps)02.2f}&\\bf{%(pc)02.2f}\\\\' % {'ps':percent_surface_correct, 'pc':percent_clutter_correct}
                else:
                    print '&',name.replace('_','-'),'&',count_surface,'&',count_surface_correct,'&',count_clutter, \
                        '&',count_clutter_correct,'&%(ps)02.2f&%(pc)02.2f\\\\' % {'ps':percent_surface_correct, 'pc':percent_clutter_correct}
                if features != 'baseline':
                    print '\\cline{2-8}'
                else:
                    print '\\hline\\hline' 
            
        print '\\hline\\hline'
        
        first = True
        for features in ordered_features:
            total_counts = testresults_total[features]  

            count_surface = total_counts['count_surface']
            count_clutter = total_counts['count_clutter']
            count_surface_correct = total_counts['count_surface_correct']
            count_clutter_correct = total_counts['count_clutter_correct']
            percent_surface_correct = float(count_surface_correct)/float(count_surface) * 100
            percent_clutter_correct = float(count_clutter_correct)/float(count_clutter) * 100
            if first == True: 
                print '\multirow{5}{*}{total}'
            first = False
            if features != 'baseline':
                name = features
            else:
                name = 'baseline algo'
            if features == 'all_post':
                print '&\\bf{',name.replace('_','-'),'}&\\bf{',count_surface,'}&\\bf{',count_surface_correct,'}&\\bf{',count_clutter, \
                    '}&\\bf{',count_clutter_correct,'}&\\bf{%(ps)02.2f}&\\bf{%(pc)02.2f}\\\\' % {'ps':percent_surface_correct, 'pc':percent_clutter_correct}
            else:
                print '&',name.replace('_','-'),'&',count_surface,'&',count_surface_correct,'&',count_clutter, \
                    '&',count_clutter_correct,'&%(ps)02.2f&%(pc)02.2f\\\\' % {'ps':percent_surface_correct, 'pc':percent_clutter_correct}
            if features != 'baseline':
                print '\\cline{2-8}'
            else:
                print '\\hline\\hline' 
                
        return testresults_total
                
    def print_testrestults_crossvalidation_latex(self, testresults_crossvalidation):

        ordered_features = ['range','color','all','all_post','baseline']
        testresults_total_crossvalidation = {}
        for features in ordered_features:
            testresults_total_crossvalidation[features] = {'count_surface' : 0, 'count_clutter' : 0, 'count_surface_correct'  : 0, 'count_clutter_correct' : 0}
        
        #print '\\hline\\hline', '\\hline\\hline' 
        latex_clearpage = False
        
        for testresults_all in testresults_crossvalidation:
            print r"""\begin{table*}[p]
              \centering
                
              \caption{Classification results }
              \label{tab:results}
              %\resizebox{1.80\columnwidth}{!} {
               \begin{tabular}{|c|c||c|c|c|c||c|c|}
                \hline
                \tbthc{Dataset}&\tbth{Features /}&\tbthc{\#points surface}&\tbth{\#points surface}&\tbthc{\#points clutter}&\tbth{\#points clutter}&\tbth{\% surface}&\tbth{\% clutter}
            \\    &\tbth{Algorithm}&&\tbth{correct}&&\tbth{correct}&\tbth{correct}&\tbth{correct}
            \\\hline"""
            testresults_total = self.print_testresults_all_latex(testresults_all)
            print r"""\end{tabular}
              %}
            \end{table*}"""
            if latex_clearpage:
                print r"""\clearpage"""
                latex_clearpage = False
            else:
                latex_clearpage = True
            for features in ordered_features:
                testresults_total_crossvalidation[features]['count_surface'] += testresults_total[features]['count_surface']
                testresults_total_crossvalidation[features]['count_clutter'] += testresults_total[features]['count_clutter']
                testresults_total_crossvalidation[features]['count_surface_correct'] += testresults_total[features]['count_surface_correct']
                testresults_total_crossvalidation[features]['count_clutter_correct'] += testresults_total[features]['count_clutter_correct']
                
       # print '\\hline\\hline', '\\hline\\hline'         
        
        print r"""\begin{table*}[p]
          \centering
          \caption{Classification results crossvalidation total}
          \label{tab:results}
          %\resizebox{1.80\columnwidth}{!} {
           \begin{tabular}{|c|c||c|c|c|c||c|c|}
            \hline
            \tbthc{Dataset}&\tbth{Features /}&\tbthc{\#points surface}&\tbth{\#points surface}&\tbthc{\#points clutter}&\tbth{\#points clutter}&\tbth{\% surface}&\tbth{\% clutter}
        \\    &\tbth{Algorithm}&&\tbth{correct}&&\tbth{correct}&\tbth{correct}&\tbth{correct}
        \\\hline"""
        first = True
        for features in ordered_features:        
            count_surface = testresults_total_crossvalidation[features]['count_surface']
            count_clutter = testresults_total_crossvalidation[features]['count_clutter']
            count_surface_correct = testresults_total_crossvalidation[features]['count_surface_correct']
            count_clutter_correct = testresults_total_crossvalidation[features]['count_clutter_correct']
            percent_surface_correct = float(count_surface_correct)/float(count_surface) * 100
            percent_clutter_correct = float(count_clutter_correct)/float(count_clutter) * 100
            
            if first == True: 
                print '\multirow{5}{*}{crossvalidation}'
            first = False
            if features != 'baseline':
                name = features
            else:
                name = 'baseline algo'
            if features == 'all_post':
                print '&\\bf{',name.replace('_','-'),'}&\\bf{',count_surface,'}&\\bf{',count_surface_correct,'}&\\bf{',count_clutter, \
                    '}&\\bf{',count_clutter_correct,'}&\\bf{%(ps)02.2f}&\\bf{%(pc)02.2f}\\\\' % {'ps':percent_surface_correct, 'pc':percent_clutter_correct}
            else:
                print '&',name.replace('_','-'),'&',count_surface,'&',count_surface_correct,'&',count_clutter, \
                    '&',count_clutter_correct,'&%(ps)02.2f&%(pc)02.2f\\\\' % {'ps':percent_surface_correct, 'pc':percent_clutter_correct}
            if features != 'baseline':
                print '\\cline{2-8}'
            else:
                print '\\hline\\hline'             
            
        print r"""\end{tabular}
          %}
        \end{table*}"""
            #print '@@@@@crossvalidation total: ',features,'@@@@@'
            #print testresults_total_crossvalidation[features]
            #print 'percent surface correct:',percent_surface_correct,'percent clutter correct:',percent_clutter_correct
       
       
    def generate_and_save_visualizations(self, id):
        feature_list = self.classifiers.keys()
        feature_list += ['labels', 'all_post']
        
        self.load_data(id, False)
        self.process_raw_data()
        
        for features in feature_list:
            self.img_labels = self.draw_mapped_labels_into_image(features)  
            self.save_labels_image(features)
            self.save_3d(features, True)
            
    def generate_and_save_visualizations_all(self):
        
        self.scan_dataset = self.scans_database.get_dataset(0)
        #get size of training set in total
        while False != self.scan_dataset:
            if self.scan_dataset.is_labeled: #former: is_testset
                self.generate_and_save_visualizations(id)
            
            #get next one
            self.scan_dataset = self.scans_database.get_next_dataset()
            
            
    def generate_and_save_visualizations_height(self):
        self.scan_dataset = self.scans_database.get_dataset(0)
        #get size of training set in total
        while False != self.scan_dataset:
            if self.scan_dataset.is_test_set:
                self.load_data(self.scan_dataset.id, False)
                self.process_raw_data()
                self.save_3d('height', True)

            #get next one
            self.scan_dataset = self.scans_database.get_next_dataset()
                
    def test_Classifiers(self):
        #loop all feature-sets
        (self.pts3d, self.scan_indices, self.intensities) = self.create_pointcloud(self.laserscans)
        self.do_all_point_cloud_mapping()
        for features, classifier in self.classifiers.iteritems():
            #needed only for display, do before assigning the correct labels!
            labels, testresults = classifier.test()
            self.save_classifier_labels(labels, testresults, features)
        
        #test all+postprocessing:
        features = 'all_post'
        labels, testresults = self.classifiers['all'].test_postprocess()
        self.save_classifier_labels(labels, testresults, features)
    
    def get_point_label(self, index):
        return self.map_polys[index]
    
        
    def save_classifier_labels(self, labels, testresults, features):
        #save data:
        dict = {'labels' : labels, 'testresults' : testresults} 
        filename = self.get_Classifier_labels_filename(features)
        print ut.getTime(), "Saving: "+filename
        ut.save_pickle(dict,filename)       
        print '...saving labels: len:', len(labels), 'testresults:',testresults

    def load_Classifier_labels(self, features):
        filename = self.get_Classifier_labels_filename(features)
        print 'loading', filename
        dict = ut.load_pickle(filename)
        print '...loading labels: len:', len(dict['labels']), 'testresults:',dict['testresults']
        return dict['labels']
    
    def get_Classifier_labels_filename(self, features):
        return self.config.path+'/data/'+self.scan_dataset.id+'_labels_Classifier_'+features+'_'+self.feature_type+'_k'+str(self.feature_neighborhood)+'_r'+str(self.feature_radius)+'.pkl'
   
        
    #as input for histogram/placement statistics/...
    def save_all_labeled_pointclouds(self):
        
        self.scan_dataset = self.scans_database.get_dataset(0)
        while False != self.scan_dataset:
            if self.scan_dataset.is_labeled:
                self.load_data(self.scan_dataset.id, False)
                print "printing label #", self.scan_dataset.id
                self.process_raw_data()
                print "successfully processed raw data"
                
                dict = {'points': self.pts3d,
                        'intensities': self.intensities,
                        'surface_height': self.scan_dataset.surface_height}
                
                filename = self.config.path+'/data/labeled_pointclouds/pointcloud_'+self.scan_dataset.id+'.pkl'
                print 'saving', filename
                ut.save_pickle(dict, filename)
                
            self.scan_dataset = self.scans_database.get_next_dataset()


