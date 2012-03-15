#!/usr/bin/python
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

import roslib; roslib.load_manifest('clutter_segmentation')
roslib.load_manifest('pr2_clutter_helper')
import acquire_pr2_data;
import rospy

from hrl_lib.util import getTime
print getTime(), 'START'

import processor
import configuration

try: from placement import Placement
except: print 'Cannot find placement.py for import.  Ignoring'
from label_object import label_object
from scan_dataset import scan_dataset

import numpy as np
import opencv as cv
import opencv.highgui as hg
print getTime(), 'IMPORTS DONE'


#------------------------
ROBOT='desktopScanner' #'PR2'#'desktopcanner'#'PR2' #'dummyScanner'=='codyRobot' #'desktopScanner'
DATASET_ID = 'pr2_example_0003'
DATA_LOCATION = '/home/jokerman/Desktop/PR2/'
z_above_floor = 0
#Theory:Imperitive to know "plane" of floor to do good classification in current scheme.
#ground_plane_rotation = '' 
#rot = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
#[above] data location must hold xml training data to load. or get error!
#[above] dataset id should be a unique name for files in subfolder data/

#-------------------------
'''Number of points to  '''
NUMBER_OF_POINTS = 1800#some large number #previously only 1000
SCALE = 1
IS_LABELED = True #What happens if false? Assume = No ROI cropping.  Might get skipped altogether.  TODO:check.
USE_POLYGONS = True #this is required right now!  why?
SAVE_INTENSITY_IMAGE = False
CalibrationVisual = True

MODE = 'all_post' # 'all', 'all_post' (ransac), 'color', 'laser'

displayOn = True 
#---------------------------

'''
    @param z_above_floor Approximate height of hokuyo above the ground.  Asssumed
    to be fixed for now.  On Cody robot the camera and hokuyo were at same z-height.
'''
def create_default_scan_dataset(unique_id = DATASET_ID, z_above_floor=1.32):
    dataset = scan_dataset()
    dataset.table_plane_translation = np.matrix([0,0,0]).T
    # approximate height of hokuyo above the ground, assume it to be fixed for now.
    dataset.ground_plane_translation = np.matrix([0,0,z_above_floor]).T 
    dataset.ground_plane_rotation = ''  #expects a 3x3 numpy matrix
    #pc.scan_dataset.is_test_set = True
    dataset.is_labeled = IS_LABELED
    dataset.id = unique_id
    return dataset

'''
    Define a region of interest polygon which crops camera image.
    Smaller size will result in a quicker processing time.
    Takes four crop values as % of the image size: xmin, xmax, ymin, ymax
    which range between 0 and 1.  Perhaps this would be more intuitive as
    actual crop values so that '0' on all values meant the original image and
    '.2' on everything crops away the 20% on all edges.
'''
##MAKE SURE THAT THESE MATCH ORIGINAL TAKE: CROP CAMERA IMAGE TO RIO
def define_ROI_polygons(w, h, crop_regions):
    '''
    # Create dummy edges for placement
    # Note this assumes we will be using a SQUARE region of interest -- not true in general.
    Usage: 
        w = image_size.width; h = image_size.height
        crop_region = (0.2,0.1,.2,.1) # % to crop left, crop right, crop top, crop bot
        surface_ROI, list_of_edges = define_ROI_polygons(w, h, crop_region) 
        pc.scan_dataset.polygons.append(surface_ROI)
        for p in list_of_edges: pc.scan_dataset.polygons.append(p)
    '''
    (left, right, top, bottom) = crop_regions
    width_min = w * left
    width_max = w * (1-right)
    height_min = h * top
    height_max = h * (1-bottom)
    p = label_object()
    p.set_label('surface')
    p.add_point((width_min,height_min))
    p.add_point((width_min,height_max))
    p.add_point((width_max,height_max))
    p.add_point((width_max,height_min))
    table_surface = p #A polygon shape (rectangle) defining most generous guess of table surface.
    print 'surface',p.get_points()
    p = label_object()
    p.set_label('edge_down')
    p.add_point((width_min,height_min))
    p.add_point((width_min,height_max))
    p.add_point((width_max,height_max))
    p.add_point((width_max,height_min))
    p.add_point((width_min,height_min))
    list_of_edges = (p,) 
    # [Above] Any number of polygons which define surface border. More can be added.
    print 'edge',p.get_points()
    return table_surface, list_of_edges

def show_image(window_name, img, wait=False):
    hg.cvStartWindowThread()
    RESIZABLE = 0
    hg.cvNamedWindow(window_name, RESIZABLE)
    hg.cvShowImage(window_name, img)
    if wait:
        print 'show_image: press any key to continue..'
        cv.highgui.cvWaitKey()

#------------------------------------

###CHANGE THIS TO THE DIRECTORY WITH RESULTS FROM CODY:
###  change 'codyRobot' to 'dummyScanner' or code tries to find camera drivers, etc.
cfg = configuration.configuration(DATA_LOCATION, ROBOT) #'dummyScanner'
pc = processor.processor(cfg)
pc.features_k_nearest_neighbors = None

# Name of dataset will allow loading of saved image / scan.
pc.scan_dataset = create_default_scan_dataset( DATASET_ID, z_above_floor)#1.32

'''
    PC.LOAD_RAW_DATA()
    Browses for content with the following names:
        data/UNIQUE_ID_image.png
        data/UNIQUE_ID_laserscans.pkl
        self.image_angle is set or zeroed               
    Sets in pc as: self.img,
                   self.laserscans,
                   sets (or zeros) self.image_angle
    
'''
if cfg.device=='PR2':
    rospy.init_node('rosSegmentCloud')
    print 'STARTING NEW COLLECTION NODE'
    ac = acquire_pr2_data.AcquireCloud()
    #ac.print_test()
    print 'FINISHED COLLECTING, SENDING DATA BACK'
               
    np_img, pc.pts3d, pc.intensities, camPts, colors, idx_list = ac.return_data_for_segmentation() 
        # >> return self.img, self.pts3d, self.intensities, 
        #        self.camPts, self.colors, self.camPts_idx
    pc.pts3d = np.array(pc.pts3d)
    N = len(idx_list)
    Nbound = len( idx_list[idx_list] )
    pc.img = cv.cvCloneImage(np.array(np_img) )
    pc.laserscans = []
    pc.scan_indices = np.array( range(N) )
    camPts_bound = camPts[:,idx_list]
    pts3d_bound = pc.pts3d[:,idx_list]
    scan_indices_bound = np.array(range(N))[idx_list]
    intensities_bound = pc.intensities[idx_list]
    pc.map = (camPts_bound, camPts, idx_list, pts3d_bound,
                            scan_indices_bound, intensities_bound)

#create dummy surface polygon to define ROI
# [Below] These polygons will be used to generate a 'mask' cv image and used to 
# limit the area of image where 3D projected points are considered.
image_size = cv.cvGetSize(pc.img)
w = image_size.width
h = image_size.height
crop_region = (0.3,0.3,0.3,0.3) # % to crop left, crop right, crop top, crop bot
surface_ROI, list_of_edges = define_ROI_polygons(w, h, crop_region) 
pc.scan_dataset.polygons.append(surface_ROI)
for p in list_of_edges: pc.scan_dataset.polygons.append(p)


''' [Below] Does 'map_laser_into_cam2D' to get camPts, and bounds on all values
    Optionally applies translate and rotate relative to a groundplane (if defined)
'''
if not cfg.device=='PR2':
    pc.do_all_point_cloud()
else:
    pc.do_all_point_cloud(map_already_exists=True)

''' [Below] MAKE SURE THESE MATCH ORIGINAL SCAN: 
    truncate 3D data to a volume (center, and h,w,d).  Numbers are in meters.
    Suggest right-arm values are: np.array([0.55,-0.4,1.0]), 0.6, 0.8, 1.3)
    Note, this increases speed by throwing out points outside likely table area.
    It also is meant to remove the 'floor' as a possible candidate for table surface fits. 
'''
pc.truncate_pointcloud_to_voi(np.array([0.55,-0.4,1.0]), 2, 2, 1.3)

''' Visual to quickly check TF'''
if CalibrationVisual: 
    overlay_img = pc.draw_mapped_laser_into_image(pc.map, pc.pts3d, pc.img)
    show_image('calibration_visual', overlay_img)

if USE_POLYGONS:
    print 'do polygon mapping'
    pc.do_polygon_mapping() #-
    '''Visual based on accurate polygons.'''
    ###pc.display_3d('labels')### 

    print 'map laser into image again'
    pc.img_mapped = pc.draw_mapped_laser_into_image(pc.map, pc.pts3d, pc.img)  #-  
    #Below: create B & W images for manually entered 'table edge and center' values we specified. 
    pc.create_polygon_images()  #called twice? 
    pc.img_mapped_polygons = pc.draw_mapped_laser_polygons_into_image(pc.map, pc.pts3d, pc.img)    
    
if SAVE_INTENSITY_IMAGE:
    print 'Begin Optional save of Intensity image'
    #This *may* involve assuming a tilting scan exists 
    pc.img_intensities = pc.create_intensity_image(pc.laserscans[0])  #-  
    
else:
    n, m = np.shape(pc.pts3d_bound)  #hack! hack!
    polygon_mapping = list(1 for x in range(m)) # 0 == background/floor/unlabeled/other
    pc.map_polys = polygon_mapping #Jas:filled with CLUTTER / SURFACE labels later
#+

###pc.display_all_data()###

pc.scan_dataset.ground_plane_normal = np.matrix([0.,0.,1.]).T

''' Comment: nonzero_indices comes from pc.map_polys comes from other labeling technique.
    I have changed the second Param below to: generate_and_save_all_neighborhood_indices=True
'''
print 'lucid - about to generate features'
SAVE_UNUSED_NEIGHBORS_TO = False #Jason had run with True, True.  
                                 #Takes a lot longer obviously!
                                 #Might slightly effect the 'global' 
                                 #feature vector on each point.
feature_data = pc.generate_features(NUMBER_OF_POINTS,False, True)  ### previously 1000, False,True
print 'lucid - done generating features'
labels, testresults = pc.load_classifier_and_test_on_dataset(MODE, feature_data) #'all_post'
###pc.display_3d('all')###
#Labels are now generated --> can be saved to file?


if False and displayOn:
    pc.display_segmentation_image('all')
    print "TEST_DISPLAY: showing segmentation image |",NUMBER_OF_POINTS,"points.  Please press a key"
    cv.highgui.cvWaitKey(0)   
    pc.save_segmentation_image('all')

#feature_data = {'point_indices':[], 'set_size':1} #dummy dict (->invalid statistics), baseline doesn't need real features
#labels, testresults = pc.load_classifier_and_test_on_dataset('baseline', feature_data)
###pc.display_3d('baseline')###

#test placement, use classification results:
pc.map_polys = pc.load_Classifier_labels(MODE) #'all_post'
#this function prints: "loading labels: len: ..."

print 'pts3d_bound is size', pc.pts3d_bound.shape
print 'map_polys is size', pc.map_polys.shape

#+-
SHOW_IN_ROS= True
if SHOW_IN_ROS:
    import roslib
    roslib.load_manifest('pr2_lcs_helper')
    from sensor_msgs.msg import PointCloud
    import acquire_pr2_data, rospy
    try: rospy.init_node('table_labeler')
    except: print 'rospy init node has already been called: skipping'
    pub = rospy.Publisher('labeled_cloud', PointCloud)
    while not rospy.is_shutdown():
        ac.pub_cloud_bound()
        colors = pc.map_polys #actually labels
        acquire_pr2_data.publish_cloud(pub, '/base_footprint', pc.pts3d_bound[:,colors!=0], intensities=colors[colors!=0])
        rospy.sleep(2)
        print 'published'
    # >> publish_cloud(pub, frame, pts3d, intensities=None, labels=None, colors=None):
#+-

pc.scan_dataset.ground_plane_rotation = np.matrix([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]).T
#estimate table plance translation from mean of classified points:
#Surface points are collected, and their average z-value is assumed as the table height
data_idx = np.where(np.asarray(pc.map_polys) == processor.LABEL_SURFACE)[0]
data = np.asarray(pc.pts3d_bound)[:,data_idx]
table_height = np.mean(data, axis=1)[2]
pc.scan_dataset.table_plane_translation = np.matrix([0.,0.,table_height]).T

#----------------------------------------------------

'''
# Save results in a nice format.
'''
if False:
    filename = pc.config.path + 'data/' + unique_name + '_segmentation_results.py'
    roslib.load_manifest('display_stuff'); import save_labeled_cloud;
    save_labeled_cloud.save_results(pc.pts3d_bound, pc.intensities, labels,
                                    pc.idx_bound, pc.pts3d)
    save_labeled_cloud.save_feature_data(feature_data['features'],
                                         feature_data['point_indices'])
    save_labeled_cloud.save_map2D(pc.map[0], table_height) #for placement.
''' Comments
    * pc.pts3d_bound #3 by Nbound
    * pc.idx_bound #1 by Nbound, indexes to re-project information into full pointcloud.
    * labels # Nbound, use if loading classifer on featuer_data
    * pc.map_polys # --> Nbound, labels created from polygons.  
                   #     Overrided by leaning-based labels, so they are identical
                   #     at the end of this document.
    * Nbound = len(intensities_bound) #--> example range(19598)
    * feature_data['features'] # Nbound x 35, actually SLIGHTLY less, only 19541.
    * feature_data['point_indices']
    * Nfeature = feature_data['set_size'] #--> slightly less than maximum.  
                                          #I assume this is at most NUMBER_OF_POINTS
    -Others:
    * pc.image_labels
    * pc.img_mapped
    * pc.img_mapped_polygons
    * testresults #Not sure what this is?
    * data_idx #indexes taken from pts3d_bound ONLY where label is nonzero
    * data #3d values of pts3d_bound where label is nonzero... 
           # However as a list so 3x longer.
    * scan_indices - an artifact from the hokuyo.  Original acquired line order?
'''   

if False: ####### Placement routine.
    object_height = 0.1
    #SCALE = 1
    resolution = [.01*SCALE, .01*SCALE]  #sets resolution of occupancy grid
    print 'NOTE: Resolution is ',100*resolution[0], 'cm' ###
    polygon = label_object()
    polygon.add_point([0,0])
    polygon.add_point([0,5*SCALE])
    polygon.add_point([10*SCALE,5*SCALE])
    polygon.add_point([10*SCALE,0])
    ###object_height = 0.1

    print 'creating placement object'
    pl = Placement(pc, resolution)  ###REPLACE WITH MY OWN CLASS DEFINITION WITH FUNCTIONs

    if displayOn:
        placement_point = pl.test_placement(polygon, object_height) 
    else:
        placement_point = pl.find_placement(polygon, object_height)#Add param True to get debug popups
        
    placement_point -= pc.scan_dataset.ground_plane_translation
    
    #Assumes 'codyRobot'==ROBOT
    #This should be optional
    import mekabot.coord_frames as mcf
    placement_point_global = mcf.thok0Tglobal(placement_point)
    print 'placement point in global coordinate frame:', placement_point_global.T
        

    if displayOn:    
        print 'displaying current placement'
        pl.display_current_placement_in_heightmap()
        cv.highgui.cvWaitKey()

    if displayOn:
        print 'importing mayavi'
        ###from enthought.mayavi import mlab
        print 'showing 3D mayavi'
        ###mlab.show()
        #What does this do?


    print getTime(), 'DONE'
    #print pts3d
    
    #==============
    
