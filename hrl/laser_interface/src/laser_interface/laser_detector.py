# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## @author Hai Nguyen/hai@gatech.edu
from pkg import *
#from opencv import highgui as hg
#import opencv as cv
import cv
import numpy as np

import pickle
import itertools as it
import pickle as pk
import time
import functools as ft

#import videre as vd
#import camera as cam
import util as ut
import random_forest as rf
import hrl_opencv.adaptors as ad

class Rect:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def as_cv_rect(self):
        return (int(round(self.x)), int(round(self.y)), int(round(self.width)), int(round(self.height)))

    def top_left(self):
        #return cv.Point(int(round(self.x)), int(round(self.y)))
        return (int(round(self.x)), int(round(self.y)))

    def bottom_right(self):
        #return cv.Point(int(round(self.x+self.width)), int(round(self.y+self.height)))
        return (int(round(self.x+self.width)), int(round(self.y+self.height)))

    def keep_inside(self, minx, maxx, miny, maxy):
        x      = bound(self.x,       minx,  maxx)
        y      = bound(self.y,       miny,  maxy)
        mx     = bound(x+self.width, minx,  maxx)
        my     = bound(y+self.height, miny, maxy)
        return Rect(x,y, mx-x, my-y)

    def __repr__(self):
        return 'Rect: ' + str(self.x) + ' ' + str(self.y)+ ' ' + str(self.width)+ ' ' + str(self.height)

def bound(value, min_val, max_val):
    return min(max(value, min_val), max_val)

def strip_extension(filename):
    a = filename.split('.')
    return a[0]

def load_pickle(filename):
    p = open(filename, 'r')
    picklelicious = pk.load(p)
    p.close()
    return picklelicious

def dump_pickle(object, filename):
    pickle_file = open(filename, 'w')
    pk.dump(object, pickle_file)
    pickle_file.close()

def bound_inside_image(x_range, y_range, bounds_obj):
    if bounds_obj.__class__ == cv.cvmat:
        width  = bounds_obj.width
        height = bounds_obj.height
    elif bounds_obj.__class__ == ().__class__:
        width = bounds_obj[0]
        height = bounds_obj[1]
    else:#last case is np array
        width  = bounds_obj.shape[1]
        height = bounds_obj.shape[0]

    start_x = max(x_range[0], 0)
    start_y = max(y_range[0], 0)
    end_x   = min(x_range[1], width)
    end_y   = min(y_range[1], height)
    return {'x': (start_x, end_x), 'y': (start_y, end_y)}

def cvsize_to_list(cvsize):
    return list(cvsize)
    #return (cvsize.width, cvsize.height)

def int_from_float(f):
    return int(round(f))

def scalar_to_matrix(s, num_val):
    vals = []
    for i in range(num_val):
        vals.append(s[i])
    return np.matrix(vals)

def np2cv(np_image):
    pil_image = ut.np2pil(np_image)
    pil_image.save("_temp.bmp", "BMP")
    return cv.LoadImage("_temp.bmp")

def make_visible_binary_image(img):
        cv.Threshold(img, img, 0, 255, cv.CV_THRESH_BINARY)

def remove_large_blobs(binary_image, max_area, max_dim=30):
    blob_statistics(binary_image, max_area, max_dim)
    return binary_image

def blob_statistics(binary_image, max_area=99999.0, max_dim=99999.0):
    #'''
    #    WARNING: this function destructively modifies binary_image if max_area is set
    #'''
    statistics                = []
    storage                   = cv.CreateMemStorage(0)
    #FindContours(image,        storage, mode=CV_RETR_LIST, method=CV_CHAIN_APPROX_SIMPLE, offset=(0, 0))
    contours = cv.FindContours(binary_image, storage, cv.CV_RETR_TREE, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))
    #number_contours, contours = cv.FindContours(binary_image, storage, cv.sizeof_CvContour, cv.CV_RETR_TREE, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))
            #TODO: FIGURE OUT WHAT THE EQUIV OF SIZEOF IS IN OPENCV2

    #moments = cv.Moments()
    original_ptr = contours
    #print 'GOT contours', len(contours), contours
    #print 'GOT contours',  contours
    #import pdb
    #pdb.set_trace()
    #for e in contours:
    #    print e, e.__class__

    while contours != None:
        #print '.'
	#import pdb
	#pdb.set_trace()
	try:
            bx, by, bwidth, bheight = cv.BoundingRect(contours, 0)
            bounding_rect = Rect(bx, by, bwidth, bheight)
            moments = cv.Moments(contours, 0)
            #area = moments.m00
            #approximation to area since cvMoments' area seem broken
            area = bounding_rect.width * bounding_rect.height
	    if False:
	        #NOT WORKING!!
	        if moments.m00 == 0.0:
                    centroid = (bounding_rect.x, bounding_rect.y)
                else:
                    centroid = (moments.m10/moments.m00, moments.m01/moments.m00)
	    else:
	        if bwidth > 0:
	    	    cx = bx + bwidth/2.
	        else:
	    	    cx = bx

	        if bheight > 0:
	    	    cy = by + bheight/2.
	        else:
	    	    cy = by
	        centroid = (cx, cy)

            if area > max_area or bounding_rect.width > max_dim or bounding_rect.height > max_dim:
                cv.DrawContours(binary_image, contours, cv.Scalar(0), cv.Scalar(0), 0, cv.CV_FILLED) 
            else:
                stats = {'area': area, 'centroid': centroid, 'rect': bounding_rect}
                statistics.append(stats)
            contours = contours.h_next()
        except:
	    break
    return statistics

## 
# Throws away blobs that are too large, selects the largest one
# 
def select_laser_blob(blobs, approx_laser_point_size=40, min_area = 1.0):
    size_filtered_blobs = []
    #rejected = False
    for b in blobs:
        #print 'select_laser_blob:', b['rect'].width, b['rect'].height, b['area'], b['centroid']
        if b['rect'].width <= approx_laser_point_size and b['rect'].height <= approx_laser_point_size and b['area'] >= min_area:
            size_filtered_blobs.append(b)

    #if len(blobs) > 0:
    #    print 'select_laser_blob: incoming', len(blobs), 'size filtered', len(size_filtered_blobs)

    if len(size_filtered_blobs) == 0:
        return None

    largest_area = 0
    largest_blob = None
    for b in size_filtered_blobs:
        if b['area'] > largest_area:
            largest_area = b['area']
            largest_blob = b
    #if len(blobs) > 0:
    #   print 'select_laser_blob:', largest_blob
    return largest_blob

def select_laser_track(tracks, min_age):
    oldest_age = -1
    oldest = None
    for t in tracks:
        if t.age() > oldest_age:
            oldest_age = t.age()
            oldest     = t
    if oldest_age >= min_age:
        if oldest.fresh:
            return oldest
        else:
            return None
    else:
        return None

def draw_detection(frame, laser_blob):
    if frame.nChannels == 1:
        color = cv.Scalar(200)
    else:
        color = cv.Scalar(255, 255, 0)

    if laser_blob != None:
        #point = cv.cvPoint(int(round(laser_blob['centroid'][0])), int(round(laser_blob['centroid'][1])))
        point = (int(round(laser_blob['centroid'][0])), int(round(laser_blob['centroid'][1])))
        cv.Circle(frame, point, 10, cv.Scalar(0,0,255), 1)
        cv.Circle(frame, point, 1, color, cv.CV_FILLED)

def draw_blobs(frame, blobs):
    if frame.nChannels == 1:
        color = cv.Scalar(200)
    else:
        color = cv.Scalar(255, 0, 0)

    for b in blobs:
        rect = blob_to_rect(b)
        if rect != None:
            cv.Rectangle(frame, rect.top_left(), rect.bottom_right(), color, 1)

class CombineMasks:
    def __init__(self, sample_image, channels=1):
        self.combined = cv.CreateImage(cv.GetSize(sample_image), 8 , channels)

    def combine(self, images):
        cv.Set(self.combined, 1)
        for img in images:
            cv.Mul(self.combined, img, self.combined)
        return self.combined

class Mask:
    def __init__(self, sample_image):
        self.thres_red_img         = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.thres_green_img       = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.thres_blue_img        = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.merged_frame          = cv.CreateImage(cv.GetSize(sample_image), 8 , 3)

    def mask(self, mask, r, g, b):
        cv.Mul(r, mask, self.thres_red_img)
        cv.Mul(g, mask, self.thres_green_img)
        cv.Mul(b, mask, self.thres_blue_img)
        cv.Merge(self.thres_blue_img, self.thres_green_img, self.thres_red_img, None, self.merged_frame);
        return self.merged_frame

class SplitColors:
    def __init__(self, sample_image):
        self.green_img             = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.red_img               = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.blue_img              = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)

    def split(self, image):
        cv.Split(image, self.blue_img, self.green_img, self.red_img, None);
        return (self.red_img, self.green_img, self.blue_img)

class BrightnessThreshold:
    def __init__(self, sample_image, thres_low=60, thres_high=200, tune=False):
        #print sample_image.__class__
        self.thres_low  = thres_low
        self.thres_high = thres_high
        self.set_thresholds([thres_low, thres_high])
        #self.csplit = SplitColors(sample_image)
        #if should_mask:
        #    self.mask   = Mask(sample_image)
        self.thresholded_low      = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.thresholded_high     = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.thresholded_combined = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        #self.should_mask = should_mask
        #self.channel = channel
        self.debug = False
        self.tune = tune

        #if tune:
        #    cv.NamedWindow('low', 1)
        #    cv.NamedWindow('high', 1)
    def set_thresholds(self, thresholds):
        self.thres_low = thresholds[0]
        self.thresh_high = thresholds[1]

    def get_thresholded_image(self):
        return self.thresholded_combined

    def threshold(self, thres_chan):
        #r, g, b = self.csplit.split(image)
        #if self.channel == 'green':
        #    thres_chan = g
        #elif self.channel == 'red':
        #    thres_chan = r
        #elif self.channel == 'blue':
        #    thres_chan = b
        if self.tune:
            result_val = 255
        else:
            result_val = 1

        start_time = time.time()
        cv.Threshold(thres_chan, self.thresholded_low, self.thres_low, result_val, cv.CV_THRESH_BINARY)
        threshold_time = time.time()
        cv.Dilate(self.thresholded_low, self.thresholded_low)
        #if self.tune:
        #    cv.ShowImage('low', self.thresholded_low)
        #    cv.WaitKey(1)
        dilate_time = time.time()
        remove_large_blobs(self.thresholded_low, LaserPointerDetector.MAX_BLOB_AREA)
        remove_time = time.time()

        cv.Threshold(thres_chan, self.thresholded_high, self.thres_high, result_val, cv.CV_THRESH_BINARY)
        threshold_time2 = time.time()
        cv.Dilate(self.thresholded_high, self.thresholded_high)
        #if self.tune:
        #    cv.ShowImage('high', self.thresholded_high)
        #    cv.WaitKey(1)
        dilate_time2 = time.time()
        remove_large_blobs(self.thresholded_high, LaserPointerDetector.MAX_BLOB_AREA)
        remove_time2 = time.time()

        cv.Or(self.thresholded_low, self.thresholded_high, self.thresholded_combined)
        or_time = time.time()


        if self.debug:
            print '      threshold_time %.4f' %(threshold_time - start_time)
            print '      dilate_time %.4f' %(dilate_time - threshold_time)
            print '      remove_time %.4f' %(remove_time - dilate_time)

            print '      threshold_time2 %.4f' %(threshold_time2 - remove_time)
            print '      dilate_time2 %.4f' %(dilate_time2 - threshold_time2)
            print '      remove_time2 %.4f' %(remove_time2 - dilate_time2)

            print '      or_time %.4f' %(or_time - remove_time2)




        #if self.should_mask:
        #    joined_image = self.mask.mask(self.thresholded_combined, r, g, b)
        #    return joined_image
        #else:
        return self.thresholded_combined

class MotionSubtract:
    def __init__(self, sample_image, adaptation_rate=0.8, threshold=10):
        self.accumulator           = cv.CreateImage(cv.GetSize(sample_image), 32, 1)
        cv.SetZero(self.accumulator)
        self.thresholded_img       = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.difference_img        = cv.CreateImage(cv.GetSize(sample_image), 32 , 1)
        self.green32_img           = cv.CreateImage(cv.GetSize(sample_image), 32 , 1)
        self.adaptation_rate       = adaptation_rate
        self.threshold             = threshold
        #if should_mask:
        #    self.mask              = Mask(sample_image)
        #self.should_mask           = should_mask
        #self.csplit                = SplitColors(sample_image)
        #self.channel               = channel

    def get_thresholded_image(self):
        return self.thresholded_img

    def subtract(self, thres_chan):
        #r, g, b = self.csplit.split(image)
        #if self.channel == 'green':
        #    thres_chan = g
        #elif self.channel == 'red':
        #    thres_chan = r
        #elif self.channel == 'blue':
        #    thres_chan = b

        cv.RunningAvg(thres_chan, self.accumulator, self.adaptation_rate)
        cv.CvtScale(thres_chan, self.green32_img)
        cv.Sub(self.green32_img, self.accumulator, self.difference_img)
        cv.Threshold(self.difference_img, self.thresholded_img, self.threshold, 1, cv.CV_THRESH_BINARY)
        cv.Dilate(self.thresholded_img, self.thresholded_img, iterations=1)
        remove_large_blobs(self.thresholded_img, max_area = LaserPointerDetector.MAX_BLOB_AREA)

        return self.thresholded_img
        #if self.should_mask:
        #    merged_frame = self.mask.mask(self.thresholded_img, r, g, b)
        #else:
        #    merged_frame = None
        #return {'image': merged_frame, 'binary': self.thresholded_img}

class ColorLearner:
    def learn_sequence(self, learn_sequence, file="ColorLearner.learn_sequence", use_entire_patch=True):
        def swap_colors(mat):
            tmp = mat[0,0]
            mat[0,0] = mat[0,2]
            mat[0,2] = tmp
        N_pixels    = 0
        summed      = np.matrix([0.0, 0.0, 0.0]) 
        squared_sum = np.matrix([0.0, 0.0, 0.0])

        for img, binary_mask, detection in learn_sequence:
            #Cut out patch
            if detection == None:
                print "ColorLearner.learn_sequence: there are no detections for this image, skipping."
                continue
            patch        = cv.CloneImage(cv.GetSubRect(img, detection['rect'].as_cv_rect()))
            binary_patch = cv.CloneImage(cv.GetSubRect(binary_mask, detection['rect'].as_cv_rect()))

            #Split it
            csplit       = SplitColors(patch)
            r,g,b        = csplit.split(patch)

            #Mask it
            mmask        = Mask(patch)
            patch_masked = mmask.mask(binary_patch, r,g,b)

            #Convert to numpy matrix
            #patch_np    = ut.cv2np(patch_masked, 'BGR')
            patch_np    = ad.cvmat2array(patch_masked)
            if use_entire_patch:
                #calculate mean
                area        = patch_np.shape[0] * patch_np.shape[1]
                N_pixels    = N_pixels + area
                patch_sum   = np.sum(np.sum(patch_np, axis=0), axis=0)
                summed      = summed + patch_sum

                #calculate variance
                squared_patch = np.power(patch_np, 2.0)
                patch_sqsum   = np.sum(np.sum(squared_patch, axis=0), axis=0)
                squared_sum   = squared_sum + patch_sqsum
            else:
                x = np.round(patch_np.shape[0] / 2)
                y = np.round(patch_np.shape[1] / 2)
                N_pixels = N_pixels + 1
                summed = summed + patch_np[x,y]
                squared_sum = squared_sum + np.power(patch_np[x,y], 2.0)

            means       = summed / N_pixels
            std         = np.power((squared_sum - (N_pixels * np.power(means, 2.0))) / (N_pixels - 1), 0.5)
            swap_colors(means)
            swap_colors(std)
            seg_pickle  = {'mean': means, 'std': std }
            dump_pickle(seg_pickle, file)

        self.color_bounds = seg_pickle

class ColorSegment(ColorLearner):
    def __init__(self, sample_image, range_lower = 1, range_upper = 1, file='ColorLearner.learn_sequence'):
        self.range_lower = range_lower
        self.range_upper = range_upper
        f = open(file, 'r')
        self.color_bounds = pickle.load(f)
        f.close()
        self.combined       = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.r              = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.g              = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.b              = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.inrange_r      = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.inrange_g      = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.inrange_b      = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.temp           = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)

    def label(self, image):
        cv.Split(image, self.b, self.g, self.r, None);
        range_scale_upper = self.range_upper
        range_scale_lower = self.range_lower
        cv.InRangeS(self.b, self.color_bounds['mean'][0,0] - range_scale_lower * self.color_bounds['std'][0,0],
                              self.color_bounds['mean'][0,0] + range_scale_upper * self.color_bounds['std'][0,0], self.inrange_b)

        cv.InRangeS(self.g, self.color_bounds['mean'][0,1] - range_scale_lower * self.color_bounds['std'][0,1],
                              self.color_bounds['mean'][0,1] + range_scale_upper * self.color_bounds['std'][0,1], self.inrange_g)

        cv.InRangeS(self.r, self.color_bounds['mean'][0,2] - range_scale_lower * self.color_bounds['std'][0,2],
                              self.color_bounds['mean'][0,2] + range_scale_upper * self.color_bounds['std'][0,2], self.inrange_r)

        #check pixels labeled in binary image
        cv.And(self.inrange_b, self.inrange_g, self.temp)
        cv.And(self.inrange_r, self.temp, self.combined)
        cv.Dilate(self.combined, self.combined)
        cv.Erode(self.combined, self.combined)
        return cv.CloneImage(self.combined)

class ColorFilter(ColorLearner):
    def __init__(self, image_size, file='ColorLearner.learn_sequence', threshold = 1.0):
        self.threshold    = threshold
        self.mask         = cv.CreateImage((image_size[0], image_size[1]), 8 , 1)
        self.color_bounds = load_pickle(file)

    def pick(self, original_image, components, binary_image):
        def std_dist(point, color_idx):
            return abs(point - self.color_bounds['mean'][0, color_idx]) / self.color_bounds['std'][0,color_idx]

        valid = []
        not_valid = []
        for component in components:
            patch = cv.CloneImage(cv.GetSubRect(original_image, component['rect'].as_cv_rect()))
            mask  = cv.CloneImage(cv.GetSubRect(binary_image, component['rect'].as_cv_rect()))
            averages = cv.Avg(patch, mask)
            b_avg = averages[0]
            g_avg = averages[1]
            r_avg = averages[2]
            
            if ((std_dist(r_avg, 0) < self.threshold) 
                    and (std_dist(g_avg, 1) < self.threshold) 
                    and (std_dist(b_avg, 2) < self.threshold)):
                valid.append(component)
            else:
                not_valid.append(component)

        cv.Copy(binary_image, self.mask)
        for v in not_valid:
            point = (int(round(v['centroid'][0])), int(round(v['centroid'][1])))
            if 0 != self.mask[point[1], point[0]]:
                cv.FloodFill(self.mask, (point[0], point[1]), cv.Scalar(0)) 
        return {'binary': self.mask, 'components': valid}


class LaserPointerDetector:

    #These are currently set using parameters on the parameter server
    SHADE_EXPOSURE               = rospy.get_param('laser_pointer_detector/SHADE_EXPOSURE') 
    SHADE_DARK_IMAGE_THRESHOLD   = rospy.get_param('laser_pointer_detector/SHADE_DARK_IMAGE_THRESHOLD') 
    SHADE_BRIGHT_IMAGE_THRESHOLD = rospy.get_param('laser_pointer_detector/SHADE_BRIGHT_IMAGE_THRESHOLD') 

    SUN_EXPOSURE                 = rospy.get_param('laser_pointer_detector/SUN_EXPOSURE') 
    SUN_DARK_IMAGE_THRESHOLD     = rospy.get_param('laser_pointer_detector/SUN_DARK_IMAGE_THRESHOLD') 
    SUN_BRIGHT_IMAGE_THRESHOLD   = rospy.get_param('laser_pointer_detector/SUN_BRIGHT_IMAGE_THRESHOLD') 

    MAX_BLOB_AREA                = rospy.get_param('laser_pointer_detector/MAX_BLOB_AREA') 
    LASER_POINT_SIZE             = rospy.get_param('laser_pointer_detector/LASER_POINT_SIZE') 
    MIN_AGE                      = rospy.get_param('laser_pointer_detector/MIN_AGE') 
    NUMBER_OF_LEARNERS           = rospy.get_param('laser_pointer_detector/NUMBER_OF_LEARNERS') 
    PCA_VARIANCE_RETAIN          = rospy.get_param('laser_pointer_detector/PCA_VARIANCE_RETAIN') 

    TRACKER_MAX_PIX_TRESHOLD     = rospy.get_param('laser_pointer_detector/TRACKER_MAX_PIX_TRESHOLD') 
    TRACKER_MAX_TIME_THRESHOLD   = rospy.get_param('laser_pointer_detector/TRACKER_MAX_TIME_THRESHOLD') 

    CLASSIFICATION_WINDOW_WIDTH  = rospy.get_param('laser_pointer_detector/CLASSIFICATION_WINDOW_WIDTH') 
    DEFAULT_DATASET_FILE         = rospy.get_param('laser_pointer_detector/DEFAULT_DATASET_FILE') 

    #SHADE_EXPOSURE               = None
    #SHADE_DARK_IMAGE_THRESHOLD   = None
    #SHADE_BRIGHT_IMAGE_THRESHOLD = None
    #                               
    #SUN_EXPOSURE                 = None
    #SUN_DARK_IMAGE_THRESHOLD     = None
    #SUN_BRIGHT_IMAGE_THRESHOLD   = None
    #                               
    #MAX_BLOB_AREA                = None
    #LASER_POINT_SIZE             = None
    #MIN_AGE                      = None
    #NUMBER_OF_LEARNERS           = None
    #PCA_VARIANCE_RETAIN          = None
    #                               
    #TRACKER_MAX_PIX_TRESHOLD     = None
    #TRACKER_MAX_TIME_THRESHOLD   = None
    #                               
    #CLASSIFICATION_WINDOW_WIDTH  = None
    #DEFAULT_DATASET_FILE         = None

    def __init__(self, sample_frame, exposure, 
            channel      = 'green', 
            dataset      = None, 
            use_color    = False, 
            use_learning = False, 
            classifier   = None):

        if dataset == None:
            dataset = self.DEFAULT_DATASET_FILE
        if exposure == self.SHADE_EXPOSURE:
            print 'LaserPointerDetector: using thresholds made for office environments (SHADE)'
            self.threshold = (self.SHADE_DARK_IMAGE_THRESHOLD, self.SHADE_BRIGHT_IMAGE_THRESHOLD)
        elif exposure == self.SUN_EXPOSURE:
            print 'LaserPointerDetector: using thresholds made for sunny environments (SUN)'
            self.threshold = (self.SUN_DARK_IMAGE_THRESHOLD, self.SUN_BRIGHT_IMAGE_THRESHOLD)
        else:
            print 'LaserPointerDetector: using thresholds made for sunny environments (SUN)'
            self.threshold = (self.SUN_DARK_IMAGE_THRESHOLD, self.SUN_BRIGHT_IMAGE_THRESHOLD)

        self.intensity_filter    = BrightnessThreshold(sample_frame, thres_low=self.threshold[0], 
                                                    thres_high=self.threshold[1])
        self.motion_filter       = MotionSubtract(sample_frame)
        if use_color:
            self.color_filter    = ColorFilter(cvsize_to_list(cv.GetSize(sample_frame)))
        else:
            self.color_filter    = None

        self.combine             = CombineMasks(sample_frame)
        if use_learning and classifier is None:
            try:
                loaded_dataset       = load_pickle(dataset)
                self.classifier      = PatchClassifier(loaded_dataset, number_of_learners=self.NUMBER_OF_LEARNERS)
            except IOError, e:
                print 'LaserPointerDetector: no data file detected, not using classifier'
                self.classifier      = None
        else:
            self.classifier      = classifier
        #self.copy                = cv.CreateImage(sample_frame, 8, 3)
        self.copy                = cv.CreateImage(cv.GetSize(sample_frame), 8, 3)
        self.tracker             = Tracker(self.TRACKER_MAX_PIX_TRESHOLD, self.TRACKER_MAX_TIME_THRESHOLD)
        self.splitter            = SplitColors(sample_frame)
        self.combined_grey_scale = cv.CreateImage(cv.GetSize(sample_frame), 8, 1)
        self.channel             = channel
        self.copy                = cv.CreateImage(cv.GetSize(sample_frame), 8, 3)
        self.debug               = False

    def set_debug(self, v):
        self.debug                  = v
        self.intensity_filter.debug = v

    def get_motion_intensity_images(self):
        return (self.motion_filter.get_thresholded_image(), self.intensity_filter.get_thresholded_image())

    def detect(self, image, verbose=False):
        cv.Copy(image, self.copy)
        image = self.copy

        start_time = time.time()
        if self.channel   == 'red':
            channel = 0
        elif self.channel == 'green':
            channel = 1
        elif self.channel == 'blue':
            channel = 2
        r, g, b               = self.splitter.split(image)
        coi = [r,g,b][channel]
        split_time = time.time()

        intensity_filtered    = self.intensity_filter.threshold(coi)
        intensity_time = time.time()
        motion_filtered       = self.motion_filter.subtract(coi)
        motion_time = time.time()
        combined              = self.combine.combine([intensity_filtered, motion_filtered])
        #combined              = self.combine.combine([motion_filtered])
        combine_time = time.time()

        #Threshold image image after combining intensity & motion filters' outputs
        cv.Threshold(combined, combined, 0, 1, cv.CV_THRESH_BINARY)
        cv.Mul(coi, combined, self.combined_grey_scale)
        intensity_motion_blob = blob_statistics(self.combined_grey_scale)
        threshold_time = time.time()

        if len(intensity_motion_blob) > 100:
            print 'Too many...', len(intensity_motion_blob)
            return image, combined, None, intensity_motion_blob

        components = intensity_motion_blob
        #print 'LaserPointerDetector.detect: Found ', len(components), 
	#print 'comp after motion & intensity filter.'
        #print 'LaserPointerDetector.detect: classifier', self.classifier

        if self.classifier is not None:
            number_components_before = len(components)
            components = self.classifier.classify(image, components)
            if number_components_before != len(components) and verbose:
                print 'LaserPointerDetector:         PatchClassifier: %d -> %d' % (number_components_before, len(components))
        classify_time = time.time()

        laser_blob = select_laser_blob(components, approx_laser_point_size=self.LASER_POINT_SIZE)
        if laser_blob != None:
            tracks        = self.tracker.track(components_to_detections([laser_blob]))
            laser_track   = select_laser_track(tracks, self.MIN_AGE)
            if laser_blob != None and laser_track == None and verbose:
                print '         Tracker: image motion filter activated.'
            if laser_track is not None:
                #print 'LASER: moved', laser_track.dist_moved, 'age', laser_track.age()
                laser_blob          = laser_track.last_detection.component
                laser_blob['track'] = laser_track
            else:
                laser_blob    = None
            #print 'LaserPointerDetector: Detected a laser point.'
        selection_time = time.time()

        if self.debug:
            print '======================================================='
            print 'split %.4f' % (split_time - start_time)
            print 'intensity %.4f' % (intensity_time - split_time)
            print 'motion %.4f' % (motion_time - intensity_time)
            print 'combine %.4f' % (combine_time - motion_time )
            print 'threshold %.4f' % (threshold_time -combine_time )
            print 'classify_time %.4f' % (classify_time - threshold_time)
            print 'selection_time %.4f' % (selection_time - classify_time)
            print 'total %.4f' % (selection_time - start_time)

        return laser_blob, intensity_motion_blob, image, combined
        #return image, combined, laser_blob, intensity_motion_blob

class PatchClassifier:
    def __init__(self, dataset=None, number_of_learners=30, classification_window_width=7):
        if dataset is not None:
            print 'PatchClassifier.__init__: dataset size', dataset.num_examples()
            self.learn(dataset, number_of_learners=number_of_learners)
            self.classification_window_width = classification_window_width

    def classify(self, image, components):
        def predict_all(c):
            instance = blob_to_input_instance(image, c, 
                    classification_window_width=self.classification_window_width)
            if instance == None:
                classification = np.matrix([False])
                return (classification, None, c)
            classification, votes = self.classifier.predict(self.dataset.reduce(instance))
            return (classification, votes, c)

        def select_valid(c):
            classification, votes, component = c
            if classification[0,0] == 0:
                return False
            else:
                return True

        def hide_votes_in_object(c):
            classification, votes, component = c
            component['votes'] = votes
            return component

        return map(hide_votes_in_object, filter(select_valid, map(predict_all, components)))

    def learn(self, dataset=None, positive_examples = None, negative_examples = None, 
            number_of_learners=30, write_file=LaserPointerDetector.DEFAULT_DATASET_FILE):
        if dataset == None and positive_examples != None and negative_examples != None:
            dataset = classifier_matrix(positive_examples, negative_examples)
            dump_pickle(dataset, write_file)
        print 'PatchClassifier: building classifier...'
        #Reduce dimensionality before using for training
        dataset.reduce_input()
        self.dataset = dataset
        self.classifier = rf.RFBreiman(dataset, number_of_learners=number_of_learners)
        print 'PatchClassifier: done building.'
        return self

def blob_to_rect(blob, classification_window_width=LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH):
    x_center, y_center = blob['centroid']
    x_center           = int(x_center)
    y_center           = int(y_center)
    x_start            = x_center - classification_window_width
    y_start            = y_center - classification_window_width
    patch_size         = classification_window_width*2+1
    r                  = Rect(x_start, y_start, patch_size, patch_size).keep_inside(0, 639, 0,479)
    if r.width < patch_size or r.height < patch_size:
        #print "classified_stream_to_classifier_matrix: not using connected comp", 
        #print blob['centroid'], 'w', r.width, 'h', r.height, 'preferred', patch_size
        return None
    else:
        return r

def blob_to_input_instance(image, blob, classification_window_width):
    patch_size = classification_window_width*2+1
    small_r    = blob_to_rect(blob, classification_window_width=classification_window_width)
    big_r      = blob_to_rect(blob, classification_window_width=classification_window_width*2)
    if big_r == None or small_r == None:
        return None
    #print image.__class__
    #import pdb
    #pdb.set_trace()
    small_patch        = cv.CloneMat(cv.GetSubRect(image, small_r.as_cv_rect()))
    big_patch          = cv.CloneMat(cv.GetSubRect(image, big_r.as_cv_rect()))
    cv.ShowImage('patch', small_patch)
    cv.ShowImage('big_patch', big_patch)
    big_patch_rescaled = cv.CreateImage((int(classification_window_width/2), int(classification_window_width/2)), 8, 3)
    cv.Resize(big_patch, big_patch_rescaled, cv.CV_INTER_LINEAR );

    #np_patch_small   = ut.cv2np(small_patch, 'BGR')
    #np_patch_big     = ut.cv2np(big_patch_rescaled, 'BGR')
    np_patch_small   = ad.cvmat2array(small_patch)
    np_patch_big     = ad.cv2array(big_patch_rescaled)
    np_resized_small = np.matrix(np_patch_small.reshape(patch_size*patch_size*3, 1))
    np_resized_big   = np.matrix(np_patch_big.reshape(np_patch_big.shape[0] * np_patch_big.shape[1] * 3, 1))
    #print 'np_patch_small.shape, np_resized_small.shape, np_patch_big.shape, np_resized_big.shape', np_patch_small.shape, np_resized_small.shape, np_patch_big.shape, np_resized_big.shape
    return np.concatenate((np_resized_small, np_resized_big), axis=0)

def blobs_list_to_classifier_matrix(img_blobs_list, 
        classification_window_width=LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH):
    instance_list = []
    for img, blobs in img_blobs_list:
        for blob in blobs:
            instance = blob_to_input_instance(img, blob, classification_window_width)
            if instance is not None:
                instance_list.append(instance)
    return ut.list_mat_to_mat(instance_list, axis=1)

def classifier_matrix(positive_examples_list, negative_examples_list):
    '''
        positive_examples_list - [(image1, blobs), (image2, blobs)...]
        negative_examples_list - [(image1, blobs), (image2, blobs)...]
    '''
    positive_examples_mat = blobs_list_to_classifier_matrix(positive_examples_list)
    negative_examples_mat = blobs_list_to_classifier_matrix(negative_examples_list)
    return matrices_to_dataset(positive_examples_list, negative_examples_list)

class Track:
    def __init__(self, id, pos, cur_time):
        self.id         = id
        self.pos        = pos
        self.start_time = cur_time
        self.end_time   = cur_time
        self.track      = [pos]
        self.last_detection = None
        self.fresh      = True
        self.dist_moved = 0.0

    def update(self, detection, cur_time):
        self.dist_moved = np.linalg.norm(self.pos - detection.pos)
        self.pos      = detection.pos
        self.end_time = cur_time 
        self.track.append(detection.pos)
        self.last_detection = detection
        self.fresh    = True

    def age(self):
        return self.end_time - self.start_time

    def last_detection_time(self, cur_time):
        return cur_time - self.end_time

def components_to_detections(components):
    ds = []
    for c in components:
        d = Detection(np.matrix(c['centroid']).T)
        d.component = c
        ds.append(d)
    return ds

class Detection:
    def __init__(self, pos):
        self.pos = pos

class Tracker:
    def __init__(self, max_pix_thres, max_time_thres):
        self.tracks = []
        self.ids = 0
        self.cur_time = 0
        self.MAX_PIX_TRESHOLD = max_pix_thres
        self.MAX_TIME_THRESHOLD = max_time_thres

    def track(self, detections):
        #Create n^2 pairs
        pairs      = []
        pairs_dist = []
        for track in self.tracks:
            track.fresh = False
            for d in detections:
                pairs.append((track, d))
                pairs_dist.append(np.linalg.norm(track.pos - d.pos))

        for d in detections:
            d.used = False

        #Sort pairs by distances
        sorted_indices = np.argsort(pairs_dist)

        #Select the top len(detections) pairs
        pairs          = np.array(pairs)
        pairs_dist     = np.array(pairs_dist)
        selected_pairs = pairs[sorted_indices[0:len(detections)]]
        selected_dists = pairs_dist[sorted_indices[0:len(detections)]]

        #Update tracks that has matches within THRESHOLD distance
        for idx in xrange(len(selected_pairs)):
            if selected_dists[idx] < self.MAX_PIX_TRESHOLD:
                track, detection = selected_pairs[idx]
                track.update(detection, self.cur_time)
                detection.used = True

        #Make new tracks with left over detections
        for d in detections:
            if not d.used:
                self.tracks.append(Track(self.ids, d.pos, self.cur_time))
                self.ids = self.ids + 1
        
        #Remove old tracks that have not been matched
        new_tracks = []
        for t in self.tracks:
            if t.last_detection_time(self.cur_time) <= self.MAX_TIME_THRESHOLD:
                new_tracks.append(t)
        self.tracks = new_tracks
        self.cur_time = self.cur_time + 1
        return self.tracks



        







        
        




       

























       

























       
























        #if self.color_filter is not None and self.classifier is not None:
        #    print 'detector: detected', len(intensity_motion_blob),
        #    print 'color',      abs(len(intensity_motion_blob) - len(color_results['components'])),
        #    print 'classified', abs(len(components)            - len(color_results['components']))
        #select_track_time = time.time()

        #print '   split_time         took %.4f s' % (split_time     - start_time)
        #print '   intensity_time     took %.4f s' % (intensity_time - split_time)
        #print '   motion_time        took %.4f s' % (motion_time    - intensity_time)
        #print '   combine_time       took %.4f s' % (combine_time   - motion_time)
        #print '   threshold_time     took %.4f s' % (threshold_time - combine_time)
        #print '   total              took %.4f s' % (time.time()    - start_time)


#def matrices_to_dataset(positive_examples_mat, negative_examples_mat):
#    print 'positive_examples_mat.shape', positive_examples_mat.shape
#    print 'negative_examples_mat.shape', negative_examples_mat.shape
#    outputs_mat = np.matrix(np.zeros((1, positive_examples_mat.shape[1] + negative_examples_mat.shape[1]), dtype='int'))
#    outputs_mat[0, 0:positive_examples_mat.shape[1]] = 1
#    dataset = rf.Dataset(np.concatenate((positive_examples_mat, negative_examples_mat), axis=1), outputs_mat)
#    return dataset

        #import os
        #import sys
        #import re

        #def files_in(folder):
        #    root, dirs, files = os.walk(folder).next()
        #    return root, files

        #def dir_in(folder):
        #    root, dirs, files = os.walk(folder).next()
        #    return root, dirs

        #def select_frames(reg, name):
        #    return None != reg.match(name)

        #def append_pref(pref, file):
        #    return pref + '/' + file

        #pc = PatchClassifier()
        #reg = re.compile('\dframe\d*\.png')
        #proot, pfiles = files_in(folder + '/positive')
        #nroot, nfiles = files_in(folder + '/negative')
        #pfiles = map(ft.partial(append_pref, proot), filter(ft.partial(select_frames, reg), pfiles))
        #nfiles = map(ft.partial(append_pref, nroot), filter(ft.partial(select_frames, reg), nfiles))














































































        #[external]
        #Tx -59.934176 
        #Ty 0.281676 
        #Tz -1.578173 
        #Rx -0.009049 
        #Ry -0.003935 
        #Rz -0.000145 
        
        #  [left camera]
        #  pwidth 640 
        #  pheight 480 
        #  dpx 0.006000 
        #  dpy 0.006000 
        #  sx 1.000000 
        #  Cx 332.544501 
        #  Cy 254.563215 
        #  f 945.598886 
        #  fy 950.594644 
        #  alpha 0.000000 
        #  kappa1 -0.454865 
        #  kappa2 1.196744 
        #  kappa3 -7.878979 
        #  tau1 0.000000 
        #  tau2 0.000000 
        #  proj 
        #    9.510000e+02 0.000000e+00 3.112438e+02 0.000000e+00 
        #    0.000000e+00 9.510000e+02 2.444868e+02 0.000000e+00 
        #    0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 
        #  rect 
        #    9.997387e-01 -4.774891e-03 2.235500e-02 
        #    4.673785e-03 9.999786e-01 4.572824e-03 
        #    -2.237635e-02 -4.467146e-03 9.997396e-01 
        #  
        #  [right camera]
        #  pwidth 640 
        #  pheight 480 
        #  dpx 0.006000 
        #  dpy 0.006000 
        #  sx 1.000000 
        #  Cx 339.749173 
        #  Cy 234.137828 
        #  f 927.703322 
        #  fy 931.820423 
        #  alpha 0.000000 
        #  kappa1 -0.375167 
        #  kappa2 -0.607004 
        #  kappa3 3.778784 
        #  tau1 0.000000 
        #  tau2 0.000000 
        #  proj 
        #    9.510000e+02 0.000000e+00 3.112438e+02 -5.699740e+04 
        #    0.000000e+00 9.510000e+02 2.444868e+02 0.000000e+00 
        #    0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 
        #  rect 
        #    9.996422e-01 -4.699764e-03 2.633178e-02 
        #    4.818754e-03 9.999785e-01 -4.457230e-03 
        #    -2.631026e-02 4.582522e-03 9.996433e-01 










