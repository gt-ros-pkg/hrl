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
import cv
import numpy as np

import itertools as it
import time
import functools as ft

import util as ut
import random_forest as rf
import laser_interface.cv_actions as cvact
import laser_interface.blob as blob

class LaserPointerDetector:
    #These are currently set using parameters on the parameter server
    def __init__(self, sample_frame, dataset_file, classifier = None):
        self.EXPOSURE = rospy.get_param('~exposure') #TODO: enforce this using dynamic reconfigure
        #self.INTENSITY_THRESHOLD_LOW = rospy.get_param('~intensity_threshold_low')
        #self.INTENSITY_THRESHOLD_HIGH = rospy.get_param('~intensity_threshold_high')

        self.MAX_BLOB_AREA = rospy.get_param('~max_blob_area') 
        self.LASER_POINT_SIZE = rospy.get_param('~laser_point_size') 
        self.MIN_AGE = rospy.get_param('~min_age') 
        self.NUMBER_OF_LEARNERS = rospy.get_param('~number_of_learners') 
        self.PCA_VARIANCE_RETAIN = rospy.get_param('~pca_variance_retain') 

        self.TRACKER_MAX_PIX_TRESHOLD = rospy.get_param('~tracker_max_pix_treshold') 
        self.TRACKER_MAX_TIME_THRESHOLD = rospy.get_param('~tracker_max_time_threshold') 

        self.CLASSIFICATION_WINDOW_WIDTH  = rospy.get_param('~classification_window_width') 
        #self.DATA_SET_FILE = rospy.get_param('~dataset_file') 
        self.COLOR_CHANNEL = rospy.get_param('~color_channel')

        #self.threshold = (self.INTENSITY_THRESHOLD_LOW, self.INTENSITY_THRESHOLD_HIGH)
        #self.threshold = (rospy.get_param('~intensity_threshold_low'), rospy.get_param('~intensity_threshold_high'))

        if classifier is None:
            try:
                #TODO, assert that dataset dimensionality is equal to classifier dimensionality
                loaded_dataset = ut.load_pickle(dataset_file)
                self.classifier = PatchClassifier(loaded_dataset, 
                                    self.NUMBER_OF_LEARNERS, 
                                    self.CLASSIFICATION_WINDOW_WIDTH)
            except IOError, e:
                rospy.logerr('LaserPointerDetector: no data file detected (not using classifier) at loc' + dataset_file)
                self.classifier = None
        else:
            self.classifier = classifier

        #Create opencv processing classes
        self.intensity_filter = cvact.BrightnessThreshold(sample_frame, self.MAX_BLOB_AREA)
        self.motion_filter = cvact.MotionSubtract(sample_frame, self.MAX_BLOB_AREA)
        self.combine = cvact.CombineMasks(sample_frame)
        self.tracker = Tracker(self.TRACKER_MAX_PIX_TRESHOLD, self.TRACKER_MAX_TIME_THRESHOLD) 
        self.splitter = cvact.SplitColors(sample_frame)

        #Caches
        self.copy = cv.CreateImage(cv.GetSize(sample_frame), 8, 3) #Used for copying input image
        self.combined_grey_scale = cv.CreateImage(cv.GetSize(sample_frame), 8, 1)
        self.channel = self.COLOR_CHANNEL

    def get_motion_intensity_images(self):
        return (self.motion_filter.get_thresholded_image(), self.intensity_filter.get_thresholded_image())

    def detect(self, image):
        cv.Copy(image, self.copy)
        image = self.copy

        if self.COLOR_CHANNEL   == 'red':
            channel = 2
        elif self.COLOR_CHANNEL == 'green':
            channel = 1
        elif self.COLOR_CHANNEL == 'blue':
            channel = 0
        r, g, b               = self.splitter.split(image)
        coi = [r,g,b][channel]
        thres_low = rospy.get_param('~intensity_threshold_low')
        thres_high = rospy.get_param('~intensity_threshold_high')
        intensity_filtered    = self.intensity_filter.threshold(thres_low, thres_high, coi)
        motion_filtered       = self.motion_filter.subtract(coi)
        combined              = self.combine.combine([intensity_filtered, motion_filtered])

        #Threshold image image after combining intensity & motion filters' outputs
        cv.Threshold(combined, combined, 0, 1, cv.CV_THRESH_BINARY)
        cv.Mul(coi, combined, self.combined_grey_scale)
        intensity_motion_blob = blob.blob_statistics(self.combined_grey_scale)

        if len(intensity_motion_blob) > 100:
            rospy.logwarn('Too many... ' + str(len(intensity_motion_blob)))
            return image, combined, None, intensity_motion_blob

        components = intensity_motion_blob #components after motion & intensity filtering
        if self.classifier is not None:
            number_components_before = len(components)
            components = self.classifier.classify(image, components)
            if number_components_before != len(components):
                rospy.logdebug( 'LaserPointerDetector:         PatchClassifier: %d -> %d' % (number_components_before, len(components)))

        laser_blob = select_laser_blob(components, approx_laser_point_size=self.LASER_POINT_SIZE)
        if laser_blob != None:
            tracks        = self.tracker.track(components_to_detections([laser_blob]))
            laser_track   = select_laser_track(tracks, self.MIN_AGE)
            if laser_blob != None and laser_track == None:
                rospy.logdebug('         Tracker: image motion filter activated.')
            if laser_track is not None:
                laser_blob          = laser_track.last_detection.component
                laser_blob['track'] = laser_track
            else:
                laser_blob    = None

        return laser_blob, intensity_motion_blob, image, combined

class PatchClassifier:
    def __init__(self, dataset, number_of_learners, classification_window_width):
        rospy.logdebug('PatchClassifier.__init__: dataset size ' + str(dataset.num_examples()))
        self.classification_window_width = classification_window_width
        rospy.loginfo('PatchClassifier: building classifier...')
        #Reduce dimensionality before using for training
        dataset.reduce_input()
        self.dataset = dataset
        self.classifier = rf.RFBreiman(dataset, number_of_learners=number_of_learners)
        rospy.loginfo('PatchClassifier: done building.')

    def classify(self, image, components):
        def predict_all(c):
            instance = blob.blob_to_input_instance(image, c, 
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

def components_to_detections(components):
    ds = []
    for c in components:
        d = Detection(np.matrix(c['centroid']).T)
        d.component = c
        ds.append(d)
    return ds

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
        point = (int(round(laser_blob['centroid'][0])), int(round(laser_blob['centroid'][1])))
        cv.Circle(frame, point, 10, cv.Scalar(0,0,255), 1)
        cv.Circle(frame, point, 1, color, cv.CV_FILLED)
