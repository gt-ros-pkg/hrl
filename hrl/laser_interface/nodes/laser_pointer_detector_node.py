#!/usr/bin/env python
# laser_interface
#
#  Copyright (c) 2008, Willow Garage, Inc.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of the <ORGANIZATION> nor the names of its
#        contributors may be used to endorse or promote products derived from
#        this software without specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
##@mainpage
#
# @b laser_interface implments a laser pointer finder taking in images from a
# stereo pair and outputing a 3D point that can be used a mouse cursor in the
# world.  This works by first filtering the stereo pair based on image
# intensity and image motion.  Large connected components are then thrown out
# leaving smaller components representative of laser points.  Each components
# is then fed to a random forest classifier to find whether the detection is a
# laser point or not.  To gather positive and negative training examples for
# the classifier, use the interface launched by the @b user_interface_node.
# By default the @b laser_pointer_detector_node gathers negative training example
# so just point the camera to an area containing motion for it to grab negative samples.
# To gather positive examples, point the camera at a static scene where the laser
# point is the only moving object then switch to 'positive' mode with the GUI
# spawned by @ user_interface_node. The modes offered by the interface are:
# - 'positive': tells the detector to gather positive examples of laser points.
# - 'rebuild' : rebuild classifier
# - 'clear' : clear out training examples gathered so far
# - 'debug' : prints timing information 
# - 'display': show images being processed
# - 'verbose': print statistics
#
# All parameters for the algorithm is stored in params.xml.  For triangulation
# this code uses the camera calibrations provided by cameras.xml.
#
# @author Hai Nguyen/hai@gatech.edu
#
#<hr>
#
#@section usage Usage
#@verbatim
#$ roslaunch launch.xml
#@endverbatim
#
#<hr>
#
#@section topic ROS topics
#
#Subscribes to (name/type):
#- @b "mouse_click"/String : 'True' or 'False' indicating if a normal desktop mouse has been clicked.
#- @b "laser_mode"/String : sets the mode to either 'debug' 'display' 'verbose' 'rebuild' 'positive' or 'clear'.
#
#Publishes to (name / type):
#- @b "cursor3d"/Point:
#
#<hr>
#
# @todo remove dependency from the opencv version modified for Videre stereo pairs.
#

from pkg import *
#from std_msgs.msg import Position
from geometry_msgs.msg import Point
from std_msgs.msg import String
import sys, time
import cv
#import opencv as cv
#import opencv.highgui as hg

import laser_interface.camera as cam
import laser_interface.random_forest as rf
import laser_interface.dimreduce as dr
import laser_interface.util as ut
from   laser_interface.laser_detector import *
from threading import RLock
#from pyrob.voice import say

def show_processed(image, masks, detection, blobs, detector):
    masker            = Mask(image)
    splitter          = SplitColors(image)
    r, g, b           = splitter.split(image)
    thresholded_image = masker.mask(masks[0], r, g, b)
    draw_detection(thresholded_image, detection)
    cv.ShowImage('thresholded', thresholded_image)

    draw_detection(image, detection)
    draw_blobs(image, blobs)

    make_visible_binary_image(masks[0])
    draw_detection(masks[0], detection)
    make_visible_binary_image(masks[1])
    make_visible_binary_image(masks[2])

    cv.ShowImage("video",       image)
    cv.ShowImage('motion',      masks[1])
    cv.ShowImage('intensity',   masks[2])

def matrix_to_dataset(examples, type=1):
    outputs_mat      = np.matrix(np.zeros((1, examples.shape[1]), dtype='int'))
    outputs_mat[:,:] = type
    return rf.Dataset(examples, outputs_mat)

def confirmation_prompt(confirm_phrase):
    print confirm_phrase
    print 'y(es)/n(no)'
    k = cv.WaitKey()
    if k == 'y':
        return True
    else:
        return False

def append_examples_from_file(dataset, file):
    try:
        loaded_set = load_pickle(file)
        dataset.append(loaded_set)
    except IOError:
        print 'append_examples_from_file: training file \'', file, '\'not found!'
    return dataset.num_examples()

def print_friendly(votes):
    new_dict = {}
    total = 0
    for k in votes.keys():
        new_key = k[0,0]
        new_dict[new_key] = votes[k]
    return new_dict

class EmbodiedLaserDetector:
    def __init__(self, geometric_camera, hardware_camera, gather_misclassified_only = True):
        self.stereo_cam                 = geometric_camera
        self.gather_misclassified_only  = gather_misclassified_only

        self.examples                   = []
        self.labels                     = []
        self.expected_class             = 0
        self.expecting_correct_labels   = True
        self.build_detectors(hardware_camera)

    def clear_examples(self):
        self.examples                   = []
        self.labels                     = []

    def build_detectors(self, hardware_camera):
        self.write()
        frames                         = hardware_camera.next()
        self.left_detector             = LaserPointerDetector(frames[0], exposure=exposure, 
                                                    dataset=LaserPointerDetector.DEFAULT_DATASET_FILE,
                                                    use_color=False, use_learning=self.gather_misclassified_only)
        self.right_detector            = LaserPointerDetector(frames[1], exposure=exposure, 
                                                    dataset=LaserPointerDetector.DEFAULT_DATASET_FILE,
                                                    classifier=self.left_detector.classifier,
                                                    use_color=False, use_learning=self.gather_misclassified_only)
        for i in xrange(10):
            frames = hardware_camera.next()
            self.left_detector.detect(frames[0])
            self.right_detector.detect(frames[1])

    def run(self, images, display=True, verbose=False, debug=False):
        left_detection, left_intensity_motion_activations, left_image, left_combined_masks = self.left_detector.detect(images[0])
        self.record(left_detection, left_image, left_intensity_motion_activations)

        right_detection, right_intensity_motion_activations, right_image, right_combined_masks = self.right_detector.detect(images[1])
        self.record(right_detection, right_image, right_intensity_motion_activations)

        if debug:
            motion, intensity = self.left_detector.get_motion_intensity_images()
            show_processed(left_image, [left_combined_masks, motion, intensity], 
                    left_detection, left_intensity_motion_activations, self.left_detector)
        elif display:
            draw_detection(left_image, left_detection)
            cv.ShowImage('video', left_image)

        if left_detection != None and right_detection != None:
            if (self.expecting_correct_labels and self.expected_class == 0):
                print 'EmbodiedLaserDetector: output suppressed, classified positive in both cameras.'
            else:
                return self.triangulate(left_detection, right_detection)
        return None

    def set_debug(self, v):
        self.debug           = v
        self.left_detector.set_debug(v)
        self.right_detector.set_debug(v)

    def triangulate(self, left_cam_detection, right_cam_detection):
        print 'EmbodiedLaserDetector.triangulate: votes', print_friendly(right_cam_detection['votes']), print_friendly(left_cam_detection['votes'])
        x  = np.matrix(left_cam_detection['centroid']).T
        xp = np.matrix(right_cam_detection['centroid']).T
        result = self.stereo_cam.triangulate_3d(x, xp)
        print '3D point located at', result['point'].T, 
        print 'distance %.2f error %.3f' % (np.linalg.norm(result['point']),  result['error'])
        if result['point'][2,0] < 0:
            #Don't return anything if point is behind camera
            print 'EmbodiedLaserDetector.triangulate: point was behind camera, ignoring'
            return None

        if result['point'][2,0] > 5:
            print 'EmbodiedLaserDetector.triangulate: was too far, ignoring'
            return None

    def record(self, picked_blob, image, other_candidates):
        if self.expecting_correct_labels:
            if self.expected_class == 1:
                #We got it wrong
                if picked_blob == None:
                    for blob in other_candidates:
                        instance = blob_to_input_instance(image, blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
                        if instance != None:
                            self.examples.append(instance)
                            self.labels.append(np.matrix([self.expected_class]))
                    if len(other_candidates) > 0:
                        print 'EmbodiedLaserDetector.record: expected 1 got 0', len(self.examples), 'instances'
                #We got it right
                elif picked_blob != None and not self.gather_misclassified_only: 
                    instance = blob_to_input_instance(image, picked_blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
                    if instance != None:
                        self.examples.append(instance)
                        self.labels.append(np.matrix([self.expected_class]))
                    print 'EmbodiedLaserDetector.record: expected 1 got 1, ', len(self.examples), 'instances'
            
            if self.expected_class == 0:
                #We got it wrong
                if picked_blob != None:
                    instance = blob_to_input_instance(image, picked_blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
                    if instance != None:
                        self.examples.append(instance)
                        self.labels.append(np.matrix([self.expected_class]))
                    print 'EmbodiedLaserDetector.record: expected 0 got 1,', len(self.examples), 'instances'
                #We got it right
                elif picked_blob == None and not self.gather_misclassified_only:
                    for blob in other_candidates:
                        instance = blob_to_input_instance(image, blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
                        if instance != None:
                            self.examples.append(instance)
                            self.labels.append(np.matrix([self.expected_class]))
                    print 'EmbodiedLaserDetector.record: expected 0 got 0,', len(self.examples), 'instances'

    def write(self):
        if not (len(self.examples) > 0):
            print 'EmbodiedLaserDetector.write: no examples to record'
            return
        #dataset        = matrix_to_dataset(ut.list_mat_to_mat(self.examples, axis=1), type=self.type)
        inputs  = ut.list_mat_to_mat(self.examples, axis = 1)
        outputs = ut.list_mat_to_mat(self.labels, axis = 1)
        print 'EmbodiedLaserDetector.write: inputs.shape, outputs.shape', inputs.shape, outputs.shape
        dim_reduce_set = rf.LinearDimReduceDataset(inputs, outputs)
        print 'EmbodiedLaserDetector.write: appending examples from disk to dataset'
        n = append_examples_from_file(dim_reduce_set, file=LaserPointerDetector.DEFAULT_DATASET_FILE)
        print 'EmbodiedLaserDetector.write: calculating pca projection vectors'
        dim_reduce_set.set_projection_vectors(dr.pca_vectors(dim_reduce_set.inputs, percent_variance=LaserPointerDetector.PCA_VARIANCE_RETAIN))
        print 'EmbodiedLaserDetector.write: writing...'
        dump_pickle(dim_reduce_set, LaserPointerDetector.DEFAULT_DATASET_FILE)
        print 'EmbodiedLaserDetector: recorded examples to disk.  Total in dataset', n
        self.examples = []
        self.labels   = []

class LaserPointerDetectorNode:
    def __init__(self, topics, exposure = LaserPointerDetector.SUN_EXPOSURE, video = None, display=False):
        if video is None:
            self.video  = cam.ROSStereoListener(topics)
            #self.video    = cam.VidereStereo(0, gain=96, exposure=exposure)
            #self.video    = cam.StereoFile('measuring_tape_red_left.avi','measuring_tape_red_right.avi')
        else:
            self.video = video

        self.video_lock       = RLock()
        self.camera_model     = cam.ROSStereoCamera('videre_stereo')
        self.detector         = EmbodiedLaserDetector(self.camera_model, self.video)
        self.exposure         = exposure
        self.display          = display
        self.verbose = False
        self.debug   = False
        if display:
            self._make_windows()

        #Subscribe
        rospy.init_node('laser_pointer_detector')
        rospy.Subscriber(MOUSE_CLICK_TOPIC, String, self._click_handler)
        rospy.Subscriber(LASER_MODE_TOPIC, String, self._mode_handler)
        #rospy.TopicSub(MOUSE_CLICK_TOPIC, String, self._click_handler)
        #rospy.TopicSub(LASER_MODE_TOPIC, String, self._mode_handler)

        #Publish
        #self.topic = rospy.TopicPub(CURSOR_TOPIC, Position)
        self.topic = rospy.Publisher(CURSOR_TOPIC, Point)

        #Ready
        #rospy.ready(sys.argv[0])

    def _click_handler(self, evt):
        message = evt.data
        if self.detector is not None:
            if message == 'True':
                print 'LaserPointerDetector.click_handler: click!'
                #user clicked, want to suppress gathering of examples
                self.detector.expecting_correct_labels   = False
            elif message == 'False':
                print 'LaserPointerDetector.click_handler: Released click!'
                #user released, want to gather negative examples
                self.detector.expected_class = 0
                self.detector.expecting_correct_labels   = True
            else:
                raise RuntimeError('unexpected click message from topic' + MOUSE_CLICK_TOPIC)

    def _mode_handler(self, evt):
        message = evt.data
        if(message == 'debug'):
            self.debug = not self.debug
            print 'LaserPointerDetector.mode_handler: debug', self.debug
        elif (message == 'display'):
            self.display = not self.display
            print 'LaserPointerDetector.mode_handler: display', self.display
        elif(message == 'verbose'):
            self.verbose = not self.verbose
            print 'LaserPointerDetector.mode_handler: verbose', self.verbose
        elif(message == 'rebuild'):
            self.video_lock.acquire()
            if self.detector is not None:
                self.detector.build_detectors(self.video)
            self.video_lock.release()
        elif(message == 'positive'):
            if self.detector is not None:
                self.detector.expected_class = 1
                self.detector.expecting_correct_labels   = True
        elif(message == 'clear'):
            self.detector.clear_examples()
        else:
            raise RuntimeError('unexpected mode message from topic' + LASER_MODE_TOPIC)
        
    def _make_windows(self):
        windows = ['video', 'right', 'thresholded', 'motion', 'intensity', 'patch', 'big_patch']
        for n in windows:
            cv.NamedWindow(n, 1)
        cv.MoveWindow("video",       0,   0)
        cv.MoveWindow("right",       800, 0)
        cv.MoveWindow("thresholded", 800, 0)
        cv.MoveWindow("intensity",   0,   600)
        cv.MoveWindow("motion",      800, 600)

    def set_debug(self, v):
        self.detector.set_debug(v)
        self.debug = v

    def run(self):
        try:
            #while not rospy.isShutdown():
            while not rospy.is_shutdown():
                self.video_lock.acquire()
                start_time     = time.time()
                frames         = list(self.video.next())
                frames[0]      = self.camera_model.camera_left.undistort_img(frames[0])
                frames[1]      = self.camera_model.camera_right.undistort_img(frames[1])
                undistort_time = time.time()
                result         = self.detector.run(frames, display=self.display, verbose=self.verbose, debug=self.debug)
                run_time       = time.time()
                self.video_lock.release()

                if result != None:
                    p = result['point']
                    self.topic.publish(Point(p[0,0], p[1,0], p[2,0]))

                if self.debug:
                    print '>> undistort %.2f' % (undistort_time - start_time)
                    print '>> run %.4f' % (run_time - undistort_time)
                    diff = time.time() - start_time
                    print 'Main: Running at %.2f fps, took %.4f s' % (1.0 / diff, diff)

                k = cv.WaitKey(10)
                if   k == 'd':
                    self.display = not self.display
                elif k == 'v':
                    self.verbose = not self.verbose
                elif k == 'g':
                    self.set_debug(not self.debug)
                elif k == 'q':
                    return

        except StopIteration, e:
            if self.state_object.__class__ == GatherExamples:
                self.state_object.write()


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('-r', '--run',  action='store_true', 
                dest='mode_run', help='classify')
    p.add_option('-o', '--office', action='store', dest='office', 
                 help='true for operating robot in office environments')
    p.add_option('-d', '--display',  action='store_true', 
                dest='display', help='show display')
    p.add_option('-t', '--time', action = 'store_true', 
                dest='time', help='display timing information')
    opt, args = p.parse_args()

    if opt.display == None:
        display = False
    else:
        display = opt.display

    if opt.office == True:
        exposure = LaserPointerDetector.SHADE_EXPOSURE
    else:
        exposure = LaserPointerDetector.SUN_EXPOSURE

    if display == False:
        cv.NamedWindow('key', 1)

    print 'Display set to', display
    print 'Exposure set to', exposure
    topics = ["/wide_stereo/left/image_color", "/wide_stereo/right/image_color"]
    lpdn = LaserPointerDetectorNode(topics, exposure = exposure, display=display)
    if opt.time != None:
        lpdn.set_debug(True)
    lpdn.run()

























































































































                #if   k == 'p':
                #elif k == 'n':
                #elif k == ' ':

#class LaserPointerDetectorNode:
#    GATHER_MISCLASSIFIED_ONLY = bool(rospy.getMaster()['laser_pointer_detector_node/GATHER_MISCLASSIFIED_ONLY'])
#    GATHER_POSITIVE_EXAMPLES  = 'GATHER_POSITIVE_EXAMPLES'
#    GATHER_NEGATIVE_EXAMPLES  = 'GATHER_NEGATIVE_EXAMPLES'
#    DETECT                    = 'DETECT'
#
#    def __init__(self, mode, exposure = LaserPointerDetector.SUN_EXPOSURE, video = None, display=False):
#        if video is None:
#            self.video    = cam.VidereStereo(0, gain=96, exposure=exposure)
#            #self.video    = cam.StereoFile('measuring_tape_red_left.avi','measuring_tape_red_right.avi')
#        else:
#            self.video = video
#        self.exposure         = exposure
#        self.video_lock       = RLock()
#        self.camera_model     = cam.ROSStereoCamera('videre_stereo')
#        self.state            = None
#        self.state_object     = None
#        if display:
#            self._make_windows()
#        self.display = display
#        self.verbose = False
#        self.debug   = False
#
#        #Subscribe
#        rospy.TopicSub(MOUSE_CLICK_TOPIC, String, self._click_handler)
#        rospy.TopicSub(LASER_MODE_TOPIC, String, self._mode_handler)
#
#        #Publish
#        self.topic = rospy.TopicPub(CURSOR_TOPIC, Position)
#
#        #Ready
#        rospy.ready(sys.argv[0])
#
#        print '======================================================'
#        print '= Usage'
#        print '======================================================'
#        print """
#                if k == 'p':
#                    self.set_state(self.GATHER_POSITIVE_EXAMPLES)
#                if k == 'n':
#                    self.set_state(self.GATHER_NEGATIVE_EXAMPLES)
#                if k == ' ':
#                    self.set_state(self.DETECT)
#                if k == 'd':
#                    self.display = not self.display
#                if k == 'v':
#                    self.verbose = not self.verbose
#                if k == 'g':
#                    self.debug   = not self.debug"""
#        self.set_state(mode)
#
#    def run(self):
#        try:
#            while not rospy.isShutdown():
#                self.video_lock.acquire()
#                start_time = time.time()
#                frames = list(self.video.next())
#                frames[0] = self.camera_model.camera_left.undistort_img(frames[0])
#                frames[1] = self.camera_model.camera_right.undistort_img(frames[1])
#                undistort_time = time.time()
#                result = self.state_object.run(frames, display=self.display, verbose=self.verbose, debug=self.debug)
#                run_time = time.time()
#                self.video_lock.release()
#                if result != None:
#                    p = result['point']
#                    self.topic.publish(Position(p[0,0], p[1,0], p[2,0]))
#
#                if self.debug:
#                    print '>> undistort %.2f' % (undistort_time - start_time)
#                    print '>> run %.4f' % (run_time - undistort_time)
#                    diff = time.time() - start_time
#                    print 'Main: Running at %.2f fps, took %.4f s' % (1.0 / diff, diff)
#                k = hg.cvWaitKey(10)
#                if k == 'p':
#                    self.set_state(self.GATHER_POSITIVE_EXAMPLES)
#                elif k == 'n':
#                    self.set_state(self.GATHER_NEGATIVE_EXAMPLES)
#                elif k == ' ':
#                    self.set_state(self.DETECT)
#                elif k == 'd':
#                    self.display = not self.display
#                elif k == 'v':
#                    self.verbose = not self.verbose
#                elif k == 'g':
#                    self.set_debug(not self.debug)
#                elif k == 'q':
#                    return
#
#        except StopIteration, e:
#            if self.state_object.__class__ == GatherExamples:
#                self.state_object.write()
#
#    def set_state(self, new_state):
#        self.video_lock.acquire()
#        print '======================================================'
#        print '=  Changing Mode to', new_state
#        print '======================================================'
#        if self.state_object.__class__ == GatherExamples:
#            self.state_object.write()
#
#        if new_state == self.GATHER_POSITIVE_EXAMPLES:
#            say('gathering positive examples')
#            self.state_object = GatherExamples(self.video, 
#                    gather_misclassified_only = self.GATHER_MISCLASSIFIED_ONLY, type = 1)
#
#        if new_state == self.GATHER_NEGATIVE_EXAMPLES:
#            say('gathering negative examples')
#            self.state_object = GatherExamples(self.video, 
#                    gather_misclassified_only = self.GATHER_MISCLASSIFIED_ONLY, type = 0)
#
#        if new_state == self.DETECT:
#            say('detecting')
#            self.state_object = DetectState(self.camera_model, self.video, self.exposure)
#
#        self.state_object.set_debug(self.debug)
#        self.state              = new_state
#        self.video_lock.release()













#class DetectState:
#    def __init__(self, geometric_camera, hardware_camera, exposure_setting):
#        self.stereo_cam     = geometric_camera
#        frames              = hardware_camera.next()
#        self.detector       = LaserPointerDetector(frames[0], exposure_setting, 
#                                                   use_color=False, use_learning=True)
#        self.detector_right = LaserPointerDetector(frames[1], exposure_setting, 
#                                                   use_color=False, use_learning=True, 
#                                                   classifier=self.detector.classifier)
#
#        for i in xrange(10):
#            frames = hardware_camera.next()
#            self.detector.detect(frames[0])
#            self.detector_right.detect(frames[1])
#
#    def set_debug(self, v):
#        self.debug = v
#        self.detector.set_debug(v)
#        self.detector_right.set_debug(v)
#
#    def run(self, images, display=True, verbose=False, debug=False):
#        l, r = images
#
#        #Detect right image
#        start_time = time.time()
#        _, _, right_cam_detection, stats = self.detector_right.detect(r, verbose=verbose)
#        right_time = time.time()
#        if debug:
#            draw_blobs(r, stats)
#            draw_detection(r, right_cam_detection)
#            hg.cvShowImage('right', r)
#
#        #Detect left image
#        image, combined, left_cam_detection, stats = self.detector.detect(l, verbose=verbose)
#        left_time = time.time()
#        if debug: 
#            motion, intensity = self.detector.get_motion_intensity_images()
#            show_processed(l, [combined, motion, intensity], left_cam_detection, stats, self.detector)
#        elif display:
#            draw_detection(l, left_cam_detection)
#            hg.cvShowImage('video', l)
#
#        if self.debug:
#            print '#   right_time %.4f' % (right_time - start_time)
#            print '#   left_time %.4f' % (left_time - right_time)
#
#        #Triangulate
#        if right_cam_detection != None and left_cam_detection != None:
#            print right_cam_detection.keys()
#            print 'DetectState: votes', print_friendly(right_cam_detection['votes']), print_friendly(left_cam_detection['votes'])
#            x  = np.matrix(left_cam_detection['centroid']).T
#            xp = np.matrix(right_cam_detection['centroid']).T
#            result = self.stereo_cam.triangulate_3d(x, xp)
#            print '3D point located at', result['point'].T, 
#            print 'distance %.2f error %.3f' % (np.linalg.norm(result['point']),  result['error'])
#            #if result['point'][0,0] < 0:
#            #    #Don't return anything if point is behind camera
#            #    print 'DetectState: point was behind camera, ignoring'
#            #    return None
#            if result['point'][2,0] < 0:
#                #Don't return anything if point is behind camera
#                print 'DetectState: point was behind camera, ignoring'
#                return None
#
#            if result['point'][2,0] > 5:
#                print 'DetectState: was too far, ignoring'
#                return None
#
#            return result
#        else:
#            return None




#class GatherExamples:
#    def __init__(self, hardware_camera, gather_misclassified_only=True, type = 1):
#        print 'GatherExamples: gather_misclassified_only', gather_misclassified_only
#        self.gather_misclassified_only = gather_misclassified_only
#        frames         = hardware_camera.next()
#        self.detector  = LaserPointerDetector(frames[0], exposure=exposure, 
#                                         dataset=LaserPointerDetector.DEFAULT_DATASET_FILE,
#                                         use_color=False, use_learning=gather_misclassified_only)
#        self.detector2 = LaserPointerDetector(frames[1], exposure=exposure, 
#                                         dataset=LaserPointerDetector.DEFAULT_DATASET_FILE,
#                                         classifier=self.detector.classifier,
#                                         use_color=False, use_learning=gather_misclassified_only)
#        for i in xrange(10):
#            frames = hardware_camera.next()
#            self.detector.detect(frames[0])
#            self.detector2.detect(frames[1])
#        self.examples = []
#        self.type     = type
#
#    def set_debug(self, v):
#        self.debug           = v
#        self.detector.set_debug(v)
#        self.detector2.set_debug(v)
#
#    def run(self, images, display=True, verbose=False, debug=False):
#        image                 = None
#        combined              = None
#        motion                = None
#        intensity             = None
#        laser_blob            = None
#        intensity_motion_blob = None
#
#        for raw_image, detect in zip(images, [self.detector, self.detector2]):
#            #before = time.time()
#            image, combined, laser_blob, intensity_motion_blob = detect.detect(raw_image)
#            #diff = time.time() - before
#            #print 'took %.2f seconds to run or %.2f fps' % (diff, 1.0/diff)
#            if self.type == 1:
#                #We got it wrong
#                if laser_blob == None:
#                    for blob in intensity_motion_blob:
#                        instance = blob_to_input_instance(image, blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
#                        if instance != None:
#                            self.examples.append(instance)
#                    if len(intensity_motion_blob) > 0:
#                        print 'expected 1 got 0', len(self.examples), 'instances'
#                #We got it right
#                elif laser_blob != None and not self.gather_misclassified_only: 
#                    instance = blob_to_input_instance(image, laser_blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
#                    if instance != None:
#                        self.examples.append(instance)
#                    print 'expected 1 got 1, ', len(self.examples), 'instances'
#
#            if self.type == 0:
#                #We got it wrong
#                if laser_blob != None:
#                    instance = blob_to_input_instance(image, laser_blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
#                    if instance != None:
#                        self.examples.append(instance)
#                    print 'expected 0 got 1,', len(self.examples), 'instances'
#                #We got it right
#                elif laser_blob == None and not self.gather_misclassified_only:
#                    for blob in intensity_motion_blob:
#                        instance = blob_to_input_instance(image, blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
#                        if instance != None:
#                            self.examples.append(instance)
#                    print 'expected 0 got 0,', len(self.examples), 'instances'
#
#            #if laser_blob != None:
#            #    instance = blob_to_input_instance(image, laser_blob, LaserPointerDetector.CLASSIFICATION_WINDOW_WIDTH)
#            #    if instance is not None:
#            #        self.examples.append(instance)
#            #        print 'got', len(self.examples), 'instances'
#            motion, intensity = detect.get_motion_intensity_images()
#
#        if display:
#            show_processed(image, [combined, motion, intensity], laser_blob, intensity_motion_blob, self.detector2)
#
#    def write(self):
#        if not (len(self.examples) > 0):
#            print 'GatherExamples: no examples to record'
#            return
#        #print 'len(self.examples)    ', len(self.examples)
#        #print 'self.examples[0].shape', self.examples[0].shape
#        #for i in self.examples:
#        #    print i.shape
#        dataset        = matrix_to_dataset(ut.list_mat_to_mat(self.examples, axis=1), type=self.type)
#        dim_reduce_set = rf.LinearDimReduceDataset(dataset.inputs, dataset.outputs)
#        print 'GatherExamples.write: appending examples from disk to dataset'
#        n = append_examples_from_file(dim_reduce_set, file=LaserPointerDetector.DEFAULT_DATASET_FILE)
#        print 'GatherExamples.write: calculating pca projection vectors'
#        dim_reduce_set.set_projection_vectors(dr.pca_vectors(dim_reduce_set.inputs, percent_variance=LaserPointerDetector.PCA_VARIANCE_RETAIN))
#        print 'GatherExamples.write: writing...'
#        dump_pickle(dim_reduce_set, LaserPointerDetector.DEFAULT_DATASET_FILE)
#        print 'GatherExamples: recorded examples to disk.  Total in dataset', n



















#def append_examples_to_file(dataset, file = LaserPointerDetector.DEFAULT_DATASET_FILE):
#    try:
#        loaded_set = load_pickle(file)
#        dataset.append(loaded_set)
#    except IOError:
#        print 'append_examples_to_file: training file \'', file, '\'not found! creating new file'
#        pass
#    dump_pickle(dataset, file)
#    return dataset.num_examples()

    #key = hg.cvWaitKey(10)
    #if detector != None:
    #    if key == 'T': #down
    #        detector.intensity_filter.thres_high = detector.intensity_filter.thres_high - 5
    #        print 'detector.intensity_filter.thres =', detector.intensity_filter.thres_high
    #    if key == 'R':
    #        detector.intensity_filter.thres_high = detector.intensity_filter.thres_high + 5
    #        print 'detector.intensity_filter.thres =', detector.intensity_filter.thres_high
    #    if key == ' ':
    #        hg.cvWaitKey()

#class GatherNegativeExamplesState:
#    def __init__(self, frames):
#        self.detector  = LaserPointerDetector(frames[0], exposure=exposure, 
#                                        dataset=PatchClassifier.DEFAULT_DATASET_FILE,
#                                        use_color=False, use_learning=False)
#        self.detector2 = LaserPointerDetector(frames[1], exposure=exposure, 
#                                        dataset=PatchClassifier.DEFAULT_DATASET_FILE,
#                                        use_color=False, use_learning=False)
#
#        for i in xrange(10):
#            frames = hardware_camera.next()
#            self.detector.detect(frames[0])
#            self.detector2.detect(frames[1])
#
#        self.negative_examples_for_classifier = []
#
#    def gather(self, frames, display=True):
#        image                 = None
#        combined              = None
#        motion                = None
#        intensity             = None
#        laser_blob            = None
#        intensity_motion_blob = None
#        for raw_image, detect in zip(frames, [self.detector, self.detector2]):
#            image, combined, laser_blob, intensity_motion_blob = detect.detect(raw_image)
#            if laser_blob != None:
#                instance = blob_to_input_instance(image, laser_blob)
#                if instance is not None:
#                    negative_examples_for_classifier.append(instance)
#                    print 'got', len(negative_examples_for_classifier), 'instances'
#            motion, intensity = detect.get_motion_intensity_images()
#
#        if display:
#            show_processed(image, [combined, motion, intensity], laser_blob, intensity_motion_blob, self.detector2)
#
#    def write(self):
#        append_examples_to_file(
#                matrix_to_dataset(
#                    ut.list_mat_to_mat(
#                        self.negative_examples_for_classifier, axis=1)))

        #l = stereo_cam.camera_left.undistort_img(l)
        #r = stereo_cam.camera_right.undistort_img(r)
        #cv.cvCopy(l, lt)
        #cv.cvCopy(r, rt)
        #l = lt
        #r = rt
        #undistort_time = time.time()
        #triangulation_time = time.time()
        #detect_time = time.time()
        #diff = time.time() - start_time
        #print 'Main: Running at %.2f fps, took %.4f s' % (1.0 / diff, diff)
        #print '   undistort     took %.4f s' % (undistort_time - start_time)
        #print '   detection     took %.4f s' % (detect_time - undistort_time)
        #print '   triangulation took %.4f s' % (triangulation_time - detect_time)



#def learn_run(exposure = LaserPointerDetector.SUN_EXPOSURE, num_examples_to_collect=200, display_during_run = True):
#    hg.cvNamedWindow("video",       1)
#    hg.cvNamedWindow("thresholded", 1)
#    hg.cvNamedWindow('motion',      1)
#    hg.cvNamedWindow('intensity',   1)
#
#    hg.cvMoveWindow("video",       0,   0)
#    hg.cvMoveWindow("thresholded", 800, 0)
#
#    hg.cvMoveWindow("intensity",   0,   600)
#    hg.cvMoveWindow("motion",      800, 600)
#    video     = cam.VidereStereo(0, gain=96, exposure=exposure)

#if display:
#    hg.cvNamedWindow("video", 1)
#    hg.cvMoveWindow("video",   0,   0)
#if debug:
#    hg.cvNamedWindow('right',       1)
#    hg.cvMoveWindow("right", 800,   0)
#    hg.cvNamedWindow("thresholded", 1)
#    hg.cvNamedWindow('motion',      1)
#    hg.cvNamedWindow('intensity',   1)
#    hg.cvMoveWindow("thresholded", 800, 0)
#    hg.cvMoveWindow("intensity",   0,   600)
#    hg.cvMoveWindow("motion",      800, 600)
#self.video = video
#cam.KNOWN_CAMERAS['videre_stereo2']































##Subscribe
#def callback(msg):
#    print "%s: %s"%(msg.name, msg.msg)
#rospy.TopicSub("chat", Msg, callback)
#

#
##Run
#while not rospy.isShutdown():
#    utter = sys.stdin.readline().strip()
#    if utter:
#        pub.publish(Msg(name, utter))

