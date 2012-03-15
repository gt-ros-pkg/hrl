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
#- @b "cursor3d"/PointStamped:
#
#<hr>
#
# @todo remove dependency from the opencv version modified for Videre stereo pairs.
#

from pkg import *
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import sys, time
import cv

import laser_interface.camera as cam
import laser_interface.random_forest as rf
import laser_interface.dimreduce as dr
import laser_interface.util as ut
from   laser_interface.laser_detector import *
import laser_interface.blob as blob
import laser_interface.cv_actions as cva
from threading import RLock

def show_processed(image, masks, detection, blobs, detector):
    masker            = cva.Mask(image)
    splitter          = cva.SplitColors(image)
    r, g, b           = splitter.split(image)
    thresholded_image = masker.mask(masks[0], r, g, b)
    draw_detection(thresholded_image, detection)
    cv.ShowImage('thresholded', thresholded_image)

    draw_detection(image, detection)
    blob.draw_blobs(image, blobs, classification_window_width=rospy.get_param('~classification_window_width'))

    cva.make_visible_binary_image(masks[0])
    draw_detection(masks[0], detection)
    cva.make_visible_binary_image(masks[1])
    cva.make_visible_binary_image(masks[2])

    cv.ShowImage("video",       image)
    cv.ShowImage('motion',      masks[1])
    cv.ShowImage('intensity',   masks[2])

def matrix_to_dataset(examples, type=1):
    outputs_mat      = np.matrix(np.zeros((1, examples.shape[1]), dtype='int'))
    outputs_mat[:,:] = type
    return rf.Dataset(examples, outputs_mat)

def confirmation_prompt(confirm_phrase):
    print confirm_phrase
    print 'y(es) / n(no)'
    k = cv.WaitKey()
    if k == 'y':
        return True
    else:
        return False

def append_examples_from_file(dataset, file):
    try:
        loaded_set = ut.load_pickle(file)
        dataset.append(loaded_set)
    except IOError:
        rospy.logerr('append_examples_from_file: training file \'' + str(file) + '\'not found!')
    return dataset.num_examples()

def print_friendly(votes):
    new_dict = {}
    total = 0
    for k in votes.keys():
        new_key = k[0,0]
        new_dict[new_key] = votes[k]
    return new_dict

##
# Build detectors for both camera in stereo, create debugging images, triangulates and return 3D points
# Decides when to gather data
class EmbodiedLaserDetector:

    def __init__(self, geometric_camera, hardware_camera, dataset_file):
        self.examples = []
        self.labels = []
        self.gather_positive_examples = False
        self.clicked = False #clicked is here as data gathering logic is here
        self.stereo_cam = geometric_camera
        self.dataset_file = dataset_file
        self.build_detectors(hardware_camera)

    def clear_examples(self):
        self.examples = []
        self.labels = []

    def build_detectors(self, hardware_camera):
        self.write()
        #import pdb
        #pdb.set_trace()
        frames = hardware_camera.next()
        self.left_detector = LaserPointerDetector(frames[0], self.dataset_file)#, exposure=exposure)
        self.right_detector = LaserPointerDetector(frames[1], self.dataset_file, #exposure=exposure, 
                classifier=self.left_detector.classifier)
        for i in xrange(10):
            frames = hardware_camera.next()
            self.left_detector.detect(frames[0])
            self.right_detector.detect(frames[1])

    def run(self, images, display=True, debug=False):
        results = None
        left_detection, left_intensity_motion_activations, left_image, left_combined_masks = \
						self.left_detector.detect(images[0])
        self.record(left_detection, left_image, left_intensity_motion_activations)

        right_detection, right_intensity_motion_activations, right_image, right_combined_masks = \
                                                self.right_detector.detect(images[1])
        self.record(right_detection, right_image, right_intensity_motion_activations)

        if debug:
            motion, intensity = self.left_detector.get_motion_intensity_images()
            show_processed(left_image, [left_combined_masks, motion, intensity], 
                    left_detection, left_intensity_motion_activations, self.left_detector)
        elif display:
            draw_detection(left_image, left_detection)
            cv.ShowImage('video', left_image)

        if left_detection != None and right_detection != None:
            results = self.triangulate(left_detection, right_detection)

        if left_detection != None and left_detection.has_key('vote'):
            rospy.loginfo('EmbodiedLaserDetector.triangulate: votes ' + str(print_friendly(left_detection['votes'])))
            
        if self.clicked:
            return results 
        else:
            return None

    def set_debug(self, v):
        self.debug = v
        self.left_detector.set_debug(v)
        self.right_detector.set_debug(v)

    def triangulate(self, left_cam_detection, right_cam_detection):
        if right_cam_detection.has_key('votes'):
            rospy.loginfo('EmbodiedLaserDetector.triangulate: votes ' \
                          + str(print_friendly(right_cam_detection['votes']))  \
                          + ' ' + str(print_friendly(left_cam_detection['votes'])))
        x  = np.matrix(left_cam_detection['centroid']).T
        xp = np.matrix(right_cam_detection['centroid']).T
        rospy.loginfo('triangulate: x' + str(x.T) + ' xp ' + str(xp.T))
        result = self.stereo_cam.triangulate_3d(x, xp)
        rospy.loginfo('3D point located at' + str(result['point'].T) + \
                     ('distance %.2f error %.3f' % (np.linalg.norm(result['point']),  result['error'])))
        if result['point'][2,0] < 0:
            #Don't return anything if point is behind camera
            rospy.loginfo('EmbodiedLaserDetector.triangulate: point was behind camera, ignoring')
            return None

        if result['point'][2,0] > 5:
            rospy.loginfo('EmbodiedLaserDetector.triangulate: was too far, ignoring')
            return None

        return result

    def record(self, picked_blob, image, other_candidates):
        def store(label):
            instance = blob.blob_to_input_instance(image, picked_blob, 
                    self.left_detector.CLASSIFICATION_WINDOW_WIDTH)
            if instance != None:
                self.examples.append(instance)
                self.labels.append(np.matrix([label]))

        if self.gather_positive_examples:
            if self.clicked:
                #store as positives
                if picked_blob != None: 
                    store(1)
                    rospy.loginfo('EmbodiedLaserDetector.record: expected 1 got 1, ' + \
                                   str(len(self.examples)))
            else:
                #store as negatives 
                if picked_blob != None:
                    store(0)
                    rospy.loginfo('EmbodiedLaserDetector.record: expected 0 (no laser detections) but got 1 (laser detection), ' + \
                                  str(len(self.examples)) + ' instances')
        else:
            if self.clicked:
                pass
                #don't store anything as this case is ambiguous
            else:
                #store as negatives (false positives)
                if picked_blob != None:
                    store(0)
                    rospy.loginfo('EmbodiedLaserDetector.record: expected 0 (no laser detections) got 1 (laser detection), ' + \
                                   str(len(self.examples)) + 'instances')

    def write(self):
        if not (len(self.examples) > 0):
            rospy.loginfo('EmbodiedLaserDetector.write: no examples to record')
            return
        inputs  = ut.list_mat_to_mat(self.examples, axis = 1)
        outputs = ut.list_mat_to_mat(self.labels, axis = 1)
        #import pdb
        #pdb.set_trace()
        rospy.loginfo('EmbodiedLaserDetector.write: inputs.shape, outputs.shape ' + str(inputs.shape) + ' ' + str(outputs.shape))
        dim_reduce_set = rf.LinearDimReduceDataset(inputs, outputs)
        rospy.loginfo('EmbodiedLaserDetector.write: appending examples from disk to dataset')
        n = append_examples_from_file(dim_reduce_set, file=self.dataset_file)
        rospy.loginfo('EmbodiedLaserDetector.write: calculating pca projection vectors')
        dim_reduce_set.set_projection_vectors(dr.pca_vectors(dim_reduce_set.inputs, percent_variance=self.left_detector.PCA_VARIANCE_RETAIN))
        rospy.loginfo('EmbodiedLaserDetector.write: writing...')
        ut.dump_pickle(dim_reduce_set, self.dataset_file)
        rospy.loginfo('EmbodiedLaserDetector: recorded examples to disk.  Total in dataset ' + str(n))
        self.examples = []
        self.labels   = []


##
# Grab images, calls detectors, sends results out based on inputs from a user interface node
class LaserPointerDetectorNode:
    def __init__(self, camera_root_topic, calibration_root_topic, 
            #exposure = LaserPointerDetector.SUN_EXPOSURE, 
            #video = None, 
            dataset_file,
            display=False):

        image_type = 'image_rect_color'
        #if video is None:
        #self.video    = cam.StereoFile('measuring_tape_red_left.avi','measuring_tape_red_right.avi')
        #else:
        #    self.video = video
        self.video  = cam.ROSStereoListener(['/' + camera_root_topic + '/left/' + image_type, 
                                             '/' + camera_root_topic + '/right/' + image_type])

        self.video_lock = RLock()
        self.camera_model = cam.ROSStereoCalibration('/' + calibration_root_topic + '/left/camera_info' , 
                                                     '/' + calibration_root_topic + '/right/camera_info')
        self.detector = EmbodiedLaserDetector(self.camera_model, self.video, dataset_file)
        
        #self.exposure = exposure
        self.display = display
        self.debug = False #Require display = True
        self.windows_made = False
        if self.display:
            self._make_windows()

        rospy.Subscriber(MOUSE_CLICK_TOPIC, String, self._click_handler)
        rospy.loginfo('Suscribed to ' + MOUSE_CLICK_TOPIC)
        rospy.Subscriber(LASER_MODE_TOPIC, String, self._mode_handler)
        rospy.loginfo('Subscribed to ' + LASER_MODE_TOPIC)
        self.topic = rospy.Publisher(CURSOR_TOPIC, PointStamped)
        rospy.loginfo('Publishing to ' + CURSOR_TOPIC)
        self.viz_topic = rospy.Publisher(VIZ_TOPIC, PoseStamped)
        rospy.loginfo('Publishing to ' + VIZ_TOPIC)

    def _click_handler(self, evt):
        message = evt.data
        if self.detector is not None:
            if message == 'True':
                self.detector.clicked = True
                rospy.loginfo('LaserPointerDetector.click_handler: click!')
            elif message == 'False':
                self.detector.clicked = False
                rospy.loginfo('LaserPointerDetector.click_handler: released click')
            else:
                raise RuntimeError('unexpected click message from topic' + MOUSE_CLICK_TOPIC)

    def _mode_handler(self, evt):
        message = evt.data
        if(message == 'debug'):
            self.debug = not self.debug
            rospy.loginfo('LaserPointerDetector.mode_handler: debug'  + str(self.debug))
            self._make_windows()

        elif (message == 'display'):
            self.display = not self.display
            rospy.loginfo('LaserPointerDetector.mode_handler: display ' + str(self.display))
            self._make_windows()

        elif(message == 'rebuild'): #Rebuild detector based on new training data
            self.video_lock.acquire()
            if self.detector is not None:
                self.detector.build_detectors(self.video)
            self.video_lock.release()

        elif(message == 'positive'): #Will toggle gathering positive examples
            if self.detector is not None:
                self.detector.gather_positive_examples = not self.detector.gather_positive_examples
                rospy.loginfo('LaserPointerDetector.mode_handler: gather_positive_examples' + str(self.detector.gather_positive_examples))

        elif(message == 'clear'):
            self.detector.clear_examples()
        else:
            raise RuntimeError('unexpected mode message from topic' + LASER_MODE_TOPIC)
        
    def _make_windows(self):
        if self.windows_made:
            return
        windows = ['video', 'right', 'thresholded', 'motion', 'intensity', 'patch', 'big_patch']
        for n in windows:
            cv.NamedWindow(n, 1)
        cv.MoveWindow("video", 0,   0)
        cv.MoveWindow("right", 800, 0)
        cv.MoveWindow("thresholded", 800, 0)
        cv.MoveWindow("intensity", 0,   600)
        cv.MoveWindow("motion", 800, 600)
        self.windows_made = True

    def set_debug(self, v):
        self.detector.set_debug(v)
        self.debug = v

    def run(self):
        i = 0
        try:
            while not rospy.is_shutdown():
                i = i + 1
                t0 = time.time()
                self.video_lock.acquire()
                frames = list(self.video.next())
                result = self.detector.run(frames, display=self.display, debug=self.debug)
                self.video_lock.release() 
                
                if result != None:
                    p = result['point']
                    ps = PointStamped()
                    ps.header.stamp = rospy.get_rostime()
                    ps.header.frame_id = rospy.get_param('laser_pointer_detector/detector_frame')
                    ps.point.x = p[0,0]
                    ps.point.y = p[1,0]
                    ps.point.z = p[2,0]
                    self.topic.publish(ps)

                    pose_stamped = PoseStamped()
                    pose_stamped.header = ps.header
                    pose_stamped.pose.position = ps.point
                    self.viz_topic.publish(pose_stamped)

                if self.display:
                    k = cv.WaitKey(10)
                t1 = time.time()

                #rospy.logdebug('Running at ' + str(1./(t1 - t0)) + ' hz.')
                if i % 100 == 0:
                    rospy.loginfo('Running at ' + str(1./(t1 - t0)) + ' hz.')

        except StopIteration, e:
            if self.state_object.__class__ == GatherExamples:
                self.state_object.write()

#if __name__ == '__main__':
#    from pkg import *
#    rospy.init_node('laser_pointer_detector')
#    print 'PARAM is', rospy.get_param('~shade_exposure')
#    import pdb
#    pdb.set_trace()
#    print 'PARAM is', rospy.get_param('~shade_exposure')
#    exit()

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    #move this to params too?
    p.add_option('-c', '--camera', action='store',
                dest='camera', default='wide_stereo', 
                help='stereo pair root topic (wide_stereo, narrow_stereo, etc)')
    #move this to params too?
    p.add_option('-k', '--calibration', action='store',
                dest='calibration', default=None,
                help='stereo pair calibration root topic (usually the same as given in -c)')
    p.add_option('-f', '--dataset', action='store',
                dest='dataset_file', default='PatchClassifier.dataset.pickle', help='dataset for classifier')

    #p.add_option('-r', '--run',  action='store_true', 
    #            dest='mode_run', help='classify')

    #p.add_option('-o', '--office', action='store_true', dest='office', 
    #             help='true for operating robot in office environments')
    p.add_option('-d', '--display',  action='store_true', 
                dest='display', default=False, help='show display')
    p.add_option('-t', '--time', action = 'store_true', 
                dest='time', help='display timing information')
    opt, args = p.parse_args()

    #Move this to a param file
    #if opt.office == True:
    #    print 'opt.office == True, using SHADE exposure'
    #    exposure = LaserPointerDetector.SHADE_EXPOSURE
    #else:
    #    exposure = LaserPointerDetector.SUN_EXPOSURE
    #    print 'opt.office == False, using SUN exposure'

    if opt.calibration == None:
        opt.calibration = opt.camera

    rospy.loginfo('===========================================================')
    rospy.loginfo('# Detections are red circles.                             =')
    rospy.loginfo('# Hypothesis blobs are blue squares.                      =')
    rospy.loginfo('===========================================================')
    #print 'Exposure set to', exposure
    rospy.init_node('laser_pointer_detector')
    lpdn = LaserPointerDetectorNode(opt.camera, opt.calibration, 
            opt.dataset_file, display=opt.display)
    if opt.time != None:
        lpdn.set_debug(True)
    lpdn.run()

