import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import actionlib

import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import hrl_pr2_lib.devices as hd
import hrl_camera.ros_camera as rc

import hai_sandbox.recognize_3d as r3d
#import hai_sandbox.msg as hmsg
import geometry_msgs.msg as gmsg
import tf
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import ml_lib.dataset as ds
import cv
import datetime
import hrl_lib.util as ut
import math

def str_from_time(ctime):
    return datetime.datetime.fromtimestamp(ctime).strftime('%Y_%m_%d_%H_%M_%S')

class InterestPointPerception(r3d.InterestPointAppBase): 

    def __init__(self, object_name, labeled_data_fname, tf_listener):
        r3d.InterestPointAppBase.__init__(self, object_name, labeled_data_fname)

        self.tf_listener = tf_listener
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        self.laser_scan = hd.LaserScanner('point_cloud_srv')
        self.prosilica = rc.Prosilica('prosilica', 'polled')
        self.prosilica_cal = rc.ROSCameraCalibration('/prosilica/camera_info')
        self.image_pub = r3d.ImagePublisher(object_name + '_perception', self.prosilica_cal)
        self.last_added = None
        self.disp = r3d.RvizDisplayThread(self.prosilica_cal)
        self.disp.start()

    def scan(self, point3d):
        rospy.loginfo('InterestPointPerception: scanning..')
        point_cloud_bl = self.laser_scan.scan(math.radians(180.), math.radians(-180.), 15)
        prosilica_image = self.prosilica.get_frame()
        image_T_laser = tfu.transform('/high_def_optical_frame', '/base_link', self.tf_listener)

        #Extract features
        self.feature_extractor = r3d.IntensityCloudData(point_cloud_bl, 
                prosilica_image, image_T_laser, self.prosilica_cal, 
                point3d, self.rec_params)

        fex = self.feature_extractor
        self.disp.display_scan(fex.point_cloud3d_orig, fex.points3d_valid_laser, fex.colors_valid,
                prosilica_image, self.prosilica_cal)

        rospy.loginfo('InterestPointPerception: extracting features..')
        #self.instances, self.points2d, self.points3d = self.feature_extractor.extract_vectorized_features()
        rospy.loginfo('InterestPointPerception: finished extraction.')

    def is_ready(self):
        return self.learner != None

    def add_example(self, point3d_bl, label, pt2d=None):
        fex = self.feature_extractor
        feature = fex.feature_vec_at_mat(point3d_bl)
        if feature == None:
            return False
        pt2d = fex.calibration_obj.project(tfu.transform_points(fex.image_T_laser, point3d_bl))
        label = np.matrix(label)
        self.add_to_dataset(feature, label, pt2d, point3d_bl)
        self.last_added = {'pt': pt2d, 'l': label}
        return True

    def select_next_instances(self, n):
        #selected_idx, selected_dist = self.learner.select_next_instances(self.instances, n)
        return self.learner.select_next_instances(self.instances, n)

    def get_likely_success_points(self):
        #do something to reduce number of points
        points3d_pos = self.classified_dataset.pt3d[:, np.where(self.classified_dataset.outputs == r3d.POSITIVE)[1].A1]
        return points3d_pos

    def draw_and_send(self):
        self.classify()
        img = cv.CloneMat(self.feature_extractor.image_cv)
        #draw classified points.
        colors = {r3d.POSITIVE: [0,255,0], r3d.NEGATIVE: [0,0,255]}
        r3d.draw_labeled_points(img, self.classified_dataset)

        #draw labeled data. 
        r3d.draw_labeled_points(img, self.dataset, colors[r3d.POSITIVE], colors[r3d.NEGATIVE])

        #draw latest addition and its label. 
        r3d.draw_points(img, self.last_added['pt'], colors[self.last_added['l']], 4)

        self.image_pub.publish(img, self.feature_extractor.calibration_obj)


if __name__ == '__main__':
    import sys

    object_name = sys.argv[1]
    datafile = sys.argv[2]
    server = InterestPointActions(object_name, datafile)
    rospy.spin() 





















    #def classify(self):
    #    r3d.InterestPointAppBase.classify(self)
    #    points3d_pos = self.classified_dataset.pt3d[:, np.where(self.classified_dataset.outputs == r3d.POSITIVE)[1].A1]
    #    return points3d_pos

    #def add_to_dataset(self, feature, label, pt2d, pt3d):
    #    if self.dataset == None:
    #        self.dataset = InterestPointDataset(feature, label, [pt2d], [pt3d], self.feature_extractor)
    #    else:
    #        self.dataset.add(feature, label, pt2d, pt3d)
    #        #pos_ex = np.sum(self.dataset.outputs)
    #        #neg_ex = self.dataset.outputs.shape[1] - pos_ex
    #        #if pos_ex > 2 and neg_ex > 2 and self.blank:
    #        #    self.train_learner()
    #        #    self.blank = False

    #def train(self):
    #    return None

    #def train(self):
    #    if self.dataset != None: 
    #        #train
    #        self.ipdetector.train(self.dataset)
    #        #classify
    #        #pdb.set_trace()
    #        results = []
    #        for i in range(self.instances.shape[1]):
    #            results.append(self.ipdetector.learner.classify(self.instances[:,i]))
    #        #pdb.set_trace()
    #        plist = [self.points2d[:, i] for i in range(self.points2d.shape[1])]
    #        p3list = [self.points3d[:, i] for i in range(self.points3d.shape[1])]
    #        self.classified_dataset = InterestPointDataset(self.instances, np.matrix(results), 
    #                                                       plist, p3list, self.feature_extractor)

        #self.object_name = object_name
        #self.ipdetector = r3d.InterestPointDetector()
        #self.dataset = None
        #self.labeled_data_fname = datafile
        #if datafile != None:
        #    self.load_labeled_data()
        #self.feature_extractor = None

    #def load_labeled_data(self):
    #    self.dataset = ut.load_pickle(self.labeled_data_fname)
    #    print 'loaded from', self.labeled_data_fname
    #    self.dataset.pt2d = [None] * len(self.dataset.pt2d)
    #    self.dataset.pt3d = [None] * len(self.dataset.pt3d)
    #    self.ipdetector = InterestPointDetector(self.dataset)
    #    self.ipdetector.train(self.dataset)

    #def add_to_dataset(self, feature, label, pt2d, pt3d):
    #    if self.dataset == None:
    #        self.dataset = InterestPointDataset(feature, label, [pt2d], [pt3d], self.feature_extractor)
    #    else:
    #        self.dataset.add(feature, label, pt2d, pt3d)
    #        pos_ex = np.sum(self.dataset.outputs)
    #        neg_ex = self.dataset.outputs.shape[1] - pos_ex
    #        if pos_ex > 2 and neg_ex > 2 and self.blank:
    #            self.train_learner()
    #            self.blank = False




#    def __init__(self, object_name, datafile):
#        self.node_name = object_name + '_active_perception_server'
#        self.object_name = object_name
#        rospy.init_node(self.node_name)
#
#        #Load learner
#        self.learner = r3d.SVMPCA_ActiveLearner()
#        self.rec_params = r3d.Recognize3DParam()
#
#        rospy.loginfo('Loading dataset: ' + datafile)
#        labeled_light_switch_dataset = ut.load_pickle(datafile)
#        rospy.loginfo('Training %s.' % object_name)
#        self.learner.train(labeled_light_switch_dataset, 
#                           labeled_light_switch_dataset.sizes['intensity'],
#                           self.rec_params.variance_keep)
#
#        rospy.loginfo('Launching ROS connections')
#
#        #Create message listeners
#        self.tf_listener = tf.TransformListener()
#        self.bridge = CvBridge()
#
#        #Create servers
#        self.find_as = actionlib.SimpleActionServer('find_' + object_name, 
#                hmsg.InterestPointLocate, self.find_cb, False)
#        self.find_as.start() 
#        #self.find_as.is_preempt_requested()
#        #self.find_as.set_preempted()
#        #self.find_as.publish_feedback(self._feedback)
#        #self.find_as.set_succeeded()
#
#        self.add_example_as = actionlib.SimpleActionServer('add_example_' + object_name, 
#                hmsg.InterestPointAddExample, self.add_example_cb, False)
#        self.add_example_as.start() 
#
#        self.pick_as = actionlib.SimpleActionServer('pick_' + object_name,
#                hmsg.InterestPointPick, self.pick_cb, False)
#        self.pick_as.start()
#
#        rospy.loginfo('Ready.')
#
#        #rospy.loginfo('Loading dataset: ' + datafile)
#        #rospy.loginfo('Training %s.' % object_name)
#        #rospy.loginfo('Launching ROS connections')
#        
#    def _find(self, cloud, image, calib, center, radius):
#        #preprocess 
#        self.rec_params.radius = goal.radius
#        image_T_laser = tfu.transform(calib.frame, cloud.header.frame_id, self.tf_listener)
#        ic_data = r3d.IntensityCloudData(cloud, image, image_T_laser, calib, center, self.rec_params)
#
#        #label
#        instances = ic_data.extract_vectorized_features()
#        results = []
#        for i in range(instances.shape[1]):
#            nlabel = self.learner.classify(instances[:, i])
#            results.append(nlabel)
#
#        results = np.matrix(results)
#
#        #draw and save results
#        image_cpy = cv.CloneImage(image)
#        r3d.draw_labeled_points(ic_data, ds.Dataset(self.instances, results), image_cpy)
#        cv.SaveImage('%s_%s.png' % (self.object_name, str_from_time(cloud.header.stamp.to_time())), image_cpy)
#
#        #want 3d location of each instance
#        positive_indices = np.where(results == r3d.POSITIVE)[1]
#        positive_points_3d = ic_data.sampled_points[:, positive_indices]
#
#        #return a random point for now
#        rindex = np.random.randint(0, len(positive_indices))
#
#        return positive_points_3d[:,rindex]
#
#
#    def find_cb(self, goal):
#        calib = rc.ROSCameraCalibration(offline=True)
#        calib.camera_info(goal.camera_info)
#        imagecv = self.bridge.imgmsg_to_cv(goal.image, encoding="bgr8")
#        centermat = np.matrix([goal.center.x, goal.center.y, goal.center.z]).T
#        round_points = self._find(goal.cloud, imagecv, centermat, goal_radius)
#
#        results = hmsg.InterestPointLocateResult()
#        results.header.frame_id = goal.cloud.header.frame_id
#        results.header.stamp = rospy.Time.now()
#        results.interest_points = [gmsg.Point(*round_point[:,i].T.A1.tolist()) for i in range(round_points.shape[1])]
#        self.find_as.set_succeeded()
#
#    def add_example_cb(self, goal):
#        pass
#
#    def pick_cb(self, goal):
#        pass
