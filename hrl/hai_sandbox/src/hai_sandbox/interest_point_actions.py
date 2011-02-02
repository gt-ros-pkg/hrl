import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import actionlib

import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import hrl_pr2_lib.devices as hd
import hrl_camera.ros_camera as rc

import hai_sandbox.recognize_3d as r3d
import hai_sandbox.msg as hmsg
import geometry_msgs.msg as gmsg
import tf
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import ml_lib.dataset as ds
import cv
import datetime

def str_from_time(ctime):
    return datetime.datetime.fromtimestamp(ctime).strftime('%Y_%m_%d_%H_%M_%S')

class InterestPointActions: 

    def __init__(self, object_name, datafile):
        self.node_name = object_name + '_active_perception_server'
        self.object_name = object_name
        rospy.init_node(self.node_name)

        #Load learner
        self.learner = r3d.SVMActiveLearnerApp()
        self.rec_params = r3d.Recognize3DParam()

        rospy.loginfo('Loading dataset: ' + datafile)
        labeled_light_switch_dataset = ut.load_pickle(datafile)
        rospy.loginfo('Training %s.' % object_name)
        self.learner.train(labeled_light_switch_dataset, 
                           labeled_light_switch_dataset.sizes['intensity']
                           self.params.variance_keep)

        rospy.loginfo('Launching ROS connections')

        #Create message listeners
        self.tf_listener = tf.TransformListener()
        self.bridge = CvBridge()

        #Create servers
        self.find_as = actionlib.SimpleActionServer('find_' + object_name, 
                hmsg.InterestPointLocate, self.find_cb, False)
        self.find_as.start() 

        self.add_example_as = actionlib.SimpleActionServer('add_example_' + object_name, 
                hmsg.InterestPointAddExample, self.add_example_cb, False)
        self.add_example_as.start() 

        self.pick_as = actionlib.SimpleActionServer('pick_' + object_name,
                hmsg.InterestPointPick, self.pick_cb, False)
        self.pick_as.start()

        rospy.loginfo('Ready.')

        
    def _find(self, cloud, image, calib, center, radius):
        #preprocess 
        self.rec_params.radius = goal.radius
        image_T_laser = tfu.transform(calib.frame, cloud.header.frame_id, self.tf_listener)
        ic_data = r3d.IntensityCloudData(cloud, image, image_T_laser, calib, center, self.rec_params)

        #label
        instances = ic_data.extract_vectorized_features()
        results = []
        for i in range(instances.shape[1]):
            nlabel = self.learner.classify(instances[:, i])
            results.append(nlabel)

        results = np.matrix(results)

        #draw and save results
        image_cpy = cv.CloneImage(image)
        r3d.draw_image_labels(ic_data, ds.Dataset(self.instances, results), image_cpy)
        cv.SaveImage('%s_%s.png' % (self.object_name, str_from_time(cloud.header.stamp.to_time())), image_cpy)

        #want 3d location of each instance
        positive_indices = np.where(results == r3d.POSITIVE)[1]
        positive_points_3d = ic_data.sampled_points[:, positive_indices]

        #return a random point for now
        rindex = np.random.randint(0, len(positive_indices))

        return positive_points_3d[:,rindex]


    def find_cb(self, goal):
        calib = rc.ROSCameraCalibration(offline=True)
        calib.camera_info(goal.camera_info)
        imagecv = self.bridge.imgmsg_to_cv(goal.image, encoding="bgr8")
        centermat = np.matrix([goal.center.x, goal.center.y, goal.center.z]).T
        round_points = self._find(goal.cloud, imagecv, centermat, goal_radius)

        results = hmsg.InterestPointLocateResult()
        results.header.frame_id = goal.cloud.header.frame_id
        results.header.stamp = rospy.Time.now()
        results.interest_points = [gmsg.Point(*round_point[:,i].T.A1.tolist()) for i in range(round_points.shape[1])]
        self.find_as.set_succeeded()

    def add_example_cb, goal):
        pass

    def pick_cb, goal):
        pass


if __name__ == '__main__':
    import sys

    object_name = sys.argv[1]
    datafile = sys.argv[2]
    server = InterestPointActions(object_name, datafile)
    rospy.spin() 







    #self.find_as.is_preempt_requested()
    #self.find_as.set_preempted()
    #self.find_as.publish_feedback(self._feedback)
    #self.find_as.set_succeeded()
