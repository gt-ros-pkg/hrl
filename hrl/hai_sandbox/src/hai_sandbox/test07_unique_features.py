import csv
import roslib; roslib.load_manifest('hai_sandbox')
from cv_bridge.cv_bridge import CvBridge, CvBridgeError
import rospy
import cv
import sys

import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
import tf.transformations as tr
import tf
import hrl_camera.ros_camera as cam
from sensor_msgs.msg import CameraInfo
import numpy as np
import hai_sandbox.features as fea
import os.path as pt
import hrl_lib.util as ut


##
# @param bagname
# @param topic
# @return features_list list of tuples [(float time, (list surf_keypoints, list surf_descriptors))...]
def find_image_features(bagname, topic):
    features_list = []
    bridge = CvBridge()
    i = 0
    for topic, msg, t in ru.bag_iter(bagname, [topic]):
        t = msg.header.stamp.to_time()
        image = bridge.imgmsg_to_cv(msg, 'bgr8')
        image_gray = fea.grayscale(image)
        surf_keypoints, surf_descriptors = fea.surf(image_gray)
        features_list.append((t, (surf_keypoints, surf_descriptors)))
        rospy.loginfo("%.3f frame %d found %d points" % (t, i, len(surf_keypoints)))
        i = i + 1
    return features_list

def csv_bag_names(fname):
    csv_file = open(fname)
    for bag_name in csv.reader(csv_file):
        yield bag_name
    csv_file.close()

if __name__ == '__main__':
    forearm_cam_l = '/l_forearm_cam/image_rect_color'
    ws_l = '/wide_stereo/left/image_rect_color'
    ws_r = '/wide_stereo/right/image_rect_color'

    #features_list = find_image_features(sys.argv[1], forearm_cam_l)
    #Find all features in all videos that we have
    fname = sys.argv[1]
    for path_complete in csv_bag_names(fname):
        path_complete = path_complete[0]
        rospy.loginfo('processing %s'% path_complete)
        features_list = find_image_features(path_complete, forearm_cam_l)

        path_bag, name_bag = pt.split(path_complete)
        root_name, _ = pt.splitext(name_bag)
        surf_path_complete = pt.join(path_bag, root_name + '.surf_pkl')

        #pickle features list
        rospy.loginfo('saving feature extraction results to %s' % surf_path_complete)
        ut.save_pickle(features_list, surf_path_complete)












