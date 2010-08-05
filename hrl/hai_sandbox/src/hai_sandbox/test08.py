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
import scipy.cluster.vq as vq

def csv_bag_names(fname):
    csv_file = open(fname)
    for bag_name in csv.reader(csv_file):
        yield bag_name
    csv_file.close()

def gen_pkl_name(path_complete, ext):
    #path_complete = path_complete[0]
    path_bag, name_bag = pt.split(path_complete)
    root_name, _ = pt.splitext(name_bag)
    surf_path_complete = pt.join(path_bag, root_name + ext)
    return surf_path_complete

def features_mat(features_list):
    features = []
    for msg_t, surf in features_list:
        features.append(np.matrix(surf[1]).T)
    return np.column_stack(tuple(features))

def features_mat_compress(fmat, k):
    #k = max(int(round(fmat.shape[1] * percent)), 1)
    rospy.loginfo('compressing to %d centers' % k)
    center_indices = np.random.permutation(fmat.shape[1])[0:k]
    initial_centers = fmat[:, center_indices]
    kresults = vq.kmeans(np.array(fmat.T), np.array(initial_centers.T))
    return np.matrix(kresults[0]).T

def compress_pkl(surf_path_complete):
    features_list = ut.load_pickle(surf_path_complete)

    rospy.loginfo('making matrix')
    fmat = features_mat(features_list)

    rospy.loginfo('compressing')
    reduced_features = features_mat_compress(fmat, 1000)
    small_pickle_fname = gen_pkl_name(path_complete, '.surf_sm_pkl')
    ut.save_pickle(reduced_features, small_pickle_fname)
    rospy.loginfo('saved to %s' % small_pickle_fname)

if __name__ == '__main__':
    ##
    # "compresss" large pkls
    path_complete = sys.argv[1]
    #for path_complete in csv_bag_names(fname):
    surf_path_complete = gen_pkl_name(path_complete, ext=".surf_pkl")
    rospy.loginfo('loading %s' % surf_path_complete)
    compress_pkl(surf_path_complete)
    rospy.loginfo('done')
    exit()
























    #forearm_cam_l = '/l_forearm_cam/image_rect_color'
    #ws_l = '/wide_stereo/left/image_rect_color'
    #ws_r = '/wide_stereo/right/image_rect_color'
    #features_list = find_image_features(sys.argv[1], forearm_cam_l)
    #Find all features in all videos that we have
    #list_of_features_list = []

        #list_of_features_list.append(reduced_features)
        #break
    #ut.save_pickle(list_of_features_list, 'reduced_all_features.pkl')
    #pdb.set_trace()
    #Put all features into one large matrix, cluster...
    #for message_t, surf in list_of_features_list:
    #    keypoints, descriptors = surf

    #Kmean wants row vectors
    #What happens if we have many duplicated points?

#whitened = vq.whiten(features)
#book = np.array((whitened[0],whitened[2]))
#vq.kmeans(whitened, book)

