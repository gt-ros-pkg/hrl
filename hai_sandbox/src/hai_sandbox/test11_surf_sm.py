import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import hrl_lib.util as ut
import csv
import scipy.spatial as sp
import hrl_camera.ros_camera as cam
import hai_sandbox.features as fea
import numpy as np
import cv
import hrl_lib.rutils as ru
from cv_bridge.cv_bridge import CvBridge, CvBridgeError
import scipy.cluster.vq as vq

def csv_bag_names(fname):
    csv_file = open(fname)
    for bag_name in csv.reader(csv_file):
        yield bag_name
    csv_file.close()

def features_mat_compress(fmat, k):
    #k = max(int(round(fmat.shape[1] * percent)), 1)
    rospy.loginfo('compressing to %d centers' % k)
    center_indices = np.random.permutation(fmat.shape[1])[0:k]
    initial_centers = fmat[:, center_indices]
    kresults = vq.kmeans(np.array(fmat.T), np.array(initial_centers.T))
    return np.matrix(kresults[0]).T

if __name__ == '__main__':
    import sys
    import pdb
    features_file = sys.argv[1]
    images_file = sys.argv[2]
    features_db = np.column_stack([ut.load_pickle(p[0]) for p in csv_bag_names(features_file)])
    features_db_reduced = features_mat_compress(features_db, 500)

    #Generate a random color for each feature
    colors = np.matrix(np.random.randint(0, 255, (3, features_db_reduced.shape[1])))
    features_tree = sp.KDTree(np.array(features_db_reduced.T))
    bridge = CvBridge()

    forearm_cam_l = '/l_forearm_cam/image_rect_color'
    cv.NamedWindow('surf', 1)

    #import pdb
    #while not rospy.is_shutdown():
    i = 0
    for topic, msg, t in ru.bag_iter(images_file, [forearm_cam_l]):
        image = bridge.imgmsg_to_cv(msg, 'bgr8')
        #image = camera.get_frame()
        image_gray = fea.grayscale(image)
        surf_keypoints, surf_descriptors = fea.surf(image_gray)
        #print len(surf_keypoints)
        #pdb.set_trace()

        #match each keypoint with one in our db & look up color
        matching_idx = [features_tree.query(d)[1] for d in surf_descriptors]
        coordinated_colors = colors[:, matching_idx]

        #nimage = fea.draw_surf(image, surf_keypoints, (0,255,0))
        nimage = fea.draw_surf2(image, surf_keypoints, coordinated_colors)
        cv.ShowImage('surf', nimage)
        cv.SaveImage('forearm_cam%d.png' % i, nimage)
        i = i + 1
        cv.WaitKey(10)
        





    #rospy.init_node('test11')
    #camera = cam.ROSImageClient(forearm_cam_l)
