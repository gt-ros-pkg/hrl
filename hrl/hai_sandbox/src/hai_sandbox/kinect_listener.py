import roslib; roslib.load_manifest('hai_sandbox')

import rospy
import feature_extractor_fpfh.msg as fmsg
import hrl_lib.rutils as ru
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class KinectListener:
    def __init__(self, topic=None):
        if topic == None:
            topic = 'fpfh_hist'
        rate = .2
        self.listener = ru.GenericListener('kinect_client', fmsg.FPFHHist, topic, rate)
        self.bridge = CvBridge()

    def read(self):
        fpfh_hist = self.listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)

        histogram = np.matrix(fpfh_hist.histograms).reshape((fpfh_hist.hist_npoints, 33)).T 
        hist_points = np.matrix(fpfh_hist.hpoints3d).reshape((fpfh_hist.hist_npoints, 3)).T 
        points3d = np.matrix(fpfh_hist.origpoints).reshape((fpfh_hist.original_npoints, 3)).T 
        points3d = points3d[:, np.where(1-np.isnan(points3d))[1].A1]
        cvimage_mat = self.bridge.imgmsg_to_cv(fpfh_hist.image, 'bgr8')
        return {'histogram': histogram, 'hpoints3d': hist_points, 'points3d': points3d, 'image': cvimage_mat}


