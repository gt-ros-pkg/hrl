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
import os

def csv_bag_names(fname):
    csv_file = open(fname)
    for bag_name in csv.reader(csv_file):
        yield bag_name
    csv_file.close()

if __name__ == '__main__':
    import sys
    fname = sys.argv[1]

    for path in csv_bag_names(fname):
        cmd ='python test08.py %s' % path[0]
        print cmd
        os.system(cmd)
