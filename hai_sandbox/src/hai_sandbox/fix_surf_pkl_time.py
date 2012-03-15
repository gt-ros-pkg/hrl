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
import itertools as it

#Load original pickle
orig_bag = sys.argv[1]
topic = '/l_forearm_cam/image_rect_color'

#Load surf pickle
print 'loading pickle', sys.argv[2]
surf_pkl = ut.load_pickle(sys.argv[2])

new_surf_data = []

print 'replacing time field'
for tmt, surf_record in it.izip(ru.bag_iter(orig_bag, [topic]), surf_pkl) :
    topic, msg, t = tmt
    surf_t, surf_data = surf_record 
    new_surf_data.append((msg.header.stamp.to_time(), surf_data))

print 'saving pickle with new time', sys.argv[2]
ut.save_pickle(new_surf_data, sys.argv[2])
