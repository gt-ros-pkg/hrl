#!/usr/bin/env python
##
# This utility is used for publishing
# OpenCV accessible camera images over
# ROS.
#
# Usage ./ros_camera OPENCV_ID
# where OPENCV_ID is a number >= 0
# representing opencv's index for 
# the particular device
#
import roslib
roslib.load_manifest('hrl_camera')
import sys
import rospy
import cv
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge.cv_bridge import CvBridge, CvBridgeError
import hrl_lib.rutils as ru
import polled_camera.srv as ps
import numpy as np
import hrl_lib.tf_utils as tfu
import tf.transformations as tr

class ROSImageClient:
    def __init__(self, topic_name):
        self.bridge = CvBridge()
        def message_extractor(ros_img):
            try:
                cv_image = self.bridge.imgmsg_to_cv(ros_img, 'bgr8')
                return cv_image, ros_img
            except CvBridgeError, e:
                return None
        self.listener = ru.GenericListener('ROSImageClient', Image, topic_name, 
                                           .1, message_extractor)
    def get_frame(self, fresh=False):
        if not fresh:
            return self.listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)[0]
        else:
            not_fresh = True
            cur_time = rospy.Time.now().to_sec()
            while not_fresh:
                img, rosmsg = self.listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)
                if not (rosmsg.header.stamp.to_sec() < cur_time):
                    not_fresh = False
                rospy.sleep(.1)
            return img


    def next(self):
        return self.get_frame()

class Prosilica(ROSImageClient):
    def __init__(self, camera_name, mode):
        if mode == 'polled':
            srv_name = '/%s/request_image' % camera_name
            self.trigger_proxy = rospy.ServiceProxy(srv_name, ps.GetPolledImage)

        self.mode = mode
        self.camera_name = camera_name
        topic_name = '/%s/image_rect_color' % camera_name
        ROSImageClient.__init__(self, topic_name)

    def get_frame(self, fresh=False):
        if self.mode == 'polled':
            #will get a rospy.service.ServiceException if mode is wrong
            rq = ps.GetPolledImageRequest()
            rq.response_namespace = '/%s' % self.camera_name
            resp = self.trigger_proxy(rq)
        return ROSImageClient.get_frame(self, fresh)

        #if not fresh:
        #    return self.listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)[0]

        #else:
        #    not_fresh = True
        #    cur_time = rospy.Time.now().to_sec()
        #    while not_fresh:
        #        img, rosmsg = self.listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)
        #        if not (rosmsg.header.stamp.to_sec() < cur_time):
        #            not_fresh = False
        #        rospy.sleep(.1)
        #    return img





##
# from camera.py in laser_interface.
class ROSCameraCalibration:
    def __init__(self, channel=None, offline=False):
        if not offline:
            rospy.Subscriber(channel, CameraInfo, self.camera_info)
        self.has_msg = False
        self.msg = None

    def wait_till_msg(self):
        r = rospy.Rate(10)
        while not self.has_msg:
            r.sleep()

    def camera_info(self, msg):
        self.distortion = np.matrix(msg.D)
        self.K = np.reshape(np.matrix(msg.K), (3,3))
        self.R = np.reshape(np.matrix(msg.R), (3,3))
        self.P = np.reshape(np.matrix(msg.P), (3,4))
        self.w = msg.width
        self.h = msg.height
        self.frame = msg.header.frame_id
        self.has_msg = True
        self.msg = msg

    ##
    # project 3D point into this camera 
    #   
    # @param p 3x1 matrix in given coord frame
    # @param tf_listener None if transformation not needed
    # @param from_frame None is default camera frame
    # @return 2x1 matrix
    def project(self, p, tf_listener=None, from_frame=None):
        if not self.has_msg:
            raise RuntimeError('Has not been initialized with a CameraInfo message (call camera_info).')
        if not(from_frame == None or from_frame == self.frame):
            p_cam = tfu.transform(self.frame, from_frame, tf_listener) \
                           * tfu.tf_as_matrix((p.A1.tolist(), tr.quaternion_from_euler(0,0,0)))
            trans, q = tfu.matrix_as_tf(p_cam)
            p = np.matrix(trans).T

        hrow = np.matrix(np.zeros((1,p.shape[1])))
        p = np.row_stack((p, hrow))
        pp = self.P * p
        pp = pp / pp[2,:]
        return pp[0:2,:]


class ROSCamera(ROSImageClient):
    def __init__(self, topic_name):
        ROSImageClient.__init__(self, topic_name)

    def set_exposure(self, exposure):
        print 'ROSCamera: **** WARNING: set_exposure unimplemented over ROS ****'

    def set_frame_rate(self, rate):
        print 'ROSCamera: **** WARNING: set_frame_rate unimplemented over ROS ****'


class ROSStereoListener:
    def __init__(self, topics, rate=30.0, name='stereo_listener'):
        self.listener = ru.GenericListener(name, [Image, Image], topics, rate)
        self.lbridge = CvBridge()
        self.rbridge = CvBridge()

    def next(self):
        lros, rros =  self.listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)
        lcv = cv.CloneMat(self.lbridge.imgmsg_to_cv(lros, 'bgr8'))
        rcv = cv.CloneMat(self.rbridge.imgmsg_to_cv(rros, 'bgr8'))
        return lcv, rcv


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'This utility is used for publishing'
        print 'OpenCV accessible camera images over'
        print 'ROS.\n'
        print 'Usage ./ros_camera OPENCV_ID'
        print 'where OPENCV_ID is a number >= 0'
        print 'representing opencv\'s index for '
        print 'the particular device'

    camera_id = int(sys.argv[1])
    topic_name = 'cvcamera' + str(camera_id)

    image_pub = rospy.Publisher(topic_name, Image)
    rospy.init_node('cvcamera', anonymous=True)

    capture = cv.CaptureFromCAM(camera_id)
    bridge = CvBridge()

    print 'Opening OpenCV camera with ID', camera_id
    print 'Publishing on topic', topic_name
    while not rospy.is_shutdown():
        try:
            cv_image = cv.CloneImage(cv.QueryFrame(capture))
            rosimage = bridge.cv_to_imgmsg(cv_image, "bgr8")
            image_pub.publish(rosimage)
        except rospy.exceptions.ROSSerializationException, e:
            print 'serialization exception'
        except CvBridgeError, e: 
            print e
            break
        except KeyboardInterrupt:
            print "Shutting down."
            break
        time.sleep(1/100.0)

    cv.DestroyAllWindows()
