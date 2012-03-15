import roslib; roslib.load_manifest('hai_sandbox')
from cv_bridge.cv_bridge import CvBridge, CvBridgeError
import rospy
import cv
import sys

import hrl_lib.tf_utils as tfu
import tf.transformations as tr
import tf
import hrl_camera.ros_camera as cam
from sensor_msgs.msg import CameraInfo
import numpy as np
import hai_sandbox.features as fea


##
# from camera.py in laser_interface.
class ROSCameraCalibration:
    def __init__(self, channel):
        rospy.Subscriber(channel, CameraInfo, self.camera_info)
        self.has_msg = False

    def camera_info(self, msg):
        self.distortion = np.matrix(msg.D)
        self.K = np.reshape(np.matrix(msg.K), (3,3))
        self.R = np.reshape(np.matrix(msg.R), (3,3))
        self.P = np.reshape(np.matrix(msg.P), (3,4))
        self.w = msg.width
        self.h = msg.height
        self.frame = msg.header.frame_id
        self.has_msg = True

    ##
    # project 3D point into this camera 
    #   
    # @param p 3x1 matrix in given coord frame
    # @param tf_listener None if transformation not needed
    # @param from_frame None is default camera frame
    # @return 2x1 matrix
    def project(self, p, tf_listener=None, from_frame=None):
        if not(from_frame == None or from_frame == self.frame):
            p_cam = tfu.transform(self.frame, from_frame, tf_listener) \
                           * tfu.tf_as_matrix((p.A1.tolist(), tr.quaternion_from_euler(0,0,0)))
            trans, q = tfu.matrix_as_tf(p_cam)
            p = np.matrix(trans).T

        p = np.row_stack((p, np.matrix([1.])))
        pp = self.P * p
        pp = pp / pp[2,0]
        return pp[0:2,0]


class GripperTipProjected:
    def __init__(self):
        forearm_cam_l = '/l_forearm_cam/image_rect_color'
        ws_l = '/wide_stereo/left/image_rect_color'
        ws_r = '/wide_stereo/right/image_rect_color'
        ws_linf = '/wide_stereo/left/camera_info'
        ws_rinf = '/wide_stereo/right/camera_info'

        self.finger_tips = ['r_gripper_l_finger_tip_link',
                       'r_gripper_r_finger_tip_link',
                       'l_gripper_l_finger_tip_link',
                       'l_gripper_r_finger_tip_link']
        
        self.camera_fr = ['r_forearm_cam_optical_frame', 
                          'l_forearm_cam_optical_frame', 
                          'wide_stereo_optical_frame']

        rospy.init_node('gripper_pose_viewer')
        #self.camera_geo = ROSCameraCalibration('/wide_stereo/left/camera_info')
        self.camera_geo = ROSCameraCalibration('/l_forearm_cam/camera_info')
        self.camera = cam.ROSImageClient(forearm_cam_l)
        self.tflistener = tf.TransformListener()


    def run(self):
        cv.NamedWindow('surf', 1)
        while not rospy.is_shutdown():
            image = self.camera.get_frame()
            image_gray = fea.grayscale(image)
            surf_keypoints, surf_descriptors = fea.surf(image_gray)
            vis_img = fea.draw_surf(image, surf_keypoints, (255, 0, 0))
        
            #Project the tip of the gripper (both of them) into the image frame
            img_ll = self.camera_geo.project(np.matrix([0,0,0.]).T, self.tflistener, self.finger_tips[2])
            img_lr = self.camera_geo.project(np.matrix([0,0,0.]).T, self.tflistener, self.finger_tips[3])

            cv.Circle(vis_img, tuple(np.matrix(np.round(img_ll), dtype='int').A1.tolist()), 30, (0, 255, 0), 1, cv.CV_AA)
            cv.Circle(vis_img, tuple(np.matrix(np.round(img_lr), dtype='int').A1.tolist()), 30, (0, 255, 0), 1, cv.CV_AA)
            cv.ShowImage('surf', vis_img)
            cv.WaitKey(10)
            

if __name__ == '__main__':
    g = GripperTipProjected()
    g.run()




#fname = sys.argv[1]
#bridge = CvBridge()
#ws_leftinfo = ROSCameraCalibration(ws_linf)
#ws_rightinfo = ROSCameraCalibration(ws_rinf)



