#!/usr/bin/python

import copy
import pygame
import pygame.image 
import numpy as np

import roslib
roslib.load_manifest('UI_segment_object')
roslib.load_manifest('camera_calibration')
roslib.load_manifest('pixel_2_3d')
roslib.load_manifest('hrl_generic_arms')

import cv

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf

from camera_calibration.calibrator import ChessboardInfo, Calibrator
from pixel_2_3d.srv import Pixel23d
from hrl_generic_arms.pose_converter import PoseConverter

class ImageListener:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.msg_img = None

    def callback(self, data):
        try:
            self.msg_img = data
        except CvBridgeError, e:
            print e
            return

    def get_cv_img(self):
        if self.msg_img is None:
            return None
        cv_img = self.bridge.imgmsg_to_cv(self.msg_img, "rgb8")
        return cv_img

    def get_pg_img(self, cv_img):
        return pygame.image.frombuffer(cv_img.tostring(), 
                                       cv.GetSize(cv_img), "RGB")
    
def umeyama_method(A, B):
    ux = np.mean(A, 1)
    uy = np.mean(B, 1)
    A_demean = A - ux
    B_demean = B - uy
    U, D, VT = np.linalg.svd(A_demean * B_demean.T)
    R = VT.T * U.T
    t = uy - R * ux
    return t, R

def ransac(A, B, err_thresh, percent_set_train=0.5, percent_set_fit=0.5, iterations=200):
    best_model = None
    best_consen_set = None
    best_error = 1000000.0
    n = int(percent_set_train * A.shape[1])
    min_num = int(percent_set_fit * A.shape[1])
    for i in range(iterations):
        shuff_inds = np.arange(A.shape[1])
        np.random.shuffle(shuff_inds)
        maybe_inds = shuff_inds[:n]
        maybe_A = A[:, maybe_inds]
        maybe_B = B[:, maybe_inds]
        t_maybe, R_maybe = umeyama_method(maybe_A, maybe_B)
        err = (R_maybe * A + t_maybe - B).A
        err_mag = np.sqrt(err[0,:] ** 2 + err[1,:] ** 2 + err[2,:] ** 2)
        if np.sum(err_mag < err_thresh) < min_num:
            print "Not enough points within thresh"
            continue
        consensus_A = A[:, err_mag < err_thresh]
        consensus_B = B[:, err_mag < err_thresh]
        t_cons, R_cons = umeyama_method(consensus_A, consensus_B)
        err_cons = (R_cons * consensus_A + t_cons - consensus_B).A
        err_mag_cons = np.sqrt(err_cons[0,:] ** 2 + err_cons[1,:] ** 2 + err_cons[2,:] ** 2)
        if np.mean(err_mag_cons) < best_error:
            best_error = np.mean(err_mag_cons)
            best_model = (t_cons, R_cons)
        print "Best error:", best_error
        print "Consensus Size:", consensus_A.shape[1]
    return best_model
        
def main():
    gray = (100,100,100)
    corner_len = 5
    chessboard = ChessboardInfo()
    chessboard.n_cols = 6
    chessboard.n_rows = 7
    chessboard.dim = 0.02273
    cboard_frame = "kinect_cb_corner"
#kinect_tracker_frame = "kinect"
#TODO
    use_pygame = False
    kinect_tracker_frame = "pr2_antenna"

    rospy.init_node("kinect_calib_test")
    img_list = ImageListener("/kinect_head/rgb/image_color")
    pix3d_srv = rospy.ServiceProxy("/pixel_2_3d", Pixel23d, True)
    tf_list = tf.TransformListener()
    if use_pygame:
        pygame.init()
        clock = pygame.time.Clock()
        screen = pygame.display.set_mode((640, 480))
    calib = Calibrator([chessboard])
    done = False
    corner_list = np.ones((2, corner_len)) * -1000.0
    corner_i = 0
    saved_corners_2d, saved_corners_3d, cb_locs = [], [], []
    while not rospy.is_shutdown():
        try:
            cb_pos, cb_quat = tf_list.lookupTransform(kinect_tracker_frame, 
                                                      cboard_frame, 
                                                      rospy.Time())
        except:
            rospy.sleep(0.001)
            continue
        cv_img = img_list.get_cv_img()
        if cv_img is not None:
            has_corners, corners, chess = calib.get_corners(cv_img)
            for corner2d in saved_corners_2d:
                cv.Circle(cv_img, corner2d, 4, [0, 255, 255])
            if has_corners:
                corner_i += 1
                corner = corners[0]
                if use_pygame:
                    for event in pygame.event.get():
                        if event.type == pygame.KEYDOWN:
                            print event.dict['key'], pygame.K_d
                            if event.dict['key'] == pygame.K_d:
                                done = True
                            if event.dict['key'] == pygame.K_q:
                                return
                    if done:
                        break
                corner_list[:, corner_i % corner_len] = corner
                if np.linalg.norm(np.var(corner_list, 1)) < 1.0:
                    corner_avg = np.mean(corner_list, 1)
                    corner_avg_tuple = tuple(corner_avg.round().astype(int).tolist())
                    cv.Circle(cv_img, corner_avg_tuple, 4, [0, 255, 0])
                    pix3d_resp = pix3d_srv(*corner_avg_tuple)
                    if pix3d_resp.error_flag == pix3d_resp.SUCCESS:
                        corner_3d_tuple = (pix3d_resp.pixel3d.pose.position.x,
                                           pix3d_resp.pixel3d.pose.position.y,
                                           pix3d_resp.pixel3d.pose.position.z)
                        if len(saved_corners_3d) == 0:
                            cb_locs.append(cb_pos)
                            saved_corners_2d.append(corner_avg_tuple)
                            saved_corners_3d.append(corner_3d_tuple)
                        else:
                            diff_arr = np.array(np.mat(saved_corners_3d) - np.mat(corner_3d_tuple))
                            if np.min(np.sqrt(np.sum(diff_arr ** 2, 1))) >= 0.03:
                                cb_locs.append(cb_pos)
                                saved_corners_2d.append(corner_avg_tuple)
                                saved_corners_3d.append(corner_3d_tuple)
                                print "Added sample", len(saved_corners_2d) - 1
                else:
                    cv.Circle(cv_img, corner, 4, [255, 0, 0])
            else:
                corner_list = np.ones((2, corner_len)) * -1000.0
        if use_pygame:
            if cv_img is None:
                screen.fill(gray)
            else:
                screen.blit(img_list.get_pg_img(cv_img), (0, 0))
            pygame.display.flip()
        rospy.sleep(0.001)
    A = np.mat(saved_corners_3d).T
    B = np.mat(cb_locs).T
    print A, B
    t, R = umeyama_method(A, B)
    print A, B, R, t
    print "-" * 60
    print "Transformation Parameters:"
    pos, quat = PoseConverter.to_pos_quat(t, R)
    print '%f %f %f %f %f %f %f' % tuple(pos + quat)
    t_r, R_r = ransac(A, B, 0.02, percent_set_train=0.5, percent_set_fit=0.6)
    print t_r, R_r
    pos, quat = PoseConverter.to_pos_quat(t_r, R_r)
    print '%f %f %f %f %f %f %f' % tuple(pos + quat)
    
if __name__ == '__main__':
    main()
