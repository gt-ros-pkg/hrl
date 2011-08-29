#! /usr/bin/python

import copy
import pygame
import numpy as np

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('rosparam')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('camera_calibration')
roslib.load_manifest('pixel_2_3d')
roslib.load_manifest('hrl_generic_arms')

import cv

import rospy
import rosparam
import rosbag
import tf
from sensor_msgs.msg import PointCloud2
from hrl_generic_arms.pose_converter import PoseConverter
from camera_calibration.calibrator import ChessboardInfo, Calibrator
from pixel_2_3d.srv import Pixel23d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from msg import PHRIPointCloudCapture

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

class PCSaver:
    def __init__(self, pc_topic, base_frame, frames, filename):
        self.base_frame = base_frame
        self.frames = frames
        self.seq = 0
        self.i = 0
        self.pcc = None
        self.bag = rosbag.Bag(filename, 'w')
        self.tf_list = tf.TransformListener()
        self.img_list = ImageListener("/kinect_head/rgb/image_color")
        self.pix3d_srv = rospy.ServiceProxy("/pixel_2_3d", Pixel23d)
        rospy.sleep(1.)
        rospy.Subscriber(pc_topic, PointCloud2, self.store_pc)

    def store_pc(self, msg):
        self.i += 1

        pcc = PHRIPointCloudCapture()
        pcc.header.seq = self.seq
        pcc.header.frame_id = self.base_frame
        pcc.header.stamp = rospy.Time.now()
        pcc.pc_capture = msg
        pcc.frame_names = self.frames

        for frame in self.frames:
            try:
                pos, quat = self.tf_list.lookupTransform(self.base_frame,  
                                                         frame, rospy.Time(0))
                pose_stmpd = PoseConverter.to_pose_stamped_msg(pos, quat)
                pose_stmpd.header = copy.deepcopy(pcc.header)
                pcc.saved_frames.append(pose_stmpd)
            except Exception,e:
                print e
                print "%d, Missing frame: %s" % (self.i, frame)
                pcc = None
        self.pcc = pcc

    def save_pc(self):
        if self.pcc is not None:
            self.bag.write("/point_cloud_captures", self.pcc)
            self.seq += 1
            print "Saved PC and frames"
        else:
            print "Invalid capture, no PC saved"

    def collection_loop(self):
        chessboard = ChessboardInfo()
        chessboard.n_cols = 6
        chessboard.n_rows = 7
        chessboard.dim = 0.02273
        calib = Calibrator([chessboard])
        cboard_frame = "kinect_cb_corner"
        pygame.init()
        clock = pygame.time.Clock()
        screen = pygame.display.set_mode((640, 480))

        while not rospy.is_shutdown():
            try:
                cb_pos, cb_quat = self.tf_list.lookupTransform('/optitrak', 
                                                               cboard_frame, 
                                                               rospy.Time())
                ot_B_cb = PoseConverter.to_homo_mat(cb_pos, cb_quat)
            except Exception, e:
                print e
                rospy.sleep(0.001)
                continue
            cv_img = self.img_list.get_cv_img()
            if cv_img is not None:
                has_corners, corners, chess = calib.get_corners(cv_img)
                if has_corners:
                    corner_a, corner_b, corner_c = corners[0], corners[6*6], corners[5]
                    capture, done = False, False
                    for event in pygame.event.get():
                        if event.type == pygame.KEYDOWN:
                            if event.dict['key'] == pygame.K_SPACE:
                                capture = True
                            if event.dict['key'] == pygame.K_d:
                                done = True
                            if event.dict['key'] == pygame.K_q:
                                return
                    if done:
                        break
                    cv.Circle(cv_img, corner_a, 4, [0, 0, 255])
                    cv.Circle(cv_img, corner_b, 4, [255, 0, 0])
                    cv.Circle(cv_img, corner_c, 4, [0, 255, 0])
                    pix3d_a = self.pix3d_srv(*corner_a)
                    pix3d_b = self.pix3d_srv(*corner_b)
                    pix3d_c = self.pix3d_srv(*corner_c)
                    if (pix3d_a.error_flag == pix3d_a.SUCCESS and
                        pix3d_b.error_flag == pix3d_b.SUCCESS and
                        pix3d_c.error_flag == pix3d_c.SUCCESS):
                        corner_3d_a = (pix3d_a.pixel3d.pose.position.x,
                                       pix3d_a.pixel3d.pose.position.y,
                                       pix3d_a.pixel3d.pose.position.z)
                        corner_3d_b = (pix3d_b.pixel3d.pose.position.x,
                                       pix3d_b.pixel3d.pose.position.y,
                                       pix3d_b.pixel3d.pose.position.z)
                        corner_3d_c = (pix3d_c.pixel3d.pose.position.x,
                                       pix3d_c.pixel3d.pose.position.y,
                                       pix3d_c.pixel3d.pose.position.z)
                        axis_x = np.array(corner_3d_b) - np.array(corner_3d_a)
                        axis_y1 = np.array(corner_3d_c) - np.array(corner_3d_a)
                        axis_z = np.cross(axis_x, axis_y1)
                        axis_x /= np.linalg.norm(axis_x)
                        axis_z /= np.linalg.norm(axis_z)
                        axis_y2 = np.cross(axis_z, axis_x)
                        R = np.mat([axis_x, axis_y2, axis_z]).T
                        kinopt_B_cb = PoseConverter.to_homo_mat(np.mat(corner_3d_a).T, R)
                        ot_B_kinopt = ot_B_cb * kinopt_B_cb ** -1
                        if capture:
                            self.save_pc(PoseConverter.to_pose_msg(ot_B_kinopt))
                    else:
                        cv.Circle(cv_img, corner_a, 4, [255, 255, 0])
            if cv_img is None:
                screen.fill(gray)
            else:
                screen.blit(self.img_list.get_pg_img(cv_img), (0, 0))
            pygame.display.flip()
            rospy.sleep(0.001)
        self.bag.close()

def main():
    rospy.init_node('phri_setup_capture')

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-l', '--load_file', dest="load_file", default="",
                 help="YAML file to load parameters from.")
    p.add_option('-b', '--bagfile', dest="bagname", default="phri_output.bag",
                 help="Bag file to save to.")
    opts, args = p.parse_args()
    assert opts.load_file != ""

    params = rosparam.load_file(opts.load_file, "phri_setup_capture")[0][0]
    base_frame = params['base_frame']
    frames = params['frames_to_save']
    pc_topic = params['pc_topic']

    pcs = PCSaver(pc_topic, base_frame, frames, opts.bagname)
#pcs.collection_loop()
    while not rospy.is_shutdown():
        if raw_input("Type 'd' when done: ") == 'd':
            break
        pcs.save_pc()
    pcs.bag.close()


if __name__ == "__main__":
    main()
