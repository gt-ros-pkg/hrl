#! /usr/bin/python

import copy
import pygame
import pygame.image 
from vec2d import *

import numpy as np

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('rosparam')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('camera_calibration')
roslib.load_manifest('pixel_2_3d')
roslib.load_manifest('hrl_generic_arms')
roslib.load_manifest('opencv2')

import cv

import rospy
import rosparam
import rosbag
import tf
from sensor_msgs.msg import PointCloud2
from hrl_generic_arms.pose_converter import PoseConverter
from camera_calibration.calibrator import ChessboardInfo, Calibrator
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from msg import PHRIPointCloudCapture

DEBUG = True

class PCSaver:
    def __init__(self, pc_topic, img_topic, base_frame, frames, filename):
        self.base_frame = base_frame
        self.frames = frames
        self.seq = 0
        self.i = 0
        self.pcc = None
        self.bag = rosbag.Bag(filename, 'w')
        self.tf_list = tf.TransformListener()
        self.img_list = ImageListener(img_topic)
        rospy.sleep(1.)
        rospy.Subscriber(pc_topic, PointCloud2, self.record_data)

    def record_data(self, msg):
        if self.i == 0:
            print "Got first PC"
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
                if not DEBUG:
                    print e
                    print "%d, Missing frame: %s" % (self.i, frame)
                pcc = None
        self.pcc = pcc

    def save_data(self, register=False):
        if DEBUG:
            self.pcc = PHRIPointCloudCapture()
        if self.pcc is not None:
            self.bag.write("/point_cloud_captures", self.pcc)
            self.seq += 1
            print "%2d Saved PC and frames" % self.seq
            if register:
                self.get_registration()
            return True
        else:
            print "Invalid capture, no PC saved"
            return False

    def get_registration(self):
        forehead = Vec2d(320, 150)
        chin = Vec2d(320, 340)
        l_eye = Vec2d(240, 200)
        r_eye = Vec2d(390, 200)
        r_mouth = Vec2d(350, 285)
        l_mouth = Vec2d(290, 285)
        control_points = [forehead, chin, l_eye, r_eye, r_mouth, l_mouth]

        pg_img = self.img_list.get_pg_img()
        pygame.init()
        screen = pygame.display.set_mode((640, 480))
        clock = pygame.time.Clock()
        selected = None
        running = True
        while not rospy.is_shutdown() and running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.dict['key'] == pygame.K_SPACE:
                        running = False
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    for p in control_points:
                        if abs(p.x - event.pos[0]) < 10 and abs(p.y - event.pos[1]) < 10 :
                            selected = p
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
                    x,y = pygame.mouse.get_pos()
                    control_points.append(Vec2d(x,y))
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    selected = None

            screen.blit(pg_img, (0, 0))

            pygame.draw.lines(screen, (255, 255, 0), False, [forehead, chin])
            pygame.draw.lines(screen, (255, 255, 0), False, [l_eye, r_eye, r_mouth, l_mouth, l_eye])

            if selected is not None:
                selected.x, selected.y = pygame.mouse.get_pos()
                pygame.draw.circle(screen, (0, 255, 0), selected, 10)

            for p in control_points:
                pygame.draw.circle(screen, (0, 0, 255), p, 4)

            pygame.display.flip()
        pygame.quit()
        print "Selected points:", control_points



class ImageListener:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.img_msg = None
        self.got_first_img = False

    def callback(self, data):
        if not self.got_first_img:
            self.got_first_img = True
            print "Got first image"
        self.img_msg = data

    def get_pg_img(self):
        try:
            cv_img = self.bridge.imgmsg_to_cv(self.img_msg, "rgb8")
        except CvBridgeError, e:
            print e
            return
        return pygame.image.frombuffer(cv_img.tostring(), 
                                       cv.GetSize(cv_img), "RGB")

def main():
    rospy.init_node('phri_setup_capture')

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-l', '--load_file', dest="load_file", default="",
                 help="YAML file to load parameters from.")
    p.add_option('-b', '--bagfile', dest="bagname", default="phri_output.bag",
                 help="Bag file to save to.")
    p.add_option('-r', '--rate', dest="rate", default="0", type="int",
                 help="""Rate in hz to capture at. If not specified or 0, 
                         will only capture when pressing enter.""")
    opts, args = p.parse_args()
    assert opts.load_file != ""

    params = rosparam.load_file(opts.load_file, "phri_setup_capture")[0][0]
    base_frame = params['base_frame']
    frames = params['frames_to_save']
    pc_topic = params['pc_topic']
    img_topic = params['img_topic']

    pcs = PCSaver(pc_topic, img_topic, base_frame, frames, opts.bagname)
    if opts.rate != 0:
        r = rospy.Rate(opts.rate)
    first = True
    while not rospy.is_shutdown():
        if opts.rate == 0:
            if raw_input("Type 'd' when done: ") == 'd':
                break
        else:
            r.sleep()
        pcs.save_data(first)
        first = False
    pcs.bag.close()


if __name__ == "__main__":
    main()
