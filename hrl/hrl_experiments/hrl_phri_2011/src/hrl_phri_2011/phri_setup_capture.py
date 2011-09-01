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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from msg import PHRIPointCloudCapture

class PCSaver:
    def __init__(self, pc_topic, base_frame, frames, filename):
        self.base_frame = base_frame
        self.frames = frames
        self.seq = 0
        self.i = 0
        self.pcc = None
        self.bag = rosbag.Bag(filename, 'w')
        self.tf_list = tf.TransformListener()
        rospy.sleep(1.)
        rospy.Subscriber(pc_topic, PointCloud2, self.record_data)

    def record_data(self, msg):
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

    def save_data(self):
        if self.pcc is not None:
            self.bag.write("/point_cloud_captures", self.pcc)
            self.seq += 1
            print "%2d Saved PC and frames" % self.seq
        else:
            print "Invalid capture, no PC saved"

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

    pcs = PCSaver(pc_topic, base_frame, frames, opts.bagname)
    if opts.rate != 0:
        r = rospy.Rate(opts.rate)
    while not rospy.is_shutdown():
        if opts.rate == 0:
            if raw_input("Type 'd' when done: ") == 'd':
                break
        else:
            r.sleep()
        pcs.save_data()
    pcs.bag.close()


if __name__ == "__main__":
    main()
