#! /usr/bin/python

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('rosparam')
roslib.load_manifest('sensor_msgs')

import rospy
import rosparam
import rosbag
from sensor_msgs.msg import PointCloud2
from hrl_generic_arms.pose_converter import PoseConverter

from kelsey_sandbox.msg import PHRIPointCloudCapture

class PCSaver:
    def __init__(self, pc_topic, base_frame, frames, filename):
        self.base_frame = base_frame
        self.frames = frames
        self.seq = 0
        self.bag = rosbag.Bag(filename, 'w')
        rospy.Subscriber(pc_topic, PointCloud2, self.store_pc)

    def store_pc(self, msg):
        pcc = PHRIPointCloudCapture()
        pcc.pc_capture = msg
        pcc.saved_frames.header.seq = self.seq
        pcc.saved_frames.header.frame_id = self.base_frame
        pcc.saved_frames.header.stamp = rospy.Time.now()
        for frame in self.frames:
            try:
                pos, quat = self.tf_list.lookupTransform(self.base_frame,  
                                                         frame, rospy.Time(0))
                pcc.saved_frames.poses.append(PoseConverter.to_pose_msg(pos, quat))
            except:
                print "FAILED TO CAPTURE POINTCLOUD, MISSING FRAME: %s" % frame
                return
        self.pcc = pcc

    def save_pc(self):
        self.bag.write("/point_cloud_captures", self.pcc)
        self.seq += 1

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
    while not rospy.is_shutdown():
        raw_input("Type 'd' when done: ")
        pcs.save_pc()
    pcs.bag.close()


if __name__ == "__main__":
    main()
