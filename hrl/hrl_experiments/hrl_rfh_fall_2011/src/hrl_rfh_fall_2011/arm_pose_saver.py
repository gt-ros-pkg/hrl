#! /usr/bin/python

import sys
import cPickle as pickle

import roslib
roslib.load_manifest("hrl_pr2_arms")
roslib.load_manifest("hrl_rfh_fall_2011")

import rospy
import rosbag
import tf

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from geometry_msgs.msg import PoseArray
from hrl_rfh_fall_2011.msg import NamedPoses

def pose_saver(tf_frame, bag_file, pickle_file):
    arm = create_pr2_arm('l', PR2ArmJTransposeTask)
    tf_list = tf.TransformListener()
    named_poses = NamedPoses()
    named_poses.pose_arr.header.frame_id = tf_frame
    pose_dict = {}
    while True:
        if rospy.is_shutdown():
            return
        pose_name = raw_input("Type the name of the pose and hit enter to save it (Type 'q' to quit) ")
        if pose_name == 'q':
            break
        arm_pose = PoseConverter.to_pose_stamped_msg("/torso_lift_link", arm.get_end_effector_pose())
        tf_pose = tf_list.transformPose(tf_frame, arm_pose)
        named_poses.pose_arr.poses.append(tf_pose)
        named_poses.names.append(pose_name)
        pose_dict[pose_name] = tf_pose
    
    if bag_file != "":
        bag = rosbag.Bag(bag_file, 'w')
        bag.write("/named_poses", named_poses)
        bag.close()

    if pickle_file != "":
        f = file(pickle_file, 'w')
        pickle.dump(pose_dict, f)
        f.close()

def pose_printer(bag_file, pickle_file):
    if bag_file != "":
        bag = rosbag.Bag(bag_file, 'r')
        for topic, message, stamp in bag.read_messages():
            print message
    if pickle_file != "":
        f = file(pickle_file, 'r')
        print pickle.load(f)
        f.close()
        

def main():
    rospy.init_node("arm_pose_saver", sys.argv)

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-b', '--bag_file', dest="bag_file", default="",
                 help="Bag file to save poses in.")
    p.add_option('-f', '--tf_frame', dest='tf_frame', default='/torso_lift_link',
                 help="TF frame to transform the poses into.")
    p.add_option('-p', '--pickle', dest="pickle", default="",
                 help="Name of the pickle to save to.")
    p.add_option('-s', '--save_poses', dest="save_poses",
                 action="store_true", default=False,
                 help="Saving poses mode.")
    p.add_option('-o', '--print_poses', dest="print_poses",
                 action="store_true", default=False,
                 help="Print the poses from the given file.")
    (opts, args) = p.parse_args()

    if opts.save_poses:
        pose_saver(opts.tf_frame, opts.bag_file, opts.pickle)

    if opts.print_poses:
        pose_printer(opts.bag_file, opts.pickle)

if __name__ == "__main__":
    main()
