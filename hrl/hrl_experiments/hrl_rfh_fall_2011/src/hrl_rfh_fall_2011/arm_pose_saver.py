#! /usr/bin/python

import sys
import cPickle as pickle

import roslib
roslib.load_manifest("hrl_pr2_arms")

import rospy
import rosbag
import tf

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from geometry_msgs.msg import PoseArray
from hrl_rfh_fall_2011.msg import NamedPoses

def pose_saver(bag_file, pickle_file):
    arm = create_pr2_arm('l', PR2ArmJTransposeTask, end_link="%s_gripper_shaver45_frame")
    tf_list = tf.TransformListener()
    named_poses = NamedPoses()
    named_poses.pose_arr.header.frame_id = '/torso_lift_link'
    pose_dict = {}
    while True:
        if rospy.is_shutdown():
            return
        pose_name = raw_input("Type the name of the pose and hit enter to save it (Type 'q' to quit) ")
        if pose_name == 'q':
            break
        tf_pose = tf_list.transformPose("/ellipse_frame", 
                                        PoseConverter.to_pose_stamped_msg("/torso_lift_link", arm.get_end_effector_pose()))
        named_poses.pose_arr.poses.append(tf_pose)
        named_poses.names.append(pose_name)
        pose_dict[pose_name] = tf_pose
    
    bag = rosbag.Bag(bag_file, 'w')
    bag.write("/named_poses", named_poses)
    bag.close()

    if pickle_file != "":
        f = file(pickle_file, 'w')
        pickle.dump(pose_dict, f)
        f.close()

def main():
    rospy.init_node("arm_pose_saver", sys.argv)

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-b', '--bag_file', dest="bag_file", default="",
                 help="Bag file to save poses in.")
    p.add_option('-p', '--pickle', dest="pickle", default="",
                 help="Name of the pickle to save to.")
    p.add_option('-s', '--save_poses', dest="save_poses",
                 action="store_true", default=False,
                 help="Saving poses mode.")
    (opts, args) = p.parse_args()

    if opts.save_poses:
        pose_saver(opts.bag_file, opts.pickle)

if __name__ == "__main__":
    main()
