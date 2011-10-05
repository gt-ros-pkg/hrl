#! /usr/bin/python

import sys
import numpy as np
import scipy.io

import roslib
roslib.load_manifest('hrl_phri_2011')
import rospy

import rosbag

def main():
    bag = rosbag.Bag(sys.argv[1], 'r')
    fixed_file = sys.argv[1].split(".")[0] + "_fixed.bag"
    fixed_bag = rosbag.Bag(fixed_file, 'w')
    for topic, tf_msg, t in bag.read_messages():
        if topic == "/tf":
            if len(tf_msg.transforms) > 0 and tf_msg.transforms[0].child_frame_id == "/tool_netft_raw_frame":
                tf_msg.transforms[0].transform.rotation.x = 0
                tf_msg.transforms[0].transform.rotation.y = 0
                tf_msg.transforms[0].transform.rotation.z = 1
                tf_msg.transforms[0].transform.rotation.w = 0
        fixed_bag.write(topic, tf_msg, t)
    bag.close()
    fixed_bag.close()
    print "Saved fixed bag to:", fixed_file


if __name__ == "__main__":
    main()
