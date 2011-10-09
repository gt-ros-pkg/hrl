#! /usr/bin/python

import sys
import numpy as np
import scipy.io

import roslib
roslib.load_manifest('hrl_phri_2011')
import rospy

import rosbag

def main():

    field_map = {
        "time_offset" : "time_offset",
        "tool_frame.transform.translation.x" : "pos_x",
        "tool_frame.transform.translation.y" : "pos_y",
        "tool_frame.transform.translation.z" : "pos_z",
        "tool_frame.transform.rotation.x" : "rot_x",
        "tool_frame.transform.rotation.y" : "rot_y",
        "tool_frame.transform.rotation.z" : "rot_z",
        "tool_frame.transform.rotation.w" : "rot_w",
        "pc_pt.x" : "pc_pt_x",
        "pc_pt.y" : "pc_pt_y",
        "pc_pt.z" : "pc_pt_z",
        "pc_normal.x" : "pc_norm_x",
        "pc_normal.y" : "pc_norm_y",
        "pc_normal.z" : "pc_norm_z",
        "pc_dist" : "pc_dist",
        "wrench.force.x" : "force_x",
        "wrench.force.y" : "force_y",
        "wrench.force.z" : "force_z",
        "wrench.torque.x" : "torque_x",
        "wrench.torque.y" : "torque_y",
        "wrench.torque.z" : "torque_z",
        "force_magnitude" : "force_mag",
        "force_normal" : "force_norm",
        "force_tangental" : "force_tan",
        "contact_period" : "contact_period",
        "time_from_contact_start" : "time_contact",
        "ell_coords.x" : "lat",
        "ell_coords.y" : "long",
        "ell_coords.z" : "height",
    }
    data = {}
    for field in field_map:
        data[field_map[field]] = []

    bag = rosbag.Bag(sys.argv[1], 'r')
    for topic, fp, t in bag.read_messages(topics=["/force_processed"]):
        for field in field_map:
            exec("data['%s'].append(fp.%s)" % (field_map[field], field))
    bag.close()

    output_file = sys.argv[1].split(".")[0] + ".mat"
    scipy.io.savemat(output_file, data)
    print "Saved mat file to:", output_file

if __name__ == "__main__":
    main()
