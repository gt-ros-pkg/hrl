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
        "/l_cart/f_cmd" : 
        {
            "wrench.force.x" : "f_cmd_x",
            "wrench.force.y" : "f_cmd_y",
            "wrench.force.z" : "f_cmd_z",
            "wrench.torque.x" : "t_cmd_x",
            "wrench.torque.y" : "t_cmd_y",
            "wrench.torque.z" : "t_cmd_z"
        },
        "/l_cart/f_err" :
        {
            "wrench.force.x" : "f_err_x",
            "wrench.force.y" : "f_err_y",
            "wrench.force.z" : "f_err_z",
            "wrench.torque.x" : "t_err_x",
            "wrench.torque.y" : "t_err_y",
            "wrench.torque.z" : "t_err_z"
        },
        "/l_cart/k_effective" :
        {
            "wrench.force.x" : "k_eff_x",
            "wrench.force.y" : "k_eff_y",
            "wrench.force.z" : "k_eff_z",
            "wrench.torque.x" : "kt_eff_x",
            "wrench.torque.y" : "kt_eff_y",
            "wrench.torque.z" : "kt_eff_z"
        },
        "/l_cart/sensor_ft" :
        {
            "wrench.force.x" : "f_sens_x",
            "wrench.force.y" : "f_sens_y",
            "wrench.force.z" : "f_sens_z",
            "wrench.torque.x" : "t_sens_x",
            "wrench.torque.y" : "t_sens_y",
            "wrench.torque.z" : "t_sens_z"
        },
        "/l_cart/sensor_raw_ft" :
        {
            "wrench.force.x" : "f_sens_raw_x",
            "wrench.force.y" : "f_sens_raw_y",
            "wrench.force.z" : "f_sens_raw_z",
            "wrench.torque.x" : "t_sens_raw_x",
            "wrench.torque.y" : "t_sens_raw_y",
            "wrench.torque.z" : "t_sens_raw_z"
        },
        "/l_cart/state/x" :
        {
            "pose.position.x" : "pos_x",
            "pose.position.y" : "pos_y",
            "pose.position.z" : "pos_z",
            "pose.orientation.x" : "rot_x",
            "pose.orientation.y" : "rot_y",
            "pose.orientation.z" : "rot_z",
            "pose.orientation.w" : "rot_w",
        },
        "/l_cart/state/xd" :
        {
            "linear.x" : "vel_x",
            "linear.y" : "vel_y",
            "linear.z" : "vel_z",
            "angular.x" : "avel_x",
            "angular.y" : "avel_y",
            "angular.z" : "avel_z"
        },
        "/l_cart/x_err" :
        {
            "linear.x" : "pos_err_x",
            "linear.y" : "pos_err_y",
            "linear.z" : "pos_err_z",
            "angular.x" : "rot_err_x",
            "angular.y" : "rot_err_y",
            "angular.z" : "rot_err_z"
        }
    }
    data = {}
    for topic in field_map:
        for field in field_map[topic]:
            data[field_map[topic][field]] = []

    bag = rosbag.Bag(sys.argv[1], 'r')
    for topic in field_map:
        for tp, fp, t in bag.read_messages(topics=[topic]):
            for field in field_map[topic]:
                exec("data['%s'].append(fp.%s)" % (field_map[topic][field], field))
    bag.close()

    output_file = sys.argv[1].split(".")[0] + ".mat"
    scipy.io.savemat(output_file, data)
    print "Saved mat file to:", output_file

if __name__ == "__main__":
    main()
