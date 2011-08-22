#! /usr/bin/python

import numpy as np
import sys
import cPickle as pickle

import roslib
roslib.load_manifest('kelsey_sandbox')
roslib.load_manifest('hrl_generic_arms')
import rospy
import rosparam
import tf
import tf.transformations as tf_trans
from hrl_generic_arms.pose_converter import PoseConverter
#from umeyama_method import umeyama_method

def capture_data():
    tf_list = tf.TransformListener()
    opti_mat, robot_mat = [], []
    print "Hit enter to collect a data sample"
    i = 0
    while not rospy.is_shutdown():
        rospy.sleep(0.3)
#if raw_input(".") == 'q':
#    break
        now = rospy.Time(0.)
        (opti_pos, opti_rot) = tf_list.lookupTransform("/pr2_antenna", 
                                                       "/optitrak", now)
        (robot_pos, robot_rot) = tf_list.lookupTransform("/wide_stereo_link", 
                                                         "/base_footprint", now)
        opti_mat.append(opti_pos)
        opti_rot_foot = np.mat([[-0.17496687,  0.03719102, -0.98387165],
                                [-0.98098458, -0.09183928,  0.17098186],
                                [-0.08399907,  0.99507908,  0.05255265]]).T
        opti_pos_foot = np.mat([[ 1.276  ],
                                [-0.81755],
                                [-0.06905]])
        foot_B_opti = PoseConverter.to_homo_mat(opti_pos_foot, opti_rot_foot) ** -1
        opti_B_ant = PoseConverter.to_homo_mat(opti_pos, opti_rot) ** -1
        wide_B_foot = PoseConverter.to_homo_mat(robot_pos, robot_rot) 
        print wide_B_foot * foot_B_opti * opti_B_ant
        offset_robot_pos = (wide_B_foot * foot_B_opti)[:3,3]
        print offset_robot_pos, opti_pos
        robot_mat.append(offset_robot_pos.T.A[0].tolist())
        i += 1
        if i % 10 == 0:
            print i, "samples collected"
    return opti_mat, robot_mat

def umeyama_method(A, B):
    ux = np.mean(A, 1)
    uy = np.mean(B, 1)
    A_demean = A - ux
    B_demean = B - uy
    U, D, VT = np.linalg.svd(A_demean * B_demean.T)
    R = VT.T * U.T
    t = uy - R * ux
    return t, R

def save_params(pos, rot, filename):
    rot_homo = np.eye(4)
    rot_homo[:3,:3] = rot
    rot_quat = tf_trans.quaternion_from_matrix(rot_homo)
    optitrak_params = { "position" : pos.T.A[0].tolist(),
                        "orientation" : rot_quat.tolist() }
    print optitrak_params
    rosparam.upload_params("optitrak_calibration", optitrak_params)
    rospy.sleep(0.5)
    rosparam.dump_params(filename, "optitrak_calibration")

def publish_transform():
    optitrak_params = rosparam.get_param("optitrak_calibration")
    remap_mat = PoseConverter.to_homo_mat(optitrak_params["position"], 
                                          optitrak_params["orientation"]) ** -1
    tf_list = tf.TransformListener()
    tf_broad = tf.TransformBroadcaster()
    small_dur = rospy.Duration(0.001)
    robot_mat = PoseConverter.to_homo_mat([0., 0., 0.], [0., 0., 0., 1.])
    opti_mat = PoseConverter.to_homo_mat([0., 0., 0.], [0., 0., 0., 1.])
    while not rospy.is_shutdown():
        try:
            now = rospy.Time(0.)

            (opti_pos, opti_rot) = tf_list.lookupTransform("/optitrak", 
                                                           "/pr2_antenna", now)
            opti_mat = PoseConverter.to_homo_mat(opti_pos, opti_rot)
            now = rospy.Time(0.)

            (robot_pos, robot_rot) = tf_list.lookupTransform("/wide_stereo_link", 
                                                             "/base_footprint", now)
            robot_mat = PoseConverter.to_homo_mat(robot_pos, robot_rot)

            odom_mat = opti_mat * remap_mat * robot_mat
            odom_pos, odom_rot = PoseConverter.to_pos_quat(odom_mat)
            tf_broad.sendTransform(odom_pos, odom_rot, rospy.Time.now(), "/base_footprint", "/optitrak")
            rospy.sleep(0.001)
        except Exception as e:
            print e


def main():
    rospy.init_node("optitrak_odom_remap")
    
    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-f', '--file', dest="filename", default="",
                 help="YAML file to save parameters in.")
    p.add_option('-l', '--load', dest="is_load",
                 action="store_true", default=False,
                 help="Load parameters from file.")
    p.add_option('-t', '--train', dest="is_train",
                 action="store_true", default=False,
                 help="Train by moving the head to different poses and capturing the relative positions.")
    p.add_option('-p', '--publish', dest="is_pub",
                 action="store_true", default=False,
                 help="Publish the transformation from optitrak to base_footprint.")
    (opts, args) = p.parse_args()
    if opts.filename == "":
        print "Bad command line parameters"
        p.print_help()
        return

    if opts.is_load:
        print "hi"
        params = rosparam.load_file(opts.filename, "optitrak_calibration")[0][0]
        rosparam.upload_params("optitrak_calibration", params)
        rospy.sleep(0.1)
    if opts.is_pub:
        publish_transform()
        return
    if opts.is_train:
        opti_mat, robot_mat = capture_data()
        opti_mat = np.mat(opti_mat).T
        robot_mat = np.mat(robot_mat).T
        pos, rot = umeyama_method(opti_mat, robot_mat)
        print np.linalg.norm(robot_mat - (rot * opti_mat + pos))
        save_params(pos, rot, opts.filename)
        return
        

if __name__ == "__main__":
    main()
