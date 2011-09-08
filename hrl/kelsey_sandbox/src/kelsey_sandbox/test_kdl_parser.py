#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("hrl_kdl_arms")
roslib.load_manifest("equilibrium_point_control")
roslib.load_manifest("rospy")
roslib.load_manifest("urdf_parser_python")
roslib.load_manifest("kdl_parser_python")
#roslib.load_manifest("python_orocos_kdl")
import rospy
import urdf_parser_python.urdf_parser as urdf
import kdl_parser_python.kdl_parser as kdlp
import PyKDL as kdl
from hrl_kdl_arms.kdl_arm_kinematics import KDLArmKinematics

chain = kdlp.chain_from_param("torso_lift_link", "r_gripper_tool_frame")

if True:
    kinematics = KDLArmKinematics(chain=chain)
    while not rospy.is_shutdown():
        q = np.random.uniform(-0.3, 0.3, 7)
        print kinematics.chain.getNrOfSegments()
        print kinematics.FK_vanilla(q, 11)
        H = kinematics.inertia(q)
        J = kinematics.jacobian_vanilla(q)
        print H.shape, H
        print np.linalg.inv(J * np.linalg.inv(H) * J.T)
        print np.log10(np.linalg.cond(H))
        print np.log10(np.linalg.cond(J * np.linalg.inv(H) * J.T))
        print np.log10(np.linalg.cond(kinematics.cart_inertia(q)))
        rospy.sleep(0.2)

