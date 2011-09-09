#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("kdl_parser_python")
roslib.load_manifest("hrl_kdl_arms")
import rospy
import urdf_parser_python.urdf_parser as urdf
import kdl_parser_python.kdl_parser as kdlp
import PyKDL as kdl
from hrl_kdl_arms.kdl_arm_kinematics import KDLArmKinematics

chain, joint_info = kdlp.chain_from_param("torso_lift_link", "r_gripper_tool_frame")

print "Joint Information:", joint_info

kinematics = KDLArmKinematics(chain=chain)
print kinematics.chain.getNrOfSegments()
while not rospy.is_shutdown():
    q = np.random.uniform(-0.3, 0.3, 7)
    print "FK", kinematics.FK_vanilla(q, 11)
    J = kinematics.jacobian_vanilla(q)
    print "Jacobian", J
    rospy.sleep(0.2)
