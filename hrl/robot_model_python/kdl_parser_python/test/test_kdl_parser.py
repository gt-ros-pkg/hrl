#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("hrl_pr2_arms")
roslib.load_manifest("rospy")
roslib.load_manifest("urdf_parser_python")
roslib.load_manifest("kdl_parser_python")
#roslib.load_manifest("python_orocos_kdl")
import rospy
import urdf_parser_python.urdf_parser as urdf
import kdl_parser_python.kdl_parser as kdlp
import PyKDL as kdl
from hrl_pr2_arms.kdl_arm_kinematics import KDLArmKinematics

def joint_list_to_kdl(q):
    if q is None:
        return None
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl

#model = urdf.create_model_from_file("/etc/ros/diamondback/urdf/robot.xml")
model = urdf.create_model_from_param()
tree = kdlp.tree_from_urdf_model(model)
chain = tree.getChain("torso_lift_link", "r_gripper_tool_frame")
#chain = tree.getChain("torso_lift_link", "r_gripper_palm_link")

if False:
    fk = kdl.ChainFkSolverPos_recursive(chain)
    while not rospy.is_shutdown():
        q = [0., 0., 0., 0., 0., 0., 0.]
        q = np.random.uniform(-0.3, 0.3, 7)
        endeffec_frame = kdl.Frame()
        fk.JntToCart(joint_list_to_kdl(q), endeffec_frame)
        p = endeffec_frame.p
        pos = np.mat([p.x(), p.y(), p.z()]).T
        M = endeffec_frame.M
        rot = np.mat([[M[0,0], M[0,1], M[0,2]], 
                      [M[1,0], M[1,1], M[1,2]], 
                      [M[2,0], M[2,1], M[2,2]]])
        print pos, rot
        rospy.sleep(0.2)

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

