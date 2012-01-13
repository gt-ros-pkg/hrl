#!/usr/bin/env python

import numpy as np

import roslib
roslib.load_manifest("kdl_parser_python")

# this is an odd little trick which puts our libraries ahead of all the others
# so that we get the new version of PyKDL instead of the old one
import sys
kpp_paths, other_paths = [], []
for path in sys.path:
    if "kdl_parser_python" in path:
        kpp_paths.append(path)
    else:
        other_paths.append(path)
sys.path = kpp_paths + other_paths

import urdf_parser_python.urdf_parser as urdf
import PyKDL as kdl

def to_kdl_vector(v):
    return kdl.Vector(v.x, v.y, v.z)

def to_kdl_rotation(r):
    return kdl.Rotation.Quaternion(r.x, r.y, r.z, r.w)

def to_kdl_frame(p):
    return kdl.Frame(to_kdl_rotation(p.orientation), to_kdl_vector(p.position))

def to_kdl_joint(jnt):
    f_parent_jnt = to_kdl_frame(jnt.parent_to_joint_origin_transform)
    if jnt.type == urdf.Joint.FIXED:
        return kdl.Joint(jnt.name, kdl.Joint.None)
    elif jnt.type == urdf.Joint.REVOLUTE:
        axis = to_kdl_vector(jnt.axis)
        return kdl.Joint(jnt.name, f_parent_jnt.p, f_parent_jnt.M * axis, kdl.Joint.RotAxis)
    elif jnt.type == urdf.Joint.CONTINUOUS:
        axis = to_kdl_vector(jnt.axis)
        return kdl.Joint(jnt.name, f_parent_jnt.p, f_parent_jnt.M * axis, kdl.Joint.RotAxis)
    elif jnt.type == urdf.Joint.PRISMATIC:
        axis = to_kdl_vector(jnt.axis)
        return kdl.Joint(jnt.name, f_parent_jnt.p, f_parent_jnt.M * axis, kdl.Joint.RotAxis)
    else:
        rospy.logwarn("Unknown joint type %s." % jnt.name)
        return kdl.Joint(jnt.name, kdl.Joint.None)

def to_kdl_rbi(i):
    origin = to_kdl_frame(i.origin)
    rbi = kdl.RigidBodyInertia(i.mass, origin.p, 
                               kdl.RotationalInertia(i.ixx, i.iyy, i.ixy, i.ixz, i.iyz))
    return rbi.rot_mult(origin.M)

def add_children_to_tree(root, tree):
    children = root.child_links

    inert = kdl.RigidBodyInertia()
    if root.inertial is not None:
        inert = to_kdl_rbi(root.inertial)
    
    jnt = to_kdl_joint(root.parent_joint)

    sgm = kdl.Segment(root.name, jnt, 
                      to_kdl_frame(root.parent_joint.parent_to_joint_origin_transform), inert)

    tree.addSegment(sgm, root.parent_joint.parent_link_name)

    for child in children:
        add_children_to_tree(child, tree)

def tree_from_urdf_model(robot_model):
    tree = kdl.Tree(robot_model.root_link.name)
    for child in robot_model.root_link.child_links:
        add_children_to_tree(child, tree)
    return tree

def tree_from_file(filename):
    robot_model = urdf.create_model_from_file(filename)
    return tree_from_urdf_model(robot_model)

def tree_from_param(param="/robot_description"):
    robot_model = urdf.create_model_from_param(param)
    return tree_from_urdf_model(robot_model)

def chain_from_file(base_link, end_link, filename):
    robot_model = urdf.create_model_from_file(filename)
    return chain_from_urdf_model(robot_model, base_link, end_link)

def chain_from_param(base_link, end_link, param="/robot_description"):
    robot_model = urdf.create_model_from_param(param)
    return chain_from_urdf_model(robot_model, base_link, end_link)

def chain_from_urdf_model(robot_model, base_link, end_link):
    tree = tree_from_urdf_model(robot_model)
    chain = tree.getChain(base_link, end_link)
    joint_info = joint_info_from_model(chain, robot_model)
    return chain, joint_info

def joint_info_from_model(chain, model):
    joint_info = {"names" : [], 
                  "types" : [],
                  "lim_mins" : [], 
                  "lim_maxs" : [], 
                  "safe_mins" : [], 
                  "safe_maxs" : []}
    for i in range(chain.getNrOfSegments()):
        seg = chain.getSegment(i)
        joint = seg.getJoint()
        if joint.getType() != joint.None:
            name = joint.getName()
            joint_info["names"].append(name)
            mdl_jnt = model.joints[name]
            joint_info["types"].append(mdl_jnt.type)
            if mdl_jnt.type == urdf.Joint.CONTINUOUS:
                joint_info["lim_mins"].append(-np.inf)
                joint_info["lim_maxs"].append(np.inf)
                joint_info["safe_mins"].append(-np.inf)
                joint_info["safe_maxs"].append(np.inf)
            else:
                joint_info["lim_mins"].append(mdl_jnt.limits.lower)
                joint_info["lim_maxs"].append(mdl_jnt.limits.upper)
                joint_info["safe_mins"].append(mdl_jnt.safety.soft_lower_limit)
                joint_info["safe_maxs"].append(mdl_jnt.safety.soft_upper_limit)
    return joint_info
