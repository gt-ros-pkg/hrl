
import roslib
#roslib.load_manifest("kelsey_sandbox")
roslib.load_manifest("python_orocos_kdl")
import urdf_interface as urdf
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
        kdl_jnt = kdl.Joint(kdl.Joint.None)
        kdl_jnt.name = jnt.name
        return kdl_jnt
    elif jnt.type == urdf.Joint.REVOLUTE:
        axis = to_kdl_vector(jnt.axis)
        kdl_jnt = kdl.Joint(kdl.Joint.None)
        kdl_jnt.type = kdl.Joint.RotAxis
        kdl_jnt.name = jnt.name
        kdl_jnt.origin = f_parent_jnt.p
        kdl_jnt.axis = f_parent_jnt.M * axis
        kdl_jnt.axis = kdl_jnt.axis / kdl_jnt.axis.Norm()
        #kdl_jnt.joint_pose.p = kdl_jnt.origin
        #kdl_jnt.joint_pose.M = kdl.Rotation.Rot2(kdl_jnt.axis, 0)
        return kdl_jnt
    elif jnt.type == urdf.Joint.CONTINUOUS:
        axis = to_kdl_vector(jnt.axis)
        kdl_jnt = kdl.Joint(kdl.Joint.None)
        kdl_jnt.type = kdl.Joint.RotAxis
        kdl_jnt.name = jnt.name
        kdl_jnt.origin = f_parent_jnt.p
        kdl_jnt.axis = f_parent_jnt.M * axis
        kdl_jnt.axis = kdl_jnt.axis / kdl_jnt.axis.Norm()
        #kdl_jnt.joint_pose.p = kdl_jnt.origin
        #kdl_jnt.joint_pose.M = kdl.Rotation.Rot2(kdl_jnt.axis, 0)
        return kdl_jnt
    elif jnt.type == urdf.Joint.PRISMATIC:
        axis = to_kdl_vector(jnt.axis)
        kdl_jnt = kdl.Joint(kdl.Joint.None)
        kdl_jnt.type = kdl.Joint.TransAxis
        kdl_jnt.name = jnt.name
        kdl_jnt.origin = f_parent_jnt.p
        kdl_jnt.axis = f_parent_jnt.M * axis
        kdl_jnt.axis = kdl_jnt.axis / kdl_jnt.axis.Norm()
        #kdl_jnt.joint_pose.p = kdl_jnt.origin
        #kdl_jnt.joint_pose.M = kdl.Rotation.Rot2(kdl_jnt.axis, 0)
        return kdl_jnt
    else:
        rospy.logwarn("Unknown joint type %s." % jnt.name)
        kdl_jnt = kdl.Joint(kdl.Joint.None)
        kdl_jnt.name = jnt.name
        return kdl_jnt

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

    sgm = kdl.Segment(jnt, to_kdl_frame(root.parent_joint.parent_to_joint_origin_transform))
    sgm.name = root.name
    sgm.I = inert

    tree.addSegment(sgm, root.parent_joint.parent_link_name)

    for child in children:
        add_children_to_tree(child, tree)

def tree_from_urdf_model(robot_model):
    tree = kdl.Tree(robot_model.root_link.name)
    for child in robot_model.root_link.child_links:
        add_children_to_tree(child, tree)
    return tree
