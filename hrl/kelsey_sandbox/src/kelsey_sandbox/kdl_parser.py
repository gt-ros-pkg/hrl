
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
