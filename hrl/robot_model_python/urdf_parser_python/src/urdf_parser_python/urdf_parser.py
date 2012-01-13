
import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("tf")
import rospy
import roslib.substitution_args

import re
import xml.dom.minidom as minidom
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
import tf.transformations as tf_trans
        
def create_pose_from_dom(dom):
    pose = Pose()
    pose.orientation.w = 1.
    xyz = dom.getAttribute("xyz")
    if len(xyz) != 0:
        xyz_list = [float(x) for x in re.findall("[0-9\-\.]+", xyz)]
        pose.position = Point(*xyz_list)
    rpy = dom.getAttribute("rpy")
    if len(rpy) != 0:
        rpy_list = [float(x) for x in re.findall("[0-9\-\.]+", rpy)]
        pose.orientation = Quaternion(*tf_trans.quaternion_from_euler(*rpy_list))
    return pose

class JointDynamics(object):
    def __init__(self):
        self.damping = 0.
        self.friction = 0.

    def init_xml(self, dom):
        damping = dom.getAttribute("damping")
        if len(damping) != 0:
            self.damping = float(damping)
        friction = dom.getAttribute("friction")
        if len(friction) != 0:
            self.friction = float(friction)

class JointLimits(object):
    def __init__(self):
        self.lower = 0.
        self.upper = 0.
        self.effort = 0.
        self.velocity = 0.

    def init_xml(self, dom):
        lower = dom.getAttribute("lower")
        if len(lower) != 0:
            self.lower = float(lower)
        upper = dom.getAttribute("upper")
        if len(upper) != 0:
            self.upper = float(upper)
        effort = dom.getAttribute("effort")
        if len(effort) != 0:
            self.effort = float(effort)
        velocity = dom.getAttribute("velocity")
        if len(velocity) != 0:
            self.velocity = float(velocity)

class JointSafety(object):
    def __init__(self):
        self.soft_upper_limit = 0.
        self.soft_lower_limit = 0.
        self.k_position = 0.
        self.k_velocity = 0.

    def init_xml(self, dom):
        soft_upper_limit = dom.getAttribute("soft_upper_limit")
        if len(soft_upper_limit) != 0:
            self.soft_upper_limit = float(soft_upper_limit)
        soft_lower_limit = dom.getAttribute("soft_lower_limit")
        if len(soft_lower_limit) != 0:
            self.soft_lower_limit = float(soft_lower_limit)
        k_position = dom.getAttribute("k_position")
        if len(k_position) != 0:
            self.k_position = float(k_position)
        k_velocity = dom.getAttribute("k_velocity")
        if len(k_velocity) != 0:
            self.k_velocity = float(k_velocity)

class JointCalibration(object):
    def __init__(self):
        self.reference_position = 0.
        self.rising = 0.
        self.falling = 0.

    def init_xml(self, dom):
        reference = dom.getAttribute("reference")
        if len(reference) != 0:
            self.reference_position = float(reference)
        rising = dom.getAttribute("rising")
        if len(rising) != 0:
            self.rising = float(rising)
        falling = dom.getAttribute("falling")
        if len(falling) != 0:
            self.falling = float(falling)

class JointMimic(object):
    def __init__(self):
        self.offset = 0.
        self.multiplier = 0.
        self.joint_name = ""

    def init_xml(self, dom):
        offset = dom.getAttribute("offset")
        if len(offset) != 0:
            self.offset = float(offset)
        multiplier = dom.getAttribute("multiplier")
        if len(multiplier) != 0:
            self.multiplier = float(multiplier)
        joint = dom.getAttribute("joint")
        if len(joint) != 0:
            self.joint_name = joint
        

class Joint(object):
    UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED = range(7)

    def __init__(self):
        self.name = ""
        self.axis = Vector3()
        self.child_link_name = ""
        self.parent_link_name = ""
        self.parent_to_joint_origin_transform = Pose()
        self.parent_to_joint_origin_transform.orientation.w = 1.
        self.dynamics = JointDynamics()
        self.limits = JointLimits()
        self.safety = JointSafety()
        self.calibration = JointCalibration()
        self.mimic = JointMimic()
        self.type = 0

    def init_xml(self, dom):
        self.name = dom.getAttribute("name")
        assert len(self.name) != 0
        if len(dom.getElementsByTagName("axis")) != 0:
            axis_xyz = dom.getElementsByTagName("axis")[0].getAttribute("xyz")
            axis_xyz_list = [float(x) for x in re.findall("[0-9\-\.]+", axis_xyz)]
            self.axis = Vector3(*axis_xyz_list)
        if len(dom.getElementsByTagName("child")) != 0:
            self.child_link_name = dom.getElementsByTagName("child")[0].getAttribute("link")
        if len(dom.getElementsByTagName("parent")) != 0:
            self.parent_link_name = dom.getElementsByTagName("parent")[0].getAttribute("link")
        if len(dom.getElementsByTagName("origin")) != 0:
            origin_dom = dom.getElementsByTagName("origin")[0]
            self.parent_to_joint_origin_transform = create_pose_from_dom(origin_dom)
        if len(dom.getElementsByTagName("dynamics")) != 0:
            dynamics_dom = dom.getElementsByTagName("dynamics")[0]
            self.dynamics.init_xml(dynamics_dom)
        if len(dom.getElementsByTagName("limit")) != 0:
            limits_dom = dom.getElementsByTagName("limit")[0]
            self.limits.init_xml(limits_dom)
        if len(dom.getElementsByTagName("safety_controller")) != 0:
            safety_dom = dom.getElementsByTagName("safety_controller")[0]
            self.safety.init_xml(safety_dom)
        if len(dom.getElementsByTagName("calibration")) != 0:
            calibration_dom = dom.getElementsByTagName("calibration")[0]
            self.calibration.init_xml(calibration_dom)
        if len(dom.getElementsByTagName("mimic")) != 0:
            mimic_dom = dom.getElementsByTagName("mimic")[0]
            self.mimic.init_xml(mimic_dom)
        joint_types = ["unknown", "revolute", "continuous", "prismatic", "floating", "planar", "fixed"]
        self.type = joint_types.index(dom.getAttribute("type"))

class Inertial(object):
    def __init__(self):
        self.origin = Pose()
        self.origin.orientation.w = 1.
        self.mass = 0.
        self.ixx = 0.
        self.ixy = 0.
        self.ixz = 0.
        self.iyy = 0.
        self.iyz = 0.
        self.izz = 0.

    def init_xml(self, dom):
        if len(dom.getElementsByTagName("origin")) != 0:
            origin_dom = dom.getElementsByTagName("origin")[0]
            self.parent_to_joint_origin_transform = create_pose_from_dom(origin_dom)
        if len(dom.getElementsByTagName("mass")) != 0:
            self.child_link_name = dom.getElementsByTagName("mass")[0].getAttribute("value")
        if len(dom.getElementsByTagName("inertia")) != 0:
            i_dom = dom.getElementsByTagName("inertia")[0]
            i_list = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
            for i_name in i_list:
                i_val = i_dom.getAttribute(i_name)
                if len(i_val) != 0:
                    exec "self.%s = float(i_val)" % (i_name)

class Geometry(object):
    SPHERE, BOX, CYLINDER, MESH = range(4)
    # TODO UNIMPLEMENTED
    def __init__(self):
        pass

class Color(object):
    def __init__(self):
        self.r = 0.
        self.g = 0.
        self.b = 0.
        self.a = 0.

    def init_xml(self, dom):
        # TODO UNIMPLEMENTED
        pass

class Material(object):
    def __init__(self):
        self.name = ""
        self.texture_filename = ""
        self.color = Color()

    def init_xml(self, dom):
        # TODO UNIMPLEMENTED
        pass

class Visual(object):
    def __init__(self):
        self.origin = Pose()
        self.origin.orientation.w = 1.
        self.geometry = Geometry()
        self.material_name = ""
        self.material = Material()
        self.group_name = ""

    def init_xml(self, dom):
        # TODO UNIMPLEMENTED
        pass

class Collision(object):
    def __init__(self):
        self.origin = Pose()
        self.origin.orientation.w = 1.
        self.geometry = Geometry()
        self.group_name = ""

    def init_xml(self, dom):
        # TODO UNIMPLEMENTED
        pass

class Link(object):
    def __init__(self):
        self.name = ""
        self.inertial = None
        self.visual = Visual()
        self.collision = Collision()
        self.visual_groups = {}
        self.collision_groups = {}
        self.parent_joint = None
        self.parent_link = None
        self.child_joints = []
        self.child_links = []

    def init_xml(self, dom):
        self.name = dom.getAttribute("name")
        assert len(self.name) != 0
        if len(dom.getElementsByTagName("inertial")) != 0:
            inertial_dom = dom.getElementsByTagName("inertial")[0]
            self.inertial = Inertial()
            self.inertial.init_xml(inertial_dom)
        if len(dom.getElementsByTagName("vision")) != 0:
            vision_dom = dom.getElementsByTagName("vision")[0]
            self.vision.init_xml(vision_dom)
        if len(dom.getElementsByTagName("collision")) != 0:
            collision_dom = dom.getElementsByTagName("collision")[0]
            self.collision.init_xml(collision_dom)

class Model(object):
    def __init__(self):
        self.joints = {}
        self.links = {}
        self.materials = {}
        self.name = ""
        self.root_link = None

    def init_xml(self, dom):
        self.name = dom.getAttribute("name")
        assert len(self.name) != 0
        for joint_dom in dom.getElementsByTagName("joint"):
            if joint_dom.parentNode.nodeName != "robot":
                continue
            joint = Joint()
            joint.init_xml(joint_dom)
            self.joints[joint.name] = joint
        for link_dom in dom.getElementsByTagName("link"):
            link = Link()
            link.init_xml(link_dom)
            self.links[link.name] = link

        # build tree links
        for joint_name in self.joints:
            joint = self.joints[joint_name]
            assert len(joint.parent_link_name) != 0 and len(joint.child_link_name) != 0
            p_link = self.links[joint.parent_link_name]
            c_link = self.links[joint.child_link_name]
            c_link.parent_link = p_link
            c_link.parent_joint = joint
            p_link.child_joints.append(joint)
            p_link.child_links.append(c_link)

        found_root = False
        for link_name in self.links:
            link = self.links[link_name]
            if link.parent_link is None:
                assert not found_root, "Found multiple roots"
                self.root_link = link
                found_root = True

        # TODO materials / geometry

def create_model_from_dom(dom):
    robot_dom = dom.getElementsByTagName("robot")[0]
    model = Model()
    model.init_xml(robot_dom)
    return model

def create_model_from_file(filename):
    dom = minidom.parse(roslib.substitution_args.resolve_args(filename))
    return create_model_from_dom(dom)

def create_model_from_string(string):
    dom = minidom.parseString(string)
    return create_model_from_dom(dom)
            
def create_model_from_param(param="/robot_description"):
    import rospy
    urdf_str = rospy.get_param(param)
    return create_model_from_string(urdf_str)

