#! /usr/bin/python

import sys
import numpy as np
import copy

import roslib
roslib.load_manifest("hrl_pr2_arms")
roslib.load_manifest("kelsey_sandbox")
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
import tf.transformations as tf_trans
from equilibrium_point_control.pose_converter import PoseConverter

import urdf_interface as urdf
import kdl_parser as kdlp
from hrl_pr2_arms.kdl_arm_kinematics import KDLArmKinematics
from equilibrium_point_control.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm_kinematics import PR2ArmKinematics
from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory, PR2ArmJTranspose

from spheroid_space import SpheroidSpace

JOINT_STATE_INDS_R = [17, 18, 16, 20, 19, 21, 22]
JOINT_STATE_INDS_L = [29, 30, 28, 32, 31, 33, 34]

class SpheroidViz:
    def __init__(self):
        rot = np.mat([[1, 0, 0], [0, 1./np.sqrt(2), -1./np.sqrt(2)], [0, 1./np.sqrt(2), 1./np.sqrt(2)]])
        self.sspace = SpheroidSpace(0.2)#, np.mat([1.0, 0.5, 0.5]).T, rot)
        self.colors = [ColorRGBA(1., 0., 0., 1.), ColorRGBA(0., 1., 0., 1.)]
        self.vis_pub = rospy.Publisher("force_torque_markers_array", MarkerArray)
        self.pose_pub = rospy.Publisher("spheroid_poses", PoseStamped)

        m = Marker()
        m.header.frame_id = "torso_lift_link"
        m.header.stamp = rospy.Time()
        m.ns = "force_torque"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        #m.points.append(Point(0, 0, 0))
        m.scale = Vector3(0.01, 0.01, 0.01)
        m.color = self.colors[0]
        #self.vis_pub.publish(m)
        self.m = m
        
        self.ma = MarkerArray()

    def publish_vector(self, m_id):
        new_m = copy.deepcopy(self.m)
        new_m.id = m_id
        u, v, p = 1, np.random.uniform(0, np.pi), np.random.uniform(0, 2 * np.pi)
        pos = self.sspace.spheroidal_to_cart((u, v, p))
        new_m.points.append(Point(*pos))

        df_du = self.sspace.partial_u((u, v, p))
        df_du *= 0.1 / np.linalg.norm(df_du)
        new_m.points.append(Point(*(pos+df_du)))
        
        self.ma.markers.append(new_m)
        self.vis_pub.publish(self.ma)

        rot_gripper = np.pi/4.

        nx, ny, nz = df_du.T.A[0] / np.linalg.norm(df_du)
        j = np.sqrt(1./(1.+ny*ny/(nz*nz)))
        k = -ny*j/nz
        norm_rot = np.mat([[-nx,  ny*k - nz*j,  0],      
                           [-ny,  -nx*k,        j],      
                           [-nz,  nx*j,         k]])
        _, norm_quat = PoseConverter.to_pos_quat(np.mat([0, 0, 0]).T, norm_rot)
        rot_angle = np.arctan(-norm_rot[2,1] / norm_rot[2,2])
        quat_ortho_rot = tf_trans.quaternion_from_euler(rot_angle + np.pi + rot_gripper, 0.0, 0.0)
        norm_quat_ortho = tf_trans.quaternion_multiply(norm_quat, quat_ortho_rot)
        ps_msg = PoseConverter.to_pose_stamped_msg("torso_lift_link", pos, norm_quat_ortho)
        self.pose_pub.publish(ps_msg)

def fix_pr2_angs(q, min_lims, max_lims):
    q_mod = np.mod(q, 2 * np.pi)
    in_lims_a = np.clip(q_mod, min_lims, max_lims) == q_mod
    in_lims_b = np.clip(q_mod - 2 * np.pi, min_lims, max_lims) == q_mod
    if np.all(np.any([in_lims_a, in_lims_b])):
        return np.where(in_lims_a, q_mod, q_mod - 2 * np.pi)
    return None


def main():
    rospy.init_node("ellipsoidal_ik")
    sspace = SpheroidSpace(0.2, np.mat([0.78, -0.28, 0.3]).T)
    kin = PR2ArmKinematics('r')
    jarm = PR2ArmJointTrajectory('r', kin)

    while not rospy.is_shutdown():
        u, v, p = 1, np.random.uniform(0, np.pi), np.random.uniform(0, 2 * np.pi)
        pos, rot = sspace.spheroidal_to_pose((u, v, p))
        q = kin.IK(pos, rot)
        if not q is None:
            print q
#print np.mod(q, 2 * np.pi)
        rospy.sleep(0.1)
#jarm.set_ep([0.1]*7, 15)

    return

    jfv = SpheroidViz()
    i = 0
    while not rospy.is_shutdown():
        jfv.publish_vector(i)
        i += 1
        rospy.sleep(0.5)
    return

if __name__ == "__main__":
    main()
