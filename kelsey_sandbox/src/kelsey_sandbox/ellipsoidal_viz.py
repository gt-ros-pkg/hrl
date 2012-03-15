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
from hrl_generic_arms.pose_converter import PoseConverter

from spheroid_space import SpheroidSpace

JOINT_STATE_INDS_R = [17, 18, 16, 20, 19, 21, 22]
JOINT_STATE_INDS_L = [29, 30, 28, 32, 31, 33, 34]

class SpheroidViz:
    def __init__(self):
        ellipse_rot = np.mat([[-1., 0., 0.], [0., -1., 0.], [0., 0., 1.]])
        self.sspace = SpheroidSpace(0.15, np.mat([0.78, -0.18, 0.1]).T, ellipse_rot)
        self.colors = [ColorRGBA(1., 0., 0., 1.), ColorRGBA(0., 1., 0., 1.)]
        self.vis_pub = rospy.Publisher("/force_torque_markers_array", MarkerArray)
        self.pose_pub = rospy.Publisher("/spheroid_poses", PoseStamped)

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
        u, v, p = 1.0, np.random.uniform(0, np.pi), np.random.uniform(0, 2 * np.pi)
        pos = self.sspace.spheroidal_to_cart((u, v, p))
        new_m.points.append(Point(*pos))

        df_du = self.sspace.partial_u((u, v, p))
        df_du *= 0.1 / np.linalg.norm(df_du)
        new_m.points.append(Point(*(pos+df_du)))
        
        self.ma.markers.append(new_m)
        self.vis_pub.publish(self.ma)

        rot_gripper = np.pi/4.

        pos, rot = self.sspace.spheroidal_to_pose((u,v,p), rot_gripper)

        ps_msg = PoseConverter.to_pose_stamped_msg("torso_lift_link", pos, rot)
        self.pose_pub.publish(ps_msg)


def main():
    rospy.init_node("test_markers")

    jfv = SpheroidViz()
    i = 0
    while not rospy.is_shutdown():
        jfv.publish_vector(i)
        i += 1
        rospy.sleep(0.1)
    return

if __name__ == "__main__":
    main()
