#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest("hrl_pr2_arms")
roslib.load_manifest("kelsey_sandbox")
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState

import urdf_interface as urdf
import kdl_parser as kdlp
from hrl_pr2_arms.kdl_arm_kinematics import KDLArmKinematics

JOINT_STATE_INDS_R = [17, 18, 16, 20, 19, 21, 22]
JOINT_STATE_INDS_L = [29, 30, 28, 32, 31, 33, 34]

class JointForceViz:
    def __init__(self, arm):
        self.arm = arm
        self.vis_pub = rospy.Publisher("force_torque_markers", Marker)
        if arm == 'r':
            self.arm_inds = JOINT_STATE_INDS_R
        else:
            self.arm_inds = JOINT_STATE_INDS_L
        model = urdf.create_model_from_file("/etc/ros/diamondback/urdf/robot.xml")
        tree = kdlp.tree_from_urdf_model(model)
        chain = tree.getChain("torso_lift_link", "r_gripper_tool_frame")
        link_dict = {0:0, 1:1, 2:2, 3:3, 4:4, 5:5, 6:6, 7:11}
        self.kinematics = KDLArmKinematics(chain=chain, link_dict=link_dict)
        self.colors = [ColorRGBA(1., 0., 0., 1.), ColorRGBA(0., 1., 0., 1.)]
        rospy.Subscriber("joint_states", JointState, self.joint_states_cb)

    def joint_states_cb(self, msg):
        q_pos = np.mat([msg.position[i] for i in self.arm_inds]).T
        q_vel = np.mat([msg.velocity[i] for i in self.arm_inds]).T
        q_effort = np.mat([msg.effort[i] for i in self.arm_inds]).T
        fk_pos, fk_rot = self.kinematics.FK(q_pos)
        J = self.kinematics.jacobian(q_pos)
        F, res, rank, sing = np.linalg.lstsq(J.T, q_effort)
        #print "Force:", F[0:3]
        #print "Torque:", F[3:6]
        self.publish_vector(fk_pos, 0.01 * F[0:3], 0)
        self.publish_vector(fk_pos, 0.04 * F[3:6], 1)

    def publish_vector(self, loc, v, m_id):
        m = Marker()
        m.header.frame_id = "torso_lift_link"
        m.header.stamp = rospy.Time()
        m.ns = "force_torque"
        m.id = m_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points.append(Point(*loc))
        m.points.append(Point(*(loc + v)))
        m.scale = Vector3(0.01, 0.02, 0.01)
        m.color = self.colors[m_id]
        self.vis_pub.publish(m)


def main():
    assert sys.argv[1] in ['r', 'l']
    rospy.init_node("test_markers")

    jfv = JointForceViz(sys.argv[1])
    rospy.spin()
    return

if __name__ == "__main__":
    main()
