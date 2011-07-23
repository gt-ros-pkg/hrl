#! /usr/bin/python

import sys

import roslib
roslib.load_manifest("kelsey_sandbox")
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState

JOINT_STATE_INDS_R = [17, 18, 16, 20, 19, 21, 22]
JOINT_STATE_INDS_L = [29, 30, 28, 32, 31, 33, 34]

class JointForceViz:
    def __init__(self, arm):
        self.arm = arm
        rospy.Subscriber("joint_states", JointState, self.joint_states_cb)
        if arm == 'r':
            self.arm_inds = JOINT_STATE_INDS_R
        else:
            self.arm_inds = JOINT_STATE_INDS_L

    def joint_states_cb(self, msg):
        jnt_effort = [msg.effort[i] for i in self.arm_inds]
        print jnt_effort

def main():
    assert sys.argv[1] in ['r', 'l']
    rospy.init_node("test_markers")

    jfv = JointForceViz(sys.argv[1])
    rospy.spin()
    return

    vis_pub = rospy.Publisher("test_markers", Marker)
    x = 0.
    while not rospy.is_shutdown():
        m = Marker()
        m.header.frame_id = "torso_lift_link"
        m.header.stamp = rospy.Time()
        m.ns = "test"
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position = Point(x, 0., 0.)
        m.pose.orientation = Quaternion(0., 0., 0., 1.)
        m.scale = Vector3(1.0, 1.0, 1.0)
        m.color = ColorRGBA(1., 0., 0., 1.)
        vis_pub.publish(m)
        x += 0.01
        rospy.sleep(0.1)

        


if __name__ == "__main__":
    main()
