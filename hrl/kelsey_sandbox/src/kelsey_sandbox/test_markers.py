#! /usr/bin/python

import roslib
roslib.load_manifest("kelsey_sandbox")
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

def main():
    rospy.init_node("test_markers")
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
