
import roslib; roslib.load_manifest('hrl_pr2_kinematics_tutorials')
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import hrl_lib.transforms as tr
import math, numpy as np

def get_marker_arrow(p_st, p_end, frame_id):
    m = Marker()
    m.header.stamp = rospy.rostime.get_rostime()
    m.header.frame_id = frame_id

    m.ns = 'basic_shapes'
    m.id = 0
    m.type = Marker.ARROW
    m.action = Marker.ADD

    pt1 = Point(p_st[0,0], p_st[1,0], p_st[2,0])
    pt2 = Point(p_end[0,0], p_end[1,0], p_end[2,0])
    m.points.append(pt1)
    m.points.append(pt2)

    m.scale.x = 0.02;
    m.scale.y = 0.05;
    m.scale.z = 0.1;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.lifetime = rospy.Duration();
    return m


if __name__ == '__main__':
    rospy.init_node('marker_test', anonymous = True)
    marker_pub = rospy.Publisher('/test_marker', Marker)

    p1 = np.matrix([0.,0.,0.]).T
    p2 = np.matrix([0.,1.,0.]).T
    while not rospy.is_shutdown():
        marker = get_marker_arrow(p1, p2, 'base_link')
        marker_pub.publish(marker)
        rospy.sleep(0.1)




