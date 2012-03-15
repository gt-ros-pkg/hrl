#!/usr/bin/python

import numpy as np, math

import roslib; roslib.load_manifest('point_cloud_ros')
import rospy
import point_cloud_ros.ros_occupancy_grid as rog
from point_cloud_ros.msg import OccupancyGrid

from visualization_msgs.msg import Marker

## return a Marker message object
# @param loc - (x,y,z)
# @param scale - (x,y,z)
# @param color - (r,g,b,a)
# @param shape - 'sphere'
# @param frame_id - string.
def simple_viz_marker(loc, scale, color, shape, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.rostime.get_rostime()
    marker.ns = 'basic_shapes'
    marker.id = 0
    if shape == 'sphere':
        marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = loc[0]
    marker.pose.position.y = loc[1]
    marker.pose.position.z = loc[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.lifetime = rospy.Duration()
    return marker

if __name__ == '__main__':
    rospy.init_node('set_og_param_node')
    og_param_pub = rospy.Publisher('/og_params', OccupancyGrid)
    marker_pub = rospy.Publisher('/occupancy_grid_viz_marker', Marker)

    rospy.logout('Ready')

    center = np.matrix([0.8, 0., 0.8]).T # for single object bag file
    #center = np.matrix([0.6, 0., 0.75]).T
    size = np.matrix([0.4, 0.4, 0.4]).T
    #resolution = np.matrix([0.01, 0.01, 0.01]).T
    resolution = np.matrix([0.005, 0.005, 0.005]).T
    occupancy_threshold = 5
    frame_id = 'base_link'

    scale = (0.02, 0.02, 0.02)
    color = (0., 1., 0., 1.)
    shape = 'sphere'
    marker = simple_viz_marker(center.A1, scale, color, shape, frame_id)
    og_param = rog.og_param_msg(center, size, resolution,
                                occupancy_threshold, frame_id)

    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        og_param_pub.publish(og_param)
        r.sleep()

    rospy.spin()


