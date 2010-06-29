#!/usr/bin/python

import numpy as np, math
import time

import roslib; roslib.load_manifest('point_cloud_ros')
import rospy

from point_cloud_ros.msg import OccupancyGrid

import hrl_tilting_hokuyo.display_3d_mayavi as d3m
import point_cloud_ros.ros_occupancy_grid as rog

def mayavi_cb(og, param_list):
    og3d = rog.og_msg_to_og3d(og)
    param_list[0] = og3d
    param_list[1] = True

def relay_cb(og, og_pub):
    rospy.logout('relay_cb called')
    og3d = rog.og_msg_to_og3d(og)
    og_new = rog.og3d_to_og_msg(og3d)
    og_pub.publish(og_new)


if __name__ == '__main__':

    og_pub = rospy.Publisher('relay_og_out', OccupancyGrid)
    rospy.Subscriber('relay_og_in', OccupancyGrid, relay_cb, og_pub)
    param_list = [None, False]
    rospy.Subscriber('occupancy_grid', OccupancyGrid, mayavi_cb, param_list)

    rospy.init_node('og_sample_python')
    rospy.logout('Ready')

    mode = rospy.get_param('~mode')

    if mode == 'mayavi':
        while not rospy.is_shutdown():
            if param_list[1] == True:
                og3d = param_list[0]
                print 'grid_shape:', og3d.grid.shape
                pts = og3d.grid_to_points()
                print pts.shape
#                param_list[1] = False
                break
            rospy.sleep(0.1)

        d3m.plot_points(pts)
        d3m.show()
    elif mode == 'relay':
        rospy.spin()







