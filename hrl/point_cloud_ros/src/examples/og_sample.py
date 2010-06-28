#!/usr/bin/python

import numpy as np, math

import roslib; roslib.load_manifest('point_cloud_ros')
import rospy

import hrl_tilting_hokuyo.display_3d_mayavi as d3m
from point_cloud_ros.msg import OccupancyGrid
import point_cloud_ros.occupancy_grid as pog

def cb(og, param_list):
    c = np.matrix([og.center.x, og.center.y, og.center.z]).T
    s = np.matrix([og.grid_size.x, og.grid_size.y, og.grid_size.z]).T
    r = np.matrix([og.resolution.x, og.resolution.y, og.resolution.z]).T
    og3d = pog.occupancy_grid_3d(c, s, r, np.array(og.data))
    param_list[0] = og3d
    param_list[1] = True

if __name__ == '__main__':
    rospy.init_node('og_sample_python')
    param_list = [None, False]
    rospy.Subscriber('occupancy_grid', OccupancyGrid, cb, param_list)
    rospy.logout('Ready')

    while not rospy.is_shutdown():
        if param_list[1] == True:
            og3d = param_list[0]
            print 'grid_shape:', og3d.grid.shape
            pts = og3d.grid_to_points()
            print pts.shape
            break
        rospy.sleep(0.1)

    d3m.plot_points(pts)
    d3m.show()


