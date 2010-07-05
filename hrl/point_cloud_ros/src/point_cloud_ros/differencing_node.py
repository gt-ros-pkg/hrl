#!/usr/bin/python

import numpy as np, math
import scipy.ndimage as ni

import roslib; roslib.load_manifest('point_cloud_ros')
import rospy

import hrl_lib.util as ut
import point_cloud_ros.occupancy_grid as pog
import point_cloud_ros.ros_occupancy_grid as rog

from point_cloud_ros.msg import OccupancyGrid


def og_cb(og_msg, param_list):
    global occupancy_difference_threshold, connected_comonents_size_threshold
    rospy.loginfo('og_cb called')
    diff_og = param_list[0]
    curr_og = rog.og_msg_to_og3d(og_msg, to_binary = False)

    if diff_og == None:
        param_list[0] = curr_og
        return

    pog.subtract(diff_og, curr_og)
    param_list[0] = curr_og

    diff_og.to_binary(occupancy_difference_threshold)
    # filter the noise
    connect_structure = np.zeros((3,3,3), dtype=int)
    connect_structure[1,1,:] = 1
#    connect_structure[1,1,0] = 0
    diff_og.grid = ni.binary_opening(diff_og.grid, connect_structure,
                                     iterations = 1)
    #    diff_og.grid, n_labels = diff_og.connected_comonents(connected_comonents_size_threshold)

    print 'np.all(diff_og == 0)', np.all(diff_og.grid == 0)
    diff_og_msg = rog.og3d_to_og_msg(diff_og)
    diff_og_msg.header.frame_id = og_msg.header.frame_id
    diff_og_msg.header.stamp = og_msg.header.stamp
    param_list[1].publish(diff_og_msg)


#------ arbitrarily set paramters -------
occupancy_difference_threshold = 5
connected_comonents_size_threshold = 10

if __name__ == '__main__':
    rospy.init_node('pc_difference_node')

    pub = rospy.Publisher('difference_occupancy_grid', OccupancyGrid)
    param_list = [None, pub]
    rospy.Subscriber('occupancy_grid', OccupancyGrid, og_cb, param_list)
    rospy.logout('Ready')

    rospy.spin()


