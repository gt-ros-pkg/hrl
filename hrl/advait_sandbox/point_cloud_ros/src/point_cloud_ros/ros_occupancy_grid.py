
import numpy as np, math

import roslib; roslib.load_manifest('point_cloud_ros')
import rospy

import hrl_tilting_hokuyo.display_3d_mayavi as d3m
from point_cloud_ros.msg import OccupancyGrid
import point_cloud_ros.occupancy_grid as pog

## convert OccupancyGrid message to the  occupancy_grid_3d object.
# @param to_binary - want the occupancy grid to be binarified.
# @return occupancy_grid_3d object
def og_msg_to_og3d(og, to_binary=True):
    c = np.matrix([og.center.x, og.center.y, og.center.z]).T
    s = np.matrix([og.grid_size.x, og.grid_size.y, og.grid_size.z]).T
    r = np.matrix([og.resolution.x, og.resolution.y, og.resolution.z]).T
    og3d = pog.occupancy_grid_3d(c, s, r, np.array(og.data),
                    og.occupancy_threshold, to_binary = to_binary)
    return og3d

## convert occupancy_grid_3d object to OccupancyGrid message.
# sets the frame to base_link and stamp to the current time.
# @return OccupancyGrid object
def og3d_to_og_msg(og3d):
    og = OccupancyGrid()

    og.center.x = og3d.center[0,0]
    og.center.y = og3d.center[1,0]
    og.center.z = og3d.center[2,0]

    og.grid_size.x = og3d.size[0,0]
    og.grid_size.y = og3d.size[1,0]
    og.grid_size.z = og3d.size[2,0]

    og.resolution.x = og3d.resolution[0,0]
    og.resolution.y = og3d.resolution[1,0]
    og.resolution.z = og3d.resolution[2,0]

    og.occupancy_threshold = 1

    og.data = og3d.grid.flatten().tolist()
    og.header.frame_id = 'base_link'
    og.header.stamp = rospy.rostime.get_rostime()
    return og

## create an OccupancyGrid msg object for the purpose of setting the
# grid parameters for pc_to_og node.
# @param center -  3x1 np matrix.
# @param size -  3x1 np matrix.
# @param resolution -  3x1 np matrix.
# @param occupancy_threshold - integer
# @param frame_id - string.
def og_param_msg(center, size, resolution, occupancy_threshold, frame_id):
    og = OccupancyGrid()

    og.center.x = center[0,0]
    og.center.y = center[1,0]
    og.center.z = center[2,0]

    og.grid_size.x = size[0,0]
    og.grid_size.y = size[1,0]
    og.grid_size.z = size[2,0]

    og.resolution.x = resolution[0,0]
    og.resolution.y = resolution[1,0]
    og.resolution.z = resolution[2,0]

    og.occupancy_threshold = occupancy_threshold
    og.header.frame_id = frame_id
    og.header.stamp = rospy.rostime.get_rostime()
    return og

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


