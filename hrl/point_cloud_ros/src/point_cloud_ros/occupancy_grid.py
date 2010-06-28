
import numpy as np, math

import roslib; roslib.load_manifest('point_cloud_ros')
import rospy

import hrl_tilting_hokuyo.display_3d_mayavi as d3m
from point_cloud_ros.msg import OccupancyGrid



class occupancy_grid_3d():

    ##
    # @param resolution - 3x1 matrix. size of each cell (in meters) along
    #                     the different directions.
    def __init__(self, center, size, resolution, data):
        self.grid_shape = size/resolution
        tlb = center + size/2
        brf = center + size/2

        self.size = size
        self.grid = np.reshape(data, self.grid_shape)
        self.grid_shape = np.matrix(self.grid.shape).T
        self.resolution = resolution
        self.center = center

    ##
    # @param array - if not None then this will be used instead of self.grid
    # @return 3xN matrix of 3d coord of the cells which have occupancy >= occupancy_threshold
    def grid_to_points(self, array=None, occupancy_threshold=1):
        if array == None:
            array = self.grid

        idxs = np.where(array>=occupancy_threshold)
        x_idx = idxs[2]
        y_idx = idxs[1]
        z_idx = idxs[0]
        
        x = x_idx * self.resolution[0,0] + self.center[0,0] - self.size[0,0]/2
        y = y_idx * self.resolution[1,0] + self.center[1,0] - self.size[1,0]/2
        z = z_idx * self.resolution[2,0] + self.center[2,0] - self.size[2,0]/2

        return np.matrix(np.row_stack([x,y,z]))


def cb(og, param_list):
    c = np.matrix([og.center.x, og.center.y, og.center.z]).T
    s = np.matrix([og.grid_size.x, og.grid_size.y, og.grid_size.z]).T
    r = np.matrix([og.resolution.x, og.resolution.y, og.resolution.z]).T
    og3d = occupancy_grid_3d(c, s, r, np.array(og.data))
    param_list[0] = og3d
    param_list[1] = True

if __name__ == '__main__':

    rospy.init_node('occupancy_grid_python')
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


