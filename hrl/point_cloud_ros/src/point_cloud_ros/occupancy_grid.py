
import numpy as np, math

class occupancy_grid_3d():

    ##
    # @param resolution - 3x1 matrix. size of each cell (in meters) along
    #                     the different directions.
    def __init__(self, center, size, resolution, data,
                 occupancy_threshold):
        self.grid_shape = size/resolution
        tlb = center + size/2
        brf = center + size/2

        self.size = size
        self.grid = np.reshape(data, self.grid_shape)
        self.grid_shape = np.matrix(self.grid.shape).T
        self.resolution = resolution
        self.center = center

        filled = (self.grid >= occupancy_threshold)
        self.grid[np.where(filled==True)] = 1
        self.grid[np.where(filled==False)] = 0

    ##
    # @param array - if not None then this will be used instead of self.grid
    # @return 3xN matrix of 3d coord of the cells which have occupancy = 1
    def grid_to_points(self, array=None):
        if array == None:
            array = self.grid

        idxs = np.where(array == 1)
        x_idx = idxs[2]
        y_idx = idxs[1]
        z_idx = idxs[0]
        
        x = x_idx * self.resolution[0,0] + self.center[0,0] - self.size[0,0]/2
        y = y_idx * self.resolution[1,0] + self.center[1,0] - self.size[1,0]/2
        z = z_idx * self.resolution[2,0] + self.center[2,0] - self.size[2,0]/2

        return np.matrix(np.row_stack([x,y,z]))


## subtract occupancy grids. og1 = abs(og1-og2)
# @param og1 - occupancy_grid_3d object.
# @param og2 - occupancy_grid_3d object.
#
# will position og2 at an appropriate location within og1 (hopefully)
# will copy points in og2 but not in og1 into og1
#
#UNTESTED:
#    * subtracting grids of different sizes.
def subtract(og1, og2):
    if np.all(og1.resolution == og2.resolution) == False:
        print 'occupancy_grid_3d.subtract: The resolution of the two grids is not the same.'
        print 'res1, res2:', og1.resolution.A1.tolist(), og2.resolution.A1.tolist()
        print 'Exiting...'
#        rospy.logfatal('Trying to subtract grids with different resolutions')
        sys.exit()

    if np.any(og1.grid_shape!=og2.grid_shape):
        print 'Grid Sizes:', og1.grid_shape.A1, og2.grid_shape.A1
        raise RuntimeError('grids are of different sizes')

    og1.grid = np.abs(og1.grid - og2.grid)


if __name__ == '__main__':
    print 'Hello World'

