
import time
import sys, os, copy
import numpy as np, math
import scipy.ndimage as ni

class occupancy_grid_3d():

    ##
    # @param resolution - 3x1 matrix. size of each cell (in meters) along
    #                     the different directions.
    def __init__(self, center, size, resolution, data,
                 occupancy_threshold, to_binary = True):
        self.grid_shape = size/resolution
        tlb = center + size/2
        brf = center + size/2

        self.size = size
        self.grid = np.reshape(data, self.grid_shape)
        self.grid_shape = np.matrix(self.grid.shape).T
        self.resolution = resolution
        self.center = center

        if to_binary:
            self.to_binary(occupancy_threshold)

    ## binarize the grid
    # @param occupancy_threshold - voxels with occupancy less than this are set to zero.
    def to_binary(self, occupancy_threshold):
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
        x_idx = idxs[0]
        y_idx = idxs[1]
        z_idx = idxs[2]
        
        x = x_idx * self.resolution[0,0] + self.center[0,0] - self.size[0,0]/2
        y = y_idx * self.resolution[1,0] + self.center[1,0] - self.size[1,0]/2
        z = z_idx * self.resolution[2,0] + self.center[2,0] - self.size[2,0]/2

        return np.matrix(np.row_stack([x,y,z]))

    ## 27-connected components.
    # @param threshold - min allowed size of connected component
    def connected_comonents(self, threshold):
        connect_structure = np.ones((3,3,3), dtype='int')
        grid = self.grid
        labeled_arr, n_labels = ni.label(grid, connect_structure)

        if n_labels == 0:
            return labeled_arr, n_labels

        labels_list = range(1,n_labels+1)
        count_objects = ni.sum(grid, labeled_arr, labels_list)
        if n_labels == 1:
            count_objects = [count_objects]

        t0 = time.time()
        new_labels_list = []

        for c,l in zip(count_objects, labels_list):
            if c > threshold:
                new_labels_list.append(l)
            else:
                labeled_arr[np.where(labeled_arr == l)] = 0

        # relabel stuff
        for nl,l in enumerate(new_labels_list):
            labeled_arr[np.where(labeled_arr == l)] = nl+1
        n_labels = len(new_labels_list)
        t1 = time.time()
        print 'time:', t1-t0
        return labeled_arr, n_labels




if __name__ == '__main__':
    print 'Hello World'

