#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#  \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)

import sys, optparse, os
import time

import math, numpy as np 
import scipy.ndimage as ni
import copy

import hrl_lib.util as ut, hrl_lib.transforms as tr


## subtract occupancy grids. og1 = og1-og2
#
# @param og1 - occupancy_grid_3d object.
# @param og2 - occupancy_grid_3d object.
#
#will position og2 at an appropriate location within og1 (hopefully)
#will copy points in og2 but not in og1 into og1
#
# points corresponding to the gird cells whose occupancy drops to
# zero will still be in grid_points_list
#UNTESTED:
#    * subtracting grids of different sizes.
#    * how the rotation_z of the occupancy grid will affect things.
def subtract(og1,og2):

    if np.all(og1.resolution==og2.resolution) == False:
        print 'occupancy_grid_3d.subtract: The resolution of the two grids is not the same.'
        print 'res1, res2:', og1.resolution.A1.tolist(), og2.resolution.A1.tolist()
        print 'Exiting...'
        sys.exit()

    sub_tlb = og2.tlb
    sub_brf = og2.brf

    sub_tlb_idx = np.round((sub_tlb-og1.brf)/og1.resolution)
    sub_brf_idx = np.round((sub_brf-og1.brf)/og1.resolution)

    x_s,y_s,z_s = int(sub_brf_idx[0,0]),int(sub_brf_idx[1,0]),int(sub_brf_idx[2,0])
    x_e,y_e,z_e = int(sub_tlb_idx[0,0]),int(sub_tlb_idx[1,0]),int(sub_tlb_idx[2,0])

    x_e = min(x_e+1,og1.grid_shape[0,0])
    y_e = min(y_e+1,og1.grid_shape[1,0])
    z_e = min(z_e+1,og1.grid_shape[2,0])
    sub_grid = og1.grid[x_s:x_e,y_s:y_e,z_s:z_e]

    if np.any(og1.grid_shape!=og2.grid_shape):
        print '#############################################################################'
        print 'WARNING: occupancy_grid_3d.subtract has not been tested for grids of different sizes.'
        print '#############################################################################'
    sub_grid = sub_grid-og2.grid
    sub_grid = np.abs(sub_grid) # for now.
    og1.grid[x_s:x_e,y_s:y_e,z_s:z_e] = sub_grid
    idxs = np.where(sub_grid>=1)
    shp = og2.grid_shape
    list_idxs = (idxs[0]+idxs[1]*shp[0,0]+idxs[2]*shp[0,0]*shp[1,0]).tolist()

    og1_list_idxs = (idxs[0]+x_s+(idxs[1]+y_s)*shp[0,0]+(idxs[2]+z_s)*shp[0,0]*shp[1,0]).tolist()
    og1_list_len = len(og1.grid_points_list)
    for og1_pts_idxs,pts_idxs in zip(og1_list_idxs,list_idxs):
        if og1_pts_idxs<og1_list_len:
            og1.grid_points_list[og1_pts_idxs] += og2.grid_points_list[pts_idxs]

## class which implements the occupancy grid
class occupancy_grid_3d():

    ##
    # @param brf - 3x1 matrix. Bottom Right Front.
    # @param tlb - 3x1 matrix (coord of center of the top left back cell)
    # @param resolution - 3x1 matrix. size of each cell (in meters) along
    #                     the different directions.
    def __init__(self, brf, tlb, resolution, rotation_z=math.radians(0.)):
        #print np.round((tlb-brf)/resolution).astype('int')+1
        self.grid = np.zeros(np.round((tlb-brf)/resolution).astype('int')+1,dtype='int')

        self.tlb = tlb
        self.brf = brf
        self.grid_shape = np.matrix(self.grid.shape).T
        self.resolution = resolution

        n_cells = self.grid.shape[0]*self.grid.shape[1]*self.grid.shape[2]
        self.grid_points_list = [[] for i in range(n_cells)]
        self.rotation_z = rotation_z

    ## returns list of 8 tuples of 3x1 points which form the edges of the grid.
    # Useful for displaying the extents of the volume of interest (VOI).
    # @return list of 8 tuples of  3x1 points which form the edges of the grid.
    def grid_lines(self, rotation_angle=0.):
        grid_size = np.multiply(self.grid_shape,self.resolution)
        rot_mat = tr.rotZ(rotation_angle)
        p5 = self.tlb
        p6 = p5+np.matrix([0.,-grid_size[1,0],0.]).T
        p8 = p5+np.matrix([0.,0.,-grid_size[2,0]]).T
        p7 = p8+np.matrix([0.,-grid_size[1,0],0.]).T

        p3 = self.brf
        p4 = p3+np.matrix([0.,grid_size[1,0],0.]).T
        p2 = p3+np.matrix([0.,0.,grid_size[2,0]]).T
        p1 = p2+np.matrix([0.,grid_size[1,0],0.]).T

        p1 = rot_mat*p1
        p2 = rot_mat*p2
        p3 = rot_mat*p3
        p4 = rot_mat*p4
        p5 = rot_mat*p5
        p6 = rot_mat*p6
        p7 = rot_mat*p7
        p8 = rot_mat*p8

        l = [(p1,p2),(p1,p4),(p2,p3),(p3,p4),(p5,p6),(p6,p7),(p7,p8),(p8,p5),(p1,p5),(p2,p6),(p4,p8),(p3,p7)]
        #l = [(p5,p6),(p5,p3),(p1,p2)]
        return l

    ## fill the occupancy grid.
    # @param pts - 3xN matrix of points.
    # @param ignore_z - not use the z coord of the points. grid will be like a 2D grid.
    #
    #each cell of the grid gets filled the number of points that fall in the cell.
    def fill_grid(self,pts,ignore_z=False):
        if ignore_z:
            idx = np.where(np.min(np.multiply(pts[0:2,:]>self.brf[0:2,:],
                                              pts[0:2,:]<self.tlb[0:2,:]),0))[1]
        else:
            idx = np.where(np.min(np.multiply(pts[0:3,:]>self.brf,pts[0:3,:]<self.tlb),0))[1]

        if idx.shape[1] == 0:
            print 'aha!'
            return
        pts = pts[:,idx.A1.tolist()]

        # Find coordinates
        p_all = np.round((pts[0:3,:]-self.brf)/self.resolution)
        # Rotate points
        pts[0:3,:] = tr.Rz(self.rotation_z).T*pts[0:3,:]

        for i,p in enumerate(p_all.astype('int').T):
            if ignore_z:
                p[0,2] = 0

            if np.any(p<0) or np.any(p>=self.grid_shape.T):
                continue

            tup = tuple(p.A1)
            self.grid_points_list[
                    tup[0] + self.grid_shape[0,0] * 
                    tup[1] + self.grid_shape[0,0] * 
                    self.grid_shape[1,0] * 
                    tup[2]].append(pts[:,i])
            self.grid[tuple(p.A1)] += 1


    def to_binary(self,thresh=1):
        ''' all cells with occupancy>=thresh set to 1, others set to 0.
        '''
        filled = (self.grid>=thresh)
        self.grid[np.where(filled==True)] = 1
        self.grid[np.where(filled==False)] = 0

    def argmax_z(self,index_min=-np.Inf,index_max=np.Inf,search_up=False,search_down=False):
        ''' searches in the z direction for maximum number of cells with occupancy==1
            call this function after calling to_binary()
            returns index.
        '''
        index_min = int(max(index_min,0))
        index_max = int(min(index_max,self.grid_shape[2,0]-1))
        z_count_mat = []
        #for i in xrange(self.grid_shape[2,0]):
        for i in xrange(index_min,index_max+1):
            z_count_mat.append(np.where(self.grid[:,:,i]==1)[0].shape[0])

        if z_count_mat == []:
            return None
        z_count_mat = np.matrix(z_count_mat).T
        max_z = np.argmax(z_count_mat)
        max_count = z_count_mat[max_z,0]
        max_z += index_min
        print '#### max_count:', max_count

        if search_up:
            max_z_temp = max_z
            for i in range(1,5):
                #if (z_count_mat[max_z+i,0]*3.0)>max_count: #A
                #if (z_count_mat[max_z+i,0]*8.0)>max_count: #B
                if (max_z+i)>index_max:
                    break
                if (z_count_mat[max_z+i-index_min,0]*5.0)>max_count: #B'
                    max_z_temp = max_z+i
            max_z = max_z_temp

        if search_down:
            max_z_temp = max_z
            for i in range(1,5):
                if (max_z-i)<index_min:
                    break
                if (max_z-i)>index_max:
                    continue

                if (z_count_mat[max_z-i-index_min,0]*5.0)>max_count:
                    max_z_temp = max_z-i
            max_z = max_z_temp

        return max_z,max_count

    def find_plane_indices(self,hmin=-np.Inf,hmax=np.Inf,assume_plane=False):
        ''' assume_plane - always return something.
            returns list of indices (z) corrresponding to horizontal plane points.
            returns [] if there is no plane
        '''
        index_min = int(max(round((hmin-self.brf[2,0])/self.resolution[2,0]),0))
        index_max = int(min(round((hmax-self.brf[2,0])/self.resolution[2,0]),self.grid_shape[2,0]-1))
        z_plane,max_count = self.argmax_z(index_min,index_max,search_up=True)
        if z_plane == None:
            print 'oink oink.'
            return []

#---------- A
#        extra_remove_meters = 0.01
#        n_more_to_remove = int(round(extra_remove_meters/self.resolution[2,0]))
#        l = range(max(z_plane-n_more_to_remove-1,0),
#                  min(z_plane+n_more_to_remove+1,self.grid_shape[2,0]-1))

#---------- B
        extra_remove_meters = 0.005
        n_more_to_remove = int(round(extra_remove_meters/self.resolution[2,0]))
        l = range(max(z_plane-10,0),
                  min(z_plane+n_more_to_remove+1,self.grid_shape[2,0]-1))
        
# figure out whether this is indeed a plane.
        if assume_plane == False:
            n_more = int(round(0.1/self.resolution[2,0]))
            l_confirm = l+ range(max(l),min(z_plane+n_more+1,self.grid_shape[2,0]-1))

            grid_2d = np.max(self.grid[:,:,l],2)
            n_plane_cells = grid_2d.sum()
            grid_2d = ni.binary_fill_holes(grid_2d) # I want 4-connectivity while filling holes.

            n_plane_cells = grid_2d.sum()
            min_plane_pts_threshold = (self.grid_shape[0,0]*self.grid_shape[1,0])/4
            print '###n_plane_cells:', n_plane_cells
            print 'min_plane_pts_threshold:', min_plane_pts_threshold
            print 'find_plane_indices grid shape:',self.grid_shape.T

            if  n_plane_cells < min_plane_pts_threshold:
                print 'occupancy_grid_3d.find_plane_indices: There is no plane.'
                print 'n_plane_cells:', n_plane_cells
                print 'min_plane_pts_threshold:', min_plane_pts_threshold
                l = []

        return l


    ## get centroids of all the occupied cells as a 3xN np matrix 
    # @param occupancy_threshold - number of points in a cell for it to be "occupied"
    # @return 3xN matrix of 3d coord of the cells which have occupancy >= occupancy_threshold
    def grid_to_centroids(self,occupancy_threshold=1):
        p = np.matrix(np.row_stack(np.where(self.grid>=occupancy_threshold))).astype('float')
        p[0,:] = p[0,:]*self.resolution[0,0]
        p[1,:] = p[1,:]*self.resolution[1,0]
        p[2,:] = p[2,:]*self.resolution[2,0]
        p += self.brf
        return p


    def grid_to_points(self,array=None,occupancy_threshold=1):
        ''' array - if not None then this will be used instead of self.grid
            returns 3xN matrix of 3d coord of the cells which have occupancy >= occupancy_threshold
        '''

        if array == None:
            array = self.grid

        idxs = np.where(array>=occupancy_threshold)
        list_idxs = (idxs[0]+idxs[1]*self.grid_shape[0,0]+idxs[2]*self.grid_shape[0,0]*self.grid_shape[1,0]).tolist()

        l = []
        for pts_idxs in list_idxs:
            l += self.grid_points_list[pts_idxs]

        if l == []:
            p = np.matrix([])
        else:
            p = np.column_stack(l)

        return p

    def labeled_array_to_points(self,array,label):
        ''' returns coordinates of centers of grid cells corresponding to
            label as a 3xN matrix.
        '''
        idxs = np.where(array==label)
        list_idxs = (idxs[0]+idxs[1]*self.grid_shape[0,0]+idxs[2]*self.grid_shape[0,0]*self.grid_shape[1,0]).tolist()

        l = []
        for pts_idxs in list_idxs:
            l += self.grid_points_list[pts_idxs]

        if l == []:
            p = np.matrix([])
        else:
            p = np.column_stack(l)

        return p

    def remove_vertical_plane(self):
        ''' removes plane parallel to the YZ plane.
            changes grid.
            returns plane_indices, slice corresponding to the vertical plane.
            points behind the plane are lost for ever!
        '''
        self.grid = self.grid.swapaxes(2,0)
        self.grid_shape = np.matrix(self.grid.shape).T

#        z_max_first,max_count = self.argmax_z(search_up=False)
#        z_max_second,max_count_second = self.argmax_z(index_min=z_max_first+int(round(0.03/self.resolution[0,0])) ,search_up=False)

        z_max_first,max_count = self.argmax_z(search_down=False)
        z_max_second,max_count_second = self.argmax_z(index_min=z_max_first+int(round(0.035/self.resolution[0,0])) ,search_down=False)
        z_max_first,max_count = self.argmax_z(search_down=False)

        #z_max = self.argmax_z(search_up=True)
        if (max_count_second*1./max_count) > 0.3:
            z_max = z_max_second
        else:
            z_max = z_max_first
        print 'z_max_first', z_max_first
        print 'z_max_second', z_max_second
        print 'z_max', z_max

        more = int(round(0.03/self.resolution[0,0]))
        plane_indices = range(max(0,z_max-more),min(z_max+more,self.grid_shape[2,0]))

        self.grid = self.grid.swapaxes(2,0)
        self.grid_shape = np.matrix(self.grid.shape).T

        ver_plane_slice = self.grid[plane_indices,:,:]
        self.grid[plane_indices,:,:] = 0
        max_x = max(plane_indices)
        behind_indices = range(max_x,self.grid_shape[0,0])
        self.grid[behind_indices,:,:] = 0
        return plane_indices,ver_plane_slice

    def remove_horizontal_plane(self, remove_below=True,hmin=-np.Inf,hmax=np.Inf,
                                extra_layers=0):
        ''' call after to_binary()
            removes points corresponding to the horizontal plane from the grid.
            remove_below - remove points below the plane also.
            hmin,hmax - min and max possible height of the plane. (meters)
            This function changes grid.

            extra_layers - number of layers above the plane to remove. Sometimes
                           I want to be over zealous while removing plane points.
                           e.g. max_fwd_without_collision

            it returns the slice which has been set to zero, in case you want to
            leave the grid unchanged.
        '''
        l = self.find_plane_indices(hmin,hmax)

        if l == []:
            print 'occupancy_grid_3d.remove_horizontal_plane: No plane found.'
            return None,l

        add_num = min(10,self.grid_shape[2,0]-max(l)-1)
        max_l = max(l)+add_num
        l_edge = l+range(max(l),max_l+1)

        grid_2d = np.max(self.grid[:,:,l_edge],2)
#        grid_2d = ni.binary_dilation(grid_2d,iterations=1) # I want 4-connectivity while filling holes.
        grid_2d = ni.binary_fill_holes(grid_2d) # I want 4-connectivity while filling holes.

        connect_structure = np.empty((3,3),dtype='int')
        connect_structure[:,:] = 1
        eroded_2d = ni.binary_erosion(grid_2d,connect_structure,iterations=2)
        grid_2d = grid_2d-eroded_2d
        idxs = np.where(grid_2d!=0)

        if max_l>max(l):
            for i in range(min(5,add_num)):
                self.grid[idxs[0],idxs[1],max(l)+i+1] = 0

        if remove_below:
            l = range(0,min(l)+1)+l

        max_z = max(l)
        for i in range(extra_layers):
            l.append(max_z+i+1)

        l_edge = l+range(max(l),max_l+1)

        plane_and_below_pts = self.grid[:,:,l_edge]
        self.grid[:,:,l] = 0 # set occupancy to zero.
        return plane_and_below_pts,l_edge

    def segment_objects(self, twod=False):
        ''' segments out objects after removing the plane.
            call after calling to_binary.
            returns labelled_array,n_labels
            labelled_array - same dimen as occupancy grid, each object has a different label.
        '''
        plane_and_below_pts,l = self.remove_horizontal_plane(extra_layers=0)
        if l == []:
            print 'occupancy_grid_3d.segment_objects: There is no plane.'
            return None,None

        if twod == False:
            labelled_arr,n_labels = self.find_objects()
        else:
            labelled_arr,n_labels = self.find_objects_2d()
        self.grid[:,:,l] = plane_and_below_pts
        return labelled_arr,n_labels

    def find_objects_2d(self):
        ''' projects all points into the xy plane and then performs
            segmentation by region growing.
        '''
        connect_structure = np.empty((3,3),dtype='int')
        connect_structure[:,:] = 1
        grid_2d = np.max(self.grid[:,:,:],2)
#        grid_2d = ni.binary_erosion(grid_2d)
#        grid_2d = ni.binary_erosion(grid_2d,connect_structure)
        labeled_arr,n_labels = ni.label(grid_2d,connect_structure)
        print 'found %d objects'%(n_labels)

        labeled_arr_3d = self.grid.swapaxes(2,0)
        labeled_arr_3d = labeled_arr_3d.swapaxes(1,2)
        print 'labeled_arr.shape:',labeled_arr.shape
        print 'labeled_arr_3d.shape:',labeled_arr_3d.shape
        labeled_arr_3d = labeled_arr_3d*labeled_arr
        labeled_arr_3d = labeled_arr_3d.swapaxes(2,0)
        labeled_arr_3d = labeled_arr_3d.swapaxes(1,0)
        labeled_arr = labeled_arr_3d
        # I still want to count cells in 3d (thin but tall objects.)

        if n_labels > 0:
            labels_list = range(1,n_labels+1)
            #count_objects = ni.sum(grid_2d,labeled_arr,labels_list)
            count_objects = ni.sum(self.grid,labeled_arr,labels_list)
            if n_labels == 1:
                count_objects = [count_objects]

            t0 = time.time()
            new_labels_list = []

            for c,l in zip(count_objects,labels_list):
                if c > 3:
                    new_labels_list.append(l)
                else:
                    labeled_arr[np.where(labeled_arr == l)] = 0

            # relabel stuff
            for nl,l in enumerate(new_labels_list):
                labeled_arr[np.where(labeled_arr == l)] = nl+1

            n_labels = len(new_labels_list)
            t1 = time.time()
            print 'time:', t1-t0

        print 'found %d objects'%(n_labels)
#        return labeled_arr,n_labels
        return labeled_arr_3d,n_labels

    def find_objects(self):
        ''' region growing kind of thing for segmentation. Useful if plane has been removed.
        '''
        connect_structure = np.empty((3,3,3),dtype='int')
        grid = copy.copy(self.grid)

        connect_structure[:,:,:] = 0

        connect_structure[1,1,:] = 1
        iterations = int(round(0.005/self.resolution[2,0]))
#        iterations=5


        #grid = ni.binary_closing(grid,connect_structure,iterations=iterations)

        connect_structure[:,:,:] = 1
        labeled_arr,n_labels = ni.label(grid,connect_structure)
        print 'ho!'
        print 'found %d objects'%(n_labels)

        if n_labels == 0:
            return labeled_arr,n_labels

        labels_list = range(1,n_labels+1)
        count_objects = ni.sum(grid,labeled_arr,labels_list)
        if n_labels == 1:
            count_objects = [count_objects]

#        t0 = time.time()
#        remove_labels = np.where(np.matrix(count_objects) <= 5)[1].A1.tolist()
#        for r in remove_labels:
#            labeled_arr[np.where(labeled_arr == r)] = 0
#        t1 = time.time()
#        labeled_arr,n_labels = ni.label(labeled_arr,connect_structure)
#        print 'time:', t1-t0

        t0 = time.time()
        new_labels_list = []

        for c,l in zip(count_objects,labels_list):
            if c > 3:
                new_labels_list.append(l)
            else:
                labeled_arr[np.where(labeled_arr == l)] = 0

        # relabel stuff
        for nl,l in enumerate(new_labels_list):
            labeled_arr[np.where(labeled_arr == l)] = nl+1

        n_labels = len(new_labels_list)
        t1 = time.time()
        print 'time:', t1-t0


        print 'found %d objects'%(n_labels)
        return labeled_arr,n_labels



if __name__ == '__main__':
    import pygame_opengl_3d_display as po3d
    import hokuyo.pygame_utils as pu
    import processing_3d as p3d

    p = optparse.OptionParser()

    p.add_option('-f', action='store', type='string', dest='pkl_file_name',
                 help='file.pkl File with the scan,pos dict.',default=None)
    p.add_option('-c', action='store', type='string', dest='pts_pkl',
                 help='pkl file with 3D points',default=None)

    opt, args = p.parse_args()
    pts_pkl = opt.pts_pkl
    pkl_file_name = opt.pkl_file_name


    #-------------- simple test ---------------
#    gr = occupancy_grid_3d(np.matrix([0.,0.,0]).T, np.matrix([1.,1.,1]).T,
#                           np.matrix([1,1,1]).T)
#    pts = np.matrix([[1.1,0,-0.2],[0,0,0],[0.7,0.7,0.3],[0.6,0.8,-0.2]]).T
#    gr.fill_grid(pts)
##    print gr.grid

    resolution = np.matrix([0.01,0.01,0.01]).T
    gr = occupancy_grid_3d(np.matrix([0.45,-0.5,-1.0]).T, np.matrix([0.65,0.05,-0.2]).T,
                           resolution)

    if pts_pkl != None:
        pts = ut.load_pickle(pts_pkl)

    elif pkl_file_name != None:
        dict = ut.load_pickle(pkl_file_name)
        pos_list = dict['pos_list']
        scan_list = dict['scan_list']
        min_angle = math.radians(-40)
        max_angle = math.radians(40)

        l1 = dict['l1']
        l2 = dict['l2']
        pts = p3d.generate_pointcloud(pos_list, scan_list, min_angle, max_angle, l1, l2)

    else:
        print 'specify a pkl file -c or -f'
        print 'Exiting...'
        sys.exit()


    print 'started filling the grid'
    t0 = time.time()
    gr.fill_grid(pts)
    t1 = time.time()
    print 'time to fill the grid:', t1-t0

    #grid_pts = gr.grid_to_points()
    grid_pts = gr.grid_to_centroids()
##    print grid_pts
    cloud = pu.CubeCloud(grid_pts,(0,0,0),(resolution/2).A1.tolist())
    pc = pu.PointCloud(pts,(100,100,100))
    lc = pu.LineCloud(gr.grid_lines(),(100,100,0))
    po3d.run([cloud,pc,lc])




