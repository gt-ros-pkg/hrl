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

import roslib; roslib.load_manifest('hrl_tilting_hokuyo')
import hrl_hokuyo.hokuyo_processing as hp
import sys, optparse, os
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import numpy as np,math
import time

import display_3d_mayavi as d3m

import occupancy_grid_3d as og3d
import scipy.ndimage as ni

import copy
import pylab as pl

color_list = [(1.,1.,0),(1.,0,0),(0,1.,1.),(0,1.,0),(0,0,1.),(0,0.4,0.4),(0.4,0.4,0),
              (0.4,0,0.4),(0.4,0.8,0.4),(0.8,0.4,0.4),(0.4,0.4,0.8),(0.4,0,0.8),(0,0.8,0.4),
              (0,0.4,0.8),(0.8,0,0.4),(0.4,0,0.4),(1.,0.6,0.02) ]

#--------------------- utility functions -------------

##returns points that are within the cuboid formed by tlb and brf.
# @param pts - 3xN numpy matrix
# @param tlb, brf - 3x1 np matrix. bottom-right-front and top-left-back
# @return 3xM numpy matrix
def points_within_cuboid(pts,brf,tlb):
    idx = np.where(np.min(np.multiply(pts>brf,pts<tlb),0))[1]
    if idx.shape[1] == 0:
        within_pts = np.empty((3,0))
    else:
        within_pts = pts[:,idx.A1.tolist()]

    return within_pts


def generate_pointcloud(pos_list, scan_list, min_angle, max_angle, l1, l2,save_scan=False,
                        max_dist=np.Inf, min_dist=-np.Inf,min_tilt=-np.Inf,max_tilt=np.Inf, get_intensities=False, reject_zero_ten=True):
    ''' pos_list - list of motor positions (in steps)
        scan_list - list of UrgScan objects at the corres positions.
        l1,l2 - parameterizing the transformation (doc/ folder)
        save_scan - pickle 3xN numpy matrix of points.
        max_dist - ignore points at distance > max_dist
        min_dist - ignore points at distance < max_dist

        min_tilt, max_tilt - min and max tilt angles (radians)
                             +ve tilts the hokuyo down.
        get_intensites - additionally return intensity values
        returns 3xN numpy matrix of 3d coords of the points, returns (3xN, 1xN) including the intensity values if get_intensites=True
    '''
    t0 = time.time()
    allpts = []
    allintensities = []
    
    pos_arr = np.array(pos_list)
    scan_arr = np.array(scan_list)
    idxs = np.where(np.multiply(pos_arr<=max_tilt,pos_arr>=min_tilt))
    pos_list = pos_arr[idxs].tolist()
    scan_list = scan_arr[idxs].tolist()

    n_scans = len(scan_list)

    if n_scans>1:
        scan_list = copy.deepcopy(scan_list)

        # remove graze in across scans.
        ranges_mat = []
        for s in scan_list:
            ranges_mat.append(s.ranges.T)
        ranges_mat = np.column_stack(ranges_mat)

        start_index = hp.angle_to_index(scan_list[0],min_angle)
        end_index = hp.angle_to_index(scan_list[0],max_angle)
        for r in ranges_mat[start_index:end_index+1]:
            hp.remove_graze_effect(r,np.matrix(pos_list),skip=1,graze_angle_threshold=math.radians(169.))

        for i,s in enumerate(scan_list):
            s.ranges = ranges_mat[:,i].T

    for p,s in zip(pos_list,scan_list):
        mapxydata = hp.get_xy_map(s,min_angle,max_angle,max_dist=max_dist,min_dist=min_dist,reject_zero_ten=reject_zero_ten,sigmoid_hack=True,get_intensities=get_intensities) # pts is 2xN
      
        if get_intensities == True:
            pts, intensities = mapxydata
            allintensities.append(intensities)
        else:
            pts = mapxydata
            
        pts = np.row_stack((pts,np.zeros(pts.shape[1]))) # pts is now 3xN
        pts = tr.Ry(-p)*(pts+np.matrix((l1,0,-l2)).T)
        allpts.append(pts)
        

    allpts = np.column_stack(allpts)
    
    
    if save_scan:
        date_name = ut.formatted_time()
        ut.save_pickle(allpts,date_name+'_cloud.pkl')

    t1 = time.time()
#    print 'Time to generate point cloud:', t1-t0
#    allpts = tr.rotZ(math.radians(5))*allpts
    if get_intensities == True:
        allintensities = np.column_stack(allintensities)
        return allpts, allintensities
    else:
        return allpts

#----------------------- navigation functions -----------------

def max_fwd_without_collision(all_pts,z_height,max_dist,display_list=None):
    ''' find the max distance that it is possible to move fwd by
        without collision.
        all_pts - 3xN matrix of 3D points in thok frame.
        z - height of zenither while taking the scan.
        max_dist - how far do I want to check for a possible collision.
        returns max_dist that the thok can be moved fwd without collision.
    '''
    brf = np.matrix([0.2,-0.4,-z_height-0.1]).T
    tlb = np.matrix([max_dist, 0.4,-z_height+1.8]).T
    resolution = np.matrix([0.05,0.05,0.02]).T

    gr = og3d.occupancy_grid_3d(brf,tlb,resolution)
    gr.fill_grid(all_pts)

    gr.to_binary(4)
    ground_z = tr.thok0Tglobal(np.matrix([0,0,-z_height]).T)[2,0]
#    gr.remove_horizontal_plane(hmax=ground_z+0.1)
    gr.remove_horizontal_plane(extra_layers=2)
    collide_pts = np.row_stack(np.where(gr.grid!=0))
    x_coords = collide_pts[0]
#    print x_coords
    if x_coords.shape[0] == 0:
        max_x = max_dist
#        height_mat = np.arrary([np.Inf])
    else:
        max_x_gr = np.min(x_coords)
#        height_mat = collide_pts[1,np.where(x_coords==max_x_gr)[0]]
#        height_mat = height_mat*gr.resolution[2,0]+gr.brf[2,0]
        max_x = max_x_gr*gr.resolution[0,0]+gr.brf[0,0]

    if display_list != None:
        collide_grid = gr.grid_to_points()
        display_list.append(pu.PointCloud(all_pts,(200,200,200)))
        display_list.append(pu.CubeCloud(collide_grid,(200,200,200),resolution))
        display_list.append(pu.CubeCloud(np.matrix((max_x,0.,0.)).T,(0,0,200),size=np.matrix([0.05,.05,0.05]).T))

#    return max_x,np.matrix(height_mat)
    return max_x

def find_goto_point_surface_1(grid,pt,display_list=None):
    ''' returns p_erratic,p_edge,surface_height
        p_erratic - point where the erratic's origin should go to. (in thok0 frame)
        p_edge - point on the edge closest to pt.
        p_erratic,p_edge are 3x1 matrices.
        surface_height - in thok0 coord frame.
    '''
    pt_thok = pt
    
    # everything is happening in the thok coord frame.
    close_pt,approach_vector = find_approach_direction(grid,pt_thok,display_list)
    if close_pt == None:
        return None,None,None

    # move perpendicular to approach direction.
#    lam = -(close_pt[0:2,0].T*approach_vector)[0,0]
#    lam = min(lam,-0.3) # atleast 0.3m from the edge.
    lam = -0.4
    goto_pt = close_pt[0:2,0] + lam*approach_vector # this is where I want the utm
                                                    # to be
    err_to_thok = tr.erraticTglobal(tr.globalTthok0(np.matrix([0,0,0]).T))
    goto_pt_erratic = -err_to_thok[0,0]*approach_vector + goto_pt # this is NOT
                # general. It uses info about the two frames. If frames move, bad
                # things can happen.

    if display_list != None:
        display_list.append(pu.CubeCloud(np.row_stack((goto_pt,close_pt[2,0])),color=(0,250,250), size=(0.012,0.012,0.012)))
        display_list.append(pu.Line(np.row_stack((goto_pt,close_pt[2,0])).A1,close_pt.A1,color=(255,20,0)))

    p_erratic = np.row_stack((goto_pt_erratic,np.matrix((close_pt[2,0]))))
    print 'p_erratic in thok0:', p_erratic.T

    return p_erratic,close_pt,close_pt[2,0]


#------------------ surface orientation ----------------
def find_closest_pt_index(pts2d,pt):
    ''' returns index (of pts2d) of closest point to pt.
        pts2d - 2xN matrix, pt - 2x1 matrix
    '''
    pt_to_edge_dists = ut.norm(pts2d-pt)
    closest_pt_index = np.argmin(pt_to_edge_dists)
    return closest_pt_index

def find_closest_pt(pts2d,pt,pt_closer=False):
    ''' returns closest point to edge (2x1 matrix)
        can return None also
    '''
    dist_pt = np.linalg.norm(pt[0:2,0])
    pts2d_r = ut.norm(pts2d)
    pts2d_a = np.arctan2(pts2d[1,:],pts2d[0,:])
    if pt_closer == False:
        k_idxs = np.where(pts2d_r<=dist_pt)
    else:
        k_idxs = np.where(pts2d_r>dist_pt)

    pts2d_r = pts2d_r[k_idxs]
    pts2d_a = pts2d_a[k_idxs]
    pts2d = ut.cart_of_pol(np.matrix(np.row_stack((pts2d_r,pts2d_a))))

    if pt_closer == False:
        edge_to_pt = pt[0:2,0]-pts2d
    else:
        edge_to_pt = pts2d-pt[0:2,0]

    edge_to_pt_a = np.arctan2(edge_to_pt[1,:],edge_to_pt[0,:])
    keep_idxs = np.where(np.abs(edge_to_pt_a)<math.radians(70))[1].A1

    if keep_idxs.shape[0] == 0:
        return None

    pts2d = pts2d[:,keep_idxs]

#    pt_to_edge_dists = ut.norm(pts2d-pt[0:2,0])
#    closest_pt_index = np.argmin(pt_to_edge_dists)
    closest_pt_index = find_closest_pt_index(pts2d,pt[0:2,0])
    closest_pt = pts2d[:,closest_pt_index]
    return closest_pt

def pushback_edge(pts2d,pt):
    ''' push pt away from the edge defined by pts2d.
        pt - 2x1, pts2d - 2xN
        returns the pushed point.
    '''
    closest_idx = find_closest_pt_index(pts2d,pt)
    n_push_points = min(min(5,pts2d.shape[1]-closest_idx-1),closest_idx)
    if closest_idx<n_push_points or (pts2d.shape[1]-closest_idx-1)<n_push_points:
        print 'processing_3d.pushback_edge: pt is too close to the ends of the pts2d array.'
        return None

    edge_to_pt = pt-pts2d[:,closest_idx-n_push_points:closest_idx+n_push_points]
    edge_to_pt_r = ut.norm(edge_to_pt)
    edge_to_pt_a = np.arctan2(edge_to_pt[1,:],edge_to_pt[0,:])

    non_zero_idxs = np.where(edge_to_pt_r>0.005)
    edge_to_pt_r = edge_to_pt_r[non_zero_idxs]
    edge_to_pt_r[0,:] = 1
    edge_to_pt_a = edge_to_pt_a[non_zero_idxs]
    edge_to_pt_unit = ut.cart_of_pol(np.row_stack((edge_to_pt_r,edge_to_pt_a)))
    push_vector = edge_to_pt_unit.mean(1)
    push_vector = push_vector/np.linalg.norm(push_vector)
    print 'push_vector:', push_vector.T
    pt_pushed = pt + push_vector*0.05

    return pt_pushed

## figure out a good direction to approach the surface.
# @param grid - occupancy grid (binary) around the point of interest.
#               assumes that it has a surface.
# @param pt - 3d point which has to be approached.
# @param display_list - if display_list are lists then point clouds etc. are added
# for visualisation.
# 
# @return - closest_pt,approach_vector. 
def find_approach_direction(grid,pt,display_list=None):
    z_plane,max_count = grid.argmax_z(search_up=True)
    z_plane_meters = z_plane*grid.resolution[2,0]+grid.brf[2,0]

    l = grid.find_plane_indices(assume_plane=True)
    print '------------ min(l)',min(l)

    z_plane_argmax,max_count = grid.argmax_z(search_up=False)
    z_plane_below = max(0,z_plane_argmax-5)
    print 'z_plane_argmax',z_plane_argmax
    print 'z_plane_below',z_plane_below
    print 'l:',l
#    l = range(z_plane_below,z_plane)+l
    copy_grid = copy.deepcopy(grid)
    plane_slices = grid.grid[:,:,l]
    copy_grid.grid[:,:,:] = 0
    copy_grid.grid[:,:,l] = copy.copy(plane_slices)
    #display_list.append(pu.PointCloud(copy_grid.grid_to_points(),color=(0,0,255)))
    #plane_pts = copy_grid.grid_to_points()

    grid_2d = np.max(grid.grid[:,:,l],2)
    grid_2d = ni.binary_dilation(grid_2d,iterations=4) # I want 4-connectivity while filling holes.
    grid_2d = ni.binary_fill_holes(grid_2d) # I want 4-connectivity while filling holes.

    labeled_arr,n_labels = ni.label(grid_2d)
    labels_list = range(1,n_labels+1)
    count_objects = ni.sum(grid_2d,labeled_arr,labels_list)
    if n_labels == 1:
        count_objects = [count_objects]
    max_label = np.argmax(np.matrix(count_objects))
    grid_2d[np.where(labeled_arr!=max_label+1)] = 0

#    connect_structure = np.empty((3,3),dtype='int')
#    connect_structure[:,:] = 1
#    eroded_2d = ni.binary_erosion(grid_2d,connect_structure,iterations=4)
#    eroded_2d = ni.binary_erosion(grid_2d)

#    grid_2d = grid_2d-eroded_2d

    labeled_arr_3d = copy_grid.grid.swapaxes(2,0)
    labeled_arr_3d = labeled_arr_3d.swapaxes(1,2)
    labeled_arr_3d = labeled_arr_3d*grid_2d
    labeled_arr_3d = labeled_arr_3d.swapaxes(2,0)
    labeled_arr_3d = labeled_arr_3d.swapaxes(1,0)
    copy_grid.grid = labeled_arr_3d
    pts3d = copy_grid.grid_to_points()
    pts2d = pts3d[0:2,:]

    dist_pt = np.linalg.norm(pt[0:2,0])
    pts2d_r = ut.norm(pts2d)
    pts2d_a = np.arctan2(pts2d[1,:],pts2d[0,:])

    max_angle = np.max(pts2d_a)
    min_angle = np.min(pts2d_a)

    max_angle = min(max_angle,math.radians(50))
    min_angle = max(min_angle,math.radians(-50))

    ang_res = math.radians(1.)
    n_bins = int((max_angle-min_angle)/ang_res)
    print 'n_bins:', n_bins
    n_bins = min(50,n_bins)
#    n_bins=50
    ang_res = (max_angle-min_angle)/n_bins
    print 'n_bins:', n_bins


    angle_list = []
    range_list = []
    for i in range(n_bins):
        angle = min_angle+ang_res*i
        idxs = np.where(np.multiply(pts2d_a<(angle+ang_res/2.),pts2d_a>(angle-ang_res/2.)))
        r_mat = pts2d_r[idxs]
        a_mat = pts2d_a[idxs]
        if r_mat.shape[1] == 0:
            continue
        min_idx = np.argmin(r_mat.A1)
        range_list.append(r_mat[0,min_idx])
        angle_list.append(a_mat[0,min_idx])

    if range_list == []:
        print 'processing_3d.find_approach_direction: No edge points found'
        return None,None

    pts2d = ut.cart_of_pol(np.matrix(np.row_stack((range_list,angle_list))))

    closest_pt_1 = find_closest_pt(pts2d,pt,pt_closer=False)
    if closest_pt_1 == None:
        dist1 = np.Inf
    else:
        approach_vector_1 = pt[0:2,0] - closest_pt_1
        dist1 = np.linalg.norm(approach_vector_1)
        approach_vector_1 = approach_vector_1/dist1

    closest_pt_2 = find_closest_pt(pts2d,pt,pt_closer=True)
    if closest_pt_2 == None:
        dist2 = np.Inf
    else:
        approach_vector_2 = closest_pt_2 - pt[0:2,0]
        dist2 = np.linalg.norm(approach_vector_2)
        approach_vector_2 = approach_vector_2/dist2

    if dist1 == np.Inf and dist2 == np.Inf:
        approach_vector_1 = np.matrix([1.,0.,0.]).T
        approach_vector_2 = np.matrix([1.,0.,0.]).T
        print 'VERY STRANGE: processing_3d.find_approach_direction: both distances are Inf'

    if dist1<0.05 or dist2<0.05:
        print 'dist1,dist2:',dist1,dist2
        t_pt = copy.copy(pt)
        if dist1<dist2 and dist1<0.02:
            t_pt[0,0] += 0.05
        elif dist2<0.02:
            t_pt[0,0] -= 0.05
        #pt_new = pushback_edge(pts2d,pt[0:2,0])
        pt_new = pushback_edge(pts2d,t_pt[0:2,0])
        if display_list != None:
            pt_new_3d = np.row_stack((pt_new,np.matrix([z_plane_meters])))
            display_list.append(pu.CubeCloud(pt_new_3d,color=(200,000,0),size=(0.009,0.009,0.009)))

        closest_pt_1 = find_closest_pt(pts2d,pt_new,pt_closer=False)
        if closest_pt_1 == None:
            dist1 = np.Inf
        else:
            approach_vector_1 = pt_new - closest_pt_1
            dist1 = np.linalg.norm(approach_vector_1)
            approach_vector_1 = approach_vector_1/dist1

        closest_pt_2 = find_closest_pt(pts2d,pt_new,pt_closer=True)
        if closest_pt_2 == None:
            dist2 = np.Inf
        else:
            approach_vector_2 = closest_pt_2 - pt_new
            dist2 = np.linalg.norm(approach_vector_2)
            approach_vector_2 = approach_vector_2/dist2

    print '----------- dist1,dist2:',dist1,dist2
    if dist2<dist1:
        closest_pt = closest_pt_2
        approach_vector = approach_vector_2
    else:
        closest_pt = closest_pt_1
        approach_vector = approach_vector_1

    print '----------- approach_vector:',approach_vector.T
    closest_pt = np.row_stack((closest_pt,np.matrix([z_plane_meters])))

    if display_list != None:
        z = np.matrix(np.empty((1,pts2d.shape[1])))
        z[:,:] = z_plane_meters
        pts3d_front = np.row_stack((pts2d,z))

        display_list.append(pu.CubeCloud(closest_pt,color=(255,255,0),size=(0.020,0.020,0.020)))
        display_list.append(pu.CubeCloud(pts3d_front,color=(255,0,255),size=grid.resolution))

        #display_list.append(pu.CubeCloud(pts3d,color=(0,255,0)))
    return closest_pt,approach_vector

#------------------- for doors ---------------------
def vertical_plane_points(grid):
    ''' changes grid
    '''
    plane_indices,ver_plane_slice = grid.remove_vertical_plane()
    grid.grid[:,:,:] = 0
    grid.grid[plane_indices,:,:] = ver_plane_slice

## returns door handle points in the thok coord frame.
def find_door_handle(grid,pt,list = None,rotation_angle=math.radians(0.),
                     occupancy_threshold=None,resolution=None):
    grid.remove_vertical_plane()
    pts = grid.grid_to_points()

    rot_mat = tr.Rz(rotation_angle)
    t_pt = rot_mat*pt
    brf = t_pt+np.matrix([-0.2,-0.3,-0.2]).T
    tlb = t_pt+np.matrix([0.2, 0.3,0.2]).T
    #resolution = np.matrix([0.02,0.0025,0.02]).T
    grid = og3d.occupancy_grid_3d(brf,tlb,resolution,rotation_z=rotation_angle)
    
    if pts.shape[1] == 0:
        return None

    grid.fill_grid(tr.Rz(rotation_angle)*pts)
    grid.to_binary(occupancy_threshold)

    labeled_arr,n_labels = grid.find_objects()

    if list == None:
        object_points_list = []
    else:
        object_points_list = list

    for l in range(n_labels):
        pts = grid.labeled_array_to_points(labeled_arr,l+1)
        obj_height = np.max(pts[2,:])-np.min(pts[2,:])
        print 'object_height:', obj_height
        if obj_height > 0.1:
            #remove the big objects
            grid.grid[np.where(labeled_arr==l+1)] = 0

    connect_structure = np.empty((3,3,3),dtype='int')
    connect_structure[:,:,:] = 0
    connect_structure[1,:,1] = 1
    # dilate away - actual width of the door handle is not affected
    # because that I will get from the actual point cloud!
    grid.grid = ni.binary_dilation(grid.grid,connect_structure,iterations=7)

    labeled_arr,n_labels = grid.find_objects()
    for l in range(n_labels):
        pts = grid.labeled_array_to_points(labeled_arr,l+1)
        
        pts2d = pts[1:3,:] # only the y-z coordinates.

        obj_width = (pts2d.max(1)-pts2d.min(1))[0,0]
        print 'processing_3d.find_door_handle: object width = ', obj_width
        if obj_width < 0.05:
            continue

        pts2d_zeromean = pts2d-pts2d.mean(1)
        e_vals,e_vecs = np.linalg.eig(pts2d_zeromean*pts2d_zeromean.T)
        max_index = np.argmax(e_vals)
        max_evec = e_vecs[:,max_index]
        ang = math.atan2(max_evec[1,0],max_evec[0,0])
        print 'processing_3d.find_door_handle: ang = ', math.degrees(ang)
        if (ang>math.radians(45) and ang<math.radians(135)) or \
           (ang>math.radians(-135) and ang<math.radians(-45)):
               # assumption is that door handles are horizontal.
               continue

        object_points_list.append(pts)
    print 'processing_3d.find_door_handle: found %d objects'%(len(object_points_list))

    closest_obj = find_closest_object(object_points_list,pt)
    return closest_obj


#--------------------- segmentation ---------------------
def find_closest_object(obj_pts_list,pt,return_idx=False):
    ''' obj_pts_list - list of 3xNi matrices of points.
        pt - point of interest. (3x1) matrix.
        return_idx - whether to return the index (in obj_pts_list) of
                     the closest object.
        returns 3xNj matrix of points which is the closest object to pt.
                None if obj_pts_list is empty.
    '''
    min_dist_list = []
    for obj_pts in obj_pts_list:
        min_dist_list.append(np.min(ut.norm(obj_pts-pt)))

    if obj_pts_list == []:
        return None
    min_idx = np.argmin(np.matrix(min_dist_list))
    cl_obj = obj_pts_list[min_idx]
    print 'processing_3d.find_closest_object: closest_object\'s centroid',cl_obj.mean(1).T

    if return_idx:
        return cl_obj,min_idx
    return cl_obj

def segment_objects_points(grid,return_labels_list=False,
                           twod=False):
    ''' grid - binary occupancy grid.
        returns list of 3xNi numpy matrices where Ni is the number of points
        in the ith object. Point refers to center of the cell of occupancy grid.
        return_labels_list - return a list of labels of the objects in
        the grid.
        returns None if there is no horizontal surface
    '''
    labeled_arr,n_labels = grid.segment_objects(twod=twod)
    if n_labels == None:
        # there is no surface, so segmentation does not make sense.
        return None

    object_points_list = []
    labels_list = []

    for l in range(n_labels):
        pts = grid.labeled_array_to_points(labeled_arr,l+1)

        pts_zeromean = pts-pts.mean(1)
        e_vals,e_vecs = np.linalg.eig(pts_zeromean*pts_zeromean.T)

        max_index = np.argmax(e_vals)
        max_evec = e_vecs[:,max_index]
        print 'max eigen vector:', max_evec.T
        pts_1d = max_evec.T * pts
        size = pts_1d.max() - pts_1d.min()
        print 'size:', size
        print 'n_points:', pts.shape[1]
        ppsoe = pts.shape[1]/(e_vals[0]+e_vals[1]+e_vals[2])
        print 'points per sum of eigenvalues:',ppsoe
#        if ppsoe<5000:
#            continue
        if size<0.05 or size>0.5:  #TODO - figure out a good threshold.
            continue
        object_points_list.append(pts)
        labels_list.append(l+1)

    if return_labels_list:
        return object_points_list, labels_list

    return object_points_list

def create_grid(brf,tlb,resolution,pos_list,scan_list,l1,l2,
                display_flag=False,show_pts=True,rotation_angle=0.,
                occupancy_threshold=1):
    ''' rotation angle - about the Z axis.
    '''
    max_dist = np.linalg.norm(tlb) + 0.2
    min_dist = brf[0,0]
    min_angle,max_angle=math.radians(-60),math.radians(60)

    all_pts = generate_pointcloud(pos_list, scan_list, min_angle, max_angle, l1, l2,
                                  max_dist=max_dist,min_dist=min_dist)
    rot_mat = tr.Rz(rotation_angle)
    all_pts_rot = rot_mat*all_pts

    gr = og3d.occupancy_grid_3d(brf,tlb,resolution,rotation_z=rotation_angle)

    gr.fill_grid(all_pts_rot)
    gr.to_binary(occupancy_threshold)

    if display_flag == True:
        if show_pts:
            d3m.plot_points(all_pts,color=(0.,0.,0.))
        cube_tups = gr.grid_lines(rotation_angle=rotation_angle)
        d3m.plot_cuboid(cube_tups)

    return gr

def create_vertical_plane_grid(pt,pos_list,scan_list,l1,l2,rotation_angle,display_list=None):
    rot_mat = tr.Rz(rotation_angle)
    t_pt = rot_mat*pt
    brf = t_pt+np.matrix([-0.2,-0.3,-0.2]).T
    tlb = t_pt+np.matrix([0.2, 0.3,0.2]).T
    resolution = np.matrix([0.005,0.02,0.02]).T
    return create_grid(brf,tlb,resolution,pos_list,scan_list,l1,l2,display_list,rotation_angle=rotation_angle,occupancy_threshold=1)

def create_scooping_grid(pt,pos_list,scan_list,l1,l2,display_flag=False):
    brf = pt+np.matrix([-0.15,-0.4,-0.2]).T
    brf[0,0] = max(0.07,brf[0,0])
    tlb = pt+np.matrix([0.25, 0.4,0.2]).T
    resolution = np.matrix([0.01,0.01,0.0025]).T
    return create_grid(brf,tlb,resolution,pos_list,scan_list,l1,l2,display_flag)

def create_segmentation_grid(pt,pos_list,scan_list,l1,l2,display_flag=False):
    brf = pt+np.matrix([-0.15,-0.2,-0.2]).T
    brf[0,0] = max(0.07,brf[0,0])
    tlb = pt+np.matrix([0.25, 0.2,0.2]).T

#   brf = np.matrix([0.05,-0.3,-0.2]).T
#   tlb = np.matrix([0.5, 0.3,0.1]).T
#   resolution = np.matrix([0.005,0.005,0.005]).T

#    resolution = np.matrix([0.005,0.005,0.0025]).T
    resolution = np.matrix([0.01,0.01,0.0025]).T
#    resolution = np.matrix([0.01,0.01,0.001]).T

    return create_grid(brf,tlb,resolution,pos_list,scan_list,l1,l2,display_flag)

def create_approach_grid(pt,pos_list,scan_list,l1,l2,display_list=None,show_pts=True):
    brf = pt + np.matrix([-0.5,-0.2,-0.3]).T
    brf[0,0] = max(0.10,brf[0,0])

    tlb = pt + np.matrix([0.3, 0.2,0.2]).T
#    resolution = np.matrix([0.005,0.005,0.005]).T
    resolution = np.matrix([0.01,0.01,0.0025]).T
#    resolution = np.matrix([0.01,0.01,0.001]).T
    return create_grid(brf,tlb,resolution,pos_list,scan_list,l1,l2,display_list,show_pts)

def create_overhead_grasp_choice_grid(pt,pos_list,scan_list,l1,l2,far_dist,display_list=None):

#    y_pos = max(pt[1,0]+0.1,0.1)
#    y_neg = min(pt[1,0]-0.1,-0.1)
#
#    brf = np.matrix([0.2,y_neg,0.0]).T
#    tlb = np.matrix([pt[0,0]+far_dist,y_pos,pt[2,0]+0.75]).T
#
#    resolution = np.matrix([0.02,0.02,0.02]).T

    y_pos = 0.1
    y_neg = -0.1
    
    r = np.linalg.norm(pt[0:2,0])
    brf = np.matrix([0.25,y_neg,0.0]).T
    tlb = np.matrix([r+far_dist,y_pos,pt[2,0]+0.75]).T

    resolution = np.matrix([0.02,0.02,0.02]).T
    rotation_angle = math.atan2(pt[1,0],pt[0,0])
    print 'rotation_angle:', math.degrees(rotation_angle)

    gr = create_grid(brf,tlb,resolution,pos_list,scan_list,l1,l2,display_list,rotation_angle=rotation_angle)

    if display_list != None:
        collide_pts = gr.grid_to_points()
        if collide_pts.shape[1] > 0:
            collide_pts = tr.Rz(rotation_angle).T*gr.grid_to_points()
            display_list.insert(0,pu.PointCloud(collide_pts,color=(0,0,0)))
    return gr


def overhead_grasp_collision(pt,grid):
    print 'collision points:', grid.grid.sum()
    if grid.grid.sum()>15:
        return True
    else:
        return False


def grasp_location_on_object(obj,display_list=None):
    ''' obj - 3xN numpy matrix of points of the object.
    '''

    pts_2d = obj[0:2,:]
    centroid_2d = pts_2d.mean(1)
    pts_2d_zeromean = pts_2d-centroid_2d
    e_vals,e_vecs = np.linalg.eig(pts_2d_zeromean*pts_2d_zeromean.T)

    # get the min size
    min_index = np.argmin(e_vals)
    min_evec = e_vecs[:,min_index]

    print 'min eigenvector:', min_evec.T
    pts_1d = min_evec.T * pts_2d
    min_size = pts_1d.max() - pts_1d.min()
    print 'spread along min eigenvector:', min_size

    max_height = obj[2,:].max()

    tlb = obj.max(1)
    brf = obj.min(1)
    print 'tlb:', tlb.T
    print 'brf:', brf.T

    resolution = np.matrix([0.005,0.005,0.005]).T
    gr = og3d.occupancy_grid_3d(brf,tlb,resolution)
    gr.fill_grid(obj)
    gr.to_binary(1)
    obj = gr.grid_to_points()

    grid_2d = gr.grid.max(2)
    grid_2d_filled = ni.binary_fill_holes(grid_2d)
    gr.grid[:,:,0] = gr.grid[:,:,0]+grid_2d_filled-grid_2d

    p = np.matrix(np.row_stack(np.where(grid_2d_filled==1))).astype('float')
    p[0,:] = p[0,:]*gr.resolution[0,0]
    p[1,:] = p[1,:]*gr.resolution[1,0]
    p += gr.brf[0:2,0]
    print 'new mean:', p.mean(1).T
    print 'centroid_2d:', centroid_2d.T
    centroid_2d = p.mean(1)


    if min_size<0.12:
        # grasp at centroid.
        grasp_point = np.row_stack((centroid_2d,np.matrix([max_height+gr.resolution[2,0]*2])))
#        grasp_point[2,0] = max_height
        gripper_angle = -math.atan2(-min_evec[0,0],min_evec[1,0])
        grasp_vec = min_evec

        if display_list != None:
            max_index = np.argmax(e_vals)
            max_evec = e_vecs[:,max_index]
            pts_1d = max_evec.T * pts_2d
            max_size = pts_1d.max() - pts_1d.min()

            v = np.row_stack((max_evec,np.matrix([0.])))
            max_end_pt1 = grasp_point + v*max_size/2.
            max_end_pt2 = grasp_point - v*max_size/2.
            display_list.append(pu.Line(max_end_pt1,max_end_pt2,color=(0,0,0)))
    else:
        #----- more complicated grasping location finder.

        for i in range(gr.grid_shape[2,0]):
            gr.grid[:,:,i] = gr.grid[:,:,i]*(i+1)

        height_map = gr.grid.max(2) * gr.resolution[2,0]
    #    print height_map
        print 'height std deviation:',math.sqrt(height_map[np.where(height_map>0.)].var())

#        connect_structure = np.empty((3,3),dtype='int')
#        connect_structure[:,:] = 1
#        for i in range(gr.grid_shape[2,0]):
#            slice = gr.grid[:,:,i]
#            slice_filled = ni.binary_fill_holes(slice)
#            slice_edge = slice_filled - ni.binary_erosion(slice_filled,connect_structure)
#            gr.grid[:,:,i] = slice_edge*(i+1)
#
#        height_map = gr.grid.max(2) * gr.resolution[2,0]
#    #    print height_map
#        print 'contoured height std deviation:',math.sqrt(height_map[np.where(height_map>0.)].var())
#

        high_pts_2d = obj[0:2,np.where(obj[2,:]>max_height-0.005)[1].A1]
        #high_pts_1d = min_evec.T * high_pts_2d
        high_pts_1d = ut.norm(high_pts_2d)
        idx1 = np.argmin(high_pts_1d)
        pt1 = high_pts_2d[:,idx1]

        idx2 = np.argmax(high_pts_1d)
        pt2 = high_pts_2d[:,idx2]

        if np.linalg.norm(pt1)<np.linalg.norm(pt2):
            grasp_point = np.row_stack((pt1,np.matrix([max_height])))
        else:
            grasp_point = np.row_stack((pt2,np.matrix([max_height])))

        vec = centroid_2d-grasp_point[0:2,0]
        gripper_angle = -math.atan2(-vec[0,0],vec[1,0])
        grasp_vec = vec/np.linalg.norm(vec)

        if display_list != None:
            pt1 = np.row_stack((pt1,np.matrix([max_height])))
            pt2 = np.row_stack((pt2,np.matrix([max_height])))
#            display_list.insert(0,pu.CubeCloud(pt1,(0,0,200),size=(0.005,0.005,0.005)))
#            display_list.insert(0,pu.CubeCloud(pt2,(200,0,200),size=(0.005,0.005,0.005)))


    if display_list != None:
        pts = gr.grid_to_points()
        size = resolution
#        size = resolution*2
#        size[2,0] = size[2,0]*2
        #display_list.insert(0,pu.PointCloud(pts,(200,0,0)))
        display_list.append(pu.CubeCloud(pts,color=(200,0,0),size=size))
        display_list.append(pu.CubeCloud(grasp_point,(0,200,200),size=(0.007,0.007,0.007)))

        v = np.row_stack((grasp_vec,np.matrix([0.])))
        min_end_pt1 = grasp_point + v*min_size/2.
        min_end_pt2 = grasp_point - v*min_size/2.

        max_evec = np.matrix((min_evec[1,0],-min_evec[0,0])).T
        pts_1d = max_evec.T * pts_2d
        max_size = pts_1d.max() - pts_1d.min()

        display_list.append(pu.Line(min_end_pt1,min_end_pt2,color=(0,255,0)))

    return grasp_point,gripper_angle



#---------------------- testing functions -------------------

def test_vertical_plane_finding():
    display_list = []
    rot_angle = dict['rot_angle']
    gr = create_vertical_plane_grid(pt,pos_list,scan_list,l1,l2,rotation_angle=rot_angle,
                                    display_list=display_list)

    vertical_plane_points(gr)
    plane_cloud = pu.PointCloud(gr.grid_to_points(),color=(0,150,0))
    display_list.insert(0,plane_cloud)
    po3d.run(display_list)

def test_find_door_handle():
    display_list = []
    rot_angle = dict['rot_angle']
#    pt[2,0] += 0.15
    print 'pt:',pt.A1.tolist()
    gr = create_vertical_plane_grid(pt,pos_list,scan_list,l1,l2,rotation_angle=rot_angle,
                                    display_list=display_list)
    grid_pts_cloud = pu.PointCloud(gr.grid_to_points(),(0,0,255))
    display_list.insert(0,grid_pts_cloud)
    copy_gr = copy.deepcopy(gr)

    obj_pts_list = []
    print 'pt:',pt.A1.tolist()

    # Do I want to change the occupancy threshold when I'm closer? (see the old function test_find_door_handle_close)
    handle_object = find_door_handle(gr,pt,obj_pts_list,rotation_angle=rot_angle,
                                     occupancy_threshold=1,resolution=np.matrix([0.02,0.0025,0.02]).T)

    copy_gr.remove_vertical_plane()
    stickout_pts_cloud = pu.PointCloud(copy_gr.grid_to_points(),(100,100,100))
    display_list.insert(0,stickout_pts_cloud)


    for i,obj_pts in enumerate(obj_pts_list):
        print 'mean:', obj_pts.mean(1).A1.tolist()
        size = [0.02,0.0025,0.02] # look at param for find_door_handle
#        size=gr.resolution.A1.tolist()
        size[0] = size[0]*2
        size[1] = size[1]*2
#        size[2] = size[2]*2
        display_list.append(pu.CubeCloud(obj_pts,color=color_list[i%len(color_list)],size=size))

    laser_point_cloud = pu.CubeCloud(pt,color=(0,200,0),size=(0.005,0.005,0.005))
    po3d.run(display_list)
    #po3d.save(display_list, raw_name+'.png')

def test_segmentation():
    gr = create_segmentation_grid(pt,pos_list,scan_list,l1,l2,
                                  display_flag=True)
    obj_pts_list = segment_objects_points(gr)
    if obj_pts_list == None:
        print 'There is no plane'
        obj_pts_list = []


    pts = gr.grid_to_points()
    d3m.plot_points(pts,color=(1.,1.,1.))
    d3m.plot_points(pt,color=(0,1,0.),mode='sphere')

    for i,obj_pts in enumerate(obj_pts_list):
        size=gr.resolution.A1.tolist()
        size[2] = size[2]*2
        d3m.plot_points(obj_pts,color=color_list[i%len(color_list)])
#        display_list.append(pu.CubeCloud(obj_pts,color=color_list[i%len(color_list)],size=size))
        #display_list.insert(0,pu.PointCloud(obj_pts,color=color_list[i%len(color_list)]))
        print 'mean:', obj_pts.mean(1).T
    
    d3m.show()


def test_grasp_location_on_object():
    display_list = []
#    display_list = None
    gr = create_segmentation_grid(pt,pos_list,scan_list,l1,l2,display_list=display_list)
    obj_pts_list = segment_objects_points(gr)
    closest_obj = find_closest_object(obj_pts_list,pt)
    grasp_location_on_object(closest_obj,display_list)

    po3d.run(display_list)

def test_plane_finding():
    ''' visualize plane finding.
    '''
#   brf = pt + np.matrix([-0.4,-0.2,-0.3]).T
#   brf[0,0] = max(brf[0,0],0.05)
#   print 'brf:', brf.T
#
#   tlb = pt + np.matrix([0.3, 0.2,0.3]).T
#   resolution = np.matrix([0.01,0.01,0.0025]).T

    brf = pt+np.matrix([-0.15,-0.25,-0.2]).T
    brf[0,0] = max(0.07,brf[0,0])
    tlb = pt+np.matrix([0.25, 0.25,0.2]).T

    resolution = np.matrix([0.01,0.01,0.0025]).T

    max_dist = np.linalg.norm(tlb) + 0.2
    min_dist = brf[0,0]

    all_pts = generate_pointcloud(pos_list, scan_list, min_angle, max_angle, l1, l2,save_scan=False,
                                  max_dist=max_dist,min_dist=min_dist)
                                  #max_dist=2.0,min_dist=min_dist)

    gr = og3d.occupancy_grid_3d(brf,tlb,resolution)
    gr.fill_grid(all_pts)
    gr.to_binary(1)
    l = gr.find_plane_indices(assume_plane=True)
    z_min = min(l)*gr.resolution[2,0]+gr.brf[2,0]
    z_max = max(l)*gr.resolution[2,0]+gr.brf[2,0]
    print 'height of plane:', (z_max+z_min)/2
    pts = gr.grid_to_points()

    plane_pts_bool = np.multiply(pts[2,:]>=z_min,pts[2,:]<=z_max)
    plane_pts = pts[:,np.where(plane_pts_bool)[1].A1.tolist()]
    above_pts =pts[:,np.where(pts[2,:]>z_max)[1].A1.tolist()]
    below_pts =pts[:,np.where(pts[2,:]<z_min)[1].A1.tolist()]


    d3m.plot_points(pt,color=(0,1,0.),mode='sphere')
    d3m.plot_points(plane_pts,color=(0,0,1.))
    d3m.plot_points(above_pts,color=(1.0,1.0,1.0))
    d3m.plot_points(below_pts,color=(1.,0.,0.))

    cube_tups = gr.grid_lines()
    d3m.plot_cuboid(cube_tups)

    d3m.show()


def test_approach():
    display_list=[]

#    gr = create_approach_grid(pt,pos_list,scan_list,l1,l2,display_list=display_list,show_pts=False)
    gr = create_approach_grid(pt,pos_list,scan_list,l1,l2,display_list=display_list,show_pts=True)
    t0 = time.time()
    p_erratic,p_edge,h = find_goto_point_surface_1(gr,pt,display_list)
    t1 = time.time()
    print 'aaaaaaaaaaaaaah:', t1-t0

    l = gr.find_plane_indices(assume_plane=True)
    z_min = min(l)*gr.resolution[2,0]+gr.brf[2,0]
    z_max = max(l)*gr.resolution[2,0]+gr.brf[2,0]
    print 'height of plane:', (z_max+z_min)/2

    print 'height of surface in thok0 coord frame:', h
    print 'p_erratic in thok0:', p_erratic.T
    display_list.append(pu.CubeCloud(pt,color=(0,255,0),size=(0.018,0.018,0.018)))
#    display_list.append(pu.CubeCloud(p_erratic,color=(0,250,250),size=(0.007,0.007,0.007)))
    display_list.insert(0,pu.PointCloud(gr.grid_to_points(),color=(100,100,100)))

    po3d.run(display_list)

def test_max_forward():
    max_dist = math.sqrt(pt[0,0]**2+pt[1,0]**2+2.0**1) + 0.3
#    max_dist = np.linalg.norm(pt[0:2]+0.3)
    min_angle,max_angle=math.radians(-40),math.radians(40)

    all_pts = generate_pointcloud(pos_list, scan_list, min_angle, max_angle, l1, l2, max_dist=max_dist,
                                  min_tilt=math.radians(-90))

    display_list = []
    max_x = max_fwd_without_collision(all_pts,0.20,max_dist,display_list)
    po3d.run(display_list)
#    print 'height_mat:', height_mat
    print 'max_x:', max_x

    dict = {'pos_list':pos_list, 'scan_list':scan_list,'l1':l1, 'l2':l2, 'pt':pt}
#    ut.save_pickle(dict,ut.formatted_time()+'_fwd_dict.pkl')

def test_choose_grasp_strategy():
    display_list = []
    far_dist = 0.15
    pt[1,0] += 0.0
    gr = create_overhead_grasp_choice_grid(pt,pos_list,scan_list,l1,l2,far_dist,display_list)
    print 'overhead collide?',overhead_grasp_collision(pt,gr)
    display_list.append(pu.CubeCloud(pt,color=(0,200,0),size=(0.005,0.005,0.005)))
#    pts = generate_pointcloud(pos_list, scan_list, min_angle, max_angle, l1, l2,save_scan=False,
#                              #max_dist=max_dist,min_dist=min_dist)
#                              max_dist=2.0,min_dist=0.1)
#    display_list.append(pu.PointCloud(pts,color=(100,100,100)))
    po3d.run(display_list)

def test_different_surfaces():
    display_list = []
#    all_pts = generate_pointcloud(pos_list, scan_list, math.radians(-40), math.radians(40), l1, l2,save_scan=False,
#                              max_dist=np.Inf, min_dist=-np.Inf,min_tilt=-np.Inf,max_tilt=np.Inf)
##    pts = all_pts[:,np.where(np.multiply(all_pts[0,:]>0.1,all_pts[0,:]<0.4))[1].A1]
##    pts = pts[:,np.where(np.multiply(pts[1,:]<0.3,pts[1,:]>-0.3))[1].A1]
##    pts = pts[:,np.where(pts[2,:]>-0.2)[1].A1]
##    display_list.append(pu.PointCloud(pts,color=(0,200,0)))
#    display_list.append(pu.PointCloud(all_pts,color=(200,0,0)))
#
#    brf = np.matrix([0.05,-0.35,-0.2]).T
#    tlb = np.matrix([0.5, 0.35,0.0]).T
#    resolution = np.matrix([0.01,0.01,0.0025]).T
#    gr = og3d.occupancy_grid_3d(brf,tlb,resolution)
#    gr.fill_grid(all_pts)
#    gr.to_binary(1)
    
#    gr = create_segmentation_grid(pt,pos_list,scan_list,l1,l2,display_list)
    gr = create_approach_grid(pt,pos_list,scan_list,l1,l2,display_list)

    l = gr.find_plane_indices(assume_plane=True)
    max_index = min(max(l)+5,gr.grid_shape[2,0]-1)
    min_index = max(min(l)-5,0)
    l = range(min_index,max_index+1)

    n_points_list = []
    height_list = []
    for idx in l:
        n_points_list.append(gr.grid[:,:,idx].sum())
        height_list.append(idx*gr.resolution[2,0]+gr.brf[2,0])

    pl.bar(height_list,n_points_list,width=gr.resolution[2,0],linewidth=0,align='center',color='y')
    max_occ = max(n_points_list)
    thresh = max_occ/5
    xmin,xmax = pl.xlim()
    t = pl.axis()
    t = (xmin+0.0017,xmax-0.001,t[2],t[3]+50)

#    pl.plot([height_list[0],height_list[-1]],[thresh,thresh],c='r')
    pl.plot([xmin,xmax],[thresh,thresh],c='b')
    pl.title('Histogram of number of points vs z-coordinate of points')
    pl.xlabel('z-coordinate (relative to the laser range finder) (meters)')
    pl.ylabel('Number of points')
    pl.axis(t)
    pl.savefig(pkl_file_name+'.png')
#    pl.show()

#    print 'Mean:', pts.mean(1).T
#    pts_zeromean = pts-pts.mean(1)
#    n_points = pts.shape[1]
#    print 'n_points:', n_points
#    e_vals,e_vecs = np.linalg.eig(pts_zeromean*pts_zeromean.T/n_points)
#
#    min_index = np.argmin(e_vals)
#    min_evec = e_vecs[:,min_index]
#    print 'min eigenvector:', min_evec.T
#    print 'min eigenvalue:', e_vals[min_index]
#    pts_1d = min_evec.T * pts
#    size = pts_1d.max() - pts_1d.min()
#    print 'spread along min eigenvector:', size

#    po3d.run(display_list)

def test_occ_grid():

    gr = create_approach_grid(pt,pos_list,scan_list,l1,l2,display_list=None,show_pts=True)

    pts = gr.grid_to_points()
    display_list=[]
    display_list.append(pu.CubeCloud(pt,color=(0,255,0),size=(0.007,0.007,0.007)))
    display_list.append(pu.PointCloud(pts,color=(200,0,0)))
    po3d.run(display_list)


if __name__ == '__main__':

    p = optparse.OptionParser()
    p.add_option('-f', action='store', type='string', dest='pkl_file_name',
                 help='file.pkl File with the scan,pos dict.')
    p.add_option('--all_pts', action='store_true', dest='show_all_pts',
                 help='show all the points in light grey')

    opt, args = p.parse_args()
    pkl_file_name = opt.pkl_file_name
    show_full_cloud = opt.show_all_pts

    str_parts = pkl_file_name.split('.')
    raw_name = str_parts[-2]
    str_parts = raw_name.split('/')
    raw_name = str_parts[-1]

    dict = ut.load_pickle(pkl_file_name)
    pos_list = dict['pos_list']
    scan_list = dict['scan_list']
    min_angle = math.radians(-40)
    max_angle = math.radians(40)

    l1 = dict['l1']
    l2 = dict['l2']
 #   l2 = -0.055
#    l2 = 0.035

    if dict.has_key('pt'):
        pt = dict['pt']
        print 'dict has key pt'
    else:
        print 'dict does NOT have key pt'
        pt = np.matrix([0.35,0.0,-0.3]).T
        dict['pt'] = pt
        ut.save_pickle(dict,pkl_file_name)

#    charlie_nih()

#    test_grasp_location_on_object()
#    test_find_door_handle()
#    test_vertical_plane_finding_close()
#	test_vertical_plane_finding()
    test_segmentation()
#    test_plane_finding()
#    test_max_forward()
#    test_approach()
#    test_choose_grasp_strategy()
#    test_different_surfaces()
#    test_occ_grid()




