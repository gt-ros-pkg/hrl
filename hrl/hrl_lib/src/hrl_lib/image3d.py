import roslib; roslib.load_manifest('hrl_lib')

import cv
import numpy as np
import hrl_lib.tf_utils as tfu
import hrl_opencv.blob as blob
import copy
import pdb

##
# calculate normal to a group of points
# @param points3d 3d points 3xn matrix
# @param p direction to return normals wrt
def calc_normal(points3d, p=np.matrix([-1,0,0.]).T):
    #pdb.set_trace()
    u, s, vh = np.linalg.svd(np.cov(points3d))
    u = np.matrix(u)

    # Pick normal
    if (u[:,2].T * p)[0,0] < 0:
        normal = -u[:,2]
    else:
        normal = u[:,2]
    return normal

##
# give indices of 2d points that are in the image frame
#
# @param points 2xn matrix of 2d points
# @param cal_obj calibration object
def indices_of_points_in_frame(points, cal_obj):
    valid_indices = np.where(np.multiply(np.multiply(points[0,:] > 0, points[0,:] < cal_obj.w-.6),
                                         np.multiply(points[1,:] > 0, points[1,:] < cal_obj.h-.6)))[1].A1
    return valid_indices


##
# Given a 2d location and a window size, cut out a square of intensity values, 
# returns (winsize*winsize)x1 array in range [0,1] just use local template image
#
# @param location
# @param bw_image
# @param winsize
# @param resize_to
# @param flatten whether to return a flat 1 column mat or the 3d matrix of pixels
def local_window(location, bw_image, winsize, resize_to=None, flatten=True):
    loc = np.matrix(np.round(location), dtype='int')
    start = loc - winsize
    patch_size = winsize*2+1
    img_width = bw_image.shape[1]
    img_height = bw_image.shape[0]

    r = blob.Rect(start[0,0], start[1,0], patch_size, patch_size).keep_inside(0, img_width-1, 0, img_height-1) #640, 480
    if r.width < patch_size or r.height < patch_size:
        return None
    else:
        #pdb.set_trace()
        #subrect_cpy = np.zeros((subrect.shape[0], subrect.shape[1], subrect.shape[2]), dtype='uint8')
        #cv.Copy(cv.fromarray(subrect), cv.fromarray(subrect_cpy))
        subrect = bw_image[r.y:r.y+r.height, r.x:r.x+r.width, :]
        #subrect_cpy = np.array(copy.copy(subrect.tolist()), dtype='uint8')
        #subrect = subrect_cpy

        if resize_to != None:
           rescaled = np.zeros((resize_to*2+1, resize_to*2+1, subrect.shape[2]), dtype='uint8')
           cv.Resize(cv.fromarray(subrect), cv.fromarray(rescaled), cv.CV_INTER_LINEAR)
           subrect = rescaled

        if flatten:
            #try:
            #    subrect = np.array(subrect.tolist(), copy=True, order='C')
            #except TypeError, e:
            #    print 'typeerror:', e
            #    pdb.set_trace()
            #    print 'error'
            try:
                intensity = np.matrix(np.reshape(subrect, (subrect.shape[0]*subrect.shape[1]*subrect.shape[2], 1))) / 255.
            except TypeError, e:
                #print 'TypeError:', e, 'retrying anyway?'
                intensity = np.matrix(np.reshape(subrect, (subrect.shape[0]*subrect.shape[1]*subrect.shape[2], 1))) / 255.
                #print intensity.shape
            #try:
            #    intensity = np.matrix(subrect.flatten()) / 255.
            #except TypeError, e:
            #    print 'TypeError:', e, '!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            #    return None
        else:
            intensity = subrect

        return intensity


##
# Combines a point cloud with an intensity image
#
# @param points_in_laser_frame 3d points in laser frame
# @param image numpy image (ex: np.asarray(cvimage))
# @param cal_obj camera calibration object
# @return 3xn int matrix of 3d points that are visible in the camera's frame
# @return 3xn int matrix of rgb values of those points in range [0,1]
def combine_scan_and_image_laser_frame(points_in_laser_frame, image_T_laser, image, cal_obj):
    points_in_image_frame = tfu.transform_points(image_T_laser, points_in_laser_frame)
    return combine_scan_and_image(points_in_image_frame, image, cal_obj)

##
# Combines a point cloud with an intensity image
#
# @param points_in_image_frame 3d points in image frame
# @param imagea numpy image (ex: np.asarray(cvimage))
# @param cal_obj camera calibration object
# @return 3xn int matrix of 3d points that are visible in the camera's frame
# @return 3xn int matrix of rgb values of those points in range [0,1]
def combine_scan_and_image(points_in_image_frame, imagea, cal_obj):
    p2d = cal_obj.project(points_in_image_frame)
    valid_indicies = indices_of_points_in_frame(p2d, cal_obj)

    vp2d = p2d[:, valid_indicies]
    vp2d = np.matrix(np.round(vp2d), dtype='int')
    #vpoints = points_in_laser_frame[:, valid_indicies]
    vpoints = points_in_image_frame[:, valid_indicies]

    # imagea = np.asarray(image)
    intensity_channels = imagea[vp2d[1,:].A1, vp2d[0,:].A1, :]
    #pdb.set_trace()
    return vpoints, (np.matrix(intensity_channels).T / 255.), vp2d

##
# Given a group of points, select the ones within rectangular volume
#
# @param center
# @param w
# @param h
# @param depth
# @param points 3xn mat
def select_rect(center, w, h, depth, points):
    limits = [[center[0,0]-w, center[0,0]+w], 
                 [center[1,0]-h, center[1,0]+h], 
                 [center[2,0]-depth, center[2,0]+depth]]
    return select_volume(limits, points), limits


##
# Given a group of points and rectangular limits return points within limits
#
# @param limits [[xmin, xmax], [ymin, ymax], [...]]
# @param points 3xn mat
def select_volume(limits, points):
    xlim, ylim, zlim = limits
    xlim_sat = np.multiply(points[0, :] > xlim[0], points[0, :] < xlim[1])
    ylim_sat = np.multiply(points[1, :] > ylim[0], points[1, :] < ylim[1])
    zlim_sat = np.multiply(points[2, :] > zlim[0], points[2, :] < zlim[1])
    selected = np.multiply(np.multiply(xlim_sat, ylim_sat), zlim_sat)
    if np.sum(selected) <= 0:
        return None
    return points[:, np.where(selected)[1].A1]

    #points_x   = points[:, np.where(np.multiply(points[0, :] > xlim[0], points[0, :] < xlim[1]))[1].A1]
    #points_xy  = points_x[:, np.where(np.multiply(points_x[1, :] > ylim[0], points_x[1, :] < ylim[1]))[1].A1]
    #points_xyz = points_xy[:, np.where(np.multiply(points_xy[2, :] > zlim[0], points_xy[2, :] < zlim[1]))[1].A1]
    #return points_xyz


