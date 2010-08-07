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



# functions to use scans from hokuyos.

import roslib
roslib.load_manifest('hrl_hokuyo')

import pygame
import pygame.locals
import hokuyo_scan as hs

import hrl_lib.transforms as tr
import hrl_lib.util as ut
#import util as uto
import math, numpy as np
import scipy.ndimage as ni
import pylab as pl

import sys, time
import optparse
import copy

import pygame_utils as pu


# Need to switch to Willow's OpenCV 2.0 python bindings at some point of time.
# from opencv.cv import *
# from opencv.highgui import *

from ctypes import POINTER

MAX_DIST=1.0
# pixel coordinates of origin
org_x,org_y = 320,240+150

color_list = [(255,255,0),(255,0,0),(0,255,255),(0,255,0),(0,0,255),(0,100,100),(100,100,0),
              (100,0,100),(100,200,100),(200,100,100),(100,100,200),(100,0,200),(0,200,100),
              (0,0,0),(0,100,200),(200,0,100),(100,0,100),(255,152,7) ]


def angle_to_index(scan, angle):
    ''' returns index corresponding to angle (radians).
    '''
    return int(min(scan.n_points-1, max(0.,int(round((angle-scan.start_angle)/scan.angular_res)))))

def index_to_angle(scan, index):
    ''' returns angle (radians) corresponding to index.
    '''
    return scan.start_angle+index*scan.angular_res

def get_xy_map(scan,start_angle=math.radians(-360),end_angle=math.radians(360),reject_zero_ten=True,max_dist=np.Inf,min_dist=-np.Inf,sigmoid_hack=False, get_intensities=False):
    """ start_angle - starting angle for points to be considered (hokuyo coord)
        end_angle - ending angle in hokuyo coord frame.
        scan - object of class HokuyoScan
        reject_zero_ten - ignore points at 0 or 10. meters
		
        returns - 2xN matrix, if get_intensities=True returns (2xN matrix, 1xN matrix)
    """
    start_index = angle_to_index(scan,start_angle)
    end_index = angle_to_index(scan,end_angle)

    if sigmoid_hack:
#------ sigmoid hack to increase ranges of points with small range(<0.3) ----
    #    short_range_idxs = np.where(scan.ranges<0.3)
    #    scan.ranges[short_range_idxs] = scan.ranges[short_range_idxs]*1.05

#--- test1
#        scan.ranges = np.multiply(scan.ranges, 1+(1-np.divide(1.,1+np.power(np.e,-5*(scan.ranges-0.3))))*0.1)

#--- test2
        scan.ranges = np.multiply(scan.ranges, 1+(1-np.divide(1.,1+np.power(np.e,-5*(scan.ranges-0.2))))*0.20)

#--- test3
#        scan.ranges = np.multiply(scan.ranges, 1+(1-np.divide(1.,1+np.power(np.e,-5*(scan.ranges-0.3))))*0.2)

#--- test4
#        scan.ranges = np.multiply(scan.ranges, 1+(1-np.divide(1.,1+np.power(np.e,-5*(scan.ranges-0.25))))*0.15)

#------------------
    xy_map = np.matrix(np.row_stack((np.multiply(scan.ranges,np.cos(scan.angles)),
                                     np.multiply(scan.ranges,np.sin(scan.angles))*-1)))

    sub_range = scan.ranges[:,start_index:end_index+1]
    keep_condition = np.multiply(sub_range<=max_dist,sub_range>=min_dist)

    if reject_zero_ten == True:
        keep_condition = np.multiply(keep_condition,np.multiply(sub_range > 0.01,sub_range <= 10.))

    xy_map = xy_map[:,start_index:end_index+1]
    
    idxs = np.where(keep_condition)
    xy_map = np.row_stack((xy_map[idxs],xy_map[idxs[0]+1,idxs[1]]))

    if get_intensities == True:
        intensities = scan.intensities[:,start_index:end_index+1]
        intensities = intensities[idxs]
        return xy_map, intensities
    else:
        return xy_map


def connected_components(p, dist_threshold):
    ''' p - 2xN numpy matrix of euclidean points points (indices give neighbours).
        dist_threshold - max distance between two points in the same connected component.
                         typical value is .02 meters
        returns a list of (p1,p2, p1_index, p2_index): (start euclidean point, end euclidean point, start index, end index) 
        p1 and p2 are 2X1 matrices
    '''
    nPoints = p.shape[1]
    q = p[:,0:nPoints-1]
    r = p[:,1:nPoints]
    diff = r-q
    dists = ut.norm(diff).T
    idx = np.where(dists>dist_threshold) # boundaries of the connected components

    end_list = idx[0].A1.tolist()
    end_list.append(nPoints-1)

    cc_list = []
    start = 0
    for i in end_list:
        cc_list.append((p[:,start], p[:,i], start, i))
        start = i+1

    return cc_list

def find_objects(scan, max_dist, max_size, min_size, min_angle, max_angle, 
                 connect_dist_thresh, all_pts=False):
    ''' max_dist - objects with centroid greater than max_dist will be ignored. (meters)
        max_size - objects greater than this are ignored. (meters)
        min_size - smaller than this are ignored (meters)
        min_angle, max_angle - part of scan to consider.
        connect_dist_thresh - points in scan greater than this will be treated as separate
                              connected components.

        all_pts == True:
            returns [ np matrix: 2xN1, 2xN2 ...] each matrix consists of all the points in the object.
        all_pts == False:
            returns list of (p,q,centroid): (end points,object centroid (2x1 matrices.))
    '''
    xy_map = get_xy_map(scan,min_angle,max_angle)
    cc_list = connected_components(xy_map,connect_dist_thresh)
    object_list = []
    all_pts_list = []
    for i,(p,q,p_idx,q_idx) in enumerate(cc_list):
        object_pts = xy_map[:,p_idx:q_idx+1]
        centroid = object_pts.sum(1)/(q_idx-p_idx+1)
        size = np.linalg.norm(p-q)
        if size>max_size:
            continue
        if size<min_size:
            continue
        if np.linalg.norm(centroid) > max_dist:
            continue
        object_list.append((p,q,centroid))
        all_pts_list.append(object_pts)

    if all_pts == True:
        return all_pts_list
    else:
        return object_list

def find_closest_object_point(scan, pt_interest=np.matrix([0.,0.]).T, min_angle=math.radians(-60),
                              max_angle=math.radians(60),max_dist=0.6,min_size=0.01,max_size=0.3):
    ''' returns 2x1 matrix - centroid of connected component in hokuyo frame closest to pt_interest
        pt_interest - 2x1 matrix in hokuyo coord frame.
        None if no object found.
    '''
    obj_list = find_objects(scan,max_dist,max_size,min_size,min_angle,max_angle,
                            connect_dist_thresh=0.02, all_pts=True)

    if obj_list == []:
        return None

    min_dist_list = []
    for pts in obj_list:
        min_dist_list.append(np.min(ut.norm(pts-pt_interest)))

    min_idx = np.argmin(np.matrix(min_dist_list))
    return obj_list[min_idx].mean(1)

def remove_graze_effect(ranges, angles, skip=1, graze_angle_threshold=math.radians(169.)):
    ''' ranges,angles - 1xN numpy matrix
        skip - which two rays to consider.
        this function changes ranges.
    '''
    nPoints = ranges.shape[1]
    p = ranges[:,0:nPoints-(1+skip)]
    q = ranges[:,(1+skip):nPoints]
    d_mat = np.abs(p-q)
    angles_diff = np.abs(angles[:,(1+skip):nPoints]-angles[:,0:nPoints-(1+skip)])
    l_mat = np.max(np.row_stack((p,q)),0)
    l_mat = np.multiply(l_mat,np.sin(angles_diff))/math.sin(graze_angle_threshold)

    thresh_exceed = d_mat>l_mat
    l1_greater = p>q
    idx_remove_1 = np.where(np.all(np.row_stack((thresh_exceed,l1_greater)),0))
    idx_remove_2 = np.where(np.all(np.row_stack((thresh_exceed,1-l1_greater)),0))
#    print 'idx_remove_1:', idx_remove_1
    p[idx_remove_1] = 1000.
    q[idx_remove_2] = 1000.

def remove_graze_effect_scan(scan, graze_angle_threshold=math.radians(169.)):
    ''' changes scan
    '''
    remove_graze_effect(scan.ranges,scan.angles,1,graze_angle_threshold)

def subtract_scans(scan2,scan1,threshold=0.01):
    if scan1.ranges.shape != scan2.ranges.shape:
        print 'hokuyo_processing.subtract_scans: the two scan.ranges have different shapes.'
        print 'remember to pass remove_graze_effect = False'
        print 'Exiting...'
        sys.exit()

    diff_range = scan2.ranges-scan1.ranges
    idxs = np.where(np.abs(diff_range)<threshold)
    #idxs = np.where(np.abs(diff_range)<0.04)
#    idxs = np.where(np.abs(diff_range)<0.01)
#    idxs = np.where(np.abs(diff_range)<0.005)
    hscan = hs.HokuyoScan(scan2.hokuyo_type,scan2.angular_res,
                          scan2.max_range,scan2.min_range,
                          scan2.start_angle,scan2.end_angle)
    hscan.ranges = copy.copy(scan2.ranges)
    hscan.ranges[idxs] = 0.
    return hscan

def find_door(start_pts_list,end_pts_list,pt_interest=None):
    ''' returns [p1x,p1y], [p2x,p2y] ([],[] if no door found)
        returns line closest to the pt_interest.
        pt_interest - 2x1 matrix
    '''
    if start_pts_list == []:
        return [],[]
#    print 'start_pts_list,end_pts_list',start_pts_list,end_pts_list
    start_pts = np.matrix(start_pts_list).T
    end_pts = np.matrix(end_pts_list).T
    line_vecs = end_pts-start_pts
    line_vecs_ang = np.arctan2(line_vecs[1,:],line_vecs[0,:])

    idxs = np.where(np.add(np.multiply(line_vecs_ang>math.radians(45),
                                              line_vecs_ang<math.radians(135)),
                                  np.multiply(line_vecs_ang<math.radians(-45),
                                              line_vecs_ang>math.radians(-135))
                          ) > 0
                   )[1].A1.tolist()

    if idxs == []:
        return [],[]
    start_pts = start_pts[:,idxs]
    end_pts = end_pts[:,idxs]
#    print 'start_pts,end_pts',start_pts.A1.tolist(),end_pts.A1.tolist()
    if pt_interest == None:
        print 'hokuyo_processing.find_door: pt_interest in None so returning the longest line.'
        length = ut.norm(end_pts-start_pts)
        longest_line_idx = np.argmax(length)
        vec_door = (end_pts-start_pts)[:,longest_line_idx]
        return start_pts[:,longest_line_idx].A1.tolist(),end_pts[:,longest_line_idx].A1.tolist()
    else:
        v = end_pts-start_pts
        q_dot_v = pt_interest.T*v
        p1_dot_v = np.sum(np.multiply(start_pts,v),0)
        v_dot_v = ut.norm(v)
        lam = np.divide((q_dot_v-p1_dot_v),v_dot_v)
        r = start_pts + np.multiply(lam,v)
        dist = ut.norm(pt_interest-r)
        edge_idxs = np.where(np.multiply(lam>1.,lam<0.))[1].A1.tolist()
        min_end_dist = np.minimum(ut.norm(start_pts-pt_interest),ut.norm(end_pts-pt_interest))
        dist[:,edge_idxs] = min_end_dist[:,edge_idxs]

        # dist - distance of closest point within the line segment
        #        or distance of the closest end point.
        # keep line segments that are within some distance threshold.

        keep_idxs = np.where(dist<0.5)[1].A1.tolist()
        if len(keep_idxs) == 0:
            return [],[]

        start_pts = start_pts[:,keep_idxs]
        end_pts = end_pts[:,keep_idxs]

        # find distance from the robot and select furthest line.
        p_robot = np.matrix([0.,0.]).T
        v = end_pts-start_pts
        q_dot_v = p_robot.T*v
        p1_dot_v = np.sum(np.multiply(start_pts,v),0)
        v_dot_v = ut.norm(v)
        lam = np.divide((q_dot_v-p1_dot_v),v_dot_v)
        r = start_pts + np.multiply(lam,v)
        dist = ut.norm(p_robot-r)

        door_idx = np.argmax(dist)
        return start_pts[:,door_idx].A1.tolist(),end_pts[:,door_idx].A1.tolist()

def xy_map_to_np_image(xy_map,m_per_pixel,dilation_count=0,padding=50):
    ''' returns binary numpy image. (255 for occupied
        pixels, 0 for unoccupied)
        2d array
    '''
    min_x = np.min(xy_map[0,:])
    max_x = np.max(xy_map[0,:])
    min_y = np.min(xy_map[1,:])
    max_y = np.max(xy_map[1,:])
    br = np.matrix([min_x,min_y]).T

    n_x = int(round((max_x-min_x)/m_per_pixel)) + padding
    n_y = int(round((max_y-min_y)/m_per_pixel)) + padding
    img = np.zeros((n_x+padding,n_y+padding),dtype='int')
    occupied_pixels = np.matrix([n_x,n_y]).T - np.round((xy_map-br)/m_per_pixel).astype('int')
    
    if dilation_count == 0:
        img[(occupied_pixels[0,:],occupied_pixels[1,:])] = 255
    else:
        img[(occupied_pixels[0,:],occupied_pixels[1,:])] = 1
        connect_structure = np.empty((3,3),dtype='int')
        connect_structure[:,:] = 1
        img = ni.binary_closing(img,connect_structure,iterations=dilation_count)
        img = ni.binary_dilation(img,connect_structure,iterations=1)
        img = img*255

    return img,n_x,n_y,br

def xy_map_to_cv_image(xy_map,m_per_pixel,dilation_count=0,padding=10):
    np_im,n_x,n_y,br = xy_map_to_np_image(xy_map,m_per_pixel,dilation_count,padding)
    cv_im = uto.np2cv(np_im)
    return cv_im,n_x,n_y,br

def hough_lines(xy_map,save_lines=False):
    ''' xy_map - 2xN matrix of points.
        returns start_list, end_list. [[p1x,p1y],[p2x,p2y]...],[[q1x,q1y]...]
                [],[] if no lines were found.
    '''
#    save_lines=True
    m_per_pixel = 0.005
    img_cv,n_x,n_y,br = xy_map_to_cv_image(xy_map,m_per_pixel,dilation_count=1,padding=50)

    time_str = str(time.time())

#    for i in range(3):
#        cvSmooth(img_cv,img_cv,CV_GAUSSIAN,3,3)
#    cvSaveImage('aloha'+str(time.time())+'.png',img_cv)

    storage = cvCreateMemStorage(0)
    method = CV_HOUGH_PROBABILISTIC
    rho = max(int(round(0.01/m_per_pixel)),1)
    rho = 1
    theta = math.radians(1)
    min_line_length = int(0.3/m_per_pixel)
    max_gap = int(0.1/m_per_pixel)
    n_points_thresh = int(0.2/m_per_pixel)

#    cvCanny(img_cv,img_cv,50,200)
#    cvSaveImage('aloha.png',img_cv)
    lines = cvHoughLines2(img_cv, storage, method, rho, theta, n_points_thresh, min_line_length, max_gap)

    if lines.total == 0:
        return [],[]
    pts_start = np.zeros((2, lines.total))
    pts_end   = np.zeros((2, lines.total))

    if save_lines:
        color_dst = cvCreateImage( cvGetSize(img_cv), 8, 3 )
        cvCvtColor( img_cv, color_dst, CV_GRAY2BGR )

    n_lines = lines.total
    for idx, line in enumerate(lines.asarrayptr(POINTER(CvPoint))):
        pts_start[0, idx] = line[0].y
        pts_start[1, idx] = line[0].x
        pts_end[0, idx]   = line[1].y
        pts_end[1, idx]   = line[1].x

    if save_lines:
        pts_start_pixel = pts_start
        pts_end_pixel = pts_end

    pts_start = (np.matrix([n_x,n_y]).T - pts_start)*m_per_pixel + br
    pts_end = (np.matrix([n_x,n_y]).T - pts_end)*m_per_pixel + br

    along_vec = pts_end - pts_start
    along_vec = along_vec/ut.norm(along_vec)
    ang_vec = np.arctan2(-along_vec[0,:],along_vec[1,:])

    res_list = []
    keep_indices = []

    for i in range(n_lines):
        ang = ang_vec[0,i]
        if ang>math.radians(90):
            ang = ang - math.radians(180)
        if ang<math.radians(-90):
            ang = ang + math.radians(180)

        rot_mat = tr.Rz(ang)[0:2,0:2]
        st = rot_mat*pts_start[:,i]
        en = rot_mat*pts_end[:,i]
        pts = rot_mat*xy_map
        x_all = pts[0,:]
        y_all = pts[1,:]
        min_x = min(st[0,0],en[0,0]) - 0.1
        max_x = max(st[0,0],en[0,0]) + 0.1
        min_y = min(st[1,0],en[1,0]) + 0.01
        max_y = max(st[1,0],en[1,0]) - 0.01

        keep = np.multiply(np.multiply(x_all>min_x,x_all<max_x),
                           np.multiply(y_all>min_y,y_all<max_y))

        xy_sub = xy_map[:,np.where(keep)[1].A1.tolist()]
        if xy_sub.shape[1] == 0:
            continue

        a,b,res = uto.fitLine_highslope(xy_sub[0,:].T, xy_sub[1,:].T)
        if res<0.0002:
            res_list.append(res)
            keep_indices.append(i)

    if keep_indices == []:
        return [],[]

    pts_start = pts_start[:,keep_indices]
    pts_end = pts_end[:,keep_indices]

    print 'number of lines:', len(keep_indices)

    if save_lines:
        ut.save_pickle(res_list,'residual_list_'+time_str+'.pkl')
        for i, idx in enumerate(keep_indices):
            s = pts_start_pixel[:,idx]
            e = pts_end_pixel[:,idx]
            cvLine(color_dst, cvPoint(int(s[1]),int(s[0])), cvPoint(int(e[1]),int(e[0])), CV_RGB(*(color_list[i])),
                   3, 8)
        cvSaveImage('lines_'+time_str+'.png',color_dst)

#    cvReleaseMemStorage(storage)
    return pts_start.T.tolist(),pts_end.T.tolist()


#------------- displaying in pygame -----------
def pixel_to_real(x,y, max_dist):
    ''' pixel to hokuyo
         x,y - NX1 matrices (N points)
         max_dist - dist which will be drawn at row 0
    '''
    return (org_y-y)*max_dist/400.,(org_x-x)*max_dist/400.
#    return org_x-(400./max_dist)*y, org_y-(400./max_dist)*x

def coord(x,y, max_dist):
    '''hokuyo coord frame to pixel (x,y) - floats
         x,y - NX1 matrices (N points)
         max_dist - dist which will be drawn at row 0
    '''
    return org_x-(400./max_dist)*y, org_y-(400./max_dist)*x

def draw_points(srf,x,y,color,max_dist,step=1):
    ''' step - set > 1 if you don't want to draw all the points.
    '''
    if len(x.A1) == 0:
        return
    x_pixel, y_pixel = coord(x.T,y.T,max_dist)
    for i in range(0,x_pixel.shape[0],step):
        pygame.draw.circle(srf, color, (int(x_pixel[i,0]+0.5), int(y_pixel[i,0]+0.5)), 2, 0)

def draw_hokuyo_scan(srf, scan, ang1, ang2, color, reject_zero_ten=True,step=1):
    ''' reject_zero_ten - don't show points with 0 or 10. range readings.
        step - set > 1 if you don't want to draw all the points.
    '''
    pts = get_xy_map(scan, ang1, ang2, reject_zero_ten=reject_zero_ten)
#    pts = get_xy_map(scan, reject_zero_ten=reject_zero_ten)
    max_dist = MAX_DIST
    draw_points(srf,pts[0,:],pts[1,:],color,max_dist,step=step)


def test_connected_comps(srf, scan):
    #colors_list = [(200,0,0), (0,255,0), (100,100,0), (100,0,100), (0,100,100)]
    n_colors = len(color_list)
    cc_list = connected_components(get_xy_map(scan,math.radians(-60),math.radians(60)),0.03)
# draw the connected components as lines
    for i,(p,q,p_idx,q_idx) in enumerate(cc_list):
        c1,c2 = p.A1.tolist(),q.A1.tolist()
        c1,c2 = coord(c1[0],c1[1], max_dist=MAX_DIST), coord(c2[0],c2[1], max_dist=MAX_DIST)
        pygame.draw.line(srf,color_list[i%n_colors],c1,c2,2)

def test_find_objects(srf, scan):
    obj_list = find_objects(scan, max_dist=0.6, max_size=0.3, min_size=0.01,
                            min_angle=math.radians(-60), max_angle=math.radians(60),
                            connect_dist_thresh=0.02)
    print 'number of objects:', len(obj_list)
    #colors_list = [(200,0,0), (0,255,0), (100,100,0), (100,0,100), (0,100,100)]
    for i,(p,q,c) in enumerate(obj_list):
        if i>4:
            break
        c1,c2 = p.A1.tolist(),q.A1.tolist()
        c1,c2 = coord(c1[0],c1[1], max_dist=MAX_DIST), coord(c2[0],c2[1], max_dist=MAX_DIST)
        pygame.draw.line(srf,color_list[i],c1,c2,2)

def test_find_closest_object_point(srf, scan):
    pt_interest = np.matrix([0.,0.]).T
#    pt_interest = np.matrix([0.3,-0.04]).T
    p = find_closest_object_point(scan, pt_interest)
    if p == None:
        return
    c1,c2 = coord(p[0,0],p[1,0], max_dist=MAX_DIST)
    pygame.draw.circle(srf, (0,200,0), (c1,c2), 3, 0)
    c1,c2 = coord(pt_interest[0,0],pt_interest[1,0], max_dist=MAX_DIST)
    pygame.draw.circle(srf, (200,200,0), (c1,c2), 3, 0)

def tune_graze_effect_init():
    sl = pu.Slider((340, 20), 10)
    return sl

def tune_graze_effect_update(sl, srf, scan):
    sl.update()
    val = sl.value/255.
    angle = 160+val*20

    remove_graze_effect_scan(scan,graze_angle_threshold=math.radians(angle))
    draw_hokuyo_scan(srf,scan,math.radians(-90), math.radians(90),color=(200,0,0),reject_zero_ten=False)
    points_removed = np.where(np.matrix(scan.ranges)>10.)[0].shape[1]

    sl.set_text('angle: %.2f, points_removed: %d'%(angle, points_removed))
    sl.render(srf)
    if sl.clicked:
        return True
    else:
        return False

def test_find_handle_init():
    fr = np.matrix([1.28184669,0.05562259]).T
    bl = np.matrix([1.19585711,-0.06184923]).T
    max_dist = MAX_DIST
    x_fr, y_fr = coord(fr[0,0],fr[1,0],max_dist)
    x_bl, y_bl = coord(bl[0,0],bl[1,0],max_dist)
    pygame.draw.rect(srf,(200,0,200),pygame.Rect(x_bl,y_bl,x_fr-x_bl,y_fr-y_bl),1)
    cv_handle_template = create_handle_template(dict['scan'],dict['bk_lt'],dict['fr_rt'])


def test_find_lines():
    pts = get_xy_map(scan,math.radians(-60),math.radians(60))
    p_start_list,p_end_list = hough_lines(pts)
    if p_start_list == []:
        return

    #-------  to test door finding ----------
#    p_start,p_end = find_door(p_start_list,p_end_list)
#    if p_start == []:
#        return
#    p_start_list,p_end_list = [p_start],[p_end]

    n_colors = len(color_list)

    for i,(p1,p2) in enumerate(zip(p_start_list,p_end_list)):
        c1,c2 = coord(p1[0],p1[1], max_dist=MAX_DIST), coord(p2[0],p2[1], max_dist=MAX_DIST)
        pygame.draw.line(srf,color_list[i%n_colors],c1,c2,2)


if __name__ == '__main__':

    p = optparse.OptionParser()
    p.add_option('-t', action='store', type='string', dest='hokuyo_type',
                 help='hokuyo_type. urg or utm')
    p.add_option('-a', action='store', type='int', dest='avg',
                 help='number of scans to average', default=1)
    p.add_option('-n', action='store', type='int', dest='hokuyo_number',
                 help='hokuyo number. 0,1,2 ...')
    p.add_option('-f', action='store_true', dest='flip',
                 help='flip the hokuyo scan')
    p.add_option('--ang_range', type='float', dest='ang_range',
                 help='max angle of the ray to display (degrees)',
                 default=360.)

    opt, args = p.parse_args()
    hokuyo_type = opt.hokuyo_type
    hokuyo_number = opt.hokuyo_number
    avg_number = opt.avg
    flip = opt.flip
    ang_range = opt.ang_range
    ang_range = math.radians(ang_range)

#------- which things to test ---------
    test_graze_effect_flag = False
    test_find_objects_flag = False
    test_find_closest_object_point_flag = False
    test_show_change_flag = False
    test_find_lines_flag = False
#--------------------------------------    
    # Initialize pygame
#    pygame.init()

    # Open a display
    srf = pygame.display.set_mode((640,480))
    pygame.display.set_caption(hokuyo_type+' '+str(hokuyo_number))

    fps = 100
    loopFlag = True
    clk = pygame.time.Clock()


    if hokuyo_type == 'utm':
        h = hs.Hokuyo('utm',hokuyo_number,start_angle=-ang_range,
                      end_angle=ang_range,flip=flip)
    elif hokuyo_type == 'urg':
        h = hs.Hokuyo('urg',hokuyo_number,flip=flip)
    else:
        print 'unknown hokuyo type: ', hokuyo_type
        print 'Exiting...'
        sys.exit()

#    scan1 = h.get_scan(avoid_duplicate=True)
#    sys.exit()

#---------- initializations -------------
    if test_graze_effect_flag:
        sl = tune_graze_effect_init()

#--------- and now loop -----------
    if test_show_change_flag:
        scan_prev = h.get_scan(avoid_duplicate=True, avg=avg_number, remove_graze=False)
        n_ch_pts_list = []
    while loopFlag:

        widget_clicked = False

        # Clear the screen
        srf.fill((255,255,255))

        # Draw the urg
        pygame.draw.circle(srf, (200,0,0), (org_x,org_y), 10, 0)

        #display all the points
        if test_graze_effect_flag or test_show_change_flag:
            # here we don't want any filtering on the scan.
            scan = h.get_scan(avoid_duplicate=True, avg=avg_number, remove_graze=False)
        else:
            scan = h.get_scan(avoid_duplicate=True, avg=avg_number, remove_graze=True)

        if test_show_change_flag == False:
            #draw_hokuyo_scan(srf,scan,math.radians(-70), math.radians(70),color=(0,0,200))
            #draw_hokuyo_scan(srf,scan,math.radians(-90), math.radians(90),color=(0,0,200))
            draw_hokuyo_scan(srf,scan,math.radians(-135),math.radians(135),color=(0,0,200))
        else:
            diff_scan = subtract_scans(scan,scan_prev)
            scan_prev = scan
            pts = get_xy_map(diff_scan,math.radians(-40), math.radians(40),reject_zero_ten=True)
            n_ch_pts_list.append(pts.shape[1])
            max_dist = MAX_DIST
            draw_points(srf,pts[0,:],pts[1,:],color=(0,200,0),max_dist=max_dist)

#        test_connected_comps(srf, scan)
        if test_find_objects_flag:
            test_find_objects(srf, scan)
        if test_find_closest_object_point_flag:
            test_find_closest_object_point(srf, scan)
        if test_graze_effect_flag:
            widget_clicked |= tune_graze_effect_update(sl,srf, scan)
        if test_find_lines_flag:
            test_find_lines()

        pygame.display.flip()

        events = pygame.event.get()
        for e in events:
            if e.type==pygame.locals.QUIT:
                loopFlag=False
            if e.type==pygame.locals.KEYDOWN:
                if e.key == 27: # Esc
                    loopFlag=False
            if widget_clicked == False:
                if e.type==pygame.locals.MOUSEMOTION:
                    if e.buttons[0] == 1:
                        # left button
                        org_x += e.rel[0]
                        org_y += e.rel[1]
                    if e.buttons[2] == 1:
                        # right button
                        MAX_DIST *= (1+e.rel[1]*0.01)
                        MAX_DIST = max(MAX_DIST,0.1)

        # Try to keep the specified framerate   
        clk.tick(fps)
    if test_show_change_flag:
        ut.save_pickle(n_ch_pts_list,ut.formatted_time()+'_ch_pts_list.pkl')





