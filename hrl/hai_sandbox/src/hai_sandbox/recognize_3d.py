import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import scipy.spatial as sp
import hrl_lib.util as ut
import sys
import tf.transformations as tr
import sensor_msgs.msg as sm
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
import numpy as np
import cv
import pdb


def indices_of_points_in_view(points, cal_obj):
    valid_indices = np.where(np.multiply(np.multiply(points[0,:] > 0, points[0,:] < cal_obj.w-.6),
                                         np.multiply(points[1,:] > 0, points[1,:] < cal_obj.h-.6)))[1].A1
    return valid_indices
    #return points[:, valid_indices]

##
#
# @param cal_obj camera calibration object
# @return 3xn int matrix of 3d points that are visible in the camera's frame
# @return 3xn int matrix of rgb values of those points
def laser_point_intensity(points_iframe, image, cal_obj):
    p2d = cal_obj.project(points_iframe)
    valid_indicies = indices_of_points_in_view(p2d, cal_obj)

    vp2d = p2d[:, valid_indicies]
    vp2d = np.matrix(np.round(vp2d), dtype='int')
    vpoints = points_iframe[:, valid_indicies]

    imagea = np.asarray(image)
    intensity_channels = imagea[vp2d[1,:].A1, vp2d[0,:].A1, :]
    return vpoints, np.matrix(intensity_channels).T

def select_volume(limits, points):
    xlim, ylim, zlim = limits
    points_x   = points[:, np.where(np.multiply(points[0, :] > xlim[0], points[0, :] < xlim[1]))[1].A1]
    points_xy  = points[:, np.where(np.multiply(points[1, :] > ylim[0], points[1, :] < ylim[1]))[1].A1]
    points_xyz = points[:, np.where(np.multiply(points[2, :] > zlim[0], points[2, :] < zlim[1]))[1].A1]
    return points_xyz


if __name__ == '__main__':
    rospy.init_node('better_recognize3d')
    highres_cloud_pub = rospy.Publisher('colorcloud_highres', sm.PointCloud)

    fname = sys.argv[1]
    data_pkl = ut.load_pickle(fname)

    #pdb.set_trace()
    orig_pc = ru.pointcloud_to_np(data_pkl['points_laser'])
    points_bf = tfu.transform_points(data_pkl['laser_T_bf'], orig_pc)
    print 'original point cloud size', orig_pc.shape[1]
    points_cam = tfu.transform_points(data_pkl['pro_T_bf'], points_bf)

    # Pair intensity and ranged data
    points_valid, colors = laser_point_intensity(points_cam, cv.LoadImageM(data_pkl['high_res']), data_pkl['prosilica_cal'])
    points_valid = np.row_stack((points_valid, colors))
    print 'number of points visible in camera',  points_valid.shape[1]

    #publish mapped cloud to verify
    #pc_msg = ru.np_to_rgb_pointcloud(points_valid, colors, 'high_def_optical_frame')
    #highres_cloud_pub.publish(pc_msg)
    #r = rospy.rate(10)
    #for i in range(10):
    #    if not rospy.is_shutdown():
    #        r.sleep()

    # cut cloud to volume
    center_point = data_pkl['touch_point']
    w = .2
    h = .2
    depth = .2
    limits = [[center_point[0,0]-w, center_point[0,0]+w], 
              [center_point[1,0]-h, center_point[1,0]+h], 
              [center_point[2,0]-depth, center_point[2,0]+depth]]
    pdb.set_trace()
    points_in_volume = select_volume(limits, points_valid)
    print 'number of points in voi', points_in_volume.shape[1]

    # sample uniformly in this volume, avoid places without points, 
    # look at 2d and 3d representation to extract features, pass through an SVM
    #    average color
    #    local geometry
    #    local intensity image of different sizes
    resolution = .05
    #tree = sp.KDTree(np.array(reduced_data.T))
    voi_tree = sp.KDTree(np.array(points_in_volume.T))
    sampled_points = []
    for x in (np.arange(limits[0][0], limits[0][1], resolution) + (resolution/2.)):
        for y in (np.arange(limits[1][0], limits[1][1], resolution) + (resolution/2.)):
            for z in (np.arange(limits[0][0], limits[2][1], resolution) + (resolution/2.)):
                sampled_points.append([x,y,z])
    sampled_points_tree = sp.KDTree(np.array(sampled_points))
    results = voi_tree.query_ball_tree(sampled_points_tree, resolution / 2.)
    pdb.set_trace()
    print 'Done!'



    #extract features
    # intensity cloud 
    #   local color histogram
    #   local geometric features
    #
    # intensity image from points
    #   depth edges
    #
    # image edges
    #
    # image keypoints?

    #tree = sp.KDTree(np.array(reduced_data.T))
    #pdb.set_trace()

















