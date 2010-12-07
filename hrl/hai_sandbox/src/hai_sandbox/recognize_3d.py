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
def laser_point_intensity(points_in_laser_frame, image_T_laser, image, cal_obj):
    points_in_image_frame = tfu.transform_points(image_T_laser, points_in_laser_frame)
    p2d = cal_obj.project(points_in_image_frame)
    valid_indicies = indices_of_points_in_view(p2d, cal_obj)

    vp2d = p2d[:, valid_indicies]
    vp2d = np.matrix(np.round(vp2d), dtype='int')
    #vpoints = points_in_laser_frame[:, valid_indicies]
    vpoints = points_in_image_frame[:, valid_indicies]

    imagea = np.asarray(image)
    intensity_channels = imagea[vp2d[1,:].A1, vp2d[0,:].A1, :]
    #pdb.set_trace()
    return vpoints, (np.matrix(intensity_channels).T / 255.)

def select_volume(limits, points):
    xlim, ylim, zlim = limits
    xlim_sat = np.multiply(points[0, :] > xlim[0], points[0, :] < xlim[1])
    ylim_sat = np.multiply(points[1, :] > ylim[0], points[1, :] < ylim[1])
    zlim_sat = np.multiply(points[2, :] > zlim[0], points[2, :] < zlim[1])
    pdb.set_trace()
    selected = np.multiply(np.multiply(xlim_sat, ylim_sat), zlim_sat)
    if np.sum(selected) <= 0:
        return None
    return points[:, np.where(selected)[1].A1]

    #points_x   = points[:, np.where(np.multiply(points[0, :] > xlim[0], points[0, :] < xlim[1]))[1].A1]
    #points_xy  = points_x[:, np.where(np.multiply(points_x[1, :] > ylim[0], points_x[1, :] < ylim[1]))[1].A1]
    #points_xyz = points_xy[:, np.where(np.multiply(points_xy[2, :] > zlim[0], points_xy[2, :] < zlim[1]))[1].A1]
    #return points_xyz


if __name__ == '__main__':
    rospy.init_node('better_recognize3d')
    highres_cloud_pub = rospy.Publisher('colorcloud_highres', sm.PointCloud)
    orig_cloud_pub = rospy.Publisher('orig_cloud_pub', sm.PointCloud)
    voi_cloud_pub = rospy.Publisher('voi_cloud_pub', sm.PointCloud)

    fname = sys.argv[1]
    data_pkl = ut.load_pickle(fname)

    print 'original frame of pointcloud is ', data_pkl['points_laser'].header.frame_id
    bl_pc = ru.pointcloud_to_np(data_pkl['points_laser'])
    #pdb.set_trace()
    print 'original point cloud size', bl_pc.shape[1]
    # points_bf = tfu.transform_points(data_pkl['laser_T_bf'], bl_pc)
    # points_cam = tfu.transform_points(data_pkl['pro_T_bf'], points_bf)

    # Pair intensity and ranged data
    image_T_laser = data_pkl['pro_T_bl']
    points_valid_pro, colors = laser_point_intensity(bl_pc, image_T_laser, 
            cv.LoadImageM(data_pkl['high_res']), data_pkl['prosilica_cal'])
    #points_valid, colors = laser_point_intensity(points_cam, cv.LoadImageM(data_pkl['high_res']), data_pkl['prosilica_cal'])

    #points_valid_pro = np.row_stack((points_valid_pro, colors))
    print 'number of points visible in camera',  points_valid_pro.shape[1]

    # cut cloud to volume
    center_point_bl = data_pkl['touch_point']
    print 'center_point is at', center_point_bl.T
    w = .5
    h = .5
    depth = .5
    limits_bl = [[center_point_bl[0,0]-w, center_point_bl[0,0]+w], 
                 [center_point_bl[1,0]-h, center_point_bl[1,0]+h], 
                 [center_point_bl[2,0]-depth, center_point_bl[2,0]+depth]]
    #pdb.set_trace()
    bl_T_pro = np.linalg.inv(data_pkl['pro_T_bl'])
    print bl_T_pro.shape, points_valid_pro[0:3,:].shape
    points_in_volume_bl = select_volume(limits_bl, tfu.transform_points(bl_T_pro, points_valid_pro[0:3,:]))
    if points_in_volume_bl == None:
        print 'uh oh'
        exit(-1)
    print 'number of points in voi', points_in_volume_bl.shape[1]

    #publish mapped cloud to verify
    pc_msg  = ru.np_to_rgb_pointcloud(points_valid_pro, colors, 'high_def_optical_frame')
    opc_msg = ru.np_to_pointcloud(bl_pc, 'base_link')
    voi_msg = ru.np_to_pointcloud(points_in_volume_bl, 'base_link')
    #pdb.set_trace()

    r = rospy.Rate(10)
    print 'publishing'
    while not rospy.is_shutdown():
    #for i in range(10):
        #if not rospy.is_shutdown():
        orig_cloud_pub.publish(opc_msg)
        highres_cloud_pub.publish(pc_msg)
        voi_cloud_pub.publish(voi_msg)
        r.sleep()

    # sample uniformly in this volume, avoid places without points, 
    # look at 2d and 3d representation to extract features, pass through an SVM
    #    average color
    #    local geometry
    #    local intensity image of different sizes
    resolution = .05
    #tree = sp.KDTree(np.array(reduced_data.T))
    voi_tree = sp.KDTree(np.array(points_in_volume_pro.T))
    sampled_points = []
    for x in (np.arange(limits_bl[0][0], limits_bl[0][1], resolution) + (resolution/2.)):
        for y in (np.arange(limits_bl[1][0], limits_bl[1][1], resolution) + (resolution/2.)):
            for z in (np.arange(limits_bl[0][0], limits_bl[2][1], resolution) + (resolution/2.)):
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

















