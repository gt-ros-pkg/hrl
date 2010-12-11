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
import laser_interface.dimreduce as dr
import laser_interface.blob as blob
import glob
import os.path as pt
import threading
import Queue as qu
import visualization_msgs.msg as vm
import hrl_lib.viz as viz


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


def indices_of_points_in_view(points, cal_obj):
    valid_indices = np.where(np.multiply(np.multiply(points[0,:] > 0, points[0,:] < cal_obj.w-.6),
                                         np.multiply(points[1,:] > 0, points[1,:] < cal_obj.h-.6)))[1].A1
    return valid_indices
    #return points[:, valid_indices]


## just use local template image
def local_window(location, bw_image, winsize):
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
        subrect = bw_image[r.y:r.y+r.height, r.x:r.x+r.width, :]
        intensity = np.reshape(subrect, (subrect.shape[0]*subrect.shape[1]*subrect.shape[2], 1))
        return intensity

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


def select_rect(center_point_bl, w, h, depth, points):
    limits_bl = [[center_point_bl[0,0]-w, center_point_bl[0,0]+w], 
                 [center_point_bl[1,0]-h, center_point_bl[1,0]+h], 
                 [center_point_bl[2,0]-depth, center_point_bl[2,0]+depth]]
    return select_volume(limits_bl, points), limits_bl


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

def calculate_features_given_point(sampled_point, voi_tree, grid_resolution, data_pkl, win_size, intensity_image_array):
    indices_list = voi_tree.query_ball_point(np.array(sampled_point), grid_resolution/2.)
    if len(indices_list) > 4:
        points_in_ball_bl = np.matrix(voi_tree.data.T[:, indices_list])
        
        #calc normal
        normal_bl = calc_normal(points_in_ball_bl[0:3,:])
        
        #calc average color
        avg_color = np.mean(points_in_ball_bl[3:,:], 1)
        mean_3d_bl = np.mean(points_in_ball_bl[0:3,:], 1)
        
        #project mean into 2d
        mean_2d = data_pkl['prosilica_cal'].project(tfu.transform_points(data_pkl['pro_T_bl'], \
                mean_3d_bl))
        
        #get local features
        features = local_window(mean_2d, intensity_image_array, win_size)
        if features != None:
            return [normal_bl, avg_color, features]

    return None


def calculate_features(fname, grid_resolution=.05, win_size=15):
    print 'processing', fname
    data_pkl = ut.load_pickle(fname)

    ##
    ## For each data pickle, generate features...
    ##
    print 'original frame of pointcloud is ', data_pkl['points_laser'].header.frame_id
    bl_pc = ru.pointcloud_to_np(data_pkl['points_laser'])
    print 'original point cloud size', bl_pc.shape[1]

    # Pair intensity and ranged data
    image_T_laser = data_pkl['pro_T_bl']
    image_fname = pt.join(pt.split(fname)[0], data_pkl['high_res'])
    intensity_image = cv.LoadImageM(image_fname)
    points_valid_pro, colors = laser_point_intensity(bl_pc, image_T_laser, intensity_image, data_pkl['prosilica_cal'])
    print 'number of points visible in camera',  points_valid_pro.shape[1]

    # cut cloud to volume
    center_point_bl = data_pkl['touch_point'][0]
    print 'center_point is at', center_point_bl.T

    bl_T_pro = np.linalg.inv(data_pkl['pro_T_bl'])
    colored_points_valid_bl = np.row_stack((tfu.transform_points(bl_T_pro, \
            points_valid_pro[0:3,:]), colors))
    points_in_volume_bl, limits_bl = select_rect(center_point_bl, .5, .5, .5, \
            colored_points_valid_bl)
    if points_in_volume_bl == None:
        print 'No 3D points in volume of interest!'
        return None
    print 'number of points in voi', points_in_volume_bl.shape[1]

    # sample uniformly in this volume
    voi_tree = sp.KDTree(np.array(points_in_volume_bl[0:3,:].T))
    closest_center_point_bl = np.matrix(voi_tree.data.T[:, voi_tree.query(center_point_bl.T, 1)[1]])
    sampled_points = []
    num_cells = 0
    empty_cells = 0
    for x in (np.arange(limits_bl[0][0], limits_bl[0][1], grid_resolution) + (grid_resolution/2.)):
        for y in (np.arange(limits_bl[1][0], limits_bl[1][1], grid_resolution) + (grid_resolution/2.)):
            for z in (np.arange(limits_bl[0][0], limits_bl[2][1], grid_resolution) + (grid_resolution/2.)):
                #ignore points too close to center point
                if (np.linalg.norm(closest_center_point_bl - np.matrix([x,y,z]).T) > grid_resolution):
                    sampled_points.append([x,y,z])
                #else:
                #    empty_cells = empty_cells + 1
                num_cells = num_cells + 1

    intensity_image_array = np.asarray(intensity_image)
    negative_samples = []
    negative_sample_points = []
    positive_samples = []
    positive_sample_points = []

    non_empty = 0
    empty_queries = 0

    for sampled_point in sampled_points:
        features = calculate_features_given_point(sampled_point, voi_tree,\
                        grid_resolution, data_pkl, win_size, intensity_image_array)
        if features != None:
            negative_samples.append(features)
            negative_sample_points.append(sampled_point)
            non_empty = non_empty + 1
        else:
            empty_queries = empty_queries + 1

    positive_samples.append(calculate_features_given_point(closest_center_point_bl.T.A1,\
                                voi_tree, grid_resolution, data_pkl, win_size, intensity_image_array))
    positive_sample_points.append(closest_center_point_bl.T.A1)
    print 'empty queries', empty_queries, 'non empty', non_empty

    return bl_pc, colored_points_valid_bl, [positive_samples, positive_sample_points], [negative_samples, negative_sample_points]

def train_model(dirname, fname, model_name):
    file_names = glob.glob(pt.join(dirname, '*.pkl'))
    #rospy.init_node('better_recognize3d')
    positive_examples = []
    negative_examples = []
    for fname in file_names:
        pos, ppoints, neg, npoints = calculate_features(dirname, fname)
        positive_examples += pos
        negative_examples += neg

    print 'num positive samples', len(positive_examples), 
    print 'num negative samples', len(negative_examples)
    ut.save_pickle([positive_examples, negative_examples], 'recognize3d.pkl')
    #Train classifier...
    #Run pca on intensity

    return saved_model


def test_model(model, test_dir):
    for d in test_dir:
        data = ut.load_pickle(d)
        display_data(d)
        points, labels = model.classify(data)
        display_classification(data, points, labels)

class DisplayThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.color_cloud_pub = rospy.Publisher('color_cloud', sm.PointCloud)
        self.cloud_pub = rospy.Publisher('orig_cloud', sm.PointCloud)
        self.labels_pub = rospy.Publisher('point_label', vm.Marker)
        self.publish_queue = qu.Queue()


    def run(self):
        r = rospy.Rate(10)
        print 'DisplayThread: running!'
        my_pub_list = []
        while not rospy.is_shutdown():
            while not self.publish_queue.empty():
                my_pub_list.append(self.publish_queue.get())
            for pub, msg in my_pub_list:
                pub.publish(msg)
            r.sleep()


    def display_scan(self, points_bl, valid_points_bl, intensity):
        #publish mapped cloud to verify
        valid_points_pro_msg  = ru.np_to_rgb_pointcloud(valid_points_bl, intensity, 'base_link')
        points_bl_msg = ru.np_to_pointcloud(points_bl, 'base_link')
        self.publish_queue.put([self.cloud_pub, points_bl_msg])
        self.publish_queue.put([self.color_cloud_pub, valid_points_pro_msg])


    def display_classification(self, points, label, frame):
        colors = []
        for i in range(labels.shape[1]):
            if labels[0,i] == 0:
                colors.append(np.matrix([1,0,0,1.]).T)
            else:
                colors.append(np.matrix([0,1,0,1.]).T)
        labels_msg = viz.list_marker(points, np.column_stack(colors), scale=.02, mtype='points', mframe=frame)
        self.publish_queue.put([self.labels_pub, labels_msg])


    def display_training_data_set(self, fname):
        points_bl, colored_points_valid_bl, pos, neg = calculate_features(fname)

        colors = [np.matrix([0,1,0,1.]).T, np.matrix([0,0,1,1.]).T]
        colors_mat = []
        points_mat = []
        for _, points in [pos, neg]:
            c = colors.pop(0)
            pcolors = np.repeat(c, len(points), axis=1)
            colors_mat.append(pcolors)
            points_mat.append(np.matrix(np.column_stack(points)))

        colors_mat = np.column_stack(colors_mat)
        points_mat = np.column_stack(points_mat)
        labels_msg = viz.list_marker(points_mat, colors_mat, scale=[.05,.05,.05], mtype='points', mframe='base_link')
        self.publish_queue.put([self.labels_pub, labels_msg])
        self.display_scan(points_bl, colored_points_valid_bl[0:3,:], colored_points_valid_bl[3:,:])


if __name__ == '__main__':
    dirname = sys.argv[1]
    #test_dir = sys.argv[2] 
    fname = 'features_recognize3d.pkl'
    model_name = 'trained_model.pkl'

    rospy.init_node('recognize3d_display')
    dt = DisplayThread()
    dt.display_training_data_set(dirname)
    dt.run()

    #if not pt.isfile(fname):
    #    train_model(dirname, fname, model_name)
    #model = load_model(model_name)

    #test_model(model, test_dir)
    ##Display ground truth

















        #indices_list = voi_tree.query_ball_point(np.array(sampled_point), grid_resolution/2.)
        #if len(indices_list) > 4:
        #    points_in_ball_bl = points_in_volume_bl[:, indices_list]
 
        #    #calc normal
        #    normal_bl = calc_normal(points_in_ball_bl[0:3,:])

        #    #calc average color
        #    avg_color = np.mean(points_in_ball_bl[3:,:], 1)
        #    mean_3d_bl = np.mean(points_in_ball_bl[0:3,:], 1)

        #    #project mean into 2d
        #    mean_2d = data_pkl['prosilica_cal'].project(tfu.transform_points(data_pkl['pro_T_bl'], mean_3d_bl))

        #    #get local features
        #    features = local_window(mean_2d, intensity_image_array, win_size)
        #    if features != None:
        #        samples.append([normal_bl, avg_color, features])
        #        non_empty = non_empty + 1
        #    else:
        #        empty_queries = empty_queries + 1
        #else:
        #    empty_queries = empty_queries + 1






    #the center point is the one positive example
    #everything else is negative...
    #sampled_points_tree = sp.KDTree(np.array(sampled_points))
    #results = voi_tree.query_ball_tree(sampled_points_tree, grid_resolution / 2.)
    #pdb.set_trace()

        #voi_msg = ru.np_to_pointcloud(points_in_volume_bl, 'base_link')
        #orig_cloud_pub.publish(opc_msg)
        #highres_cloud_pub.publish(pc_msg)
        #voi_cloud_pub.publish(voi_msg)
