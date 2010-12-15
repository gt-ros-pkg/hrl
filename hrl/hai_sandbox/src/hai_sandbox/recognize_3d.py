import cv
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

##
# given a 2d location and a window size, cut out a square of intensity values, returns (winsize*winsize)x1 array in range [0,1]
# just use local template image
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
        intensity = np.matrix(np.reshape(subrect, (subrect.shape[0]*subrect.shape[1]*subrect.shape[2], 1))) / 255.
        return intensity

##
#
# @param cal_obj camera calibration object
# @return 3xn int matrix of 3d points that are visible in the camera's frame
# @return 3xn int matrix of rgb values of those points in range [0,1]
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

def calculate_features_given_point(sampled_point, voi_tree, intensity_paired_with_3d_points, \
        grid_resolution, data_pkl, win_size, intensity_image_array, verbose=False):
    indices_list = voi_tree.query_ball_point(np.array(sampled_point), grid_resolution/2.)
    if len(indices_list) > 4:
        points_in_ball_bl = np.matrix(voi_tree.data.T[:, indices_list])
        intensity_in_ball_bl = intensity_paired_with_3d_points[:, indices_list]
        
        #calc normal
        normal_bl = calc_normal(points_in_ball_bl[0:3,:])
        
        #calc average color
        avg_color = np.mean(intensity_in_ball_bl, 1)
        mean_3d_bl = np.mean(points_in_ball_bl[0:3,:], 1)
        #pdb.set_trace()
        
        #project mean into 2d
        mean_2d = data_pkl['prosilica_cal'].project(tfu.transform_points(data_pkl['pro_T_bl'], \
                mean_3d_bl))
        
        #get local features
        features = local_window(mean_2d, intensity_image_array, win_size)
        if features != None:
            return [normal_bl, avg_color, features]
        else:
            if verbose:
                print '>> local_window outside of image'
    else:
        print '>> not enough neighbors!'

    return None

#def calculate_features_online():
#    pass


def calculate_features(fname, data_pkl, grid_resolution, win_size):
    ##
    ## For each data pickle, generate features...
    ##
    #print data_pkl.__class__, len(data_pkl)
    #if data_pkl.__class__ == list:
    #    pdb.set_trace()
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
        intensity_paired_with_3d_points = points_in_volume_bl[3:,:]
        features = calculate_features_given_point(sampled_point, voi_tree, intensity_paired_with_3d_points,\
                        grid_resolution, data_pkl, win_size, intensity_image_array)
        if features != None:
            negative_samples.append(features)
            negative_sample_points.append(sampled_point)
            non_empty = non_empty + 1
        else:
            empty_queries = empty_queries + 1

    positive_samples.append(calculate_features_given_point(closest_center_point_bl.T.A1,\
                                voi_tree, intensity_paired_with_3d_points, grid_resolution, data_pkl, \
                                win_size, intensity_image_array, verbose=True))
    if positive_samples[0] == None:
        print 'OH NO! there is NO valid positive example for this dataset'

    positive_sample_points.append(closest_center_point_bl.T.A1)
    print 'empty queries', empty_queries, 'non empty', non_empty

    return bl_pc, colored_points_valid_bl, [positive_samples, positive_sample_points], [negative_samples, negative_sample_points]

class Recognize3D:

    def __init__(self):
        self.POSITIVE = 1.0
        self.NEGATIVE = 0.

        self.projection_basis = None
        self.intensities_mean = None
        self.size_intensity = None
        self.raw_data_dim = None

        fake_train = np.matrix(np.row_stack((np.matrix(range(8)), np.matrix(range(3,3+8)))), dtype='float32')
        print 'fake_train.shape', fake_train.shape
        fake_resp = np.matrix([1,0], dtype='float32')
        self.svm = cv.SVM(fake_train, fake_resp)
        self.display = DisplayThread()

    def load(self, model_name):
        print 'loading model ', model_name
        self.svm.load(model_name)
        #construct an svm of the same size as real data, 6 + 2
        p = ut.load_pickle(pt.splitext(model_name)[0] + '.pkl')
        self.projection_basis = p['projection_basis']
        self.intensities_mean = p['intensity_mean']
        self.raw_data_dim         = p['raw_data_dim']
        self.size_intensity   = p['size_intensity']

    def save(self, model_name):
        print 'saving model to ', model_name
        self.svm.save(model_name)
        ut.save_pickle({'projection_basis': self.projection_basis,
                        'intensity_mean': self.intensity_mean,
                        'raw_data_dim': self.raw_data_dim,
                        'size_intensity': self.size_intensity},\
                        pt.splitext(model_name)[0] + '.pkl')

    ##
    #
    # @param x a column vector of size (self.raw_data_dim x 1)
    def classify(self, x):
        start_intensities = self.raw_data_dim - self.size_intensity
        other_fea   = x[:start_intensities, 0]
        intensities = x[start_intensities:, 0]
        reduced_intensities = self.projection_basis.T * intensities
        x_reduced = np.row_stack((other_fea, reduced_intensities))
        x_reduced = np.matrix(x_reduced.T, dtype='float32').copy()
        #print 'x_reduced.shape', x_reduced.shape
        return self.svm.predict(x_reduced)


    def test_training_set(self, dataset_filename, grid_resolution, win_size):
        data_pkl = ut.load_pickle(dataset_filename)
        points_bl, colored_points_valid_bl, pos, neg = calculate_features(dataset_filename, data_pkl, grid_resolution, win_size)
        #pos_fea, pos_points = pos
        #neg_fea, neg_points = neg

        all_fea = []
        all_fea += pos[0]
        all_fea += neg[0]
        labels = []
        for feas in all_fea:
            label = self.classify(np.row_stack(feas))
            labels.append(label)

        all_pts = []
        all_pts.append(np.matrix(pos[1]).T)
        all_pts.append(np.matrix(neg[1]).T)

        #call display
        self.display.display_scan(points_bl, 
                colored_points_valid_bl[0:3,:], 
                colored_points_valid_bl[3:, :])
        self.display.display_classification(np.column_stack(all_pts), 
                np.matrix(labels), 'base_link')
        self.display.run()


    def train(self, dirname, features_file_name, variance_keep, \
            grid_resolution=.05, win_size=15):
        print 'training using data in', dirname
        #pdb.set_trace()
        if not pt.isfile(features_file_name):
            data_files = glob.glob(pt.join(dirname, '*.pkl'))
            #rospy.init_node('better_recognize3d')
    
            positive_examples = []
            negative_examples = []
    
            positive_points = []
            negative_points = []
    
            print 'extracting features'
            for data_file_name in data_files:
                #pos, ppoints, neg, npoints = calculate_features(dirname, fname)
                print 'processing', data_file_name
                data_pkl = ut.load_pickle(data_file_name)
                points_bl, colored_points_valid_bl, pos, neg = calculate_features(data_file_name, data_pkl, grid_resolution, win_size)
                positive_examples += pos[0]
                negative_examples += neg[0]
                positive_points += pos[1]
                negative_points += neg[1]
    
            print 'num positive samples', len(positive_examples), 
            print 'num negative samples', len(negative_examples)
            print 'saving features'
            ut.save_pickle([positive_examples, negative_examples, positive_points, negative_points], features_file_name)
        else:
            print 'features has been calculated already (yay!) loading from', features_file_name
            positive_examples, negative_examples, positive_points, negative_points = ut.load_pickle(features_file_name)
    
        #Turn raw features into matrix format
        size_intensity = None
        all_x = []
        all_y = []
        for examples, label in [(positive_examples, self.POSITIVE), (negative_examples, self.NEGATIVE)]:
            #normal_bl 3x1 mat
            #avg_color 3x1 mat
            #intensity nx1 mat
            pdb.set_trace()
            normal_bls, avg_colors, intensities = zip(*examples)
            normal_bls = np.column_stack(normal_bls) #each column is a different sample
            avg_colors = np.column_stack(avg_colors)
            intensities = np.column_stack(intensities)
            if size_intensity == None:
                size_intensity = intensities.shape[0]
    
            #pdb.set_trace()
            xs = np.row_stack((normal_bls, avg_colors, intensities)) #stack features
            ys = np.zeros((1, len(examples))) + label
            all_x.append(xs)
            all_y.append(ys)
    
        #pdb.set_trace()
        train     = np.column_stack(all_x)
        responses = np.column_stack(all_y)
        self.raw_data_dim = train.shape[0]
    
        #projection basis should be the same dimension as the data
        start_intensities = train.shape[0] - size_intensity
        intensities_rows  = train[start_intensities:, :]
        intensity_mean    = np.mean(intensities_rows, 1)
    
        print 'constructing pca basis'
        projection_basis = dr.pca_vectors(intensities_rows, variance_keep)
        reduced_intensities = projection_basis.T * (intensities_rows - intensity_mean  )
        #assert(intensities_rows.shape[0] == projection_basis.shape[0])
        train = np.row_stack((train[:start_intensities, :], reduced_intensities))
    
        print 'training classifier'
        #Train classifier...
        #Run pca on intensity
        #train => float32 mat, each row is an example
        #responses => float32 mat, each column is a corresponding response
        train = np.matrix(train.T, dtype='float32').copy()
        responses = np.matrix(responses, dtype='float32').copy()
        svm = cv.SVM(train, responses)
        svm.train(train, responses)

        self.svm = svm
        self.projection_basis = projection_basis
        self.intensity_mean = intensity_mean  
        self.size_intensity = size_intensity




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


    def display_classification(self, points, labels, frame):
        colors = []
        for i in range(labels.shape[1]):
            if labels[0,i] == 0:
                colors.append(np.matrix([1,0,0,1.]).T)
            else:
                colors.append(np.matrix([0,1,0,1.]).T)
        labels_msg = viz.list_marker(points, np.column_stack(colors), scale=[.02, .02, .02], mtype='points', mframe=frame)
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
    test_name = sys.argv[2]
    #test_dir = sys.argv[2] 
    features_file_name = 'features_recognize3d.pkl'
    model_name = 'recognize_3d_trained_svm.xml'

    rospy.init_node('recognize3d_display')
    #train_model(dirname, features_file_name, model_name, .95)
    #test_on_training_data(model_name, dirname)
    
    GRID_RESOLUTION = .025
    WIN_SIZE = 15
    VARIANCE_KEEP = .95

    r3d = Recognize3D()
    r3d.train(dirname, features_file_name, VARIANCE_KEEP, GRID_RESOLUTION, WIN_SIZE)
    r3d.save(model_name)
    r3d.load(model_name)
    r3d.test_training_set(test_name, GRID_RESOLUTION, WIN_SIZE)
    

    #dt = DisplayThread()
    #dt.display_training_data_set(dirname)
    #dt.run()

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
