import cv
import roslib; roslib.load_manifest('hai_sandbox')

import rospy
import sensor_msgs.msg as sm
import visualization_msgs.msg as vm

import numpy as np
import scipy.spatial as sp
import threading
import Queue as qu
import os.path as pt
import glob

import ml_lib.dataset as ds
import ml_lib.dimreduce as dr

import hrl_lib.blob as blob
import hrl_lib.image3d as i3d
import hrl_lib.util as ut
import hrl_lib.viz as viz
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu

import pdb


def load_data_from_dir(dirname, grid_resolution, win_size, voi_bounds_laser):
    data_files = glob.glob(pt.join(dirname, '*.pkl'))
    data = []
    
    print 'extracting features'
    for data_file_name in data_files:
        #pos, ppoints, neg, npoints = calculate_features_from_files(dirname, fname)
        print 'processing', data_file_name
        #data_pkl = ut.load_pickle(data_file_name)
        ic_data = load_data_from_file(data_file_name, grid_resolution, win_size, voi_bounds_laser)
        data.append(ic_data)
    return data


def load_data_from_file(fname, grid_resolution, win_size, voi_bounds_laser):
    #load pickle
    data_pkl = ut.load_pickle(fname)
    print 'original frame of pointcloud is ', data_pkl['points_laser'].header.frame_id

    #use pickle to load image
    image_fname = pt.join(pt.split(fname)[0], data_pkl['high_res'])
    intensity_image = cv.LoadImageM(image_fname)

    center_point_bl = data_pkl['touch_point'][0]
    print 'robot touched point cloud at point', center_point_bl.T

    #make the data object 
    return IntensityCloudData(data_pkl['points_laser'], intensity_image, 
            data_pkl['pro_T_bl'], data_pkl['prosilica_cal'],
            grid_resolution, win_size, voi_center_laser, voi_bounds_laser)


class IntensityCloudData:

    def __init__(self, pointcloud_msg, cvimage_mat, 
            image_T_laser, calibration_obj, 
            grid_resolution, win_size, 
            voi_center_laser, voi_bounds_laser):

        self.pointcloud_msg = pointcloud_msg
        self.image_T_laser = image_T_laser
        self.image_arr = np.asarray(cvimage_mat)
        self.grid_resolution = grid_resolution
        self.win_size = win_size
        self.calibration_obj = calibration_obj
        self.voi_bl = voi_bounds_laser
        self.voi_center_laser = voi_center_laser

        #Quantities that will be calculated
        #from _associate_intensity
        self.points_valid_image = None 
        self.valid_colors = None

        #from _limit_to_voi
        self.points_voi_laser = None
        self.limits_laser = None
        self.voi_tree = None
        self.intensity_paired = None

        #from _generate_loc_in_voi
        self.sampled_points = None

        #from _calculate_features
        self.feature_list = None
        self.feature_loc_list = None

        #feature sizes
        self.sizes = None


    def _associate_intensity(self):
        bl_pc = ru.pointcloud_to_np(self.pointcloud_msg)
        print 'original point cloud size', bl_pc.shape[1]
        self.points_valid_image, self.valid_colors = \
                i3d.combine_scan_and_image_laser_frame(bl_pc, self.image_T_laser,\
                                            self.image_arr, self.calibration_obj)
        print 'number of points visible in camera',  points_valid_image.shape[1]


    def _limit_to_voi(self):
        laser_T_image = np.linalg.inv(self.image_T_laser)

        #combined matrix (3xn 3d points) + (mxn color points) = [(m+3) x n matrix]
        colored_points_valid_laser = \
                np.row_stack((tfu.transform_points(laser_T_image,\
                                                   points_valid_pro[0:3,:]),\
                               colors)) 

        self.points_voi_laser, self.limits_laser = \
                i3d.select_rect(self.voi_center_laser, 
                                self.voi_bl[0], self.voi_bl[1], self.voi_bl[2], 
                                colored_points_valid_laser)

        self.voi_tree = sp.KDTree(np.array(self.points_voi_laser[0:3,:].T))
        self.intensity_paired = self.points_voi_laser[3:,:]

        if self.points_voi_laser == None:
            msg = 'No 3D points in volume of interest!'
            print msg
            raise RuntimeError(msg)
        print 'number of points in voi', self.points_voi_laser.shape[1]


    # sample uniformly in voi 
    def _generate_loc_in_voi(self):
        lim = self.limits_laser
        res = self.grid_resolution
        self.sampled_points = []
        for x in (np.arange(lim[0][0], lim[0][1], res) + (res/2.)):
            for y in (np.arange(lim[1][0], lim[1][1], res) + (res/2.)):
                for z in (np.arange(lim[0][0], lim[2][1], res) + (res/2.)):
                    self.sampled_points.append([x,y,z])


    def _calculate_features(self):
        self._associate_intensity()
        self._limit_to_voi()
        self._generate_loc_in_voi()

        self.feature_list = []
        self.feature_loc_list = []
        non_empty = 0
        empty_queries = 0
        for sampled_point in self.sampled_points:
            feature = self.feature_vec_at(sampled_point)
            if feature != None:
                self.feature_list.append(feature)
                self.feature_loc_list.append(sampled_point)
                non_empty = non_empty + 1
            else:
                empty_queries = empty_queries + 1
        print 'empty queries', empty_queries, 'non empty', non_empty
        #return bl_pc, colored_points_valid_bl, [positive_samples, positive_sample_points], \
        #        [negative_samples, negative_sample_points]


    ##
    #
    # @param point3d - point to calculate features for 3x1 matrix
    # @param verbose
    def feature_vec_at(self, point3d, verbose=False):
        indices_list = self.voi_tree.query_ball_point(np.array(point3d), self.grid_resolution/2.)
        if len(indices_list) > 4:
            #pdb.set_trace()
            points_in_ball_bl = np.matrix(self.voi_tree.data.T[:, indices_list])
            intensity_in_ball_bl = self.intensity_paired[:, indices_list]
            
            #calc normal
            normal_bl = i3d.calc_normal(points_in_ball_bl[0:3,:])
            
            #calc average color
            avg_color = np.mean(intensity_in_ball_bl, 1)
            mean_3d_bl = np.mean(points_in_ball_bl[0:3,:], 1)
            
            #project mean into 2d
            mean_2d = self.calibration_obj.project(tfu.transform_points(self.image_T_laser,\
                    mean_3d_bl))
            
            #get local features
            features1 = i3d.local_window(mean_2d, self.image_arr, self.win_size)
            features2 = i3d.local_window(mean_2d, self.image_arr, self.win_size*2, resize_to=self.win_size)
            features4 = i3d.local_window(mean_2d, self.image_arr, self.win_size*4, resize_to=self.win_size)
            features8 = i3d.local_window(mean_2d, self.image_arr, self.win_size*8, resize_to=self.win_size)
            local_intensity = np.row_stack((features1, features2, features4, features8))
            #features = features1
            if features1 != None and features2 != None and features4 != None and features8 != None:
                if self.sizes == None:
                    self.sizes['intensity'] = local_intensity.shape[0]
                    self.sizes['normal'] = normal_bl.shape[0]
                    self.sizes['color'] = avg_color.shape[0]
                return [normal_bl, avg_color, local_intensity]
            else:
                if verbose:
                    print '>> local_window outside of image'
        else:
            if verbose:
                print '>> not enough neighbors!', len(indices_list)
        return None

    ##
    #
    # @return a matrix mxn where m is the number of features and n the number of examples
    def make_dataset(self):
        normal_bls, avg_colors, intensities = zip(self.feature_list)
        normal_bls = np.column_stack(normal_bls) #each column is a different sample
        avg_colors = np.column_stack(avg_colors)
        intensities = np.column_stack(intensities)
        xs = np.row_stack((normal_bls, avg_colors, intensities)) #stack features
        return ds.Dataset(xs, None)
        #return xs, intensities
        #return Dataset(inputs, None)

    def get_location3d(self, dataset_index):
        return loc3d

    #def get_location2d(self, dataset_index):
    #    return loc2d


class SVMActiveLearnerApp:

    def __init__(self):
        self.POSITIVE = 1.0
        self.NEGATIVE = 0.

        self.classifier = 'svm' #or 'knn'
        self.classifiers = {}
        self.n = 3.
        fake_train = np.matrix(np.row_stack((np.matrix(range(8)), np.matrix(range(3,3+8)))), dtype='float32')
        fake_resp = np.matrix([1,0], dtype='float32')
        self.classifiers['svm'] = cv.SVM(fake_train, fake_resp)

        self.intensities_index = None
        self.intensities_mean = None
        self.projection_basis = None
        self.reduced_dataset = None


    def select_next_instance(self, instances):
        return selected_index


    def update_model(self, instance, label):
        return None


    def load(self, fname):
        self.classifier['svm'].load(fname)
        p = ut.load_pickle(pt.splitext(model_name)[0] + '.pkl')
        self.intensities_index = p['intensities_index']
        self.intensities_mean  = p['intensities_mean']
        self.projection_basis  = p['projection_basis']
        self.reduced_dataset   = p['reduced_dataset']


    def save(self, fname):
        self.classifier['svm'].save(fname)
        ut.save_pickle({'intensities_index':self.intensities_index,
                        'intensities_mean': self.intensities_mean, 
                        'projection_basis': self.projection_basis,
                        'reduced_dataset':  self.reduced_dataset},\
                         pt.splitext(fname)[0] + '.pkl')

    def classify(self, instance):
        projected_instance = np.matrix(self.partial_pca_project(instance), dtype='float32').copy()
        if self.classifier == 'knn':
            labels = self.reduced_dataset.outputs[:, self.classifiers['knn'].query(x_reduced, self.n)[1].tolist()[0]]
            if np.sum(labels) > (self.n/2.):
                return self.POSITIVE
            else:
                return self.NEGATIVE

        elif self.classifier == 'svm':
            return self.classifiers['svm'].predict(x_reduced)

    def train(self, dirname, features_file_name, variance_keep, \
                    grid_resolution=.05, win_size=15, 
                    voi_bounds_laser=[.5, .5, .5]):

        #Cache feature computations
        if not pt.isfile(features_file_name):
            ic_list = load_data_from_dir(dirname, grid_resolution, win_size, voi_bounds_laser)
            ut.save_pickle(ic_list)
        else:
            print 'features has been calculated already (yay!) loading from', features_file_name
            ic_list = ut.load_pickle(features_file_name)

        #Combine individual scans into one large dataset
        dataset = None
        for ic in self.ic_list:
            if dataset == None:
                dataset = ic.make_dataset()
            else:
                dataset.append(ic.make_dataset())

        #TODO: somehow generate labels for these datasets...
        train = dataset.inputs
        responses = dataset.outputs

        #Calculate PCA vectors
        self.intensities_index = dataset.num_attributes() - ic_list[0].sizes['intensity']
        self.calculate_pca_vectors(self.intensities_index, train)
        train = self.partial_pca_project(train)
    
        print 'training classifier'
        #train => float32 mat, each row is an example
        #responses => float32 mat, each column is a corresponding response
        train = np.matrix(train.T, dtype='float32').copy()
        responses = np.matrix(responses, dtype='float32').copy()

        self.classifiers['svm'] = cv.SVM(train, responses)
        self.classifiers['svm'].train(train, responses)
        self.classifiers['knn'] = sp.KDTree(np.array(train))
        self.reduced_dataset = ds.Dataset(train, responses)


    def calculate_pca_vectors(self, data, variance_keep):
        data_in = data[self.intensities_index:, :]
        self.intensities_mean = np.mean(data_in, 1)
        print 'Constructing PCA basis'
        self.projection_basis = dr.pca_vectors(data_in, variance_keep)
        print 'PCA basis size:', self.projection_basis.shape


    def partial_pca_project(self, instances)
        instances_in = instances[self.intensities_index:, :]
        reduced_intensities = self.projection_basis.T * (instances_in - self.intensities_mean)
        return np.row_stack((instances[:self.intensities_index, :], reduced_intensities))




class RvizDisplayThread(threading.Thread):

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


    def display_training_data_set(self, fname, grid_resolution, win_size):
        data_pkl = ut.load_pickle(fname)
        points_bl, colored_points_valid_bl, pos, neg = calculate_features_from_files(fname, data_pkl, grid_resolution, win_size)

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
    import sys

    #mode = 'rebalance'
    mode = 'test'
    GRID_RESOLUTION = .03
    WIN_SIZE = 5
    features_file_name = 'features_recognize3d.pkl'
    model_name = 'recognize_3d_trained_svm.xml'
    rospy.init_node('recognize3d_display')

    if mode == 'test':
        dirname = sys.argv[1]
        #test_name = sys.argv[2]
        #test_dir = sys.argv[2] 
        #train_model(dirname, features_file_name, model_name, .95)
        #test_on_training_data(model_name, dirname)
        
        VARIANCE_KEEP = .95
        r3d = Recognize3D()
        r3d.train(dirname, features_file_name, VARIANCE_KEEP, GRID_RESOLUTION, WIN_SIZE)
        r3d.save(model_name)
        #r3d.load(model_name)
        #r3d.test_training_set(test_name, GRID_RESOLUTION, WIN_SIZE)

    elif mode == 'rebalance':
        test_name = sys.argv[1]
        r3d = Recognize3D()
        r3d.load(model_name)
        print 'rebalancing dataset'
        #r3d.rebalance_data_set_retrain()
        #r3d.kmedoids_rebalance_data_set_retrain()
        print 'knn data mat size:', r3d.knn.data.shape
        r3d.test_training_set(test_name, GRID_RESOLUTION, WIN_SIZE)

    elif mode == 'active_learn':
        r3d = Recognize3D()


    else:
        fname = sys.argv[1]
        dt = DisplayThread()
        dt.display_training_data_set(fname, GRID_RESOLUTION, WIN_SIZE)
        dt.run()
    



##
# Different representations floating around
#
# ML Dataset
#
# list of features that can be concatenated to form datset inputs (has meta
#           information about feature labels and how to reconstruct them)
#
# .pkl and image on disk 
#         want to be able to generate features from a directory
#         want to be able to generate features from a single file

#
# Want a class that loads pickles and turn them into feature representation (annotated dataset)
# Want to be able to generate annotated datasets from live sensor data.
# * annotated datasets can produce regular datasets and produce various displays of themselves
# * annotated datasets can be cached

# Want to be able to pick a feature vector and turn it back into a 2d/3d point
# 





















#class Recognize3D:
#
#    def __init__(self):
#        self.POSITIVE = 1.0
#        self.NEGATIVE = 0.
#
#        self.projection_basis = None
#        self.intensities_mean = None
#        self.size_intensity = None
#        self.raw_data_dim = None
#        self.training_data = None
#        self.knn = None
#
#        fake_train = np.matrix(np.row_stack((np.matrix(range(8)), np.matrix(range(3,3+8)))), dtype='float32')
#        print 'fake_train.shape', fake_train.shape
#        fake_resp = np.matrix([1,0], dtype='float32')
#        self.svm = cv.SVM(fake_train, fake_resp)
#        self.display = DisplayThread()
#        self.classifier = 'knn'
#        if self.classifier == 'knn':
#            print 'USING KNN FOR QUERY PREDICTIONS'
#        else:
#            print 'USING SVM FOR QUERY PREDICTIONS'
#
#    def load(self, model_name):
#        print 'loading model ', model_name
#        self.svm.load(model_name)
#        #construct an svm of the same size as real data, 6 + 2
#        p = ut.load_pickle(pt.splitext(model_name)[0] + '.pkl')
#        self.projection_basis = p['projection_basis']
#        self.intensities_mean = p['intensities_mean']
#        self.raw_data_dim     = p['raw_data_dim']
#        self.size_intensity   = p['size_intensity']
#        self.training_data    = p['training_data']
#        self.knn = sp.KDTree(np.array(self.training_data[0]))
#
#    def save(self, model_name):
#        print 'saving model to ', model_name
#        self.svm.save(model_name)
#        ut.save_pickle({'projection_basis': self.projection_basis,
#                        'intensities_mean': self.intensities_mean,
#                        'raw_data_dim': self.raw_data_dim,
#                        'training_data': self.training_data,
#                        'size_intensity': self.size_intensity},\
#                        pt.splitext(model_name)[0] + '.pkl')
#
#    def kmedoids_rebalance_data_set_retrain(self):
#        train, responses = self.training_data
#        num_positive_ex = np.sum(responses == self.POSITIVE)
#        pos_ex = train[np.where(responses == self.POSITIVE)[1].A1, :]
#        neg_ex = train[np.where(responses == self.NEGATIVE)[1].A1, :]
#
#        K = num_positive_ex
#        dtw = mlpy.Dtw(onlydist=True)
#        km = mlpy.Kmedoids(k = K, dist=dtw)
#        #pdb.set_trace()
#        medoid_indices, nonmedoid_indices, membership, total_cost = km.compute(np.array(neg_ex))
#        subset_of_neg_examples = neg_ex[medoid_indices, :]
#
#        rebalanced_train = np.matrix(np.row_stack((pos_ex, subset_of_neg_examples)), dtype='float32').copy()
#        rebalanced_resp = np.matrix(np.column_stack((np.matrix(num_positive_ex * [self.POSITIVE]),\
#                                                     np.matrix(len(medoid_indices) * [self.NEGATIVE]))), dtype='float32').copy()
#
#        self.svm = cv.SVM(rebalanced_train, rebalanced_resp)
#        self.svm.train(rebalanced_train, rebalanced_resp)
#        self.training_data = [rebalanced_train, rebalanced_resp]
#        self.knn = sp.KDTree(np.array(rebalanced_train))
#
#    def rebalance_data_set_retrain(self):
#        train, responses = self.training_data
#        num_positive_ex = np.sum(responses == self.POSITIVE)
#        pos_ex = train[np.where(responses == self.POSITIVE)[1].A1, :]
#        neg_ex = train[np.where(responses == self.NEGATIVE)[1].A1, :]
#        print 'pos_ex', pos_ex.shape
#        print 'neg_ex', neg_ex.shape
#
#        neg_indices = np.random.permutation(range(neg_ex.shape[0]))[:num_positive_ex]
#        subset_of_neg_examples = neg_ex[neg_indices, :]
#        print 'subset of neg_ex', subset_of_neg_examples.shape
#
#        rebalanced_train = np.matrix(np.row_stack((pos_ex, subset_of_neg_examples)), dtype='float32').copy()
#        rebalanced_resp = np.matrix(np.column_stack((np.matrix(num_positive_ex * [self.POSITIVE]),\
#                                                     np.matrix(len(neg_indices) * [self.NEGATIVE]))), dtype='float32').copy()
#        print 'rebalanced train & resp', rebalanced_train.shape, rebalanced_resp.shape
#
#        self.svm = cv.SVM(rebalanced_train, rebalanced_resp)
#        self.svm.train(rebalanced_train, rebalanced_resp)
#        self.training_data = [rebalanced_train, rebalanced_resp]
#        self.knn = sp.KDTree(np.array(rebalanced_train))
#
#    ##
#    #
#    # @param x a column vector of size (self.raw_data_dim x 1)
#    def classify(self, x):
#        start_intensities = self.raw_data_dim - self.size_intensity
#        other_fea   = x[:start_intensities, 0]
#        intensities = x[start_intensities:, 0]
#        reduced_intensities = self.projection_basis.T * (intensities - self.intensities_mean)
#        x_reduced = np.row_stack((other_fea, reduced_intensities))
#        x_reduced = np.matrix(x_reduced.T, dtype='float32').copy()
#        #print 'x_reduced.shape', x_reduced.shape
#
#        if self.classifier=='knn':
#            responses = self.training_data[1]
#            #if np.sum(responses[:, self.knn.query(x_reduced, 1)[1].tolist()[0]]) > 0:
#            NEIGHBORS = 3.
#            if np.sum(responses[:, self.knn.query(x_reduced, NEIGHBORS)[1].tolist()[0]]) > (NEIGHBORS/2.):
#                return self.POSITIVE
#            else:
#                return self.NEGATIVE
#        elif self.classifier == 'svm':
#            return self.svm.predict(x_reduced)
#
#
#    def test_training_set(self, dataset_filename, grid_resolution, win_size):
#        data_pkl = ut.load_pickle(dataset_filename)
#        points_bl, colored_points_valid_bl, pos, neg = calculate_features_from_files(dataset_filename, data_pkl, grid_resolution, win_size)
#        #pos_fea, pos_points = pos
#        #neg_fea, neg_points = neg
#
#        all_fea = []
#        all_fea += pos[0]
#        all_fea += neg[0]
#        labels = []
#        num_pos = 0
#        for feas in all_fea:
#            #pdb.set_trace()
#            label = self.classify(np.row_stack(feas))
#            labels.append(label)
#            num_pos += label
#        #pdb.set_trace()
#        print 'positive tested as ', self.classify(np.row_stack(pos[0][0]))
#
#        all_pts = []
#        all_pts.append(np.matrix(pos[1]).T)
#        all_pts.append(np.matrix(neg[1]).T)
#
#        #call display
#        print 'Num positives', num_pos
#        self.display.display_scan(points_bl, 
#                colored_points_valid_bl[0:3,:], 
#                colored_points_valid_bl[3:, :])
#        self.display.display_classification(np.column_stack(all_pts), 
#                np.matrix(labels), 'base_link')
#        self.display.run()
#
#
#    def extract_features_from_dir(self, dirname, features_file_name, grid_resolution, win_size):
#        print 'training using data in', dirname
#        #pdb.set_trace()
#        if not pt.isfile(features_file_name):
#            data_files = glob.glob(pt.join(dirname, '*.pkl'))
#            #rospy.init_node('better_recognize3d')
#    
#            positive_examples = []
#            negative_examples = []
#    
#            positive_points = []
#            negative_points = []
#    
#            print 'extracting features'
#            for data_file_name in data_files:
#                #pos, ppoints, neg, npoints = calculate_features_from_files(dirname, fname)
#                print 'processing', data_file_name
#                data_pkl = ut.load_pickle(data_file_name)
#                points_bl, colored_points_valid_bl, pos, neg = calculate_features_from_files(data_file_name, \
#                        data_pkl, grid_resolution, win_size)
#                positive_examples += pos[0]
#                negative_examples += neg[0]
#                positive_points   += pos[1]
#                negative_points   += neg[1]
#    
#            print 'num positive samples', len(positive_examples), 
#            print 'num negative samples', len(negative_examples)
#            print 'saving features'
#            ut.save_pickle([positive_examples, negative_examples, positive_points, negative_points], \
#                            features_file_name)
#        else:
#            print 'features has been calculated already (yay!) loading from', features_file_name
#            positive_examples, negative_examples, positive_points, negative_points = ut.load_pickle(\
#                    features_file_name)
#
#        return positive_examples, negative_examples, positive_points, negative_points 
#
#    ##
#    # @return a matrix mxn where m is the number of features and n the number of examples
#    def create_training_instances_from_raw_features(self, examples):
#        normal_bls, avg_colors, intensities = zip(*examples)
#        normal_bls = np.column_stack(normal_bls) #each column is a different sample
#        avg_colors = np.column_stack(avg_colors)
#        intensities = np.column_stack(intensities)
#        xs = np.row_stack((normal_bls, avg_colors, intensities)) #stack features
#        return xs, intensities
#
#    ##
#    #
#    def train(self, dirname, features_file_name, variance_keep, \
#            grid_resolution=.05, win_size=15):
#
#        #Get raw eatures
#        positive_examples, negative_examples, positive_points, negative_points = self.extract_features_from_dir(\
#                dirname, features_file_name, grid_resolution, win_size)
#    
#        #Turn raw features into matrix format
#        size_intensity = None
#        all_x = []
#        all_y = []
#        for examples, label in [(positive_examples, self.POSITIVE), (negative_examples, self.NEGATIVE)]:
#            xs, intensities = self.create_training_instances_from_raw_features(examples)
#            if size_intensity == None:
#                size_intensity = intensities.shape[0]
#            ys = np.zeros((1, len(examples))) + label
#            all_x.append(xs)
#            all_y.append(ys)
#    
#        #pdb.set_trace()
#        train     = np.column_stack(all_x)
#        responses = np.column_stack(all_y)
#        #self.raw_data_dim = train.shape[0]
#    
#        #projection basis should be the same dimension as the data
#        start_intensities = train.shape[0] - size_intensity
#        intensities_rows  = train[start_intensities:, :]
#        intensities_mean    = np.mean(intensities_rows, 1)
#    
#        print 'constructing pca basis'
#        projection_basis = dr.pca_vectors(intensities_rows, variance_keep)
#        print '>> PCA basis size:', projection_basis.shape
#        reduced_intensities = projection_basis.T * (intensities_rows - intensities_mean  )
#        #assert(intensities_rows.shape[0] == projection_basis.shape[0])
#        train = np.row_stack((train[:start_intensities, :], reduced_intensities))
#    
#        print 'training classifier'
#        #Train classifier...
#        #Run pca on intensity
#        #train => float32 mat, each row is an example
#        #responses => float32 mat, each column is a corresponding response
#        train = np.matrix(train.T, dtype='float32').copy()
#        responses = np.matrix(responses, dtype='float32').copy()
#
#        svm = cv.SVM(train, responses)
#        svm.train(train, responses)
#
#        self.svm = svm
#        self.training_data = [train, responses]
#        self.projection_basis = projection_basis
#        self.intensities_mean = intensities_mean  
#        self.size_intensity = size_intensity
#        self.knn = sp.KDTree(np.array(train))
#        #pdb.set_trace()
#        #sampled_points_tree = sp.KDTree(np.array(sampled_points))
#
#    def smart_example_selection_trainer(self, dirname):
#        data_files = glob.glob(pt.join(dirname, '*.pkl'))
#        #for data_file_name in data_files:
#        first_scene_name = data_files[0]
#        first_scene_pickle = ut.load_pickle(first_scene_name)
#        #TODO get more reasonable features
#        fpoints_bl, fcolored_points_valid_bl, fpos, fneg = calculate_features_from_files(first_scene_pickle, \
#                first_scene_pickle, grid_resolution, win_size)
#
#        #start with one scene 
#        #1 positive and 1 negative
#        fpos_ex, _ = self.create_training_instances_from_raw_features(fpos[0]) #need new features beside 
#                                                                               #from RAW intensity
#        fneg_ex, _ = self.create_training_instances_from_raw_features(fneg[0])
#        train = np.matrix(np.column_stack((fpos_ex, fneg_ex[:,0])).T, dtype='float32').copy()
#        response = np.matrix([1,0.], dtype='float32').copy()
#
#        #for each new scene:
#        for scene_name in data_files[1:]:
#            #extract patches
#            scene_pickle = ut.load_pickle(scene_name)
#            points_bl, colored_points_valid_bl, pos, neg = calculate_features_from_files(scene_name, \
#                    scene_pickle, grid_resolution, win_size)
#            all_neg_from_scene = self.create_training_instances_from_raw_features(neg[0]).T
#            all_pos_from_scene = self.create_training_instances_from_raw_features(pos[0]).T
#            all_ex_from_scene = np.row_stack((all_neg_from_scene, all_pos_from_scene))
#            responses_scene = np.column_stack((np.matrix([self.NEGATIVE] * all_neg_from_scene.shape[0]),
#                                               np.matrix([self.POSITIVE] * all_pos_from_scene.shape[1])))
#
#            #rank each patch by distance to all positive examples
#            all_pos_samples = train[np.where(responses == self.POSITIVE)[1].A1, :] #all pos examples so far
#            ex_dists = []
#            for ex_idx in range(all_ex_from_scene.shape[0]):
#                ex = all_ex_from_scene[ex_idx, :]
#                diff = all_pos_samples - ex
#                dists = np.power(np.sum(np.power(diff, 2), 1), .5)
#                ex_dists.append(np.min(dists))
#
#            closest_ex_is_not_positive = True
#            LARGE_NUM = 999999999999
#            while closest_ex_is_not_positive:
#                #take the closest patch and classify it
#                closest_idx = np.argmin(ex_dists)
#                if ex_dists[closest_idx] == LARGE_NUM:
#                    break
#                ex_dists[closest_idx] = LARGE_NUM #make sure we won't select this idx again
#                closest_ex = all_ex_from_scene[closest_idx, :]
#                closest_ex_label = responses_scene[0, closest_idx]
#
#                #add example regardless because it is the closest point to our positive examples
#                train = np.matrix(np.column_stack((train, closest_ex)).T, dtype='float32').copy()
#                response = np.column_stack((response, np.matrix([closest_ex_label], dtype='float32'))).copy()
#                
#                if closest_ex_label == self.POSITIVE:
#                    closest_ex_is_not_positive = False
#
#                if closest_ex_label == self.NEGATIVE: 
#                    closest_ex_is_not_positive = True
#
#        return train, response 
