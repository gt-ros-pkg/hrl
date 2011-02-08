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
#import pdb
#pdb.set_trace()
import libsvm.svm as svm
import libsvm.svmutil as su

import copy
import os

import ml_lib.dataset as ds
import ml_lib.dimreduce as dr

import hrl_opencv.blob as blob
import hrl_lib.image3d as i3d
import hrl_lib.util as ut
import hrl_lib.viz as viz
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
import hrl_lib.prob as pr

import pdb

POSITIVE = 1.0
NEGATIVE = 0.

def load_data_from_dir(dirname, grid_resolution, win_size, win3d_size, voi_bounds_laser):
    data_files = glob.glob(pt.join(dirname, '*.pkl'))
    data = []
    
    print 'extracting features'
    for data_file_name in data_files:
        print 'processing', data_file_name
        #data_pkl = ut.load_pickle(data_file_name)
        ic_data = load_data_from_file(data_file_name, grid_resolution, win_size, win3d_size, voi_bounds_laser)
        data.append(ic_data)
    return data

def load_data_from_file(fname, rec_param):
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
                              center_point_bl, rec_param)
            #grid_resolution, win_size, win3d_size, center_point_bl, voi_bounds_laser)

class IntensityCloudData:

    def __init__(self, pointcloud_msg, cvimage_mat, 
            image_T_laser, calibration_obj, voi_center_laser, rec_param):
            #grid_resolution, win_size, win3d_size, 
            #voi_center_laser, voi_bounds_laser):

        self.params = rec_param
        #All parameters
        # self.n_samples = 5000
        # self.uni_mix = .7
        # self.uncertainty = .15

        # self.grid_resolution = grid_resolution
        # self.win_size = win_size
        # self.win3d_size = win3d_size
        # self.voi_bl = voi_bounds_laser

        #Data
        self.pointcloud_msg = pointcloud_msg
        self.image_T_laser = image_T_laser
        self.image_arr = np.asarray(cvimage_mat)
        self.image_cv = cvimage_mat

        self.calibration_obj = calibration_obj
        self.voi_center_laser = voi_center_laser

        #Quantities that will be calculated
        #from _associate_intensity
        self.points_valid_image = None 
        self.colors_valid = None

        #from _limit_to_voi
        #self.points_voi_laser = None
        self.limits_laser = None
        self.voi_tree = None
        self.points3d_valid_laser = None
        self.points2d_valid = None
        #self.intensity_paired = None

        #from _grid_sample_voi
        self.sampled_points = None
        self.sampled_points2d = None

        #from _calculate_features
        self.feature_list = None
        self.feature_locs = None

        #feature sizes
        self.sizes = None

        self._associate_intensity()
        self._limit_to_voi()
        #self._grid_sample_voi()
        self._random_sample_voi()
        #fill out sizes dict
        self.feature_vec_at_2d(np.matrix([calibration_obj.w/2., 
                                         calibration_obj.h/2.]).T)

    def _associate_intensity(self):
        bl_pc = ru.pointcloud_to_np(self.pointcloud_msg)
        print 'original point cloud size', bl_pc.shape[1]
        self.points_valid_image, self.colors_valid, self.points2d_valid = \
                i3d.combine_scan_and_image_laser_frame(bl_pc, self.image_T_laser,\
                                            self.image_arr, self.calibration_obj)
        print 'number of points visible in camera', self.points_valid_image.shape[1]

    def _limit_to_voi(self):
        laser_T_image = np.linalg.inv(self.image_T_laser)
        #combined matrix (3xn 3d points) + (mxn color points) = [(m+3) x n matrix]
        all_columns = \
                np.row_stack((tfu.transform_points(laser_T_image,\
                                                   self.points_valid_image[0:3,:]),\
                               self.colors_valid,
                               self.points2d_valid)) 

        valid_columns, self.limits_laser = \
                i3d.select_rect(self.voi_center_laser, 
                                self.params.voi_bl[0], 
                                self.params.voi_bl[1], 
                                self.params.voi_bl[2], 
                                all_columns)

        ncolors = self.colors_valid.shape[0]
        self.points3d_valid_laser = valid_columns[0:3,:]
        self.colors_valid = valid_columns[3:3+ncolors,:]
        self.points2d_valid = valid_columns[-2:, :]

        self.voi_tree = sp.KDTree(np.array(self.points3d_valid_laser.T))
        self.voi_tree_2d = sp.KDTree(np.array(self.points2d_valid.T))

        if valid_columns == None:
            msg = 'No 3D points in volume of interest!'
            print msg
            raise RuntimeError(msg)

        print 'number of points in voi', valid_columns.shape[1]

    # sample uniformly in voi 
    def _grid_sample_voi(self):
        lim = self.limits_laser
        res = self.params.grid_resolution
        self.sampled_points = []
        for x in (np.arange(lim[0][0], lim[0][1], res) + (res/2.)):
            for y in (np.arange(lim[1][0], lim[1][1], res) + (res/2.)):
                for z in (np.arange(lim[0][0], lim[2][1], res) + (res/2.)):
                    self.sampled_points.append([x,y,z])

    def _in_limits(self, p3d):
        l = self.limits_laser
        if (l[0][0] < p3d[0,0]) and (p3d[0,0] < l[0][1]) and \
           (l[1][0] < p3d[1,0]) and (p3d[1,0] < l[1][1]) and \
           (l[2][0] < p3d[2,0]) and (p3d[2,0] < l[2][1]):
            return True
        else:
            return False

    def _random_sample_voi(self):
        #to generate initial distribution of points
        #   randomly, uniformly sample for n points in 2d, add in prior information if available (gaussians)
        #        for each 2d point p, associate with the 3d point closest to the camera
        #        throw away all points not in VOI
        self.sampled_points = []
        self.sampled_points2d = []
        print 'generating _random_sample_voi'
        laser_T_image = np.linalg.inv(self.image_T_laser)
        gaussian = pr.Gaussian(np.matrix([ 0,      0,                          0.]).T, \
                               np.matrix([[1.,     0,                          0], \
                                          [0, self.params.uncertainty**2,      0], \
                                          [0,      0, self.params.uncertainty**2]]))


        distr = {'u': 0, 'g':0}
        while len(self.sampled_points) < self.params.n_samples:
            if np.random.rand() < self.params.uni_mix:
                d = 'u'
                x = np.random.randint(0, self.calibration_obj.w)
                y = np.random.randint(0, self.calibration_obj.h)
            else:
                d = 'g'
                #pt = self.calibration_obj.project(tfu.transform_points(self.image_T_laser, self.voi_center_laser))
                #x = int(round(pt[0,0] + np.random.randn() * 50))
                #y = int(round(pt[1,0] + np.random.randn() * 50))
                gaussian_noise = gaussian.sample()
                gaussian_noise[0,0] = 0
                sampled3d_pt_laser = self.voi_center_laser + gaussian_noise
                sampled3d_pt_image = tfu.transform_points(self.image_T_laser, sampled3d_pt_laser)
                sampled2d_pt = self.calibration_obj.project(sampled3d_pt_image)
                pt = np.round(sampled2d_pt)
                x = int(pt[0,0])
                y = int(pt[1,0])
                if x < 0 or x > (self.calibration_obj.w-.6) or y < 0 or (y > self.calibration_obj.h-.6):
                    continue

            #get projected 3d points within radius of N pixels
            indices_list = self.voi_tree_2d.query_ball_point(np.array([x,y]), 5.)
            if len(indices_list) < 1:
                continue

            #select 3d point closest to the camera (in image frame)
            points3d_image = tfu.transform_points(self.image_T_laser, self.points3d_valid_laser[:, indices_list])
            closest_p3d_image = points3d_image[:, np.argmin(points3d_image[2,:])]
            closest_p3d_laser = tfu.transform_points(laser_T_image, closest_p3d_image)

            #check if point is in VOI
            if self._in_limits(closest_p3d_laser):
                self.sampled_points.append(closest_p3d_laser.T.A1.tolist())
                self.sampled_points2d.append([x,y])
                distr[d] = distr[d] + 1
                #pdb.set_trace()
                #print len(self.sampled_points)

        #print '>>>>>> u', distr['u'] / float(self.params.n_samples)
        #print '>>>>>> g', distr['g'] / float(self.params.n_samples)

    def _caculate_features_at_sampled_points(self):
        self.feature_list = []
        feature_loc_list = []
        feature_loc2d_list = []
        non_empty = 0
        empty_queries = 0
        for i, sampled_point_laser in enumerate(self.sampled_points):
            #if i == 6639:
            #if i == 157184:
            #    pdb.set_trace()
            sampled_point_laser = np.matrix(sampled_point_laser).T
            feature, point2d = self.feature_vec_at(sampled_point_laser)
            if feature != None:
                self.feature_list.append(feature)
                feature_loc_list.append(sampled_point_laser)
                feature_loc2d_list.append(point2d)
                non_empty = non_empty + 1
            else:
                empty_queries = empty_queries + 1
        print 'empty queries', empty_queries, 'non empty', non_empty
        if len(feature_loc_list) > 0:
            self.feature_locs = np.column_stack(feature_loc_list)
            self.feature_locs2d = np.column_stack(feature_loc2d_list)
        else:
            self.feature_locs = np.matrix([])
            self.feature_locs2d = np.matrix([])
        #return bl_pc, colored_points_valid_bl, [positive_samples, positive_sample_points], \
        #        [negative_samples, negative_sample_points]

    def feature_vec_at_2d(self, loc2d, viz=False):
        #pdb.set_trace()
        indices = self.voi_tree_2d.query(np.array(loc2d.T), k=1)[1]
        closest_pt2d = self.points2d_valid[:, indices[0]]
        closest_pt3d = self.points3d_valid_laser[:, indices[0]]
        return self.feature_vec_at(closest_pt3d, viz=viz)[0], closest_pt3d, closest_pt2d

    def feature_vec_at_2d_mat(self, loc2d):
        indices = self.voi_tree_2d.query(np.array(loc2d.T), k=1)[1]
        closest_pt2d = self.points2d_valid[:, indices[0]]
        closest_pt3d = self.points3d_valid_laser[:, indices[0]]
        return self.feature_vec_at_mat(closest_pt3d), closest_pt3d, closest_pt2d

    def feature_vec_at_mat(self, point3d_laser, verbose=False):
        f = self.feature_vec_at(point3d_laser, verbose)[0]
        if f != None:
            return np.row_stack(f)

    ##
    #
    # @param point3d_laser - point to calculate features for 3x1 matrix in laser frame
    # @param verbose
    def feature_vec_at(self, point3d_laser, verbose=False, viz=False):
        indices_list = self.voi_tree.query_ball_point(np.array(point3d_laser.T), self.params.win3d_size)[0]
        if len(indices_list) > 4:
            #pdb.set_trace()
            points_in_ball_bl = np.matrix(self.voi_tree.data.T[:, indices_list])
            intensity_in_ball_bl = self.colors_valid[:, indices_list]
            
            #calc normal
            normal_bl = i3d.calc_normal(points_in_ball_bl[0:3,:])
            
            #calc average color
            avg_color = np.mean(intensity_in_ball_bl, 1)
            #mean_3d_bl = np.mean(points_in_ball_bl[0:3,:], 1)
            
            #project mean into 2d
            #mean_2d = self.calibration_obj.project(tfu.transform_points(self.image_T_laser,\
            #        mean_3d_bl))
            point2d_image = self.calibration_obj.project(tfu.transform_points(self.image_T_laser,\
                    point3d_laser))
            
            #get local features
            if viz == True:
                flatten=False
            else:
                flatten=True

            invalid_location = False
            local_intensity = []
            for multiplier in [1,2,4,8,16,32]:
                if multiplier == 1:
                    features = i3d.local_window(point2d_image, self.image_arr, self.params.win_size, flatten=flatten)
                else:
                    features = i3d.local_window(point2d_image, self.image_arr, self.params.win_size*multiplier, 
                                                resize_to=self.params.win_size, flatten=flatten)
                if features == None:
                    invalid_location = True
                    break
                else:
                    local_intensity.append(features)

            #features1 = i3d.local_window(point2d_image, self.image_arr, self.win_size, flatten=flatten)
            #features2 = i3d.local_window(point2d_image, self.image_arr, self.win_size*2, 
            #                            resize_to=self.win_size, flatten=flatten)
            #features4 = i3d.local_window(point2d_image, self.image_arr, self.win_size*4, 
            #                             resize_to=self.win_size, flatten=flatten)
            #features8 = i3d.local_window(point2d_image, self.image_arr, self.win_size*8, 
            #                             resize_to=self.win_size, flatten=flatten)
            #features16 = i3d.local_window(point2d_image, self.image_arr, self.win_size*16, 
            #                             resize_to=self.win_size, flatten=flatten)
            #features32 = i3d.local_window(point2d_image, self.image_arr, self.win_size*32, 
            #                             resize_to=self.win_size, flatten=flatten)

            #features = features1

            #if features1 != None and features2 != None and features4 != None and features8 != None:
            if not invalid_location:
                if not viz:
                    local_intensity = np.row_stack(local_intensity)
                if self.sizes == None:
                    #pdb.set_trace()
                    self.sizes = {}
                    self.sizes['intensity'] = local_intensity.shape[0]
                    self.sizes['normal'] = normal_bl.shape[0]
                    self.sizes['color'] = avg_color.shape[0]

                return [normal_bl, avg_color, local_intensity], point2d_image
            else:
                if verbose:
                    print '>> local_window outside of image'
        else:
            if verbose:
                print '>> not enough neighbors!', len(indices_list)
        return None, None

    ##
    #
    # @return a matrix mxn where m is the number of features and n the number of examples
    def extract_vectorized_features(self):
        self._caculate_features_at_sampled_points()
        #pdb.set_trace()
        normal_bls, avg_colors, intensities = zip(*self.feature_list)
        normal_bls = np.column_stack(normal_bls) #each column is a different sample
        avg_colors = np.column_stack(avg_colors)
        intensities = np.column_stack(intensities)
        xs = np.row_stack((normal_bls, avg_colors, intensities)) #stack features
        return xs, self.feature_locs2d, self.feature_locs
    #np.matrix(self.sampled_points2d).T, np.matrix(self.sampled_points).T
        #return ds.Dataset(xs, None)
        #return xs, intensities
        #return Dataset(inputs, None)

    def get_location2d(self, instance_indices):
        #pdb.set_trace()
        sampled_points3d_laser = self.feature_locs[:,instance_indices]
        return self.calibration_obj.project(tfu.transform_points(self.image_T_laser, \
                sampled_points3d_laser))

    #def get_location3d(self, dataset_index):
    #    return loc3d

class SVM:
    ##
    # zero is on negative side of decision boundary
    # 0  /  1
    # -  /  +
    def __init__(self, dataset):
        samples = dataset.inputs.T.tolist()
        labels = dataset.outputs.T.A1.tolist()
        self.model = su.svm_train(labels, samples)
        self.nsamples = len(samples)

        #problem = svm.svm_problem(labels, samples)
        ##param = svm.svm_parameter(C=10, nr_weight=2, weight_label=[1,0], weight=[10,1])
        #param = svm.svm_parameter(C=10, kernel_type=svm.RBF)#, nr_weight=2, weight_label=[1,0], weight=[10,1])
        #self.model = svm.svm_model(problem, param)
        #self.nclasses = self.model.get_nr_class()
        #pdb.set_trace()
        #print 'trained svm'
    def sv_indices(self):
        return np.where(self.model.get_support_vectors(self.nsamples))[0]

    def predict(self, instances):
        xi = instances.T.tolist()
        #pdb.set_trace()
        return su.svm_predict([0]*instances.shape[1], xi, self.model)[0]
        #return self.model.predict()

    def distances(self, instances):
        #d = self.model.predict_values()
        xi = instances.T.tolist()
        #pdb.set_trace()
        dists = su.svm_predict([0]*instances.shape[1], xi, self.model)[2]
        return (np.matrix(dists)).T.A1.tolist()
        #nc = self.model.get_nr_class()
        #ddict = {}
        #ii = 0
        #for i in range(nc):
        #    for j in range(i+1, nc):
        #        ddict[(i,j)] = dists[ii]
        #        ii = ii+1
        #if nc == 2:
        #    #pdb.set_trace()
        #    #print ddict
        #    return - ddict[(NEGATIVE, POSITIVE)]
        #else:
        #    return ddict

class DataScale:

    def __init__(self):
        self.maxes = None
        self.mins = None
        self.ranges = None

    def scale(self, data):
        if self.maxes == None:
            self.maxes = np.max(data,1)
            self.mins = np.min(data,1)
            self.ranges = self.maxes - self.mins
            self.ranges[np.where(self.ranges < .001)[0], :] = 1.
            #pdb.set_trace()
        scaled = (((data - self.mins) / self.ranges) * 2.) - 1
        return scaled


class SVMPCA_ActiveLearner:
    def __init__(self):
        self.classifier = 'svm' #or 'knn'
        self.classifiers = {}
        self.n = 3.
        # fake_train = np.matrix(np.row_stack((np.matrix(range(8)), np.matrix(range(3,3+8)))), dtype='float32')
        # fake_resp = np.matrix([1,0], dtype='float32')
        # self.classifiers['svm'] = cv.SVM(fake_train, fake_resp)

        #self.intensities_index = None
        self.intensities_mean = None
        self.projection_basis = None
        self.reduced_dataset = None
        self.dataset = None

    def get_closest_instances(self, instances, n=1):
        p_instances = np.matrix(self.partial_pca_project(instances))
        s_instances = self.scale.scale(p_instances)
        #distances = []
        distances = np.array(self.classifiers['svm'].distances(s_instances))
        #for i in range(s_instances.shape[1]):
        #    d = self.classifiers['svm'].distances(s_instances[:,i])
        #    distances.append(d)
        #distances = np.array(distances)
        selected_indices = np.argsort(np.abs(distances))[0:n]
        return selected_indices.tolist(), distances[selected_indices]

        #selected_index = np.argmin(np.abs(distances))
        #pdb.set_trace()
        #return selected_index, distances[selected_index]

    def select_next_instances(self, instances, n=1):
        data_idx, data_dists = self.get_closest_instances(instances, n)
        data_dist = abs(data_dists[0])
        sv_dist = abs(self.sv_dist[0])

        #we've converged if distance to datapoint closest to decision boundary
        #is no closer than distance to the support vectors. 
        #So we return nothing as a proposal.
        print 'SVMPCA_ActiveLearner: support vector dist %f data dist %f' % (sv_dist, data_dist)
        if sv_dist <= data_dist:
            return None, None
        else:
            return data_idx, data_dists

    #def train(self, dataset, intensities_size, variance_keep=.95):
    def train(self, dataset, intensities_size, variance_keep=.95):
        #TODO: somehow generate labels for these datasets...
        self.dataset = dataset
        train = dataset.inputs
        responses = dataset.outputs

        #Calculate PCA vectors
        #self.intensities_index = dataset.num_attributes() - intensities_size 
        self.intensities_index = self.dataset.metadata[-1].extent[0]
        #ic_list[0].sizes['intensity']
        #pdb.set_trace()
        self._calculate_pca_vectors(train, variance_keep)
        #pdb.set_trace()
        train = self.partial_pca_project(train)
    
        print 'SVMPCA_ActiveLearner.train: Training classifier.'
        #train => float32 mat, each row is an example
        #responses => float32 mat, each column is a corresponding response
        train = np.matrix(train, dtype='float32').copy()
        responses = np.matrix(responses, dtype='float32').copy()

        self.reduced_dataset = ds.Dataset(train.copy(), responses.copy())
        #  self.classifiers['svm'] = cv.SVM(train.T.copy(), responses.copy())
        #  self.classifiers['svm'].train(train.T.copy(), responses.copy())
        #  param.kernel_type = 'rbf'
        #  self.classifiers['svm'] = svm.svm_model(problem, param)
        self.scale = DataScale()
        self.rescaled_dataset = ds.Dataset(self.scale.scale(self.reduced_dataset.inputs), 
                                           self.reduced_dataset.outputs)
        self.classifiers['svm'] = SVM(self.rescaled_dataset)
        #self.classifiers['knn'] = sp.KDTree(np.array(train.T.copy()))
        self.classifiers['knn'] = sp.KDTree(np.array(self.rescaled_dataset.inputs.T))

        sv_instances = self.dataset.inputs[:, self.classifiers['svm'].sv_indices()]
        #pdb.set_trace()
        self.sv_idx, self.sv_dist = self.get_closest_instances(sv_instances, 1)

    def classify(self, instances):
        projected_inst = np.matrix(self.partial_pca_project(instances), dtype='float32').copy()
        scaled_inst = self.scale.scale(projected_inst)
        if self.classifier == 'knn':
            labels = self.reduced_dataset.outputs[:, \
                    self.classifiers['knn'].query(scaled_inst, self.n)[1].tolist()[0]]
            if np.sum(labels) > (self.n/2.):
                return POSITIVE
            else:
                return NEGATIVE

        elif self.classifier == 'svm':
            #pdb.set_trace()
            r = self.classifiers['svm'].predict(scaled_inst)
            return r

    def _calculate_pca_vectors(self, data, variance_keep):
        data_in = data[self.intensities_index:, :]
        self.intensities_mean = np.mean(data_in, 1)
        print 'SVMPCA_ActiveLearner._calculate_pca_vectors: Constructing PCA basis'
        self.projection_basis = dr.pca_vectors(data_in, variance_keep)
        print 'SVMPCA_ActiveLearner._calculate_pca_vectors: PCA basis size -', self.projection_basis.shape

    def partial_pca_project(self, instances):
        instances_in = instances[self.intensities_index:, :]
        reduced_intensities = self.projection_basis.T * (instances_in - self.intensities_mean)
        return np.row_stack((instances[:self.intensities_index, :], reduced_intensities))

    #def refine_location(self, idx, ic_data_set):
    #    while not_converged:
    #        # look for projected 3d points close by (2d, at least n pixels away)
    #        # extract features of 3d point
    #        # look at decision distance to these features
    #
    #        #get feature vector for loc2d
    #        ic_data.
    #        d = self.classifiers['svm'].distances()

    #def load(self, fname):
    #    print 'SVM SAVING LOADING NOT IMPLEMENTED'
    #    #  self.classifier['svm'].load(fname)
    #    p = ut.load_pickle(pt.splitext(model_name)[0] + '.pkl')
    #    self.intensities_index = p['intensities_index']
    #    self.intensities_mean  = p['intensities_mean']
    #    self.projection_basis  = p['projection_basis']
    #    self.reduced_dataset   = p['reduced_dataset']

    #def save(self, fname):
    #    #self.classifier['svm'].save(fname)
    #    print 'SVM SAVING LOADING NOT IMPLEMENTED'
    #    ut.save_pickle({'intensities_index':self.intensities_index,
    #                    'intensities_mean': self.intensities_mean, 
    #                    'projection_basis': self.projection_basis,
    #                    'reduced_dataset':  self.reduced_dataset},\
    #                     pt.splitext(fname)[0] + '.pkl')

#    def train_from_dir(self, dirname, features_file_name, variance_keep, \
#                    grid_resolution=.05, win_size=15, win3d_size=.04,
#                    voi_bounds_laser=[.5, .5, .5]):
#
#        #Cache feature computations
#        if not pt.isfile(features_file_name):
#            ic_list = load_data_from_dir(dirname, grid_resolution, win_size, win3d_size, voi_bounds_laser)
#            ut.save_pickle(ic_list)
#        else:
#            print 'features has been calculated already (yay!) loading from', features_file_name
#            ic_list = ut.load_pickle(features_file_name)
#
#        #Combine individual scans into one large dataset
#        dataset = None
#        for ic in self.ic_list:
#            if dataset == None:
#                dataset = ic.extract_vectorized_features()
#            else:
#                dataset = np.column_stack((dataset, ic.extract_vectorized_features()))
#                #dataset.append(ic.extract_vectorized_features())
#
#        self.train(dataset, ic_list[0].sizes['intensity'])

class InterestPointDataset(ds.Dataset):

    def __init__(self, inputs, outputs, pt2d, pt3d, feature_extractor):
        ds.Dataset.__init__(self, inputs, outputs)
        self.pt2d = pt2d
        self.pt3d = pt3d
        offset = 0
        if feature_extractor != None:
            for k in ['normal', 'color', 'intensity']:
                start_idx = offset
                end_idx = offset + feature_extractor.sizes[k]
                self.add_attribute_descriptor(ds.AttributeDescriptor(k, (start_idx, end_idx)))
                offset = end_idx

    #def add(self, pt2d, pt3d, features, label):
    def add(self, features, label, pt2d, pt3d):
        self.inputs = np.column_stack((self.inputs, features))
        self.outputs = np.column_stack((self.outputs, label))
        self.pt2d.append(pt2d)#np.column_stack((self.pt2d, pt2d))
        self.pt3d.append(pt3d)# = np.column_stack((self.pt3d, pt3d))

    def copy(self):
        ipd = InterestPointDataset(self.inputs.copy(), self.outputs.copy(), 
                copy.copy(self.pt2d), copy.copy(self.pt3d), None)
        ipd.metadata = copy.deepcopy(self.metadata)
        return ipd


class Recognize3DParam:

    def __init__(self):
        #Data extraction parameters
        self.grid_resolution = .01
        self.win_size = 5
        self.win3d_size = .02
        self.voi_bl = [.5, .5, .5]
        self.radius = .5

        #sampling parameters
        self.n_samples = 5000
        self.uni_mix = .5
        self.uncertainty = .15

        #variance
        self.variance_keep = .98

def draw_labeled_points(image, dataset, pos_color=[255,102,55], neg_color=[0,184,245], scale=1.):
    pt2d = np.column_stack(dataset.pt2d)
    for l, color in [(POSITIVE, pos_color), (NEGATIVE, neg_color)]:
        cols = np.where(l == dataset.outputs)[1]
        #locs2d = np.matrix(np.round(ic_data.get_location2d(cols.A1)/scale), 'int')
        locs2d = np.matrix(np.round(pt2d[:, cols.A1]/scale), 'int')
        #locs2d = []
        #for idx in cols:
        #locs2d.append(np.matrix(np.round(dataset.pt2d[idx]/scale), 'int'))
        #locs2d = np.column_stack(locs2d)
        #pdb.set_trace()
        draw_points(image, locs2d, color)

def draw_points(img, img_pts, color, size=1):
    for i in range(img_pts.shape[1]):
        center = tuple(np.matrix(np.round(img_pts[:,i]),'int').T.A1.tolist())
        cv.Circle(img, center, size, color, -1)

def draw_dataset(dataset, img, scale=1.):
    npt2d = []
    ppt2d = []
    for i in range(dataset.inputs.shape[1]):
        if dataset.pt2d[i] != None:
            if POSITIVE == dataset.outputs[0,i]:
                ppt2d.append(dataset.pt2d[i]/scale)
            if NEGATIVE == dataset.outputs[0,i]:
                npt2d.append(dataset.pt2d[i]/scale)
    if len(ppt2d) > 0:
        draw_points(img, np.column_stack(ppt2d), [0,255,0], 2)
    if len(npt2d) > 0:
        draw_points(img, np.column_stack(npt2d), [0,0,255], 2)


class InterestPointAppBase:

    def __init__(self, object_name, datafile):
        #pdb.set_trace()
        self.object_name = object_name

        self.learner = None
        self.rec_params = Recognize3DParam()

        #updating/modifying dataset
        self.have_not_trained_learner = True
        self.dataset = None
        self.dataset_cpy = None

        #manage labeling of scanned data
        self.feature_extractor = None
        self.instances = None
        self.points2d = None
        self.points3d = None
        self.classified_dataset = None

        #loading/saving dataset
        self.labeled_data_fname = datafile
        if datafile != None and pt.isfile(datafile):
            self.load_labeled_data()
        else:
            self.labeled_data_fname = object_name + '_labeled.pkl'
       
    def load_labeled_data(self):
        self.dataset = ut.load_pickle(self.labeled_data_fname)
        print 'Loaded from', self.labeled_data_fname
        self.dataset.pt2d = [None] * len(self.dataset.pt2d)
        self.dataset.pt3d = [None] * len(self.dataset.pt3d)
        #self.ipdetector = InterestPointDetector(self.dataset)
        self.train()
        self.have_not_trained_learner = False

    def has_enough_data(self):
        pos_ex = np.sum(self.dataset.outputs)
        neg_ex = self.dataset.outputs.shape[1] - pos_ex
        if pos_ex > 2 and neg_ex > 2:
            return True

    def add_to_dataset(self, feature, label, pt2d, pt3d):
        if self.dataset == None:
            self.dataset = InterestPointDataset(feature, label, [pt2d], [pt3d], self.feature_extractor)
        else:
            self.dataset_cpy = self.dataset.copy()
            self.dataset.add(feature, label, pt2d, pt3d)
            if self.have_not_trained_learner:
                self.train()

    def undo_last_add(self):
        if self.dataset_cpy != None:
            self.dataset = self.dataset_cpy
            self.dataset_cpy = None

    def train(self):
        if self.dataset != None and self.has_enough_data():
            self.learner = SVMPCA_ActiveLearner()
            self.learner.train(self.dataset, self.dataset.metadata[2].extent[0],
                               self.rec_params.variance_keep)
            self.have_not_trained_learner = False
    
    def classify(self):
        #results = []
        #for i in range(self.instances.shape[1]):
        #    results.append(self.learner.classify(self.instances[:,i]))
        #pdb.set_trace()
        results = self.learner.classify(self.instances)
        plist = [self.points2d[:, i] for i in range(self.points2d.shape[1])]
        p3list = [self.points3d[:, i] for i in range(self.points3d.shape[1])]
        self.classified_dataset = InterestPointDataset(self.instances, np.matrix(results), 
                                                           plist, p3list, self.feature_extractor)


class RvizDisplayThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        rospy.init_node('display')

        self.color_cloud_pub = rospy.Publisher('color_cloud', sm.PointCloud)
        self.cloud_pub = rospy.Publisher('orig_cloud', sm.PointCloud)
        self.pc_publishers = {}

        self.labels_pub = rospy.Publisher('point_label', vm.Marker)
        self.publish_queue = qu.Queue()
        self.run_pub_list = []

    def run(self):
        r = rospy.Rate(10)
        print 'DisplayThread: running!'
        self.run_pub_list = []
        while not rospy.is_shutdown():
            self.step()
            r.sleep()
            #while not self.publish_queue.empty():
            #    self.run_pub_list.append(self.publish_queue.get())
            #for pub, msg in self.run_pub_list:
            #    pub.publish(msg)

    def step(self):
        while not self.publish_queue.empty():
            self.run_pub_list.append(self.publish_queue.get())
        for pub, msg in self.run_pub_list:
            pub.publish(msg)

    def display_pc(self, channel, points, frame='base_link'):
        if not self.pc_publishers.has_key(channel):
            self.pc_publishers[channel] = rospy.Publisher(channel, sm.PointCloud)
        #pdb.set_trace()
        pc = ru.np_to_pointcloud(points, frame)
        self.publish_queue.put([self.pc_publishers[channel], pc])

    def display_scan(self, points_bl, valid_points_bl, intensity):
        #publish mapped cloud to verify
        #pdb.set_trace()
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

    def display_training_data_set(self, fname, grid_resolution, win_size, win3d_size):
        data_pkl = ut.load_pickle(fname)
        ic_data = load_data_from_file(fname, grid_resolution, win_size, win3d_size, voi_bounds_laser=[.5, .5, .5])

        #points_bl, colored_points_valid_bl, pos, neg = calculate_features_from_files(fname, data_pkl, 
        #        grid_resolution, win_size)

        #colors = [np.matrix([0,1,0,1.]).T, np.matrix([0,0,1,1.]).T]
        #colors_mat = []
        #points_mat = []
        #for _, points in [pos, neg]:
        #    c = colors.pop(0)
        #    pcolors = np.repeat(c, len(points), axis=1)
        #    colors_mat.append(pcolors)
        #    points_mat.append(np.matrix(np.column_stack(points)))

        #colors_mat = np.column_stack(colors_mat)
        #points_mat = np.column_stack(points_mat)
        #labels_msg = viz.list_marker(points_mat, colors_mat, scale=[.05,.05,.05], mtype='points', mframe='base_link')
        #self.publish_queue.put([self.labels_pub, labels_msg])
        self.display_scan(ic_data.pointcloud_msg, 
                ic_data.points_valid_image[0:3,:], 
                ic_data.points_valid_image[3:,:])


class CVGUI4(InterestPointAppBase):

    def __init__(self, raw_data_fname, object_name, labeled_data_fname=None):
        InterestPointAppBase.__init__(self, object_name, labeled_data_fname)
        self.raw_data_fname = raw_data_fname
        self.feature_extractor = None
        self.load_scan(self.raw_data_fname)

        #Setup displays
        img = self.feature_extractor.image_cv
        self.curr_feature_list = None
        self.scale = 3
        self.frame_number = 0
        self.small_img = cv.CreateMat(img.rows/self.scale, img.cols/self.scale, cv.CV_8UC3)
        self.disp = RvizDisplayThread()
        self.selected = None

        cv.Resize(img, self.small_img)
        cv.NamedWindow('image', cv.CV_WINDOW_AUTOSIZE)
        self.win_names = ['level1', 'level2', 'level4', 'level8', 'level16', 'level32']
        for w in self.win_names:
            cv.NamedWindow(w, 0)
        cv.SetMouseCallback('image', self.mouse_cb, None)
        if not self.have_not_trained_learner:
            self.feature_extractor.feature_vec_at_2d(np.matrix([img.rows/2, img.cols/2.0]).T)
            self.classify()

    def load_scan(self, raw_data_fname):
        self.feature_extractor = load_data_from_file(raw_data_fname, self.rec_params)
        feature_cache_fname = pt.splitext(raw_data_fname)[0] + '_features.pkl'
        if not pt.isfile(feature_cache_fname):
            self.instances, self.points2d, self.points3d = self.feature_extractor.extract_vectorized_features()
            ut.save_pickle([self.instances, self.points2d, self.points3d], feature_cache_fname)
            print 'Saved cache to', feature_cache_fname
        else:
            self.instances, self.points2d, self.points3d = ut.load_pickle(feature_cache_fname)
            print 'Loaded cache from', feature_cache_fname

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            label = POSITIVE
        elif event == cv.CV_EVENT_RBUTTONDOWN:
            label = NEGATIVE
        else:
            return

        #Extract features
        loc = np.matrix([x, y]).T * self.scale
        feature_vec, closest_pt3d, closest_pt2d = self.feature_extractor.feature_vec_at_2d_mat(loc)
        if feature_vec == None:
            return
        feature_list, _, _ = self.feature_extractor.feature_vec_at_2d(loc, viz=True)
        self.curr_feature_list = feature_list[-1]

        #Add data point
        self.add_to_dataset(feature_vec, np.matrix([label]), closest_pt2d, closest_pt3d)
        self.draw()

    def run(self):
        self.disp.display_scan(ru.pointcloud_to_np(self.feature_extractor.pointcloud_msg), 
                               self.feature_extractor.points3d_valid_laser, 
                               self.feature_extractor.colors_valid)
        self.draw()

        while not rospy.is_shutdown():
            self.disp.step()
            k = cv.WaitKey(33)
            if k != -1:
                if k == ord(' '):
                    self.train()
                    self.classify()
                    self.draw()

                if k == ord('l'):
                    selected_idx, selected_dist = self.learner.select_next_instances(self.instances)
                    if selected_idx != None:
                        self.selected = {'idx': selected_idx, 
                                         'd': selected_dist, 
                                         'loc2d': self.points2d[:, selected_idx], 
                                         'label': None}
                        flist, _, _ = self.feature_extractor.feature_vec_at_2d(self.points2d[:,selected_idx], viz=True)
                        if flist != None:
                            self.curr_feature_list = flist[-1]
                        self.draw()
                    else:
                        print '======================='
                        print '>> LEARNER CONVERGED <<'
                        print '======================='
                    
                if k == ord('p'):
                    if self.selected != None:
                        self.selected['label'] = POSITIVE
                        self.draw()

                if k == ord('n'):
                    if self.selected != None:
                        self.selected['label'] = NEGATIVE
                        self.draw()

                if k == ord('a'):
                    if self.selected != None and self.selected['label'] != None:
                        idx = self.selected['idx']
                        self.add_to_dataset(self.instances[:,idx], np.matrix([self.selected['label']]),
                                            self.points2d[:,idx], self.points3d[:,idx])
                        self.train()
                        self.classify()
                        self.selected = None
                        self.draw()

                if k == ord('u'):
                    self.undo_last_add()
                    self.train()
                    self.classify()
                    self.draw()

                if k == ord('s'):
                    if self.dataset != None:
                        print 'saved to', self.labeled_data_fname
                        ut.save_pickle(self.dataset, self.labeled_data_fname)

                if k == ord('o'):
                    self.load_labeled_data()

    def draw(self):
        img = cv.CloneMat(self.small_img)
        draw_dataset(self.dataset, img, self.scale)
        self._draw_classified_dataset(img)
        self._draw_selected(img)
        cv.ShowImage('image', img)
        self._draw_features()

        cv.SaveImage('active_learn%d.png' % self.frame_number, img)
        self.frame_number = self.frame_number + 1
        return img

        ## draw labeled points
        #npt2d = []
        #ppt2d = []
        #for i in range(self.dataset.inputs.shape[1]):
        #    if self.dataset.pt2d[i] != None:
        #        if POSITIVE == self.dataset.outputs[0,i]:
        #            ppt2d.append(self.dataset.pt2d[i]/self.scale)
        #        if NEGATIVE == self.dataset.outputs[0,i]:
        #            npt2d.append(self.dataset.pt2d[i]/self.scale)

        ##print 'neg points', npt2d
        ##print 'pos points', ppt2d

        ##pdb.set_trace()
        #if len(ppt2d) > 0:
        #    draw_points(img, np.column_stack(ppt2d), [0,255,0], 2)
        #if len(npt2d) > 0:
        #    draw_points(img, np.column_stack(npt2d), [0,0,255], 2)

        #pdb.set_trace()

    def _draw_selected(self, img):
        if self.selected == None:
            return
        if self.selected['label'] == None:
            color = [255, 255, 255]

        elif self.selected['label'] == POSITIVE:
            color = [0,255,0]

        elif self.selected['label'] == NEGATIVE:
            color = [0,0,255]

        draw_points(img, self.selected['loc2d']/self.scale, color, 4)

    def _draw_classified_dataset(self, img):
        if self.classified_dataset == None:
            return
        #pdb.set_trace()
        draw_labeled_points(img, img, scale=self.scale)

    def _draw_features(self):
        if self.curr_feature_list == None:
            return
        for imgnp, win in zip(self.curr_feature_list, self.win_names):
            cv.ShowImage(win, imgnp)

#if __name__ == '__main__':
#    
#    labels = [0, 0, 1, 1]
#    samples = [[1, 1], [1, -1], [-1, 1], [-1, -1]]
#    dataset = ds.Dataset(np.matrix(samples).T, np.matrix(labels))
#    s = SVM(dataset)
#    for i in range(len(samples)):
#        print 'SAMPLE', i
#        print s.distances(np.matrix(samples[i]).T)
#    exit()

if __name__ == '__main__':
    import sys
    #pops up a window displaying prosilica image
    #cvgui = CVGUI(sys.argv[1])
    #cvgui.run()

    object_name = sys.argv[1]
    raw_data_fname = sys.argv[2]
    if len(sys.argv) > 3:
        labeled_data_fname = sys.argv[3]
    else:
        labeled_data_fname = None

    cvgui = CVGUI4(raw_data_fname, object_name, labeled_data_fname)
    cvgui.run()


    #clicking allows user to label

    #computer can propose





#
#for each positively labeled example do a gradient search using SVM distances
#

#TODO
# DONE what's up with points at image boundaries??
# DONE loading and saving samples
# DONE Sample image a little better
#        Issue: not enough precision, increasing density increases computation time too much
# Hook up to robot
# Add translation fix

#
# Get rid of overlapping points
# Don't calculate features on the fly, precompute PCA/ICA for task?
# Get rid of negative proposed points
# Add hard decicision boundaries such as positive points cannot possibly be outside this area
# Optimize SVM parameters
#


































































#class InterestPointDetector:
#
#    def __init__(self, dataset=None):
#        #Load learner
#        #self.object_name = name
#        self.learner = None
#
#        #self.cur_dataset = None
#        #self.dataset_queue = []
#        if dataset != None:
#            self.train(dataset)
#
#    #def _draw_save_results(self, cloud, image, dataset):
#    #    image_cpy = cv.CloneImage(image)
#    #    r3d.draw_image_labels(ic_data, dataset, image_cpy)
#    #    cv.SaveImage('%s_%s.png' % (self.object_name, str_from_time(cloud.header.stamp.to_time())), image_cpy)
#    #self._draw_save_results(cloud, image, instances, results)
#
#    def find(self, cloud, image, image_T_laser, calib, center, radius):
#        #extract features 
#        self.rec_params.radius = radius
#        ic_data = IntensityCloudData(cloud, image, image_T_laser, calib, center, self.rec_params)
#
#        #label
#        instances, points_3d, points_2d = ic_data.extract_vectorized_features()
#        results = []
#        for i in range(instances.shape[1]):
#            nlabel = self.learner.classify(instances[:, i])
#            results.append(nlabel)
#        results = np.matrix(results)
#
#        #want 3d location of each instance
#        positive_indices = np.where(results == r3d.POSITIVE)[1]
#        points_3d = points_3d[:, positive_indices]
#        points_2d = points_2d[:, positive_indices]
#        dataset = ds.Dataset(instances, results)
#        #return a random point for now
#        #rindex = np.random.randint(0, len(positive_indices))
#        return points_2d, points_3d, dataset
#
#    def train(self, dataset):
#        #if self._append_queued_up_dataset():
#        self.learner = SVMPCA_ActiveLearner()
#        #pdb.set_trace()
#        self.learner.train(dataset, dataset.metadata[2].extent[0],
#                           self.rec_params.variance_keep)
#
#    def save(self, fname):
#        #self._append_queued_up_dataset()
#        ut.save_pickle(self.cur_dataset, fname)
#
#    #def add_labeled_data(self, dataset):
#    #    self.dataset_queue.append(dataset)
#
#    #def _append_queued_up_dataset(self):
#    #    data_updated = False
#    #    if self.cur_dataset == None:
#    #        if len(self.dataset_queue) > 0:
#    #            self.cur_dataset = self.dataset_queue.pop(0)
#    #            data_updated = True
#    #        else:
#    #            return False
#    #    else:
#    #        for d in self.dataset_queue:
#    #            self.cur_dataset.inputs = np.column_stack((self.cur_dataset.inputs, d.inputs))
#    #            self.cur_dataset.outputs = np.column_stack((self.cur_dataset.outputs, d.outputs))
#    #            data_updated = True
#    #    return data_updated


#class CVGUI3(InterestPointManager):
#
#    def __init__(self, raw_data_fname, object_name, labeled_data_fname=None):
#        InterestPointManager.__init__(self, object_name, labeled_data_fname)
#        self.raw_data_fname = raw_data_fname
#        self.feature_extractor = None
#        self.load_scan(self.raw_data_fname)
#
#        #Setup displays
#        img = self.feature_extractor.image_cv
#        self.curr_feature_list = None
#        self.scale = 3
#        self.frame_number = 0
#        self.small_img = cv.CreateMat(img.rows/self.scale, img.cols/self.scale, cv.CV_8UC3)
#        self.disp = RvizDisplayThread()
#        self.selected = None
#
#        cv.Resize(img, self.small_img)
#        cv.NamedWindow('image', cv.CV_WINDOW_AUTOSIZE)
#        self.win_names = ['level1', 'level2', 'level4', 'level8', 'level16', 'level32']
#        for w in self.win_names:
#            cv.NamedWindow(w, 0)
#        cv.SetMouseCallback('image', self.mouse_cb, None)
#        if not self.blank:
#            self.feature_extractor.feature_vec_at_2d
#            self.classify()
#
#    def load_scan(self, raw_data_fname):
#        self.feature_extractor = load_data_from_file(raw_data_fname, self.ipdetector.rec_params)
#        feature_cache_fname = pt.splitext(raw_data_fname)[0] + '_features.pkl'
#        if not pt.isfile(feature_cache_fname):
#            self.instances, self.points2d, self.points3d = self.feature_extractor.extract_vectorized_features()
#            ut.save_pickle([self.instances, self.points2d, self.points3d], feature_cache_fname)
#            print 'Saved cache to', feature_cache_fname
#        else:
#            self.instances, self.points2d, self.points3d = ut.load_pickle(feature_cache_fname)
#            print 'Loaded cache from', feature_cache_fname
#
#    def mouse_cb(self, event, x, y, flags, param):
#        if event == cv.CV_EVENT_LBUTTONDOWN:
#            label = POSITIVE
#        elif event == cv.CV_EVENT_RBUTTONDOWN:
#            label = NEGATIVE
#        else:
#            return
#
#        #Extract features
#        loc = np.matrix([x, y]).T * self.scale
#        feature_vec, closest_pt3d, closest_pt2d = self.feature_extractor.feature_vec_at_2d_mat(loc)
#        if feature_vec == None:
#            return
#        feature_list, _, _ = self.feature_extractor.feature_vec_at_2d(loc, viz=True)
#        self.curr_feature_list = feature_list[-1]
#
#        #Add data point
#        self.add_to_dataset(feature_vec, np.matrix([label]), closest_pt2d, closest_pt3d)
#        self.draw()
#
#    def run(self):
#        self.disp.display_scan(ru.pointcloud_to_np(self.feature_extractor.pointcloud_msg), 
#                               self.feature_extractor.points3d_valid_laser, 
#                               self.feature_extractor.colors_valid)
#        self.draw()
#
#        while not rospy.is_shutdown():
#            self.disp.step()
#            k = cv.WaitKey(33)
#            if k != -1:
#                if k == ord(' '):
#                    self.train()
#                    self.classify()
#                    self.draw()
#
#                if k == ord('l'):
#                    selected_idx, selected_dist = self.ipdetector.learner.select_next_instances(self.instances)
#                    self.selected = {'idx': selected_idx, 
#                                     'd': selected_dist, 
#                                     'loc2d': self.points2d[:, selected_idx], 
#                                     'label': None}
#                    flist, _, _ = self.feature_extractor.feature_vec_at_2d(self.points2d[:,selected_idx], viz=True)
#                    if flist != None:
#                        self.curr_feature_list = flist[-1]
#                    self.draw()
#                    
#                if k == ord('p'):
#                    if self.selected != None:
#                        self.selected['label'] = POSITIVE
#                        self.draw()
#
#                if k == ord('n'):
#                    if self.selected != None:
#                        self.selected['label'] = NEGATIVE
#                        self.draw()
#
#                if k == ord('a'):
#                    if self.selected != None and self.selected['label'] != None:
#                        idx = self.selected['idx']
#                        self.add_to_dataset(self.instances[:,idx], np.matrix([self.selected['label']]),
#                                            self.points2d[:,idx], self.points3d[:,idx])
#                        self.train()
#                        self.classify()
#                        self.selected = None
#                        self.draw()
#
#                if k == ord('s'):
#                    if self.dataset != None:
#                        print 'saved to', self.labeled_data_fname
#                        ut.save_pickle(self.dataset, self.labeled_data_fname)
#
#                if k == ord('o'):
#                    self.load_labeled_data()
#
#    def draw(self):
#        img = cv.CloneMat(self.small_img)
#
#        # draw labeled points
#        npt2d = []
#        ppt2d = []
#        for i in range(self.dataset.inputs.shape[1]):
#            if self.dataset.pt2d[i] != None:
#                if POSITIVE == self.dataset.outputs[0,i]:
#                    ppt2d.append(self.dataset.pt2d[i]/self.scale)
#                if NEGATIVE == self.dataset.outputs[0,i]:
#                    npt2d.append(self.dataset.pt2d[i]/self.scale)
#
#        #print 'neg points', npt2d
#        #print 'pos points', ppt2d
#
#        #pdb.set_trace()
#        if len(ppt2d) > 0:
#            draw_points(img, np.column_stack(ppt2d), [0,255,0], 2)
#        if len(npt2d) > 0:
#            draw_points(img, np.column_stack(npt2d), [0,0,255], 2)
#
#        #pdb.set_trace()
#        self._draw_classified_dataset(img)
#        self._draw_selected(img)
#        cv.ShowImage('image', img)
#        self._draw_features()
#
#        cv.SaveImage('active_learn%d.png' % self.frame_number, img)
#        self.frame_number = self.frame_number + 1
#        return img
#
#    def _draw_selected(self, img):
#        if self.selected == None:
#            return
#        if self.selected['label'] == None:
#            color = [255, 255, 255]
#
#        elif self.selected['label'] == POSITIVE:
#            color = [0,255,0]
#
#        elif self.selected['label'] == NEGATIVE:
#            color = [0,0,255]
#
#        draw_points(img, self.selected['loc2d']/self.scale, color, 4)
#
#    def _draw_classified_dataset(self, img):
#        if self.classified_dataset == None:
#            return
#        #pdb.set_trace()
#        draw_labeled_points(self.feature_extractor, self.classified_dataset, img, scale=self.scale)
#
#    def _draw_features(self):
#        if self.curr_feature_list == None:
#            return
#        for imgnp, win in zip(self.curr_feature_list, self.win_names):
#            cv.ShowImage(win, imgnp)
#
#
#class CVGUI2:
#
#    def __init__(self, raw_data_fname, object_name, labeled_data_fname=None):
#        self.object_name = object_name
#        self.raw_data_fname = raw_data_fname
#        self.labeled_data_fname = labeled_data_fname
#        if self.labeled_data_fname != None:
#            self.load_labeled_data()
#        else:
#            self.labeled_data_fname = object_name + '_labeled.pkl'
#
#        #Preprocess
#        self.ipdetector = InterestPointDetector()
#        self.feature_extractor = load_data_from_file(raw_data_fname, self.ipdetector.rec_params)
#        feature_cache_fname = pt.splitext(self.raw_data_fname)[0] + '_features.pkl'
#        if not pt.isfile(feature_cache_fname):
#            self.instances, self.points2d, self.points3d = self.feature_extractor.extract_vectorized_features()
#            ut.save_pickle([self.instances, self.points2d, self.points3d], feature_cache_fname)
#            print 'Saved cache to', feature_cache_fname
#        else:
#            self.instances, self.points2d, self.points3d = ut.load_pickle(feature_cache_fname)
#            print 'Loaded cache from', feature_cache_fname
#
#        #Setup windows
#        self.scale = 3
#        img = self.feature_extractor.image_cv
#        self.small_img = cv.CreateMat(img.rows/self.scale, img.cols/self.scale, cv.CV_8UC3)
#        cv.Resize(img, self.small_img)
#        cv.NamedWindow('image', cv.CV_WINDOW_AUTOSIZE)
#        self.win_names = ['level1', 'level2', 'level4', 'level8', 'level16', 'level32']
#        for w in self.win_names:
#            cv.NamedWindow(w, 0)
#        cv.SetMouseCallback('image', self.mouse_cb, None)
#        self.frame_number = 0
#        self.disp = RvizDisplayThread()
#
#        self.blank = True
#        self.curr_feature_list = None
#        self.classified_dataset = None
#        self.dataset = None
#        self.selected = None
#
#    def load_labeled_data(self):
#        if pt.isfile(self.labeled_data_fname):
#            self.dataset = ut.load_pickle(self.labeled_data_fname)
#            print 'loaded from', self.labeled_data_fname
#            self.dataset.pt2d = [None] * len(self.dataset.pt2d)
#            self.dataset.pt3d = [None] * len(self.dataset.pt3d)
#            self.ipdetector = InterestPointDetector( self.dataset)
#            self.ipdetector.train(self.dataset)
#        else:
#            print 'load_labeled_data: ', self.labeled_data_fname, 'does not exist'
#
#
#    def add_to_dataset(self, feature, label, pt2d, pt3d):
#        if self.dataset == None:
#            self.dataset = InterestPointDataset(feature, label, [pt2d], [pt3d], self.feature_extractor)
#        else:
#            self.dataset.add(feature, label, pt2d, pt3d)
#            pos_ex = np.sum(self.dataset.outputs)
#            neg_ex = self.dataset.outputs.shape[1] - pos_ex
#            if pos_ex > 2 and neg_ex > 2 and self.blank:
#                self.train_learner()
#                self.blank = False
#
#    def train_learner(self):
#        if self.dataset != None: 
#            #train
#            self.ipdetector.train(self.dataset)
#            #classify
#            #pdb.set_trace()
#            results = []
#            for i in range(self.instances.shape[1]):
#                results.append(self.ipdetector.learner.classify(self.instances[:,i]))
#            #pdb.set_trace()
#            plist = [self.points2d[:, i] for i in range(self.points2d.shape[1])]
#            p3list = [self.points3d[:, i] for i in range(self.points3d.shape[1])]
#            self.classified_dataset = InterestPointDataset(self.instances, np.matrix(results), 
#                                                           plist, p3list, self.feature_extractor)
#        
#    def mouse_cb(self, event, x, y, flags, param):
#        if event == cv.CV_EVENT_LBUTTONDOWN:
#            label = POSITIVE
#        elif event == cv.CV_EVENT_RBUTTONDOWN:
#            label = NEGATIVE
#        else:
#            return
#
#        #Extract features
#        loc = np.matrix([x, y]).T * self.scale
#        feature_vec, closest_pt3d, closest_pt2d = self.feature_extractor.feature_vec_at_2d_mat(loc)
#        if feature_vec == None:
#            return
#        feature_list, _, _ = self.feature_extractor.feature_vec_at_2d(loc, viz=True)
#        self.curr_feature_list = feature_list[-1]
#
#        #Add data point
#        #pdb.set_trace()
#        self.add_to_dataset(feature_vec, np.matrix([label]), closest_pt2d, closest_pt3d)
#        self.draw()
#
#    def run(self):
#        cv.ShowImage('image', self.small_img)
#        self.disp.display_scan(ru.pointcloud_to_np(self.feature_extractor.pointcloud_msg), 
#                               self.feature_extractor.points3d_valid_laser, 
#                               self.feature_extractor.colors_valid)
#
#        while not rospy.is_shutdown():
#            self.disp.step()
#            k = cv.WaitKey(33)
#            if k != -1:
#                if k == ord(' '):
#                    self.train_learner()
#
#                if k == ord('l'):
#                    selected_idx, selected_dist = self.ipdetector.learner.select_next_instances(self.instances)
#                    self.selected = {'idx': selected_idx, 
#                                     'd': selected_dist, 
#                                     'loc2d': self.points2d[:, selected_idx], 
#                                     'label': None}
#                    flist, _, _ = self.feature_extractor.feature_vec_at_2d(self.points2d[:,selected_idx], viz=True)
#                    if flist != None:
#                        self.curr_feature_list = flist[-1]
#                    self.draw()
#                    
#                if k == ord('p'):
#                    if self.selected != None:
#                        self.selected['label'] = POSITIVE
#                        self.draw()
#
#                if k == ord('n'):
#                    if self.selected != None:
#                        self.selected['label'] = NEGATIVE
#                        self.draw()
#
#                if k == ord('a'):
#                    if self.selected != None and self.selected['label'] != None:
#                        idx = self.selected['idx']
#                        #pdb.set_trace()
#                        self.add_to_dataset(self.instances[:,idx], np.matrix([self.selected['label']]),
#                                            self.points2d[:,idx], self.points3d[:,idx])
#                        self.train_learner()
#                        self.selected = None
#                        self.draw()
#
#                if k == ord('s'):
#                    if self.dataset != None:
#                        print 'saved to', self.labeled_data_fname
#                        ut.save_pickle(self.dataset, self.labeled_data_fname)
#
#                if k == ord('o'):
#                    self.load_labeled_data()
#
#    def draw(self):
#        img = cv.CloneMat(self.small_img)
#
#        # draw labeled points
#        npt2d = []
#        ppt2d = []
#        for i in range(self.dataset.inputs.shape[1]):
#            if self.dataset.pt2d[i] != None:
#                if POSITIVE == self.dataset.outputs[0,i]:
#                    ppt2d.append(self.dataset.pt2d[i]/self.scale)
#                if NEGATIVE == self.dataset.outputs[0,i]:
#                    npt2d.append(self.dataset.pt2d[i]/self.scale)
#
#        #print 'neg points', npt2d
#        #print 'pos points', ppt2d
#
#        #pdb.set_trace()
#        if len(ppt2d) > 0:
#            draw_points(img, np.column_stack(ppt2d), [0,255,0], 2)
#        if len(npt2d) > 0:
#            draw_points(img, np.column_stack(npt2d), [0,0,255], 2)
#
#        #pdb.set_trace()
#        self._draw_classified_dataset(img)
#        self._draw_selected(img)
#        cv.ShowImage('image', img)
#        self._draw_features()
#
#        cv.SaveImage('active_learn%d.png' % self.frame_number, img)
#        self.frame_number = self.frame_number + 1
#        return img
#
#    def _draw_selected(self, img):
#        if self.selected == None:
#            return
#        if self.selected['label'] == None:
#            color = [255, 255, 255]
#
#        elif self.selected['label'] == POSITIVE:
#            color = [0,255,0]
#
#        elif self.selected['label'] == NEGATIVE:
#            color = [0,0,255]
#
#        draw_points(img, self.selected['loc2d']/self.scale, color, 4)
#
#    def _draw_classified_dataset(self, img):
#        if self.classified_dataset == None:
#            return
#        #pdb.set_trace()
#        draw_labeled_points(self.feature_extractor, self.classified_dataset, img, scale=self.scale)
#
#    def _draw_features(self):
#        if self.curr_feature_list == None:
#            return
#        for imgnp, win in zip(self.curr_feature_list, self.win_names):
#            cv.ShowImage(win, imgnp)
        #pdb.set_trace()
        #spts = np.matrix(self.ic_data.sampled_points).T
        #self.disp.display_pc('sampled_points', spts)
        
        # stree = sp.KDTree(np.array(spts.T))
        # indices = stree.query_ball_point(self.ic_data.points3d_valid_laser[:,1000].T, self.grid_resolution)
        # pdb.set_trace()
        # self.disp.display_pc('selected_sampled_points', spts[:, indices.tolist()[0]])

        # #self.ic_data.voi_tree.query_ball_point(np.array(self.ic_data.sampled_points

        #self.disp.run()























#if __name__ == '__obsolete__':
#    import sys
#
#    #mode = 'rebalance'
#    mode = 'test'
#    GRID_RESOLUTION = .03
#    WIN_SIZE = 5
#    features_file_name = 'features_recognize3d.pkl'
#    model_name = 'recognize_3d_trained_svm.xml'
#    rospy.init_node('recognize3d_display')
#
#    if mode == 'test':
#        dirname = sys.argv[1]
#        #test_name = sys.argv[2]
#        #test_dir = sys.argv[2] 
#        #train_model(dirname, features_file_name, model_name, .95)
#        #test_on_training_data(model_name, dirname)
#        
#        VARIANCE_KEEP = .95
#        r3d = Recognize3D()
#        r3d.train(dirname, features_file_name, VARIANCE_KEEP, GRID_RESOLUTION, WIN_SIZE)
#        r3d.save(model_name)
#        #r3d.load(model_name)
#        #r3d.test_training_set(test_name, GRID_RESOLUTION, WIN_SIZE)
#
#    elif mode == 'rebalance':
#        test_name = sys.argv[1]
#        r3d = Recognize3D()
#        r3d.load(model_name)
#        print 'rebalancing dataset'
#        #r3d.rebalance_data_set_retrain()
#        #r3d.kmedoids_rebalance_data_set_retrain()
#        print 'knn data mat size:', r3d.knn.data.shape
#        r3d.test_training_set(test_name, GRID_RESOLUTION, WIN_SIZE)
#
#    elif mode == 'active_learn':
#        r3d = Recognize3D()
#
#
#    else:
#        fname = sys.argv[1]
#        dt = DisplayThread()
#        dt.display_training_data_set(fname, GRID_RESOLUTION, WIN_SIZE)
#        dt.run()
    



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
















#class InterestPointManager:
#
#    def __init__(self, object_name, datafile):
#        #pdb.set_trace()
#        self.object_name = object_name
#        self.ipdetector = InterestPointDetector()
#
#        #updating/modifying dataset
#        self.blank = True
#        self.dataset = None
#
#        #manage labeling of scanned data
#        self.feature_extractor = None
#        self.instances = None
#        self.points2d = None
#        self.points3d = None
#        self.classified_dataset = None
#
#        #loading/saving dataset
#        self.labeled_data_fname = datafile
#        if datafile != None:
#            self.load_labeled_data()
#        else:
#            self.labeled_data_fname = object_name + '_labeled.pkl'
#       
#    def load_labeled_data(self):
#        self.dataset = ut.load_pickle(self.labeled_data_fname)
#        print 'loaded from', self.labeled_data_fname
#        self.dataset.pt2d = [None] * len(self.dataset.pt2d)
#        self.dataset.pt3d = [None] * len(self.dataset.pt3d)
#        #self.ipdetector = InterestPointDetector(self.dataset)
#        self.ipdetector.train(self.dataset)
#        self.blank = False
#
#    def add_to_dataset(self, feature, label, pt2d, pt3d):
#        if self.dataset == None:
#            self.dataset = InterestPointDataset(feature, label, [pt2d], [pt3d], self.feature_extractor)
#        else:
#            self.dataset.add(feature, label, pt2d, pt3d)
#            pos_ex = np.sum(self.dataset.outputs)
#            neg_ex = self.dataset.outputs.shape[1] - pos_ex
#            if pos_ex > 2 and neg_ex > 2 and self.blank:
#                self.train()
#                self.blank = False
#
#    def train(self):
#        if self.dataset != None: 
#            #train
#            self.ipdetector.train(self.dataset)
#
#    def classify(self):
#        results = []
#        for i in range(self.instances.shape[1]):
#            results.append(self.ipdetector.learner.classify(self.instances[:,i]))
#        plist = [self.points2d[:, i] for i in range(self.points2d.shape[1])]
#        p3list = [self.points3d[:, i] for i in range(self.points3d.shape[1])]
#        self.classified_dataset = InterestPointDataset(self.instances, np.matrix(results), 
#                                                           plist, p3list, self.feature_extractor)

#class CVGUI:
#
#    def __init__(self, fname):
#        self.fname = fname
#
#        #self.grid_resolution = .01
#        #self.win_size = 5
#        #self.win3d_size = .02
#        #voi_bounds_laser = [.5, .5, .5]
#
#        self.params = Recognize3DParam()
#        self.scale = 3.
#        #self.variance_keep = .98
#
#        print 'Loading data.'
#        self.ic_data = load_data_from_file(fname, self.params)
#                                            #self.grid_resolution, self.win_size, 
#                                            #self.win3d_size, voi_bounds_laser)
#        print 'Calculating features.'
#        self.features_name = 'features_cache.pkl'
#        if not pt.isfile(self.features_name):
#            self.ic_dataset = self.ic_data.extract_vectorized_features()
#            print 'Saving features.'
#            ut.save_pickle([self.ic_dataset, self.ic_data.feature_locs], self.features_name)
#        else:
#            print 'Loading features.'
#            self.ic_dataset, self.ic_data.feature_locs = ut.load_pickle(self.features_name)
#        self.classified_dataset = None
#
#        #initialize by clicking several positive points and several negative points
#        img = self.ic_data.image_cv
#        self.small_img = cv.CreateMat(img.rows/self.scale, img.cols/self.scale, cv.CV_8UC3)
#        cv.Resize(self.ic_data.image_cv, self.small_img)
#        cv.NamedWindow('image',   cv.CV_WINDOW_AUTOSIZE)
#        self.win_names = ['level1', 'level2', 'level4', 'level8', 'level16', 'level32']
#        for w in self.win_names:
#            cv.NamedWindow(w, 0)
#
#        cv.SetMouseCallback('image', self.mouse_cb, None)
#        self.learner = None
#        self.data = {'pos': [], 'neg': []}
#
#        self.selected = None
#        self.curr_feature_list = None
#        self.disp = RvizDisplayThread()
#        self.frame_number = 0
#        self.loaded_data = None
#
#
#    def draw(self):
#        img = cv.CloneMat(self.small_img)
#        for k, color in [['pos', [0,255,0]], ['neg', [0,0,255]]]:
#            pts = [img_pt for fea, img_pt in self.data[k]]
#            if len(pts) > 0:
#                draw_points(img, np.column_stack(pts), color, 2)
#        self._draw_classified_dataset(img)
#        self._draw_selected(img)
#        cv.ShowImage('image', img)
#        self._draw_features()
#
#        cv.SaveImage('active_learn%d.png' % self.frame_number, img)
#        self.frame_number = self.frame_number + 1
#        return img
#
#    def _draw_classified_dataset(self, img):
#        if self.classified_dataset == None:
#            return
#        #pdb.set_trace()
#        draw_labeled_points(self.ic_data, self.classified_dataset, img)
#
#
#    def _draw_selected(self, img):
#        if self.selected == None:
#            return
#        if self.selected['label'] == None:
#            color = [255, 255, 255]
#
#        elif self.selected['label'] == POSITIVE:
#            color = [0,255,0]
#
#        elif self.selected['label'] == NEGATIVE:
#            color = [0,0,255]
#
#        draw_points(img, self.selected['loc2d']/self.scale, color, 4)
#
#    def _draw_features(self):
#        if self.curr_feature_list == None:
#            return
#        for imgnp, win in zip(self.curr_feature_list, self.win_names):
#            cv.ShowImage(win, imgnp)
#
#    def mouse_cb(self, event, x, y, flags, param):
#        if event == cv.CV_EVENT_LBUTTONDOWN:
#            label = 'pos'
#        elif event == cv.CV_EVENT_RBUTTONDOWN:
#            label = 'neg'
#        else:
#            return
#
#        loc = np.matrix([x, y]).T * self.scale
#        feature_vec, closest_pt3d, closest_pt2d = self.ic_data.feature_vec_at_2d_mat(loc)
#        if feature_vec == None:
#            return
#        #img_pt = np.matrix(np.round(closest_pt2d / self.scale), dtype='int').T.A1.tolist()
#        img_pt = np.matrix(np.round(closest_pt2d / self.scale), dtype='int')
#        self.data[label].append([feature_vec, img_pt])
#
#        #if enough examples, train our classifier
#        if len(self.data['pos']) > 2 and len(self.data['neg']) > 2:
#            if self.learner == None:
#                self.train_learner()
#
#        #pdb.set_trace()
#        feature_list, _, _ = self.ic_data.feature_vec_at_2d(loc, viz=True)
#        if feature_list != None:
#            self.curr_feature_list = feature_list[-1]
#        self.draw()
#
#    def _data_dict_to_dataset(self, data_dict):
#        xlist = []
#        ylist = []
#        for k, label in [['pos', POSITIVE], ['neg', NEGATIVE]]:
#            for fea, _ in data_dict[k]:
#                xlist.append(fea)
#                ylist.append(label)
#
#        print '_data_dict_to_dataset: labels', ylist, 'num ex', len(ylist)
#        if len(xlist) > 0:
#            labeled_set = ds.Dataset(np.column_stack(xlist), np.column_stack(ylist))
#            return labeled_set
#        else:
#            return None
#
#    def train_learner(self):
#        #train a new learner
#        self.learner = SVMPCA_ActiveLearner()
#        #xlist = []
#        #ylist = []
#        #for k, label in [['pos', POSITIVE], ['neg', NEGATIVE]]:
#        #    for fea, _ in self.data[k]:
#        #        xlist.append(fea)
#        #        ylist.append(label)
#
#        #print 'retraining.. labels', ylist, 'num ex', len(ylist)
#        #labeled_set = ds.Dataset(np.column_stack(xlist), np.column_stack(ylist))
#        #pdb.set_trace()
#        labeled_set = self._data_dict_to_dataset(self.data)
#        if self.loaded_data != None:
#            ldata = self._data_dict_to_dataset(self.loaded_data)
#            if labeled_set != None:
#                labeled_set.inputs = np.column_stack((ldata.inputs, labeled_set.inputs))
#                labeled_set.outputs = np.column_stack((ldata.outputs, labeled_set.outputs))
#            else:
#                labeled_set = ldata
#
#        #pdb.set_trace()
#        labeled_set.sizes = self.ic_data.sizes
#        if labeled_set == None:
#            return
#
#        #if self.loc2d
#        #pdb.set_trace()
#        print 'labeled_set.inputs.shape', labeled_set.inputs.shape
#        self.learner.train(labeled_set, self.ic_data.sizes['intensity'], self.params.variance_keep)
#
#        results = []
#        for i in range(self.ic_dataset.shape[1]):
#            nlabel = self.learner.classify(self.ic_dataset[:, i])
#            results.append(nlabel)
#        #pdb.set_trace()
#        results = np.matrix(results)
#        self.classified_dataset = ds.Dataset(self.ic_dataset, results)
#        print 'positives', np.sum(results), 'total', results.shape
#
#
#    def run(self):
#        #pdb.set_trace()
#        self.disp.display_scan(ru.pointcloud_to_np(self.ic_data.pointcloud_msg), 
#                               self.ic_data.points3d_valid_laser, 
#                               self.ic_data.colors_valid)
#
#        cv.ShowImage('image', self.small_img)
#        while not rospy.is_shutdown():
#            self.disp.step()
#            k = cv.WaitKey(33)
#            if k != -1:
#                if k == ord(' '):
#                    self.train_learner()
#                    #self.draw()
#                    cv.SaveImage('active_learn%d.png' % self.frame_number, self.draw())
#                    self.frame_number = self.frame_number + 1
#
#                if k == ord('l'):
#                    selected_idx, selected_d = self.learner.select_next_instances(self.classified_dataset.inputs)
#                    loc = self.ic_data.get_location2d(selected_idx)
#                    selected_locs2d = np.matrix(np.round(loc/self.scale), 'int')
#                    self.selected = {'idx': selected_idx, 'd': selected_d, 'loc2d': selected_locs2d, 'label':None}
#
#                    feature_list, _, _ = self.ic_data.feature_vec_at_2d(loc, viz=True)
#                    if feature_list != None:
#                        self.curr_feature_list = feature_list[-1]
#                    cv.SaveImage('active_learn%d.png' % self.frame_number, self.draw())
#                    self.frame_number = self.frame_number + 1
#                    print selected_idx, selected_d, selected_locs2d
#
#                if k == ord('p'):
#                    if self.selected != None:
#                        self.selected['label'] = POSITIVE
#                    #self.draw()
#                    cv.SaveImage('active_learn%d.png' % self.frame_number, self.draw())
#                    self.frame_number = self.frame_number + 1
#
#                if k == ord('n'):
#                    if self.selected != None:
#                        self.selected['label'] = NEGATIVE
#                    #self.draw()
#                    cv.SaveImage('active_learn%d.png' % self.frame_number, self.draw())
#                    self.frame_number = self.frame_number + 1
#
#                if k == ord('a'):
#                    if self.selected != None and self.selected['label'] != None:
#                        if self.selected['label'] == POSITIVE:
#                            label = 'pos'
#                        else:
#                            label = 'neg'
#                        print '>> adding', label, 'example'
#                        feature_vec = self.classified_dataset.inputs[:, self.selected['idx']]
#                        self.data[label].append([feature_vec, self.selected['loc2d']])
#                        print 'retraining'
#                        self.train_learner()
#                        print 'DONE'
#                        self.selected = None
#                    #self.draw()
#                    cv.SaveImage('active_learn%d.png' % self.frame_number, self.draw())
#                    self.frame_number = self.frame_number + 1
#
#                if k == ord('s'):
#                    if self.loaded_data != None:
#                        self.data['pos'] += self.loaded_data['pos']
#                        self.data['neg'] += self.loaded_data['neg']
#                    ut.save_pickle(self.data, 'active_learned_set.pkl')
#
#                if k == ord('o'):
#                    self.loaded_data = ut.load_pickle('active_learned_set.pkl')
#                    print 'loaded'
#                    self.ic_data.feature_vec_at_2d(np.matrix([500, 500.]).T)
#                    print 'training'
#                    print 'drawing'
#                    self.train_learner()
#                    cv.SaveImage(os.path.splitext(self.fname)[0] + '.png', self.draw())
#                    print 'done'
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
