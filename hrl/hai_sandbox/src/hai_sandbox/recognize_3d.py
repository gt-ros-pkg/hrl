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
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

UNLABELED = 2.0
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
    print 'Original frame of pointcloud is ', data_pkl['points_laser'].header.frame_id

    #use pickle to load image
    image_fname = pt.join(pt.split(fname)[0], data_pkl['high_res'])
    intensity_image = cv.LoadImageM(image_fname)

    #center_point_bl = data_pkl['touch_point'][0]
    center_point_bl = data_pkl['touch_point']
    print 'Robot touched point cloud at point', center_point_bl.T
    #pdb.set_trace()

    #make the data object 
    return IntensityCloudData(data_pkl['points_laser'], intensity_image, 
                              data_pkl['pro_T_bl'], data_pkl['prosilica_cal'], 
                              center_point_bl, rec_param), data_pkl

class IntensityCloudData:

    def __init__(self, pointcloud_msg, cvimage_mat, 
            image_T_laser, calibration_obj, voi_center_laser, rec_param):

        self.params = rec_param

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
        self.limits_laser = None
        self.voi_tree = None
        self.points3d_valid_laser = None
        self.points2d_valid = None

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
        #fill out sizes dict
        self.feature_vec_at_2d(np.matrix([calibration_obj.w/2., 
                                         calibration_obj.h/2.]).T)

    def _associate_intensity(self):
        bl_pc = ru.pointcloud_to_np(self.pointcloud_msg)
        print 'Original point cloud size', bl_pc.shape[1]
        self.points_valid_image, self.colors_valid, self.points2d_valid = \
                i3d.combine_scan_and_image_laser_frame(bl_pc, self.image_T_laser,\
                                            self.image_arr, self.calibration_obj)
        print 'Number of points visible in camera', self.points_valid_image.shape[1]
        self.point_cloud3d_orig = bl_pc

    def _limit_to_voi(self):
        laser_T_image = np.linalg.inv(self.image_T_laser)
        #combined matrix (3xn 3d points) + (mxn color points) = [(m+3) x n matrix]
        all_columns = \
                np.row_stack((tfu.transform_points(laser_T_image,\
                                                   self.points_valid_image[0:3,:]),\
                               self.colors_valid,
                               self.points2d_valid)) 

        #pdb.set_trace()
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

        print 'Number of points in voi', valid_columns.shape[1]

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
                gaussian_noise = gaussian.sample()
                gaussian_noise[0,0] = 0
                sampled3d_pt_laser = self.voi_center_laser + gaussian_noise
                sampled3d_pt_image = tfu.transform_points(self.image_T_laser, sampled3d_pt_laser)
                sampled2d_pt = self.calibration_obj.project(sampled3d_pt_image)
                pt = np.round(sampled2d_pt)
                x = int(pt[0,0])
                y = int(pt[1,0])
                if x < 0 or x > (self.calibration_obj.w-.6) or y < 0 or (y > self.calibration_obj.h-.6):
                    #print 'missed 1'
                    continue

            #get projected 3d points within radius of N pixels
            indices_list = self.voi_tree_2d.query_ball_point(np.array([x,y]), 20.)
            if len(indices_list) < 1:
                #print 'MISSED 2'
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
                if len(self.sampled_points) % 500 == 0:
                    print len(self.sampled_points)


    def _caculate_features_at_sampled_points(self):
        self.feature_list = []
        feature_loc_list = []
        feature_loc2d_list = []
        non_empty = 0
        empty_queries = 0
        for i, sampled_point_laser in enumerate(self.sampled_points):
            if i % 500 == 0:
                print '_caculate_features_at_sampled_points:', i

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
    def feature_vec_at_old(self, point3d_laser, verbose=False, viz=False):
        indices_list = self.voi_tree.query_ball_point(np.array(point3d_laser.T), self.params.win3d_size)[0]
        if len(indices_list) > 4:
            points_in_ball_bl = np.matrix(self.voi_tree.data.T[:, indices_list])
            intensity_in_ball_bl = self.colors_valid[:, indices_list]
            
            #calc normal
            normal_bl = i3d.calc_normal(points_in_ball_bl[0:3,:])
            
            #calc average color
            avg_color = np.mean(intensity_in_ball_bl, 1)
            
            #project mean into 2d
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
    # @param point3d_laser - point to calculate features for 3x1 matrix in laser frame
    # @param verbose
    def feature_vec_at(self, point3d_laser, verbose=False, viz=False, k=4):
        #project into 2d & get intensity window
        point2d_image = self.calibration_obj.project(\
                            tfu.transform_points(self.image_T_laser, point3d_laser))

        #get local features
        if viz == True:
            flatten=False
        else:
            flatten=True

        #find closest neighbors in 3D to get normal
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

        #Get 3d normal
        indices_list = self.voi_tree.query(np.array(point3d_laser.T), k=k)[1]
        points_in_ball_bl = np.matrix(self.voi_tree.data.T[:, indices_list])
        normal_bl = i3d.calc_normal(points_in_ball_bl[0:3,:])

        if not invalid_location:
            if not viz:
                local_intensity = np.row_stack(local_intensity)
            if self.sizes == None:
                self.sizes = {}
                self.sizes['intensity'] = local_intensity.shape[0]
                self.sizes['normal'] = normal_bl.shape[0]

            return [normal_bl, local_intensity], point2d_image
        else:
            if verbose:
                print '>> local_window outside of image'

            return None, None

    ##
    #
    # @return a matrix mxn where m is the number of features and n the number of examples
    def extract_vectorized_features(self):
        self._random_sample_voi()
        self._caculate_features_at_sampled_points()
        normal_bls, intensities = zip(*self.feature_list)
        normal_bls = np.column_stack(normal_bls) #each column is a different sample
        intensities = np.column_stack(intensities)

        xs = np.row_stack((normal_bls, intensities)) #stack features
        return xs, self.feature_locs2d, self.feature_locs

    def get_location2d(self, instance_indices):
        #pdb.set_trace()
        sampled_points3d_laser = self.feature_locs[:,instance_indices]
        return self.calibration_obj.project(tfu.transform_points(self.image_T_laser, \
                sampled_points3d_laser))

class SVM:
    ##
    # zero is on negative side of decision boundary
    # 0  /  1
    # -  /  +
    def __init__(self, dataset, params):
        samples = dataset.inputs.T.tolist()
        labels = dataset.outputs.T.A1.tolist()
        #pdb.set_trace()
        print 'SVM: trianing with params => ', params
        self.model = su.svm_train(labels, samples, params)
        self.nsamples = len(samples)

    def sv_indices(self):
        return np.where(self.model.get_support_vectors(self.nsamples))[0]

    def predict(self, instances):
        xi = instances.T.tolist()
        return su.svm_predict([0]*instances.shape[1], xi, self.model)[0]

    def distances(self, instances):
        xi = instances.T.tolist()
        dists = su.svm_predict([0]*instances.shape[1], xi, self.model)[2]
        return (np.matrix(dists)).T.A1.tolist()


def dataset_to_libsvm(dataset, filename):
    f = open(filename, 'w')

    for i in range(dataset.outputs.shape[1]):
        if dataset.outputs[0,i] == POSITIVE:
            f.write('+1 ')
        else:
            f.write('-1 ')
        #instance
        for j in range(dataset.inputs.shape[0]):
            f.write('%d:%f ' % (j, dataset.inputs[j, i]))
        f.write('\n')

    f.close()


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

        self.intensities_mean = None
        self.projection_basis = None
        self.reduced_dataset = None
        self.dataset = None

    def get_closest_instances(self, instances, n=1):
        p_instances = np.matrix(self.partial_pca_project(instances))
        s_instances = self.scale.scale(p_instances)
        distances = np.array(self.classifiers['svm'].distances(s_instances))
        selected_indices = np.argsort(np.abs(distances))[0:n]
        return selected_indices.tolist(), distances[selected_indices]

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

    def train(self, dataset, intensities_size, svm_params, variance_keep=.95):
        #TODO: somehow generate labels for these datasets...
        self.dataset = dataset
        train = dataset.inputs
        responses = dataset.outputs

        #Calculate PCA vectors
        self.intensities_index = self.dataset.metadata[-1].extent[0]
        self._calculate_pca_vectors(train, variance_keep)
        train = self.partial_pca_project(train)
    
        print 'SVMPCA_ActiveLearner.train: Training classifier.'
        #train => float32 mat, each row is an example
        #responses => float32 mat, each column is a corresponding response
        train = np.matrix(train, dtype='float32').copy()
        responses = np.matrix(responses, dtype='float32').copy()

        self.reduced_dataset = ds.Dataset(train.copy(), responses.copy())
        self.scale = DataScale()
        self.rescaled_dataset = ds.Dataset(self.scale.scale(self.reduced_dataset.inputs), 
                                           self.reduced_dataset.outputs)

        #dataset_to_libsvm(self.rescaled_dataset, 'interest_point_data.libsvm')
        self.classifiers['svm'] = SVM(self.rescaled_dataset, svm_params)
        self.classifiers['knn'] = sp.KDTree(np.array(self.rescaled_dataset.inputs.T))

        sv_instances = self.dataset.inputs[:, self.classifiers['svm'].sv_indices()]
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


class InterestPointDataset(ds.Dataset):

    def __init__(self, inputs, outputs, pt2d, pt3d, feature_extractor, scan_ids=None, idx_in_scan=None, sizes=None):
        ds.Dataset.__init__(self, inputs, outputs)
        self.pt2d = pt2d
        self.pt3d = pt3d
        self.scan_ids = scan_ids
        if scan_ids != None:
            print 'scan id class', self.scan_ids.__class__
        self.idx_in_scan = idx_in_scan
        offset = 0
        if feature_extractor != None or sizes != None:
            if feature_extractor != None:
                sizes = feature_extractor.sizes
            for k in ['normal', 'intensity']:
                start_idx = offset 
                end_idx = offset + sizes[k]
                self.add_attribute_descriptor(ds.AttributeDescriptor(k, (start_idx, end_idx)))
                offset = end_idx


    #def add_multiple(features, labels, pt2d, pt3d, scan_id=None, idx_in_scan=None):
    #    self.inputs = np.column_stack((self.inputs, features))
    #    self.outputs = np.column_stack((self.outputs, labels))
    #    self.pt2d += pt2d
    #    self.pt3d += pt3d
    #    if scan_idx != None:
    #        self.scan_id += scan_id
    #    if idx_in_scan != None:
    #        self.idx_in_scan += idx_in_scan


    #def add(self, pt2d, pt3d, features, label):
    def add(self, features, label, pt2d, pt3d, scan_id=None, idx_in_scan=None):
        print 'added point', pt2d[0,0], pt2d[1,0]
        if np.abs(pt2d[1,0]) > 10000:
            pdb.set_trace()
        if np.abs(pt2d[0,0]) > 10000:
            pdb.set_trace()
        self.inputs = np.column_stack((self.inputs, features))
        self.outputs = np.column_stack((self.outputs, label))
        self.pt2d = np.column_stack((self.pt2d, pt2d))
        #self.pt2d.append(pt2d)#np.column_stack((self.pt2d, pt2d))
        self.pt3d = np.column_stack((self.pt3d, pt3d))
        #self.pt3d.append(pt3d)# = np.column_stack((self.pt3d, pt3d))
        if scan_id != None:
            self.scan_ids = np.column_stack((self.scan_ids, scan_id))
            #print 'scan id class', self.scan_ids.__class__
        if idx_in_scan != None:
            self.idx_in_scan = np.column_stack((self.idx_in_scan, idx_in_scan))
        
        #print 'new pt2d shape', self.pt2d.shape
        #self.scan_ids.append(scan)
        #self.idx_in_scan.append(idx_in_scan)

    def remove(self, instance_idx):
        self.inputs = np.column_stack((self.inputs[:,0:instance_idx], self.inputs[:,instance_idx+1:]))
        self.outputs = np.column_stack((self.outputs[:,0:instance_idx], self.outputs[:,instance_idx+1:]))
        self.pt2d = np.column_stack((self.pt2d[:,0:instance_idx], self.pt2d[:,instance_idx+1:]))
        self.pt3d = np.column_stack((self.pt3d[:,0:instance_idx], self.pt3d[:,instance_idx+1:]))
        if self.scan_ids != None:
            self.scan_ids = np.column_stack((self.scan_ids[:,0:instance_idx], self.scan_ids[:,instance_idx+1:]))
        if self.idx_in_scan != None:
            self.idx_in_scan = np.column_stack((self.idx_in_scan[:,0:instance_idx], self.idx_in_scan[:,instance_idx+1:]))

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
        self.svm_params = '-s 0 -t 2 -g .0625 -c 4'

        #sampling parameters
        #self.n_samples = 5000
        self.n_samples = 5000
        self.uni_mix = .3
        self.uncertainty = .05
        #print "Uncertainty is:", self.uncertainty

        #variance
        self.variance_keep = .98

def draw_labeled_points(image, dataset, pos_color=[255,102,55], neg_color=[0,184,245], scale=1.):
    #pt2d = np.column_stack(dataset.pt2d)
    pt2d = dataset.pt2d 
    for l, color in [(POSITIVE, pos_color), (NEGATIVE, neg_color)]:
        cols = np.where(l == dataset.outputs)[1]
        locs2d = np.matrix(np.round(pt2d[:, cols.A1]/scale), 'int')
        draw_points(image, locs2d, color)

def draw_points(img, img_pts, color, size=1, thickness=-1):
    for i in range(img_pts.shape[1]):
        center = tuple(np.matrix(np.round(img_pts[:,i]),'int').T.A1.tolist())
        cv.Circle(img, center, size, color, thickness)

def draw_dataset(dataset, img, scale=1., size=2, scan_id=None):
    npt2d = []
    ppt2d = []
    for i in range(dataset.inputs.shape[1]):
        if dataset.pt2d[0,i] != None:
            if scan_id != None and dataset.scan_ids != None:
                if scan_id != dataset.scan_ids[0,i]:
                    continue
            if POSITIVE == dataset.outputs[0,i]:
                ppt2d.append(dataset.pt2d[:,i]/scale)
            if NEGATIVE == dataset.outputs[0,i]:
                npt2d.append(dataset.pt2d[:,i]/scale)
    if len(ppt2d) > 0:
        draw_points(img, np.column_stack(ppt2d), [0,255,0], size)
    if len(npt2d) > 0:
        draw_points(img, np.column_stack(npt2d), [0,0,255], size)


class InterestPointAppBase:

    def __init__(self, object_name, datafile):
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
        self.dataset.pt2d = np.zeros((2, len(self.dataset.pt2d))) + np.nan
        self.dataset.pt3d = np.zeros((3, len(self.dataset.pt2d))) + np.nan
        #self.dataset.pt2d = [None] * len(self.dataset.pt2d)
        #self.dataset.pt3d = [None] * len(self.dataset.pt3d)
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
            self.dataset = InterestPointDataset(feature, label, pt2d, pt3d, self.feature_extractor)
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
            self.learner.train(self.dataset, self.dataset.metadata[1].extent[0],
                               self.rec_params.variance_keep)
            self.have_not_trained_learner = False
            self.classify()
    
    def classify(self):
        results = self.learner.classify(self.instances)
        plist = [self.points2d[:, i] for i in range(self.points2d.shape[1])]
        p3list = [self.points3d[:, i] for i in range(self.points3d.shape[1])]
        #pdb.set_trace()
        self.classified_dataset = InterestPointDataset(self.instances, np.matrix(results), 
                                                           plist, p3list, self.feature_extractor)


class ImagePublisher:

    def __init__(self, channel, cal=None):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(channel + '/image', Image)
        if cal != None:
            self.image_cal_pub = rospy.Publisher(channel + '/camera_info', sm.CameraInfo)
            self.cal = cal
        else:
            self.image_cal_pub = None

    def publish(self, cv_image): 
        self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
        if self.image_cal_pub != None:
            self.image_cal_pub.publish(self.cal.msg)


class RvizDisplayThread(threading.Thread):

    def __init__(self, cal=None):
        threading.Thread.__init__(self)
        try:
            rospy.init_node('display')
        except Exception,e:
            print e

        self.color_cloud_pub = rospy.Publisher('color_cloud', sm.PointCloud)
        self.cloud_pub = rospy.Publisher('orig_cloud', sm.PointCloud)
        self.pc_publishers = {}

        self.labels_pub = rospy.Publisher('point_label', vm.Marker)
        self.publish_queue = qu.Queue()
        self.image_pub = ImagePublisher('rviz_image', cal)
        self.run_pub_list = []

    def run(self):
        r = rospy.Rate(10)
        print 'DisplayThread: running!'
        self.run_pub_list = []
        while not rospy.is_shutdown():
            self.step()
            r.sleep()

    def step(self):
        while not self.publish_queue.empty():
            self.run_pub_list.append(self.publish_queue.get())
        for pub, msg in self.run_pub_list:
            pub.publish(msg)

    def display_pc(self, channel, points, frame='base_link'):
        if not self.pc_publishers.has_key(channel):
            self.pc_publishers[channel] = rospy.Publisher(channel, sm.PointCloud)
        pc = ru.np_to_pointcloud(points, frame)
        self.publish_queue.put([self.pc_publishers[channel], pc])

    def display_scan(self, points_bl, valid_points_bl, intensity, image=None, cal=None):
        #publish mapped cloud to verify
        valid_points_pro_msg  = ru.np_to_rgb_pointcloud(valid_points_bl, intensity, 'base_link')
        points_bl_msg = ru.np_to_pointcloud(points_bl, 'base_link')
        self.publish_queue.put([self.cloud_pub, points_bl_msg])
        self.publish_queue.put([self.color_cloud_pub, valid_points_pro_msg])
        if image != None:
            self.publish_queue.put([self.image_pub, image])

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
        ic_data, _ = load_data_from_file(fname, grid_resolution, win_size, win3d_size, voi_bounds_laser=[.5, .5, .5])

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
        self.feature_extractor, _ = load_data_from_file(raw_data_fname, self.rec_params)
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
        #if self.learner != None:
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
        if self.dataset != None:
            draw_dataset(self.dataset, img, self.scale)
        self._draw_classified_dataset(img)
        self._draw_selected(img)
        cv.ShowImage('image', img)

        self._draw_features()

        cv.SaveImage('active_learn%d.png' % self.frame_number, img)
        self.frame_number = self.frame_number + 1
        return img

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
        draw_labeled_points(img, self.classified_dataset, scale=self.scale)

    def _draw_features(self):
        if self.curr_feature_list == None:
            return
        for imgnp, win in zip(self.curr_feature_list, self.win_names):
            cv.ShowImage(win, imgnp)


def inverse_indices(indices_exclude, num_elements):
    temp_arr = np.zeros(num_elements)+1
    temp_arr[indices_exclude] = 0
    return np.where(temp_arr)[0]


def preprocess_scan_extract_features(raw_data_fname):
    rec_params = Recognize3DParam()
    feature_extractor, data_pkl = load_data_from_file(raw_data_fname, rec_params)
    image_fname = pt.join(pt.split(raw_data_fname)[0], data_pkl['high_res'])
    print 'Image name is', image_fname
    img = cv.LoadImageM(image_fname)
    feature_cache_fname = pt.splitext(raw_data_fname)[0] + '_features_dict.pkl'
    instances, points2d, points3d = feature_extractor.extract_vectorized_features()
    labels = np.matrix([UNLABELED] * instances.shape[1])

    preprocessed_dict = {'instances': instances,
                         'points2d': points2d,
                         'points3d': points3d,
                         'image': image_fname,
                         'labels': labels,
                         'sizes': feature_extractor.sizes}
    ut.save_pickle(preprocessed_dict, feature_cache_fname)
    return feature_cache_fname


def preprocess_data_in_dir(dirname):
    print 'Preprocessing data in', dirname
    data_files = glob.glob(pt.join(dirname, '*dataset.pkl'))
    print 'Found %d scans' % len(data_files)

    for n in data_files:
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Loading', n
        cn = preprocess_scan_extract_features(n)
        print 'Saved to', cn
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'

def make_point_exclusion_test_set(training_dataset, all_data_dir):
    #pdb.set_trace()
    scan_names = glob.glob(pt.join(all_data_dir, '*_features_dict.pkl'))
    dset = {'inputs': [], 'outputs': [], 'pt2d': [], 'pt3d': []}
    sizes = None
    i = 0
    for sn in scan_names:
        #load scan
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'Loading scan', sn
        cn = ut.load_pickle(sn)
        if sizes == None:
            sizes = cn['sizes']
        selected_pts_in_train = np.where(training_dataset.scan_ids == sn)[1]
        print '  There are %d points in scan' % cn['instances'].shape[1]

        #if there are points in training set that are in this scan, unselect them
        if len(selected_pts_in_train) > 0:
            print '  There are %d of these points in the training set.' % len(selected_pts_in_train)
            scan_ind = training_dataset.idx_in_scan[0, selected_pts_in_train].A1
            select_ind = inverse_indices(scan_ind, cn['instances'].shape[1])
            assert((len(scan_ind) + len(select_ind)) == cn['instances'].shape[1])
        else:
            select_ind = np.array(range(cn['instances'].shape[1]))
        print '  Including %d points from this dataset' % len(select_ind)
        
        for datak, dsetk in [('instances', 'inputs'), ('labels', 'outputs'),\
                  ('points2d', 'pt2d'), ('points3d', 'pt3d')]:
            dset[dsetk].append(cn[datak][:, select_ind])
        i = i + 1
        if i > 2:
            break

    return InterestPointDataset(np.column_stack(dset['inputs']),\
                                np.column_stack(dset['outputs']),\
                                None, None, None, sizes = sizes)


class ScanLabeler:

    def __init__(self, dirname):
        self.scan_dir_name = dirname
        self.scan_names = glob.glob(pt.join(dirname, '*_features_dict.pkl'))

        self.current_scan_pred = None
        self.current_scan = None
        self.cdisp = None

        self.scan_idx = 0
        self.scale = 1/3.
        self.mode = 'GROUND_TRUTH'
        self.training_sname = pt.split(dirname)[0] + '_training_set.pkl'
        self.frame_number = 0

        self.rec_params = Recognize3DParam()
        self.learner = None
        self.dataset = None #Training dataset

        cv.NamedWindow('Scan', cv.CV_WINDOW_AUTOSIZE)
        self.load_scan(self.scan_names[self.scan_idx])
        cv.SetMouseCallback('Scan', self.mouse_cb, None)


    def classify(self):
        if self.learner != None:
            print 'Classifying..'
            #results = np.matrix(self.learner.classify(self.current_scan['instances']))
            print '>>> THIS SCAN'
            #pdb.set_trace()
            results = self.print_accuracy(self.current_scan['instances'], self.current_scan['labels'])
            self.current_scan_pred = InterestPointDataset(self.current_scan['instances'], results,
                                        self.current_scan['points2d'], self.current_scan['points3d'], None)
            if np.any(self.current_scan['labels'] == UNLABELED):
                return

            #pdb.set_trace()

            #ncorrect = np.sum(self.current_scan['labels'] == results)
            #print 'This scan: %.2f' % (100.*float(ncorrect) / float(self.current_scan['labels'].shape[1])), '% correct'

        
    def load_scan(self, fname):
        print 'Loading scan', fname
        self.cdisp = {}
        self.current_scan = ut.load_pickle(fname)
        #image_fname = pt.join(pt.split(fname)[0], self.current_scan['image'])
        img = cv.LoadImageM(self.current_scan['image'])
        self.cdisp['tree'] = sp.KDTree(np.array(self.current_scan['points2d'].T))
        self.cdisp['cv'] = cv.CreateMat(int(round(img.rows*self.scale)), 
                                        int(round(img.cols*self.scale)), cv.CV_8UC3)
        cv.Resize(img, self.cdisp['cv'])
        print 'loaded!'
        self.classify()
        self.draw()

    def save_current(self):
        print 'saving current scan\'s data'
        fname = self.scan_names[self.scan_idx]
        ut.save_pickle(self.current_scan, fname)
        print 'done saving!'

    def load_next(self):
        self.scan_idx = max(min(self.scan_idx+1, len(self.scan_names)-1), 0)
        self.load_scan(self.scan_names[self.scan_idx])

    def load_prev(self):
        self.scan_idx = min(max(self.scan_idx-1, 0), len(self.scan_names)-1)
        self.load_scan(self.scan_names[self.scan_idx])

    def draw(self):
        img = cv.CloneMat(self.cdisp['cv'])

        #Draw ground truth labels
        #pdb.set_trace()
        pidx = np.where(self.current_scan['labels'] == POSITIVE)[1].A1.tolist()
        nidx = np.where(self.current_scan['labels'] == NEGATIVE)[1].A1.tolist()
        uidx = np.where(self.current_scan['labels'] == UNLABELED)[1].A1.tolist()

        if len(pidx) > 0:
            ppoints = self.current_scan['points2d'][:, pidx]
            draw_points(img, ppoints * self.scale, [0,255,0], 2, 1)

        if len(nidx) > 0:
            npoints = self.current_scan['points2d'][:, nidx]
            draw_points(img, npoints * self.scale, [0,0,255], 2, 1)

        if len(uidx) > 0:
            upoints = self.current_scan['points2d'][:, uidx]
            draw_points(img, upoints * self.scale, [255,255,255], 2, 1)

        #Draw training set
        if self.dataset != None:
            #pdb.set_trace()
            draw_dataset(self.dataset, img, 1./self.scale, 4, scan_id=self.scan_names[self.scan_idx])

        if self.current_scan_pred != None:
            draw_labeled_points(img, self.current_scan_pred, scale=1./self.scale)

        cv.ShowImage('Scan', img)
        cv.SaveImage('active_learn%d.png' % self.frame_number, img)
        self.frame_number = self.frame_number + 1
        print '- refreshed display -'

    def step(self):
        k = cv.WaitKey(33)
        if k == ord('n'):
            print 'Loading next scan..'
            self.load_next()

        if k == ord('p'):
            print 'loading previous scan..'
            self.load_prev()

        if k == ord('0'):
            self.current_scan['labels'][0, :] = NEGATIVE
            self.draw()
            print 'Labeling everything as negative!'

        if k == ord('`'):
            if self.mode == 'TRAINING_SET':
                self.mode = 'GROUND_TRUTH'
            else:
                self.mode = 'TRAINING_SET'
            print 'ScanLabeler GUI mode changed to', self.mode

        if k == ord('c'):
            self.save_current()
            
        if k == ord('s'):
            if self.dataset != None:
                ut.save_pickle(self.dataset, self.training_sname)
                print 'saved to', self.training_sname

        if k == ord('l'):
            if self.dataset == None and pt.isfile(self.training_sname):
                print 'loading training set from disk'
                self.dataset = ut.load_pickle(self.training_sname)
                print 'loaded', self.training_sname
                self.draw()

        if k == ord('t'):
            if self.dataset != None:
                print 'Making test set.'
                curr_test_set = make_point_exclusion_test_set(self.dataset, self.scan_dir_name)
                print 'Test set has %d instances' % curr_test_set.inputs.shape[1]
                fname = raw_input('Enter test set name to save to: ')
                print 'Saving to', fname
                ut.save_pickle(curr_test_set, fname)
                print 'Saved!'

        if k == ord('r'):
            self.train()
            self.classify()
            self.draw()

        if k == ord(' '):
            if self.learner != None:
                selected_idx, selected_dist = self.learner.select_next_instances(self.current_scan['instances'])
                if selected_idx != None:
                    self.add_to_training_set(selected_idx)
                    self.train()
                    self.classify()
                    self.draw()
                else:
                    print '======================='
                    print '>> LEARNER CONVERGED <<'
                    print '======================='

        if k == ord('z'):
            d = self.dataset
            d1 = InterestPointDataset(self.current_scan['instances'], self.current_scan['labels'],
                    self.current_scan['points2d'], self.current_scan['points3d'], 
                    None, sizes=self.current_scan['sizes'])
            self.load_next()
            d1.add(self.current_scan['instances'], self.current_scan['labels'],
                    self.current_scan['points2d'], self.current_scan['points3d'])

            self.dataset = d1
            self.train()
            print 'generating libsvm dataset'
            dfname = raw_input('enter dataset filename: ')
            dataset_to_libsvm(self.learner.rescaled_dataset, dfname)
            self.dataset = d
            print 'done'

    def automated_run(self):
        # Run learner until convergence on initial scan
        self.dataset = ut.load_pickle(self.training_sname)
        self.train()
        self.classify()
        self.draw()
        k = cv.WaitKey(33)

        not_converged = True
        i = 0
        while not_converged:
            selected_idx, selected_dist = self.learner.select_next_instances(self.current_scan['instances'])
            if selected_idx != None:
                self.add_to_training_set(selected_idx)
                self.train()
                fname = 'drawer_dataset_%d.libsvm' % i
                dataset_to_libsvm(self.learner.rescaled_dataset, fname)
                i = i + 1
                print 'saved to', fname
                self.classify()
                self.draw()
                k = cv.WaitKey(33)

            else:
                print '======================='
                print '>> LEARNER CONVERGED <<'
                print '======================='
                print 'saving dataset to', self.training_sname
                ut.save_pickle(self.dataset, self.training_sname)
                print 'saved!'
                not_converged = False

    def run(self):
        self.automated_run()
        # Make a test set
        while not rospy.is_shutdown():
            self.step()

    def mouse_cb(self, event, x, y, flags, param):
        if not (event == cv.CV_EVENT_LBUTTONDOWN or \
                event == cv.CV_EVENT_RBUTTONDOWN or \
                event == cv.CV_EVENT_MBUTTONDOWN):
            return
        # Grab closest thing, change its label, and redraw
        ck = [self.cdisp['tree'].query(np.array([x,y]) / self.scale, k=1)[1]]
        cr = self.cdisp['tree'].query_ball_point(np.array([x,y]) / self.scale, 10.)
        
        print 'k nearest', len(ck), 'radius', len(cr)

        if len(ck) > len(cr):
            closest_idx = ck
        else:
            closest_idx = cr
        #print 'clicked at img loc', np.array([x,y]) / self.scale
        #print 'len(closest_idx)', len(closest_idx)

        if self.mode == 'GROUND_TRUTH':
            if event == cv.CV_EVENT_LBUTTONDOWN:
                label = POSITIVE
            elif event == cv.CV_EVENT_RBUTTONDOWN:
                label = NEGATIVE
            elif event == cv.CV_EVENT_MBUTTONDOWN:
                label = UNLABELED
            else:
                return

            print 'Current mode is', self.mode
            #print 'old labels', self.current_scan['labels'][0, closest_idx]
            self.current_scan['labels'][0, closest_idx] = label
            #print 'new labels', self.current_scan['labels'][0, closest_idx]

        
        if self.mode == 'TRAINING_SET':
            # Try to add selected point
            if event == cv.CV_EVENT_LBUTTONDOWN:
                self.add_to_training_set(closest_idx)
                ## Don't add examples that have not been labeled (remove examples with the label UNLABELED)
                #labeled_idx = []
                #for idx in closest_idx:
                #    if self.current_scan['labels'][0, idx] != UNLABELED:
                #        labeled_idx.append(idx)

                ## Don't add examples that have been added (keep index of examples added from this scan).
                ## Search the list of labeled examples.
                #filtered_idx = []
                #if self.dataset != None:
                #    for idx in labeled_idx:
                #        #pdb.set_trace()
                #        matched_idx = np.where(self.dataset.idx_in_scan == idx)[1].A1
                #        if len(matched_idx) > 0:
                #            if np.any(self.scan_names[self.scan_idx] == self.dataset.scan_ids[:, matched_idx]):
                #                continue
                #        filtered_idx.append(idx)
                #else:
                #    filtered_idx += labeled_idx

                #pinfo = [self.current_scan[k][:, filtered_idx] for k in \
                #                ['instances', 'labels', 'points2d', 'points3d']]
                #pinfo.append(np.matrix([self.scan_names[self.scan_idx]] * len(filtered_idx)))
                #pinfo.append(np.matrix(filtered_idx))
                #self.add_to_dataset(pinfo)

            # Try to remove the selected point
            elif event == cv.CV_EVENT_RBUTTONDOWN:
                self.remove_from_training_set(closest_idx)
                #if self.dataset != None:
                #    for idx in closest_idx:
                #        matched_idx = np.where(self.dataset.idx_in_scan == idx)[1].A1
                #        if len(matched_idx) > 0:
                #            sm = np.where(self.scan_names[self.scan_idx] == self.dataset.scan_ids[:, matched_idx])[1]
                #            if len(sm) > 0:
                #                to_remove = matched_idx[sm]
                #                print 'removing', len(sm), 'points'
                #                for ridx in to_remove:
                #                    self.dataset.remove(ridx)
        self.draw()

    def add_to_training_set(self, indices):
        # Don't add examples that have not been labeled (remove examples with the label UNLABELED)
        labeled_idx = []
        for idx in indices:
            if self.current_scan['labels'][0, idx] != UNLABELED:
                labeled_idx.append(idx)
        
        # Don't add examples that have been added (keep index of examples added from this scan).
        # Search the list of labeled examples.
        filtered_idx = []
        if self.dataset != None:
            for idx in labeled_idx:
                #pdb.set_trace()
                matched_idx = np.where(self.dataset.idx_in_scan == idx)[1].A1
                if len(matched_idx) > 0:
                    if np.any(self.scan_names[self.scan_idx] == self.dataset.scan_ids[:, matched_idx]):
                        continue
                filtered_idx.append(idx)
        else:
            filtered_idx += labeled_idx
        
        filtered_idx = [filtered_idx[0]]
        pinfo = [self.current_scan[k][:, filtered_idx] for k in \
                        ['instances', 'labels', 'points2d', 'points3d']]
        pinfo.append(np.matrix([self.scan_names[self.scan_idx]] * len(filtered_idx)))
        pinfo.append(np.matrix(filtered_idx))

        if self.dataset == None:
            self.dataset = InterestPointDataset(pinfo[0], pinfo[1], pinfo[2], \
                    pinfo[3], None, pinfo[4], pinfo[5], sizes=self.current_scan['sizes'])
        else:
            self.dataset.add(pinfo[0], pinfo[1], \
                    pinfo[2], pinfo[3], pinfo[4], pinfo[5])

        print '>> Number of examples in dataset:', self.dataset.inputs.shape[1]

    def remove_from_training_set(self, indices):
        if self.dataset != None:
            for idx in indices:
                matched_idx = np.where(self.dataset.idx_in_scan == idx)[1].A1
                if len(matched_idx) > 0:
                    sm = np.where(self.scan_names[self.scan_idx] == self.dataset.scan_ids[:, matched_idx])[1]
                    if len(sm) > 0:
                        to_remove = matched_idx[sm]
                        print 'removing', len(sm), 'points'
                        for ridx in to_remove:
                            self.dataset.remove(ridx)

    def has_enough_data(self):
        pos_ex = np.sum(self.dataset.outputs)
        neg_ex = self.dataset.outputs.shape[1] - pos_ex
        if pos_ex > 2 and neg_ex > 2:
            return True

    def train(self):
        if self.dataset != None and self.has_enough_data():
            nneg = np.sum(self.dataset.outputs == NEGATIVE)
            npos = np.sum(self.dataset.outputs == POSITIVE)
            print '================= Training ================='
            print 'NEG examples', nneg
            print 'POS examples', npos
            print 'TOTAL', self.dataset.outputs.shape[1]
            neg_to_pos_ratio = float(nneg)/float(npos)
            pdb.set_trace()
            weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
            self.learner = SVMPCA_ActiveLearner()
            self.learner.train(self.dataset, self.dataset.metadata[1].extent[0],
                               self.rec_params.svm_params + weight_balance,
                               self.rec_params.variance_keep)

            #correct = np.sum(self.dataset.outputs == np.matrix(self.learner.classify(self.dataset.inputs)))
            #pdb.set_trace()
            #print 'Test set: %.2f' % (100.* (float(correct)/float(self.dataset.outputs.shape[1]))), '% correct'
            print '>>>> TEST SET PEFORMANCE'
            self.print_accuracy(self.dataset.inputs, self.dataset.outputs)
            print '=================  DONE  =================' 

    def print_accuracy(self, instances, true_labels):
        predicted = np.matrix(self.learner.classify(instances))

        posidx = np.where(true_labels == POSITIVE)[1].A1
        negidx = np.where(true_labels == NEGATIVE)[1].A1
        print 'Confusion matrix:'
        if len(negidx) > 0:
            m00 = 100. * float(np.sum(NEGATIVE == predicted[:, negidx])) / len(negidx)
            m01 = 100. * float(np.sum(POSITIVE == predicted[:, negidx])) / len(negidx)
            print '-   %5.2f, %5.2f' % (m00, m01)

        if len(posidx) > 0:
            m10 = 100. * float(np.sum(NEGATIVE == predicted[:, posidx])) / len(posidx)
            m11 = 100. * float(np.sum(POSITIVE == predicted[:, posidx])) / len(posidx)
            print '+  %5.2f, %5.2f' % (m10, m11)

        print '   Total %5.2f' % (100.* (float(np.sum(true_labels == predicted)) / true_labels.shape[1]))
        return predicted

        #print 'POS correct %.2f' % 100. * float(pcor) / len(posidx)
        #print 'NEG correct %.2f' % 100. * float(ncor) / len(negidx)

    def add_to_dataset(self, pinfo):
        if self.dataset == None:
            self.dataset = InterestPointDataset(pinfo[0], pinfo[1], pinfo[2], \
                    pinfo[3], None, pinfo[4], pinfo[5], sizes=self.current_scan['sizes'])
        else:
            self.dataset.add(pinfo[0], pinfo[1], \
                    pinfo[2], pinfo[3], pinfo[4], pinfo[5])


if __name__ == '__main__':
    import sys
    mode = 'label'

    if mode == 'standalone':
        object_name = sys.argv[1]
        raw_data_fname = sys.argv[2]
        if len(sys.argv) > 3:
            labeled_data_fname = sys.argv[3]
        else:
            labeled_data_fname = None

        cvgui = CVGUI4(raw_data_fname, object_name, labeled_data_fname)
        cvgui.run()

    if mode == 'preprocess':
        preprocess_data_in_dir(sys.argv[1])

    if mode == 'label':
        s = ScanLabeler(sys.argv[1])
        s.run()




