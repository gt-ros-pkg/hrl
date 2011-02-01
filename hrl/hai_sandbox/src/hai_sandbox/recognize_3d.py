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
import svm

import ml_lib.dataset as ds
import ml_lib.dimreduce as dr

import hrl_opencv.blob as blob
import hrl_lib.image3d as i3d
import hrl_lib.util as ut
import hrl_lib.viz as viz
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu

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


def load_data_from_file(fname, grid_resolution, win_size, win3d_size, voi_bounds_laser):
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
            grid_resolution, win_size, win3d_size, center_point_bl, voi_bounds_laser)


class IntensityCloudData:

    def __init__(self, pointcloud_msg, cvimage_mat, 
            image_T_laser, calibration_obj, 
            grid_resolution, win_size, win3d_size, 
            voi_center_laser, voi_bounds_laser):

        self.pointcloud_msg = pointcloud_msg
        self.image_T_laser = image_T_laser
        self.image_arr = np.asarray(cvimage_mat)
        self.image_cv = cvimage_mat
        self.grid_resolution = grid_resolution
        self.win_size = win_size
        self.win3d_size = win3d_size
        self.calibration_obj = calibration_obj
        self.voi_bl = voi_bounds_laser
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

        #from _generate_loc_in_voi
        self.sampled_points = None

        #from _calculate_features
        self.feature_list = None
        self.feature_locs = None

        #feature sizes
        self.sizes = None

        self._associate_intensity()
        self._limit_to_voi()
        self._generate_loc_in_voi()


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
                                self.voi_bl[0], self.voi_bl[1], self.voi_bl[2], 
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
    def _generate_loc_in_voi(self):
        lim = self.limits_laser
        res = self.grid_resolution
        self.sampled_points = []
        for x in (np.arange(lim[0][0], lim[0][1], res) + (res/2.)):
            for y in (np.arange(lim[1][0], lim[1][1], res) + (res/2.)):
                for z in (np.arange(lim[0][0], lim[2][1], res) + (res/2.)):
                    self.sampled_points.append([x,y,z])


    def _calculate_features_at_sampled_intervals(self):
        self.feature_list = []
        feature_loc_list = []
        non_empty = 0
        empty_queries = 0
        for i, sampled_point_laser in enumerate(self.sampled_points):
            #if i == 6639:
            #if i == 157184:
            #    pdb.set_trace()
            sampled_point_laser = np.matrix(sampled_point_laser).T
            feature = self.feature_vec_at(sampled_point_laser)
            if feature != None:
                self.feature_list.append(feature)
                feature_loc_list.append(sampled_point_laser)
                non_empty = non_empty + 1
            else:
                empty_queries = empty_queries + 1
        print 'empty queries', empty_queries, 'non empty', non_empty
        if len(feature_loc_list) > 0:
            self.feature_locs = np.column_stack(feature_loc_list)
        else:
            self.feature_locs = np.matrix([])
        #return bl_pc, colored_points_valid_bl, [positive_samples, positive_sample_points], \
        #        [negative_samples, negative_sample_points]


    def feature_vec_at_2d(self, loc2d, viz=False):
        #pdb.set_trace()
        indices = self.voi_tree_2d.query(np.array(loc2d.T), k=1)[1]
        closest_pt2d = self.points2d_valid[:, indices[0]]
        closest_pt3d = self.points3d_valid_laser[:, indices[0]]
        return self.feature_vec_at(closest_pt3d, viz=viz), closest_pt2d


    def feature_vec_at_2d_mat(self, loc2d):
        indices = self.voi_tree_2d.query(np.array(loc2d.T), k=1)[1]
        closest_pt2d = self.points2d_valid[:, indices[0]]
        closest_pt3d = self.points3d_valid_laser[:, indices[0]]
        #pdb.set_trace()
        return self.feature_vec_at_mat(closest_pt3d), closest_pt2d


    def feature_vec_at_mat(self, point3d_laser, verbose=False):
        f = self.feature_vec_at(point3d_laser, verbose)
        if f != None:
            return np.row_stack(f)


    ##
    #
    # @param point3d_laser - point to calculate features for 3x1 matrix in laser frame
    # @param verbose
    def feature_vec_at(self, point3d_laser, verbose=False, viz=False):
        indices_list = self.voi_tree.query_ball_point(np.array(point3d_laser.T), self.win3d_size)[0]
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
                    features = i3d.local_window(point2d_image, self.image_arr, self.win_size, flatten=flatten)
                else:
                    features = i3d.local_window(point2d_image, self.image_arr, self.win_size*multiplier, 
                                                resize_to=self.win_size, flatten=flatten)
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
        self._calculate_features_at_sampled_intervals()
        #pdb.set_trace()
        normal_bls, avg_colors, intensities = zip(*self.feature_list)
        normal_bls = np.column_stack(normal_bls) #each column is a different sample
        avg_colors = np.column_stack(avg_colors)
        intensities = np.column_stack(intensities)
        xs = np.row_stack((normal_bls, avg_colors, intensities)) #stack features
        return xs
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
        problem = svm.svm_problem(labels, samples)
        #param = svm.svm_parameter(C=10, nr_weight=2, weight_label=[1,0], weight=[10,1])
        param = svm.svm_parameter(C=10, kernel_type=svm.RBF)#, nr_weight=2, weight_label=[1,0], weight=[10,1])
        self.model = svm.svm_model(problem, param)
        self.nclasses = self.model.get_nr_class()

    def predict(self, instance):
        return self.model.predict(instance.T.A1.tolist())

    def distances(self, instance):
        d = self.model.predict_values(instance.T.A1.tolist())
        if self.nclasses == 2:
            return d[(NEGATIVE, POSITIVE)]
        else:
            return d


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


class SVMActiveLearnerApp:

    def __init__(self):

        self.classifier = 'svm' #or 'knn'
        self.classifiers = {}
        self.n = 3.
        #  fake_train = np.matrix(np.row_stack((np.matrix(range(8)), np.matrix(range(3,3+8)))), dtype='float32')
        #  fake_resp = np.matrix([1,0], dtype='float32')
        #  self.classifiers['svm'] = cv.SVM(fake_train, fake_resp)

        self.intensities_index = None
        self.intensities_mean = None
        self.projection_basis = None
        self.reduced_dataset = None
        self.dataset = None

    def select_next_instance(self, instances):
        p_instances = np.matrix(self.partial_pca_project(instances))
        s_instances = self.scale.scale(p_instances)
        distances = []
        for i in range(s_instances.shape[1]):
            d = self.classifiers['svm'].distances(s_instances[:,i])
            distances.append(d)
        distances = np.array(distances)
        selected_index = np.argmin(np.abs(distances))
        #pdb.set_trace()
        return selected_index, distances[selected_index]


    def train(self, dataset, intensities_size, variance_keep=.95):
        #TODO: somehow generate labels for these datasets...
        self.dataset = dataset
        train = dataset.inputs
        responses = dataset.outputs

        #Calculate PCA vectors
        self.intensities_index = dataset.num_attributes() - intensities_size 
        #ic_list[0].sizes['intensity']
        #pdb.set_trace()
        self._calculate_pca_vectors(self.intensities_index, train, variance_keep)
        #pdb.set_trace()
        train = self.partial_pca_project(train)
    
        print 'training classifier'
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

    def load(self, fname):
        print 'SVM SAVING LOADING NOT IMPLEMENTED'
        #  self.classifier['svm'].load(fname)
        p = ut.load_pickle(pt.splitext(model_name)[0] + '.pkl')
        self.intensities_index = p['intensities_index']
        self.intensities_mean  = p['intensities_mean']
        self.projection_basis  = p['projection_basis']
        self.reduced_dataset   = p['reduced_dataset']

    def save(self, fname):
        #self.classifier['svm'].save(fname)
        print 'SVM SAVING LOADING NOT IMPLEMENTED'
        ut.save_pickle({'intensities_index':self.intensities_index,
                        'intensities_mean': self.intensities_mean, 
                        'projection_basis': self.projection_basis,
                        'reduced_dataset':  self.reduced_dataset},\
                         pt.splitext(fname)[0] + '.pkl')

    def classify(self, instance):
        projected_inst = np.matrix(self.partial_pca_project(instance), dtype='float32').copy()
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

    def _calculate_pca_vectors(self, intensities_index, data, variance_keep):
        data_in = data[self.intensities_index:, :]
        self.intensities_mean = np.mean(data_in, 1)
        print 'Constructing PCA basis'
        self.projection_basis = dr.pca_vectors(data_in, variance_keep)
        print 'PCA basis size:', self.projection_basis.shape

    def partial_pca_project(self, instances):
        instances_in = instances[self.intensities_index:, :]
        reduced_intensities = self.projection_basis.T * (instances_in - self.intensities_mean)
        return np.row_stack((instances[:self.intensities_index, :], reduced_intensities))

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
#                dataset = ic.make_dataset()
#            else:
#                dataset = np.column_stack((dataset, ic.make_dataset()))
#                #dataset.append(ic.make_dataset())
#
#        self.train(dataset, ic_list[0].sizes['intensity'])


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


class CVGUI:

    def __init__(self, fname):
        self.grid_resolution = .01
        self.win_size = 5
        self.win3d_size = .02
        voi_bounds_laser = [.5, .5, .5]
        self.scale = 3.
        self.variance_keep = .98
        print 'Loading data.'
        self.ic_data = load_data_from_file(fname, self.grid_resolution, self.win_size, self.win3d_size, voi_bounds_laser)
        print 'Calculating features.'
        if not pt.isfile('features_cache.pkl'):
            self.ic_dataset = self.ic_data.make_dataset()
            print 'Saving features.'
            ut.save_pickle([self.ic_dataset, self.ic_data.feature_locs], 'features_cache.pkl')
        else:
            print 'Loading features.'
            self.ic_dataset, self.ic_data.feature_locs = ut.load_pickle('features_cache.pkl')

        self.classified_dataset = None

        #initialize by clicking several positive points and several negative points
        img = self.ic_data.image_cv
        self.small_img = cv.CreateMat(img.rows/self.scale, img.cols/self.scale, cv.CV_8UC3)
        cv.Resize(self.ic_data.image_cv, self.small_img)
        cv.NamedWindow('image',   cv.CV_WINDOW_AUTOSIZE)
        cv.NamedWindow('level1',  0)
        cv.NamedWindow('level2',  0)
        cv.NamedWindow('level4',  0)
        cv.NamedWindow('level8',  0)
        cv.NamedWindow('level16', 0)
        cv.NamedWindow('level32', 0)

        cv.SetMouseCallback('image', self.mouse_cb, None)
        self.learner = None
        self.data = {'pos': [], 'neg': []}

        self.selected = None
        self.curr_feature_list = None
        self.disp = RvizDisplayThread()

#
#
#to generate initial distribution of points
#   randomly, uniformly sample for n points in 2d, add in prior information if available (gaussians)
#        for each 2d point p, associate with the 3d point closest to the camera
#        throw away all points not in VOI
#

#
#for each positively labeled example do a gradient search using SVM distances
#



#TODO
# DONE what's up with points at image boundaries??
# DONE loading and saving samples
# Sample image a little better
#     Issue: not enough precision, increasing density increases computation time too much
# Hook up to robot
# Add translation fix

#
# Get rid of overlapping points
# Don't calculate features on the fly, precompute PCA/ICA for task?
# Get rid of negative proposed points
# Add hard decicision boundaries such as positive points cannot possibly be outside this area
# Optimize SVM parameters
#

    def draw(self):
        img = cv.CloneMat(self.small_img)
        for k, color in [['pos', [0,255,0]], ['neg', [0,0,255]]]:
            pts = [img_pt for fea, img_pt in self.data[k]]
            if len(pts) > 0:
                self.draw_points(img, np.column_stack(pts), color, 2)
        self._draw_classified_dataset(img)
        self._draw_selected(img)
        cv.ShowImage('image', img)
        self._draw_features()

    def draw_points(self, img, img_pts, color, size=1):
        for i in range(img_pts.shape[1]):
            cv.Circle(img, tuple(img_pts[:,i].T.A1.tolist()), size, color, -1)

    def _draw_classified_dataset(self, img):
        if self.classified_dataset == None:
            return

        for l, color in [(POSITIVE, [255,102,55]), (NEGATIVE, [0,184,245])]:
            cols = np.where(l == self.classified_dataset.outputs)[1]
            locs2d = np.matrix(np.round(self.ic_data.get_location2d(cols.A1)/self.scale), 'int')
            self.draw_points(img, locs2d, color)

    def _draw_selected(self, img):
        if self.selected == None:
            return
        if self.selected['label'] == None:
            color = [255, 255, 255]

        elif self.selected['label'] == POSITIVE:
            color = [0,255,0]

        elif self.selected['label'] == NEGATIVE:
            color = [0,0,255]

        self.draw_points(img, self.selected['loc2d'], color, 4)

    def _draw_features(self):
        for imgnp, win in zip(self.curr_feature_list, ['level1', 'level2', 'level4', 'level8', 'level16', 'level32']):
            cv.ShowImage(win, imgnp)

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            label = 'pos'
        elif event == cv.CV_EVENT_RBUTTONDOWN:
            label = 'neg'
        else:
            return

        loc = np.matrix([x, y]).T * self.scale
        feature_vec, closest_pt2d = self.ic_data.feature_vec_at_2d_mat(loc)
        if feature_vec == None:
            return
        #img_pt = np.matrix(np.round(closest_pt2d / self.scale), dtype='int').T.A1.tolist()
        img_pt = np.matrix(np.round(closest_pt2d / self.scale), dtype='int')
        self.data[label].append([feature_vec, img_pt])

        #if enough examples, train our classifier
        if len(self.data['pos']) > 2 and len(self.data['neg']) > 2:
            if self.learner == None:
                self.train_learner()

        #pdb.set_trace()
        feature_list, _ = self.ic_data.feature_vec_at_2d(loc, viz=True)
        if feature_list != None:
            self.curr_feature_list = feature_list[-1]
        self.draw()

    def train_learner(self):
        #train a new learner
        self.learner = SVMActiveLearnerApp()
        xlist = []
        ylist = []
        for k, label in [['pos', POSITIVE], ['neg', NEGATIVE]]:
            for fea, _ in self.data[k]:
                xlist.append(fea)
                ylist.append(label)
        #pdb.set_trace()
        print 'retraining.. labels', ylist, 'num ex', len(ylist)
        labeled_set = ds.Dataset(np.column_stack(xlist), np.column_stack(ylist))
        self.learner.train(labeled_set, self.ic_data.sizes['intensity'], self.variance_keep)

        results = []
        for i in range(self.ic_dataset.shape[1]):
            nlabel = self.learner.classify(self.ic_dataset[:, i])
            results.append(nlabel)
        #pdb.set_trace()
        results = np.matrix(results)
        self.classified_dataset = ds.Dataset(self.ic_dataset, results)
        print 'positives', np.sum(results), 'total', results.shape

    def run(self):
        #pdb.set_trace()
        self.disp.display_scan(ru.pointcloud_to_np(self.ic_data.pointcloud_msg), 
                               self.ic_data.points3d_valid_laser, 
                               self.ic_data.colors_valid)

        cv.ShowImage('image', self.small_img)
        while not rospy.is_shutdown():
            self.disp.step()
            k = cv.WaitKey(33)
            if k != -1:
                if k == ord(' '):
                    self.train_learner()
                    self.draw()

                if k == ord('l'):
                    selected_idx, selected_d = self.learner.select_next_instance(self.classified_dataset.inputs)
                    loc = self.ic_data.get_location2d(selected_idx)
                    selected_locs2d = np.matrix(np.round(loc/self.scale), 'int')
                    self.selected = {'idx': selected_idx, 'd': selected_d, 'loc2d': selected_locs2d, 'label':None}

                    feature_list, _ = self.ic_data.feature_vec_at_2d(loc, viz=True)
                    if feature_list != None:
                        self.curr_feature_list = feature_list[-1]
                    self.draw()
                    print selected_idx, selected_d, selected_locs2d

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
                        if self.selected['label'] == POSITIVE:
                            label = 'pos'
                        else:
                            label = 'neg'
                        print '>> adding', label, 'example'
                        feature_vec = self.classified_dataset.inputs[:, self.selected['idx']]
                        self.data[label].append([feature_vec, self.selected['loc2d']])
                        print 'retraining'
                        self.train_learner()
                        print 'DONE'
                        self.selected = None
                    self.draw()

                if k == ord('s'):
                    ut.save_pickle(self.data, 'active_learned_set.pkl')

                if k == ord('o'):
                    self.data = ut.load_pickle('active_learned_set.pkl')
                    self.train_learner()
                    self.draw()



if __name__ == '__main__':
    import sys

    #pops up a window displaying prosilica image
    cvgui = CVGUI(sys.argv[1])
    cvgui.run()


    #clicking allows user to label

    #computer can propose































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
