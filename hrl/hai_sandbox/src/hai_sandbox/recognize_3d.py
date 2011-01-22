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
#def local_window(location, bw_image, winsize):
def local_window(location, bw_image, winsize, resize_to=None):
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
        if resize_to != None:
           rescaled = np.zeros((resize_to*2+1, resize_to*2+1, subrect.shape[2]), dtype='uint8')
           cv.Resize(subrect, rescaled, cv.CV_INTER_LINEAR)
           subrect = rescaled
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


##
#
# @param sampled_point point to calculate features for
# @param voi_tree tree of 3d points
# @param intensity_paired_with_3d_points 
# @param grid_resolution resolution of grid points
# @param data_pkl 
# @param win_size
# @param intensity_image_array
# @param verbose
#
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
        features1 = local_window(mean_2d, intensity_image_array, win_size)
        features2 = local_window(mean_2d, intensity_image_array, win_size*2, resize_to=win_size)
        features4 = local_window(mean_2d, intensity_image_array, win_size*4, resize_to=win_size)
        features8 = local_window(mean_2d, intensity_image_array, win_size*8, resize_to=win_size)
        features = np.row_stack((features1, features2, features4, features8))
        #features = features1
        if features1 != None and features2 != None and features4 != None and features8 != None:
        #if features != None:
            return [normal_bl, avg_color, features]
        else:
            if verbose:
                print '>> local_window outside of image'
    else:
        if verbose:
            print '>> not enough neighbors!', len(indices_list)

    return None

#def calculate_features_online():
#    pass


##
# Calculate data vectors in a given image and pickle
# @param fname file name
# @param data_pkl
# @param grid_resolution grid resolution to sample
# @param win_size size of patch to collect
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

    return bl_pc, colored_points_valid_bl, [positive_samples, positive_sample_points], \
            [negative_samples, negative_sample_points]


class Recognize3D:

    def __init__(self):
        self.POSITIVE = 1.0
        self.NEGATIVE = 0.

        self.projection_basis = None
        self.intensities_mean = None
        self.size_intensity = None
        self.raw_data_dim = None
        self.training_data = None
        self.knn = None

        fake_train = np.matrix(np.row_stack((np.matrix(range(8)), np.matrix(range(3,3+8)))), dtype='float32')
        print 'fake_train.shape', fake_train.shape
        fake_resp = np.matrix([1,0], dtype='float32')
        self.svm = cv.SVM(fake_train, fake_resp)
        self.display = DisplayThread()
        self.classifier = 'knn'
        if self.classifier == 'knn':
            print 'USING KNN FOR QUERY PREDICTIONS'
        else:
            print 'USING SVM FOR QUERY PREDICTIONS'


    def load(self, model_name):
        print 'loading model ', model_name
        self.svm.load(model_name)
        #construct an svm of the same size as real data, 6 + 2
        p = ut.load_pickle(pt.splitext(model_name)[0] + '.pkl')
        self.projection_basis = p['projection_basis']
        self.intensities_mean = p['intensities_mean']
        self.raw_data_dim     = p['raw_data_dim']
        self.size_intensity   = p['size_intensity']
        self.training_data    = p['training_data']
        self.knn = sp.KDTree(np.array(self.training_data[0]))

    def save(self, model_name):
        print 'saving model to ', model_name
        self.svm.save(model_name)
        ut.save_pickle({'projection_basis': self.projection_basis,
                        'intensities_mean': self.intensities_mean,
                        'raw_data_dim': self.raw_data_dim,
                        'training_data': self.training_data,
                        'size_intensity': self.size_intensity},\
                        pt.splitext(model_name)[0] + '.pkl')

    def kmedoids_rebalance_data_set_retrain(self):
        train, responses = self.training_data
        num_positive_ex = np.sum(responses == self.POSITIVE)
        pos_ex = train[np.where(responses == self.POSITIVE)[1].A1, :]
        neg_ex = train[np.where(responses == self.NEGATIVE)[1].A1, :]

        K = num_positive_ex
        dtw = mlpy.Dtw(onlydist=True)
        km = mlpy.Kmedoids(k = K, dist=dtw)
        #pdb.set_trace()
        medoid_indices, nonmedoid_indices, membership, total_cost = km.compute(np.array(neg_ex))
        subset_of_neg_examples = neg_ex[medoid_indices, :]

        rebalanced_train = np.matrix(np.row_stack((pos_ex, subset_of_neg_examples)), dtype='float32').copy()
        rebalanced_resp = np.matrix(np.column_stack((np.matrix(num_positive_ex * [self.POSITIVE]),\
                                                     np.matrix(len(medoid_indices) * [self.NEGATIVE]))), dtype='float32').copy()

        self.svm = cv.SVM(rebalanced_train, rebalanced_resp)
        self.svm.train(rebalanced_train, rebalanced_resp)
        self.training_data = [rebalanced_train, rebalanced_resp]
        self.knn = sp.KDTree(np.array(rebalanced_train))

    def rebalance_data_set_retrain(self):
        train, responses = self.training_data
        num_positive_ex = np.sum(responses == self.POSITIVE)
        pos_ex = train[np.where(responses == self.POSITIVE)[1].A1, :]
        neg_ex = train[np.where(responses == self.NEGATIVE)[1].A1, :]
        print 'pos_ex', pos_ex.shape
        print 'neg_ex', neg_ex.shape

        neg_indices = np.random.permutation(range(neg_ex.shape[0]))[:num_positive_ex]
        subset_of_neg_examples = neg_ex[neg_indices, :]
        print 'subset of neg_ex', subset_of_neg_examples.shape

        rebalanced_train = np.matrix(np.row_stack((pos_ex, subset_of_neg_examples)), dtype='float32').copy()
        rebalanced_resp = np.matrix(np.column_stack((np.matrix(num_positive_ex * [self.POSITIVE]),\
                                                     np.matrix(len(neg_indices) * [self.NEGATIVE]))), dtype='float32').copy()
        print 'rebalanced train & resp', rebalanced_train.shape, rebalanced_resp.shape

        self.svm = cv.SVM(rebalanced_train, rebalanced_resp)
        self.svm.train(rebalanced_train, rebalanced_resp)
        self.training_data = [rebalanced_train, rebalanced_resp]
        self.knn = sp.KDTree(np.array(rebalanced_train))
        

    ##
    #
    # @param x a column vector of size (self.raw_data_dim x 1)
    def classify(self, x):
        start_intensities = self.raw_data_dim - self.size_intensity
        other_fea   = x[:start_intensities, 0]
        intensities = x[start_intensities:, 0]
        reduced_intensities = self.projection_basis.T * (intensities - self.intensities_mean)
        x_reduced = np.row_stack((other_fea, reduced_intensities))
        x_reduced = np.matrix(x_reduced.T, dtype='float32').copy()
        #print 'x_reduced.shape', x_reduced.shape

        if self.classifier=='knn':
            responses = self.training_data[1]
            #if np.sum(responses[:, self.knn.query(x_reduced, 1)[1].tolist()[0]]) > 0:
            NEIGHBORS = 3.
            if np.sum(responses[:, self.knn.query(x_reduced, NEIGHBORS)[1].tolist()[0]]) > (NEIGHBORS/2.):
                return self.POSITIVE
            else:
                return self.NEGATIVE
        elif self.classifier == 'svm':
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
        num_pos = 0
        for feas in all_fea:
            #pdb.set_trace()
            label = self.classify(np.row_stack(feas))
            labels.append(label)
            num_pos += label
        #pdb.set_trace()
        print 'positive tested as ', self.classify(np.row_stack(pos[0][0]))

        all_pts = []
        all_pts.append(np.matrix(pos[1]).T)
        all_pts.append(np.matrix(neg[1]).T)

        #call display
        print 'Num positives', num_pos
        self.display.display_scan(points_bl, 
                colored_points_valid_bl[0:3,:], 
                colored_points_valid_bl[3:, :])
        self.display.display_classification(np.column_stack(all_pts), 
                np.matrix(labels), 'base_link')
        self.display.run()


    def extract_features_from_dir(self, dirname, features_file_name, grid_resolution, win_size):
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
                points_bl, colored_points_valid_bl, pos, neg = calculate_features(data_file_name, \
                        data_pkl, grid_resolution, win_size)
                positive_examples += pos[0]
                negative_examples += neg[0]
                positive_points   += pos[1]
                negative_points   += neg[1]
    
            print 'num positive samples', len(positive_examples), 
            print 'num negative samples', len(negative_examples)
            print 'saving features'
            ut.save_pickle([positive_examples, negative_examples, positive_points, negative_points], \
                            features_file_name)
        else:
            print 'features has been calculated already (yay!) loading from', features_file_name
            positive_examples, negative_examples, positive_points, negative_points = ut.load_pickle(\
                    features_file_name)

        return positive_examples, negative_examples, positive_points, negative_points 

    ##
    # @return a matrix mxn where m is the number of features and n the number of examples
    def create_training_instances_from_raw_features(self, examples):
        normal_bls, avg_colors, intensities = zip(*examples)
        normal_bls = np.column_stack(normal_bls) #each column is a different sample
        avg_colors = np.column_stack(avg_colors)
        intensities = np.column_stack(intensities)
        xs = np.row_stack((normal_bls, avg_colors, intensities)) #stack features
        return xs, intensities

    ##
    #
    def train(self, dirname, features_file_name, variance_keep, \
            grid_resolution=.05, win_size=15):

        #Get raw eatures
        positive_examples, negative_examples, positive_points, negative_points = self.extract_features_from_dir(\
                dirname, features_file_name, grid_resolution, win_size)
    
        #Turn raw features into matrix format
        size_intensity = None
        all_x = []
        all_y = []
        for examples, label in [(positive_examples, self.POSITIVE), (negative_examples, self.NEGATIVE)]:
            xs, intensities = self.create_training_instances_from_raw_features(examples)
            if size_intensity == None:
                size_intensity = intensities.shape[0]
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
        intensities_mean    = np.mean(intensities_rows, 1)
    
        print 'constructing pca basis'
        projection_basis = dr.pca_vectors(intensities_rows, variance_keep)
        print '>> PCA basis size:', projection_basis.shape
        reduced_intensities = projection_basis.T * (intensities_rows - intensities_mean  )
        #assert(intensities_rows.shape[0] == projection_basis.shape[0])
        train = np.row_stack((train[:start_intensities, :], reduced_intensities))
    
        print 'training classifier'
        #Train classifier...
        #Run pca on intensity
        #train => float32 mat, each row is an example
        #responses => float32 mat, each column is a corresponding response
        traiyon = np.matrix(train.T, dtype='float32').copy()
        responses = np.matrix(responses, dtype='float32').copy()

        svm = cv.SVM(train, responses)
        svm.train(train, responses)

        self.svm = svm
        self.training_data = [train, responses]
        self.projection_basis = projection_basis
        self.intensities_mean = intensities_mean  
        self.size_intensity = size_intensity
        self.knn = sp.KDTree(np.array(train))
        #pdb.set_trace()
        #sampled_points_tree = sp.KDTree(np.array(sampled_points))

    def smart_example_selection_trainer(self, dirname):
        data_files = glob.glob(pt.join(dirname, '*.pkl'))
        #for data_file_name in data_files:
        first_scene_name = data_files[0]
        first_scene_pickle = ut.load_pickle(first_scene_name)
        #TODO get more reasonable features
        fpoints_bl, fcolored_points_valid_bl, fpos, fneg = calculate_features(first_scene_pickle, \
                first_scene_pickle, grid_resolution, win_size)

        #start with one scene 
        #1 positive and 1 negative
        fpos_ex, _ = self.create_training_instances_from_raw_features(fpos[0]) #need new features beside 
                                                                               #from RAW intensity
        fneg_ex, _ = self.create_training_instances_from_raw_features(fneg[0])
        train = np.matrix(np.column_stack((fpos_ex, fneg_ex[:,0])).T, dtype='float32').copy()
        response = np.matrix([1,0.], dtype='float32').copy()

        #for each new scene:
        for scene_name in data_files[1:]:
            #extract patches
            scene_pickle = ut.load_pickle(scene_name)
            points_bl, colored_points_valid_bl, pos, neg = calculate_features(scene_name, \
                    scene_pickle, grid_resolution, win_size)
            all_neg_from_scene = self.create_training_instances_from_raw_features(neg[0]).T
            all_pos_from_scene = self.create_training_instances_from_raw_features(pos[0]).T
            all_ex_from_scene = np.row_stack((all_neg_from_scene, all_pos_from_scene))
            responses_scene = np.column_stack((np.matrix([self.NEGATIVE] * all_neg_from_scene.shape[0]),
                                               np.matrix([self.POSITIVE] * all_pos_from_scene.shape[1])))

            #rank each patch by distance to all positive examples
            all_pos_samples = train[np.where(responses == self.POSITIVE)[1].A1, :] #all pos examples so far
            ex_dists = []
            for ex_idx in range(all_ex_from_scene.shape[0]):
                ex = all_ex_from_scene[ex_idx, :]
                diff = all_pos_samples - ex
                dists = np.power(np.sum(np.power(diff, 2), 1), .5)
                ex_dists.append(np.min(dists))

            closest_ex_is_not_positive = True
            LARGE_NUM = 999999999999
            while closest_ex_is_not_positive:
                #take the closest patch and classify it
                closest_idx = np.argmin(ex_dists)
                if ex_dists[closest_idx] == LARGE_NUM:
                    break
                ex_dists[closest_idx] = LARGE_NUM #make sure we won't select this idx again
                closest_ex = all_ex_from_scene[closest_idx, :]
                closest_ex_label = responses_scene[0, closest_idx]

                #add example regardless because it is the closest point to our positive examples
                train = np.matrix(np.column_stack((train, closest_ex)).T, dtype='float32').copy()
                response = np.column_stack((response, np.matrix([closest_ex_label], dtype='float32'))).copy()
                
                if closest_ex_label == self.POSITIVE:
                    closest_ex_is_not_positive = False

                if closest_ex_label == self.NEGATIVE: 
                    closest_ex_is_not_positive = True

        return train, response 


















        #train 
        #   take highest ranking patch, classify
        #   patch + => -, then add as negative
        #         - => +, then add as positive
        #         else this is correct don't add anything
        #   if 
        ##
        


    #def active_trainer(self, dirname, features_file_name, variance_keep, \
    #        grid_resolution=.05, win_size=15):
    #    print 'training using data in', dirname
    #    #pdb.set_trace()
    #    if not pt.isfile(features_file_name):
    #        data_files = glob.glob(pt.join(dirname, '*.pkl'))
    #        #rospy.init_node('better_recognize3d')
    #
    #        positive_examples = []
    #        negative_examples = []
    #
    #        positive_points = []
    #        negative_points = []
    #
    #        print 'extracting features'
    #        for data_file_name in data_files:
    #            #pos, ppoints, neg, npoints = calculate_features(dirname, fname)
    #            print 'processing', data_file_name
    #            data_pkl = ut.load_pickle(data_file_name)
    #            points_bl, colored_points_valid_bl, pos, neg = calculate_features(data_file_name, data_pkl, grid_resolution, win_size)
    #            positive_examples += pos[0]
    #            negative_examples += neg[0]
    #            positive_points += pos[1]
    #            negative_points += neg[1]
    #
    #        print 'num positive samples', len(positive_examples), 
    #        print 'num negative samples', len(negative_examples)
    #        print 'saving features'
    #        ut.save_pickle([positive_examples, negative_examples, positive_points, negative_points], features_file_name)
    #    else:
    #        print 'features has been calculated already (yay!) loading from', features_file_name
    #        positive_examples, negative_examples, positive_points, negative_points = ut.load_pickle(features_file_name)
    #
    #    #Turn raw features into matrix format
    #    size_intensity = None
    #    all_x = []
    #    all_y = []
    #    for examples, label in [(positive_examples, self.POSITIVE), (negative_examples, self.NEGATIVE)]:
    #        #normal_bl 3x1 mat
    #        #avg_color 3x1 mat
    #        #intensity nx1 mat
    #        normal_bls, avg_colors, intensities = zip(*examples)
    #        normal_bls = np.column_stack(normal_bls) #each column is a different sample
    #        avg_colors = np.column_stack(avg_colors)
    #        intensities = np.column_stack(intensities)
    #        if size_intensity == None:
    #            size_intensity = intensities.shape[0]
    #
    #        xs = np.row_stack((normal_bls, avg_colors, intensities)) #stack features
    #        ys = np.zeros((1, len(examples))) + label
    #        all_x.append(xs)
    #        all_y.append(ys)
    #
    #    train     = np.column_stack(all_x)
    #    responses = np.column_stack(all_y)
    #    self.raw_data_dim = train.shape[0]
    #
    #    #projection basis should be the same dimension as the data
    #    start_intensities = train.shape[0] - size_intensity
    #    intensities_rows  = train[start_intensities:, :]
    #    intensities_mean    = np.mean(intensities_rows, 1)
    #
    #    print 'constructing pca basis'
    #    projection_basis = dr.pca_vectors(intensities_rows, variance_keep)
    #    print '>> PCA basis size:', projection_basis.shape
    #    reduced_intensities = projection_basis.T * (intensities_rows - intensities_mean  )
    #    assert(intensities_rows.shape[0] == projection_basis.shape[0])
    #    train = np.row_stack((train[:start_intensities, :], reduced_intensities))
    #
    #    #print 'training classifier'
    #    #Train classifier...
    #    #Run pca on intensity
    #    #train => float32 mat, each row is an example
    #    #responses => float32 mat, each column is a corresponding response

    #    #These are all the examples without dimensionality reduction
    #    train = np.matrix(train.T, dtype='float32').copy()
    #    responses = np.matrix(responses, dtype='float32').copy()
    #    
    #    num_positive_ex = np.sum(responses == self.POSITIVE)
    #    pos_ex = train[np.where(responses == self.POSITIVE)[1].A1, :]
    #    subset_of_neg_examples = neg_ex[neg_indices, :]
    #    performance_improves = True

    #    while performance_improves:
    #        #train a classifier
    #        #test
    #        #positive
    #        rebalanced_train = np.matrix(np.row_stack((pos_ex, subset_of_neg_examples)), dtype='float32').copy()
    #        rebalanced_resp = np.matrix(np.column_stack((np.matrix(num_positive_ex * [self.POSITIVE]),\
    #                                                     np.matrix(len(neg_indices) * [self.NEGATIVE]))), dtype='float32').copy()
    #        svm = cv.SVM(train, responses)
    #        svm.train(train, responses)

    #    self.svm = svm
    #    self.training_data = [train, responses]
    #    self.projection_basis = projection_basis
    #    self.intensities_mean = intensities_mean  
    #    self.size_intensity = size_intensity
    #    self.knn = sp.KDTree(np.array(train))

    #    #initialize with all positive examples, and 2 means of negative examples
    #    #for each iteration, add negative examples until scores of positive examples goes down

    #    #pdb.set_trace()
    #    #sampled_points_tree = sp.KDTree(np.array(sampled_points))


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


    def display_training_data_set(self, fname, grid_resolution, win_size):
        data_pkl = ut.load_pickle(fname)
        points_bl, colored_points_valid_bl, pos, neg = calculate_features(fname, data_pkl, grid_resolution, win_size)

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

    mode = 'rebalance'
    GRID_RESOLUTION = .03
    WIN_SIZE = 5
    features_file_name = 'features_recognize3d.pkl'
    model_name = 'recognize_3d_trained_svm.xml'
    rospy.init_node('recognize3d_display')

    if mode == 'test':
        dirname = sys.argv[1]
        test_name = sys.argv[2]
        #test_dir = sys.argv[2] 
        #train_model(dirname, features_file_name, model_name, .95)
        #test_on_training_data(model_name, dirname)
        
        VARIANCE_KEEP = .95
        r3d = Recognize3D()
        r3d.train(dirname, features_file_name, VARIANCE_KEEP, GRID_RESOLUTION, WIN_SIZE)
        r3d.save(model_name)
        #r3d.load(model_name)
        r3d.test_training_set(test_name, GRID_RESOLUTION, WIN_SIZE)

    elif mode == 'rebalance':
        test_name = sys.argv[1]
        r3d = Recognize3D()
        r3d.load(model_name)
        print 'rebalancing dataset'
        #r3d.rebalance_data_set_retrain()
        r3d.kmedoids_rebalance_data_set_retrain()
        print 'knn data mat size:', r3d.knn.data.shape
        r3d.test_training_set(test_name, GRID_RESOLUTION, WIN_SIZE)

    elif mode == 'active_learn':
        r3d = Recognize3D()


    else:
        fname = sys.argv[1]
        dt = DisplayThread()
        dt.display_training_data_set(fname, GRID_RESOLUTION, WIN_SIZE)
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
