import roslib; roslib.load_manifest('hai_sandbox')
import cv
import numpy as np


##
# Generalized from Probabilistic robotics for N != weights.shape[0]
def sample_points(weights, N):
    assert(weights.shape[0] >= N)

    M = weights.shape[0]
    weights = weights / np.sum(weights)
    r = np.random.rand() * (1.0/M)
    c = weights[0,0]
    i = 0
    Xt = []

    indices = np.sort(np.random.permutation(np.arange(1, M+1))[0:N]).tolist()
    for m in indices:
        U = r + (m - 1) * (1.0/M)
        while U > c:
            i = i + 1
            c = c + weights[i,0]
        Xt.append(i)
    return Xt

def test_sample_points():
    w = np.matrix([.1,.4,.5]).T
    count = [0., 0., 0.]

    for i in range(6000.):
        idx = sample_points(w, 2)
        for x in idx:
            count[x] += 1
    print np.matrix(count) / np.sum(count)
#test_sample_points()


def intensity_pyramid_feature(self, point2d_image, image, winsize, multipliers, flatten=True):
    invalid_location = False
    local_intensity = []
    for multiplier in multipliers:
        if multiplier == 1:
            features = i3d.local_window(point2d_image, image, win_size, flatten=flatten)
        else:
            features = i3d.local_window(point2d_image, image, win_size*multiplier, 
                                        resize_to=win_size, flatten=flatten)
        if features == None:
            invalid_location = True
            break
        else:
            local_intensity.append(features)
    if invalid_location:
        return None
    else:
        if flatten:
            return np.row_stack(local_intensity)
        else:
            return local_intensity


class IntensityCloudFeatureExtractor:

    def __init__(self, pointcloud_bl, cvimage_mat, expected_loc_bl, distance_feature_points, 
                image_T_bl, camera_calibration, params):

        self.pointcloud_bl = pointcloud_bl
        self.cvimage_mat = cvimage_mat
        self.expected_loc_bl = expected_loc_bl
        self.distance_feature_points = distance_feature_points

        self.image_T_bl = image_T_bl
        self.camera_calibration = camera_calibration
        self.params = params

    def _subsample(self):
        rospy.loginfo('Subsampling using PCL')
        ex = FPFH()
        fpfh_hist, fpfh_cloud, self.pc_sub_samp_bl = ex.get_features(self.pointcloud_bl)

    def _sample_points(self):
        #evaluate all points
        gaussian = pr.Gaussian(self.expected_loc_bl, \
                               np.matrix([[self.params.uncertainty_x**2, 0,      0], \
                                          [0, self.params.uncertainty_y**2,      0], \
                                          [0,      0, self.params.uncertainty_z**2]]))

        pdf = gaussian.pdf_mat()
        probs = pdf(self.pc_sub_samp_bl)

        #sample unique points
        pt_indices = list(set(sample_points(probs.T, self.params.n_samples)))

        #only keep those that are in bound of points
        sampled_pts3d_bl = self.pc_sub_samp_bl[:, pt_indices]
        sampled_pts3d_image = tfu.transform_points(self.image_T_bl, sampled_pts3d_bl)
        sampled_pts2d = self.calibration_obj.project(sampled_pts3d_image)
        sampled_pix2d = np.matrix(np.round(sampled_pts2d))

        #throw away points that are outside of bounds
        x = sampled_pix2d[0,:]
        y = sampled_pix2d[1,:]
        good_pts = np.where((x >= 0) + (x < self.calibration_obj.w) \
                          + (y >= 0) + (y < self.calibration_obj.h))[1].A1

        sampled_pts3d_bl = sampled_pts3d_bl[:, good_pts]
        sampled_pix2d = sampled_pix2d[:, good_pts]
        return sampled_pts3d_bl, sampled_pix2d

    def feature_vec_at(self, point3d_bl, point2d_image):
        fea_calculated = []

        #Get synthetic distance points
        if self.distance_feature_points != None:
            distance_feas = np.power(np.sum(np.power(self.distance_feature_points - point3d_bl, 2), 0), .5).T
            fea_calculated.append(distance_feas)

        #Get intensity features 
        intensity = intensity_pyramid_feature(point2d_image, self.cvimage_mat, 
                self.params.win_size, self.params.win_multipliers, True)
        if intensity == None:
            return None
        else:
            fea_calculated.append(intensity)

        return fea_calculated

    def extract_vectorized_features(self):
        sampled_pts3d_bl, sampled_pix2d = self._sample_points()
        features_l = []
        pts_with_features = []
        for i in range(sampl3d_pts3d_bl.shape[1]):
            features = self.feature_vec_at(sampled_pts3d_bl[:,i], sampled_pix2d[:,i])
            if features != None:
                features_l.append(features)
                pts_with_features.append(i)

        features_by_type = zip(*features_l)
        xs = np.row_stack([np.column_stack(f) for f in features_by_type])
        return xs, sampled_pix2d[:, pts_with_features], sampled_pts3d_bl[:, pts_with_features]



#class IntensityCloudData:
#
#    def __init__(self, pointcloud, cvimage_mat, 
#                 image_T_bl, calibration_obj, 
#                 voi_center_bl, expected_loc_bl, 
#                 distance_feature_points, 
#                 rec_param):
#
#        self.params = rec_param
#
#        if pointcloud.__class__ == np.matrix:
#            self.pointcloud = pointcloud
#        else:
#            assert(pointcloud.header.frame_id == 'base_link')
#            self.pointcloud = ru.pointcloud_to_np(pointcloud)
#
#        #Data
#        #self.pointcloud_msg = pointcloud_msg
#        self.image_T_bl = image_T_bl
#        self.image_arr = np.asarray(cvimage_mat)
#        self.image_cv = cvimage_mat
#
#        self.calibration_obj = calibration_obj
#        self.voi_center_bl = voi_center_bl
#        self.expected_loc_bl = expected_loc_bl
#        self.distance_feature_points = distance_feature_points
#
#        #Quantities that will be calculated
#        #from _associate_intensity
#        self.points_valid_image = None 
#        self.colors_valid = None
#
#        #from _limit_to_voi
#        self.limits_bl = None
#        self.voi_tree = None
#        self.points3d_valid_bl = None
#        self.points2d_valid = None
#
#        #from _grid_sample_voi
#        self.sampled_points = None
#        self.sampled_points2d = None
#
#        #from _calculate_features
#        self.feature_list = None
#        self.feature_locs = None
#
#        #feature sizes
#        self.sizes = None
#
#        self._fpfh()
#        self._associate_intensity()
#        self._limit_to_voi()
#        #self._grid_sample_voi()
#        #fill out sizes dict
#        self.feature_vec_at_2d(np.matrix([calibration_obj.w/2., 
#                                         calibration_obj.h/2.]).T)
#
#
#    def _limit_to_voi(self):
#        bl_T_image = np.linalg.inv(self.image_T_bl)
#        #combined matrix (3xn 3d points) + (mxn color points) = [(m+3) x n matrix]
#        all_columns = \
#                np.row_stack((tfu.transform_points(bl_T_image,\
#                                                   self.points_valid_image[0:3,:]),\
#                               self.colors_valid,
#                               self.points2d_valid)) 
#
#        #pdb.set_trace()
#        valid_columns, self.limits_bl = \
#                i3d.select_rect(self.voi_center_bl, 
#                                self.params.voi_bl[0], 
#                                self.params.voi_bl[1], 
#                                self.params.voi_bl[2], 
#                                all_columns)
#
#        ncolors = self.colors_valid.shape[0]
#        self.points3d_valid_bl = valid_columns[0:3,:]
#        self.colors_valid = valid_columns[3:3+ncolors,:]
#        self.points2d_valid = valid_columns[-2:, :]
#
#        self.voi_tree = sp.KDTree(np.array(self.points3d_valid_bl.T))
#        self.voi_tree_2d = sp.KDTree(np.array(self.points2d_valid.T))
#
#        if valid_columns == None:
#            msg = 'No 3D points in volume of interest!'
#            print msg
#            raise RuntimeError(msg)
#
#        print 'Number of points in voi', valid_columns.shape[1]
#
#
#    def _in_limits(self, p3d):
#        l = self.limits_bl
#        if (l[0][0] < p3d[0,0]) and (p3d[0,0] < l[0][1]) and \
#           (l[1][0] < p3d[1,0]) and (p3d[1,0] < l[1][1]) and \
#           (l[2][0] < p3d[2,0]) and (p3d[2,0] < l[2][1]):
#            return True
#        else:
#            return False
#
#    def _random_sample_voi(self):
#        #to generate initial distribution of points
#        #   randomly, uniformly sample for n points in 2d, add in prior information if available (gaussians)
#        #        for each 2d point p, associate with the 3d point closest to the camera
#        #        throw away all points not in VOI
#        self.sampled_points = []
#        self.sampled_points2d = []
#
#        print 'generating _random_sample_voi'
#        bl_T_image = np.linalg.inv(self.image_T_bl)
#        gaussian = pr.Gaussian(np.matrix([ 0,      0,                          0.]).T, \
#                               np.matrix([[self.params.uncertainty_x**2, 0,      0], \
#                                          [0, self.params.uncertainty_y**2,      0], \
#                                          [0,      0, self.params.uncertainty_z**2]]))
#        gaussian_noise = np.matrix([0, 0, 0.0]).T #We want to try the given point first
#
#
#        distr = {'u': 0, 'g':0}
#        while len(self.sampled_points) < self.params.n_samples:
#            if np.random.rand() < self.params.uni_mix:
#                d = 'u'
#                x = np.random.randint(0, self.calibration_obj.w)
#                y = np.random.randint(0, self.calibration_obj.h)
#            else:
#                d = 'g'
#                sampled3d_pt_bl = self.expected_loc_bl + gaussian_noise
#                gaussian_noise = gaussian.sample()
#                gaussian_noise[0,0] = 0
#                #sampled3d_pt_bl = self.voi_center_bl + gaussian_noise
#                sampled3d_pt_image = tfu.transform_points(self.image_T_bl, sampled3d_pt_bl)
#                sampled2d_pt = self.calibration_obj.project(sampled3d_pt_image)
#                pt = np.round(sampled2d_pt)
#                x = int(pt[0,0])
#                y = int(pt[1,0])
#                if x < 0 or x > (self.calibration_obj.w-.6) or y < 0 or (y > self.calibration_obj.h-.6):
#                    #print 'missed 1'
#                    continue
#
#            #get projected 3d points within radius of N pixels
#            #indices_list = self.voi_tree_2d.query_ball_point(np.array([x,y]), 20.)
#            indices_list = self.voi_tree_2d.query_ball_point(np.array([x,y]), 10.)
#            if len(indices_list) < 1:
#                #print 'MISSED 2'
#                continue
#
#            #select 3d point closest to the camera (in image frame)
#            points3d_image = tfu.transform_points(self.image_T_bl, self.points3d_valid_bl[:, indices_list])
#            closest_z_idx = np.argmin(points3d_image[2,:])
#            closest_p3d_image = points3d_image[:, closest_z_idx]
#            closest_p2d = self.points2d_valid[:, closest_z_idx]
#            closest_p3d_bl = tfu.transform_points(bl_T_image, closest_p3d_image)
#
#            #check if point is in VOI
#            if self._in_limits(closest_p3d_bl):
#                self.sampled_points.append(closest_p3d_bl.T.A1.tolist())
#                self.sampled_points2d.append(closest_p2d.T.A1.tolist())
#                #self.sampled_points2d.append([x,y])
#                distr[d] = distr[d] + 1
#                if len(self.sampled_points) % 500 == 0:
#                    print len(self.sampled_points)
#
#
#    def _caculate_features_at_sampled_points(self):
#        self.feature_list = []
#        feature_loc_list = []
#        feature_loc2d_list = []
#        non_empty = 0
#        empty_queries = 0
#        for i, sampled_point_bl in enumerate(self.sampled_points):
#            if i % 500 == 0:
#                print '_caculate_features_at_sampled_points:', i
#
#            sampled_point_bl = np.matrix(sampled_point_bl).T
#            feature, point2d = self.feature_vec_at(sampled_point_bl)
#            if feature != None:
#                self.feature_list.append(feature)
#                feature_loc_list.append(sampled_point_bl)
#                feature_loc2d_list.append(point2d)
#                non_empty = non_empty + 1
#            else:
#                #pdb.set_trace()
#                empty_queries = empty_queries + 1
#        print 'empty queries', empty_queries, 'non empty', non_empty
#        if len(feature_loc_list) > 0:
#            self.feature_locs = np.column_stack(feature_loc_list)
#            self.feature_locs2d = np.column_stack(feature_loc2d_list)
#        else:
#            self.feature_locs = np.matrix([])
#            self.feature_locs2d = np.matrix([])
#
#    def feature_vec_at_2d(self, loc2d, viz=False):
#        #pdb.set_trace()
#        indices = self.voi_tree_2d.query(np.array(loc2d.T), k=1)[1]
#        closest_pt2d = self.points2d_valid[:, indices[0]]
#        closest_pt3d = self.points3d_valid_bl[:, indices[0]]
#        return self.feature_vec_at(closest_pt3d, viz=viz)[0], closest_pt3d, closest_pt2d
#
#    def feature_vec_at_2d_mat(self, loc2d):
#        indices = self.voi_tree_2d.query(np.array(loc2d.T), k=1)[1]
#        closest_pt2d = self.points2d_valid[:, indices[0]]
#        closest_pt3d = self.points3d_valid_bl[:, indices[0]]
#        return self.feature_vec_at_mat(closest_pt3d), closest_pt3d, closest_pt2d
#
#    def feature_vec_at_mat(self, point3d_bl, verbose=False):
#        f = self.feature_vec_at(point3d_bl, verbose)[0]
#        if f != None:
#            return np.row_stack(f)
#
#    ##
#    #
#    # @param point3d_bl - point to calculate features for 3x1 matrix in bl frame
#    # @param verbose
#    def feature_vec_at(self, point3d_bl, verbose=False, viz=False, k=4):
#        #project into 2d & get intensity window
#        try:
#            point2d_image = self.calibration_obj.project(\
#                                tfu.transform_points(self.image_T_bl, point3d_bl))
#        except Exception, e:
#            print e
#            pdb.set_trace()
#
#        #Calc intensity features
#        flatten = not viz
#
#        # Find closest neighbors in 3D to get normal
#        invalid_location = False
#        local_intensity = []
#        #for multiplier in [1,2,4,8,16,32]:
#        #pdb.set_trace()
#        for multiplier in self.params.win_multipliers:
#            if multiplier == 1:
#                #pdb.set_trace()
#                features = i3d.local_window(point2d_image, self.image_arr, self.params.win_size, flatten=flatten)
#            else:
#                #pdb.set_trace()
#                features = i3d.local_window(point2d_image, self.image_arr, self.params.win_size*multiplier, 
#                                            resize_to=self.params.win_size, flatten=flatten)
#            if features == None:
#                invalid_location = True
#                break
#            else:
#                local_intensity.append(features)
#
#        #Calc 3d normal
#        # indices_list = self.voi_tree.query(np.array(point3d_bl.T), k=k)[1]
#        # points_in_ball_bl = np.matrix(self.voi_tree.data.T[:, indices_list])
#        # normal_bl = i3d.calc_normal(points_in_ball_bl[0:3,:])
#
#        #Get fpfh
#        indices_list = self.fpfh_tree.query(np.array(point3d_bl.T), k=k)[1]
#        hists = self.fpfh_hist[:, indices_list[0]]
#        fpfh = np.mean(hists, 1)
#        #if np.any(np.abs(fpfh) > 99999.):
#        #    pdb.set_trace()
#
#        #Get distance to expected location
#        if self.expected_loc_bl != None:
#            expected_loc_fea = np.power(np.sum(np.power(self.expected_loc_bl - point3d_bl, 2), 0), .5).T
#
#        #Get synthetic distance poins
#        distance_feas = None
#        if self.distance_feature_points != None:
#            distance_feas = np.power(np.sum(np.power(self.distance_feature_points - point3d_bl, 2), 0), .5).T
#
#        if not invalid_location:
#            if not viz:
#                local_intensity = np.row_stack(local_intensity)
#                #if np.any(np.abs(local_intensity) > 99999.):
#                #    pdb.set_trace()
#            if self.sizes == None:
#                self.sizes = {}
#                self.sizes['expected_loc'] = expected_loc_fea.shape[0]
#                if distance_feas != None:
#                    self.sizes['distance'] = distance_feas.shape[0]
#                self.sizes['fpfh'] = fpfh.shape[0]
#                self.sizes['intensity'] = local_intensity.shape[0]
#            fea_calculated = []
#
#            if expected_loc_fea != None:
#                fea_calculated.append(expected_loc_fea)
#
#            if distance_feas != None:
#                fea_calculated.append(distance_feas)
#
#            fea_calculated.append(fpfh)
#            fea_calculated.append(local_intensity)
#            #pdb.set_trace()
#
#            #return [distance_feas, fpfh, local_intensity], point2d_image
#            return fea_calculated, point2d_image
#        else:
#            if verbose:
#                print '>> local_window outside of image'
#
#            return None, None
#
#    ##
#    #
#    # @return a matrix mxn where m is the number of features and n the number of examples
#    def extract_vectorized_features(self):
#        self._random_sample_voi()
#        self._caculate_features_at_sampled_points()
#        features_by_type = zip(*self.feature_list)
#        xs = np.row_stack([np.column_stack(f) for f in features_by_type])
#        #distances = np.column_stack(distances)
#        #fpfhs = np.column_stack(fpfhs) #each column is a different sample
#        #intensities = np.column_stack(intensities)
#        #xs = np.row_stack((distances, fpfhs, intensities)) #stack features
#        return xs, self.feature_locs2d, self.feature_locs
#
#    def get_location2d(self, instance_indices):
#        #pdb.set_trace()
#        sampled_points3d_bl = self.feature_locs[:,instance_indices]
#        return self.calibration_obj.project(tfu.transform_points(self.image_T_bl, \
#                sampled_points3d_bl))



###
## test this!
## weights is a column vector!
#def sample_points(weights, M):
#    assert(weights.shape[0] == M)
#    weights = weights / np.sum(weights)
#    r = np.random.rand() * (1.0/M)
#    c = weights[0,0]
#    i = 0
#    Xt = []
#
#    for m in range(1,M+1):
#        U = r + (m - 1) * (1.0/M)
#        while U > c:
#            i = i + 1
#            c = c + weights[i,0]
#        Xt.append(i)
#    return Xt


