import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import cv
import numpy as np
import feature_extractor_fpfh.srv as fsrv
import hrl_lib.image3d as i3d
import hrl_lib.rutils as ru
import hrl_lib.prob as pr
import hrl_lib.tf_utils as tfu
import pdb


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


def intensity_pyramid_feature(point2d_image, np_image_arr, win_size, multipliers, flatten=True):
    invalid_location = False
    local_intensity = []
    for multiplier in multipliers:
        if multiplier == 1:
            features = i3d.local_window(point2d_image, np_image_arr, win_size, flatten=flatten)
        else:
            features = i3d.local_window(point2d_image, np_image_arr, win_size*multiplier, 
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

class Subsampler:
    def __init__(self):
        self.proxy = rospy.ServiceProxy('subsample', fsrv.SubsampleCalc)

    def subsample(self, points3d, frame='base_link'):
        req = fsrv.SubsampleCalcRequest()
        req.input = ru.np_to_pointcloud(points3d, frame)
        res = self.proxy(req)
        return ru.pointcloud_to_np(res.output)


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
        self.subsampler_service = Subsampler()
        self.sizes = None #Important but access should be limited to decouple code

    def get_sizes(self):
        return self.sizes

    def _subsample(self):
        rospy.loginfo('Subsampling using PCL')
        rospy.loginfo('before %s' % str(self.pointcloud_bl.shape))
        self.pc_sub_samp_bl = self.subsampler_service.subsample(self.pointcloud_bl)
        rospy.loginfo('after %s' % str(self.pc_sub_samp_bl.shape))

    def _sample_points(self):
        rospy.loginfo('Sampling points')
        #evaluate all points
        gaussian = pr.Gaussian(self.expected_loc_bl, \
                               np.matrix([[self.params.uncertainty_x**2, 0,      0], \
                                          [0, self.params.uncertainty_y**2,      0], \
                                          [0,      0, self.params.uncertainty_z**2]]))

        pdf = gaussian.pdf_mat()
        probs = np.matrix(pdf(self.pc_sub_samp_bl))

        #sample unique points
        n_samples = min(self.params.n_samples, self.pc_sub_samp_bl.shape[1])
        pt_indices = list(set(sample_points(probs.T, n_samples)))

        #only keep those that are in bound of points
        sampled_pts3d_bl = self.pc_sub_samp_bl[:, pt_indices]
        sampled_pts3d_image = tfu.transform_points(self.image_T_bl, sampled_pts3d_bl)
        sampled_pts2d = self.camera_calibration.project(sampled_pts3d_image)
        sampled_pix2d = np.matrix(np.round(sampled_pts2d))

        #throw away points that are outside of bounds
        x = sampled_pix2d[0,:]
        y = sampled_pix2d[1,:]
        good_pts = np.where((x >= 0) + (x < self.camera_calibration.w) \
                          + (y >= 0) + (y < self.camera_calibration.h))[1].A1

        sampled_pts3d_bl = sampled_pts3d_bl[:, good_pts]
        sampled_pix2d = sampled_pix2d[:, good_pts]

        rospy.loginfo('got %s good points' % str(sampled_pix2d.shape[1]))
        return sampled_pts3d_bl, sampled_pix2d

    def feature_vec_at(self, point3d_bl, point2d_image):
        fea_calculated = []

        #Get synthetic distance points
        distance_feas = None
        if self.distance_feature_points != None:
            distance_feas = np.power(np.sum(np.power(self.distance_feature_points - point3d_bl, 2), 0), .5).T
            fea_calculated.append(distance_feas)

        #Get intensity features 
        intensity = intensity_pyramid_feature(point2d_image, np.asarray(self.cvimage_mat), 
                self.params.win_size, self.params.win_multipliers, True)
        #pdb.set_trace()
        if intensity == None:
            return None
        else:
            fea_calculated.append(intensity)

        if self.sizes == None:
            self.sizes = {}
            if distance_feas != None:
                self.sizes['distance'] = distance_feas.shape[0]
            self.sizes['intensity'] = intensity.shape[0]

        return fea_calculated

    def extract_features(self):
        self._subsample()
        sampled_pts3d_bl, sampled_pix2d = self._sample_points()
        features_l = []
        pts_with_features = []

        rospy.loginfo('Extracting features')
        for i in range(sampled_pts3d_bl.shape[1]):
            features = self.feature_vec_at(sampled_pts3d_bl[:,i], sampled_pix2d[:,i])
            if features != None:
                features_l.append(features)
                pts_with_features.append(i)
            if i % 500 == 0:
                rospy.loginfo(i)

        features_by_type = zip(*features_l)
        xs = np.row_stack([np.column_stack(f) for f in features_by_type])
        rospy.loginfo('Finished feature extraction.')
        return xs, sampled_pix2d[:, pts_with_features], sampled_pts3d_bl[:, pts_with_features]


