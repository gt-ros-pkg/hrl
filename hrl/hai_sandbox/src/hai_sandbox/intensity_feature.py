import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import cv
import numpy as np
import feature_extractor_fpfh.srv as fsrv
import hrl_lib.image3d as i3d
import hrl_lib.rutils as ru
import hrl_lib.prob as pr
import hrl_lib.tf_utils as tfu


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

    def _subsample(self):
        rospy.loginfo('Subsampling using PCL')
        self.pc_sub_samp_bl = self.subsampler_service.subsample(pointcloud_bl)

    def _sample_points(self):
        rospy.loginfo('Sampling points')
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

    def extract_features(self):
        self._subsample()
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


