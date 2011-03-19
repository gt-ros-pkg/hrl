## @author Hai Nguyen/hai@gatech.edu
import roslib
roslib.load_manifest('ml_lib')

import numpy as np
import dataset as ds
import hrl_lib.util as ut

def pca_gain_threshold(s, percentage_change_threshold=.15):
    if s.__class__ != np.ndarray:
        raise ValueError('Need ndarray as input.') 
    shifted      = np.concatenate((s[1:].copy(), np.array([s[-1]])), axis=1)
    diff         = s - shifted
    percent_diff = diff / s
    positions    = np.where(percent_diff < percentage_change_threshold)
    return positions[0][0]

def pca_variance_threshold(eigen_values, percent_variance=.9):
    eigen_sum    = np.sum(eigen_values)
    #print 'pca_variance_threshold: eigen_sum', eigen_sum
    eigen_normed = np.cumsum(eigen_values) / eigen_sum
    positions    = np.where(eigen_normed >= percent_variance)
    print 'pca_variance_threshold: percent_variance', percent_variance
    #print 'eigen_normed', eigen_normed
    #import pdb
    #pdb.set_trace()
    if positions[0].shape[0] == 0:
        return eigen_normed.shape[0]-1
    else:
        return positions[0][0]

def pca(data):
    cov_data = np.cov(data)
    u, s, vh = np.linalg.svd(cov_data)
    return u,s,vh

def pca_eigenface_trick(data):
    u, s, vh = np.linalg.svd(data.T * data)
    orig_u = data * u
    orig_u = orig_u / ut.norm(orig_u)
    return orig_u, s, None

def pca_vectors(data, percent_variance):
    #import pdb
    #pdb.set_trace()
    if data.shape[1] < data.shape[0]:
        print 'pca_vectors: using pca_eigenface_trick since number of data points is less than dim'
        u, s, _ = pca_eigenface_trick(data)
        #u1, s1, _ = pca(data)
        #print 'S', s
        #print 'S1', s1
    else:
        print 'pca_vectors: using normal PCA...'
        u, s, _ = pca(data)
    #u, s, _ = pca(data)
    #import pdb
    #pdb.set_trace()

    number_of_vectors = pca_variance_threshold(s, percent_variance=percent_variance)
    return np.matrix(u[:,0:number_of_vectors+1])

def randomized_vectors(dataset, number_of_vectors):
    rvectors = np.matrix(np.random.random_sample((dataset.num_attributes(), number_of_vectors))) * 2 - 1.0
    lengths  = np.diag(1.0 / np.power(np.sum(np.power(rvectors, 2), axis=0), 0.5))
    return rvectors * lengths

##
# Dataset which performs linear dimensionality reduction
class LinearDimReduceDataset(ds.Dataset):

    def __init__(self, inputs, outputs):
        ds.Dataset.__init__(self, inputs, outputs)
        self.original_inputs = inputs

    ##
    # Project this dataset's input instances (stores original instances) 
    def reduce_input(self):
        self.original_inputs = self.inputs
        self.inputs =  self.projection_basis.T * self.inputs

    ##
    # Projection vectors are assumed to be column vectors
    def set_projection_vectors(self, vec):
        self.projection_basis = vec

    ##
    # Project data points onto the linear embedding
    # @param data_points to project
    def reduce(self, data_points):
        return self.projection_basis.T * data_points

    ##
    # Performs PCA dim reduction
    # @param percent_variance 
    def pca_reduce(self, percent_variance):
        self.set_projection_vectors(pca_vectors(self.inputs, percent_variance))

    ##
    # Performs randomized vector dim reduction
    # @param number_of_vectors number of vectors to generate/use
    def randomized_vectors_reduce(self, number_of_vectors):
        self.set_projection_vectors(randomized_vectors(self.inputs, number_of_vectors))

    ##
    # returns full dimensioned inputs
    def get_original_inputs(self):
        return self.original_inputs


