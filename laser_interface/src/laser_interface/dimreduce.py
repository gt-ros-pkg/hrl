# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## @author Hai Nguyen/hai@gatech.edu
import numpy as np
#from pylab import *

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
    positions    = np.where(eigen_normed > percent_variance)
    print 'pca_variance_threshold: percent_variance', percent_variance
    #print positions
    return positions[0][0]

def pca(data):
    cov_data = np.cov(data)
    u, s, vh = np.linalg.svd(cov_data)
    return u,s,vh

def pca_vectors(data, percent_variance):
    u, s, vh = pca(data)
    number_of_vectors = pca_variance_threshold(s, percent_variance=percent_variance)
    return np.matrix(u[:,0:number_of_vectors+1])

def randomized_vectors(dataset, number_of_vectors):
    rvectors = np.matrix(np.random.random_sample((dataset.num_attributes(), number_of_vectors))) * 2 - 1.0
    lengths  = np.diag(1.0 / np.power(np.sum(np.power(rvectors, 2), axis=0), 0.5))
    return rvectors * lengths


