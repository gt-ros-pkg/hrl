#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#  \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)


#
# standard stuff, but with a bit of visualization.
#

import numpy as np, math

# @return numpy matrix where each column is a new sample.
def sample_gaussian(mean, cov, n_samples):
    mean = np.array(mean).flatten()
    cov = np.array(cov)
    s = np.random.multivariate_normal(mean, cov, (n_samples,))
    return np.matrix(s).T

# x - numpy matrix. each column is a new datapoint
def pca(x):
    U, sig, _ = np.linalg.svd(np.matrix(np.cov(x)))
    return U, sig

# untested.
def dimen_reduc(x, n_dim):
    U, sig = pca(x)
    proj_mat = np.matrix(U[:,0:n_dim])
    mn = np.mean(x, 1)
    x_projected = proj_mat.T * (x - mn)
    return x_projected.A


if __name__ == '__main__':
    import matplotlib.pyplot as pp
    import roslib; roslib.load_manifest('hrl_lib')
    import hrl_lib.matplotlib_util as mpu

    pp.axis('equal')

    mn = np.matrix((1., 5.)).T
    cov = np.matrix([[2.,50.],[50, 100]])

    s = sample_gaussian(mn, cov, 1000)
    P = np.cov(s)
    mpu.plot_ellipse_cov(mn, P, 'k', 'w')

    pp.plot(s[0,:].A1, s[1,:].A1, '.y', ms=3)

    U, sig = pca(s)
    mn = np.mean(s, 1)
    pp.plot(mn[0,:].A1, mn[1,:].A1, 'ok', ms=5)

    p1 = mn + U[:,0] * math.sqrt(sig[0])
    p2 = mn + U[:,1] * math.sqrt(sig[1])
    pp.plot([p1[0,0], mn[0,0], p2[0,0]], [p1[1,0], mn[1,0], p2[1,0]],
            'k-', linewidth=2)

    pp.show()




