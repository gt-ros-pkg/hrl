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


import numpy as np
import circular_buffer as cb

# Code copied and modified from: http://www.scipy.org/Cookbook/KalmanFiltering
class Kalman1D():
    # Q - process variance
    # R - measurement variance
    # P - variance in my estimate
    def __init__(self, P, Q, R):
        self.P_orig = float(P)
        self.Q_orig = float(Q)
        self.R_orig = float(R)
        self.reset()

    # rest the filter.
    def reset(self):
        self.P = self.P_orig
        self.Q = self.Q_orig
        self.R = self.R_orig
        self.xhat = None

    def predict(self, z):
        if self.xhat == None:
            self.xhat = z
        #time update
        xhatminus = self.xhat
        Pminus = self.P + self.Q
        #measurement update
        K = Pminus / (Pminus + self.R)
        self.xhat = xhatminus + K * (z-xhatminus)
        self.P = (1-K) * Pminus
        return self.xhat
        

class Mean():
    # shape - () for scalar, (m,n) for mxn array
    def __init__(self, size, shape):
        self.buf = cb.CircularBuffer(size, shape)

    def reset(self):
        self.buf.clear()

    def predict(self, z):
        self.buf.append(z)
        return np.mean(self.buf.to_list(), 0)

class Median():
    # shape - () for scalar, (m,n) for mxn array
    def __init__(self, size, shape):
        self.buf = cb.CircularBuffer(size, shape)

    def reset(self):
        self.buf.clear()

    def predict(self, z):
        self.buf.append(z)
        return np.median(self.buf.to_list(), 0)



# x - 1D np array
def filter_array(x, filt):
    x_filt = []
    for z in x:
        x_filt.append(filt.predict(z))
#    print 'final value of P:', kf.P
    return np.array(x_filt)





