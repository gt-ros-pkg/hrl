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

class CircularBuffer():
    # element_shape = () if you want a buffer of scalars.
    def __init__(self, max_size, element_shape):
        shp = [max_size] + list(element_shape)
        self.buf = np.zeros(shp)
        self.size = max_size
        self.n_vals = 0
        self.idx = -1 # index with the up to date data.

    def append(self, val):
        self.idx = (self.idx + 1) % self.size
        self.n_vals = min(self.n_vals + 1, self.size)
        self.buf[self.idx] = val

    # convert the data into a list
    def to_list(self):
        start_idx = (self.idx - self.n_vals + 1) % self.size
        end_idx = self.idx
        if self.n_vals == 0:
            return []
        if end_idx >= start_idx:
            l = self.buf[start_idx:end_idx+1].tolist()
        else:
            l = self.buf[start_idx:].tolist()
            l += self.buf[:end_idx+1].tolist()
        return l

    # get the last n elements.
    def get_last(self, n):
        if n > self.n_vals:
            raise IndexError('asking for too many elements')
        end_idx = self.idx
        start_idx = end_idx - (n-1)
        if start_idx < 0:
            a1 = self.buf[start_idx:]
            a2 = self.buf[:end_idx+1]
            a = np.concatenate((a1,a2))
        else:
            a = self.buf[start_idx:end_idx+1]
        return a

    def clear(self):
        self.n_vals = 0
        self.idx = -1

    def __getitem__(self, i):
        if i >= self.n_vals or -i > self.n_vals:
            raise IndexError('index out of bounds')

        start_idx = (self.idx + 1 - self.n_vals)
        i = (start_idx + i) % self.n_vals
        return self.buf[i]

    def __repr__(self):
        return str(self.to_list())

    def __len__(self):
        return self.n_vals

