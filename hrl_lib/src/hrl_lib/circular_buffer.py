#
# Copyright (c) 2009, 2011 Georgia Tech Research Corporation
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

#  \author Advait Jain and Charlie Kemp (Healthcare Robotics Lab, Georgia Tech.)


import numpy as np
import copy


class CircularBuffer():
    # element_shape = () if you want a buffer of scalars.
    def __init__(self, max_size, element_shape):
        shp = [max_size] + list(element_shape)
        self.buf = np.zeros(shp)
        self.size = max_size
        self.n_vals = 0
        self.end_idx = -1 # index with the up to date data.
        self.start_idx = 0 # index with the up to date data.

    def append(self, val):
        self.end_idx = (self.end_idx + 1) % self.size
        if self.n_vals != 0 and self.end_idx == self.start_idx:
            self.start_idx = (self.start_idx + 1) % self.size

        self.n_vals = min(self.n_vals + 1, self.size)
        self.buf[self.end_idx] = val

    # remove item from the right
    def pop(self):
        if self.n_vals == 0:
            raise IndexError('pop from empty buffer')
        v = self.buf[self.end_idx]
        self.end_idx = (self.end_idx - 1) % self.size
        self.n_vals = self.n_vals - 1
        return v

    # convert the data into a list
    def to_list(self):
        return self.get_array().tolist()
    
    # convert circular buffer to an array.
    # returns np array of len self.n_vals
    def get_array(self):
        start_idx = self.start_idx
        end_idx = self.end_idx
        if self.n_vals == 0:
            return np.array([])
        if end_idx >= start_idx:
            arr = copy.copy(self.buf[start_idx:end_idx+1])
        else:
            arr1 = self.buf[start_idx:]
            arr2 = self.buf[:end_idx+1]
            arr = np.concatenate((arr1, arr2))
        return arr

    # get the last n elements.
    def get_last(self, n):
        if n > self.n_vals:
            raise IndexError('asking for too many elements')
        end_idx = self.end_idx
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
        self.end_idx = -1
        self.start_idx = 0

    def convert_index(self, i):
        return (self.start_idx + i) % self.n_vals

    def check_index(self, i):
        if i >= self.n_vals or -i > self.n_vals:
            raise IndexError('index (i = %d) out of bounds, since n_vals = %d' % (i, self.n_vals))

    def __setitem__(self, i, val):
        self.check_index(i)
        i = self.convert_index(i)
        self.buf[i] = val

    def __getitem__(self, i):
        #print 'input argument for __getitem__ =', i
        if type(i) is type(slice(1)):
            start = i.start
            stop = i.stop
            step = i.step
            
            # clip the values of the slice to be within range
            if start is None:
                start = 0
            if stop is None:
                stop = self.n_vals
            if step is None:
                step = 1
            if start < 0:
                start = 0
            if stop > self.n_vals:
                stop = self.n_vals
            if start > stop:
                start = stop

            # convert the values to be indices to the circular buffer
            equal_indices = start==stop
            start = self.convert_index(start)
            stop = self.convert_index(stop - 1) + 1
            
            # return the requested range
            if equal_indices or (step >= self.n_vals):
                #print "self.buf[start:start] = self.buf[%d:%d]" % (start,start)
                return self.buf[start:start]
             
            if start < stop:
                #print "self.buf[start:stop:step] = self.buf[%d:%d:%d]" % (start,stop,step)
                return self.buf[start:stop:step]
            else:
                wrap_start = ((self.n_vals - start) + step) % step
                #print "np.concatenate([self.buf[start::step], self.buf[wrap_start:stop:step]]) ="
                #print "np.concatenate([self.buf[%d::%d], self.buf[%d:%d:%d]])" % (start,step,wrap_start,stop,step)
                return np.concatenate([self.buf[start::step], self.buf[wrap_start:stop:step]])
        else:
            self.check_index(i)
            i = self.convert_index(i)
            return self.buf[i]

    def __repr__(self):
        return str(self.to_list())

    def __len__(self):
        return self.n_vals



if __name__ == '__main__':
    cb1 = CircularBuffer(5, ())
    cb1.append(1)
    cb1.append(2)
    cb1.append(3)
    cb1.append(4)
    cb1.append(5)
    cb1.append(6)
    cb1.append(7)
    cb1.append(8)

    print 'cb1:', cb1
    print 'cb1.buf:', cb1.buf
    print 'cb1[0::3]', cb1[0::3]
    print 'cb1.get_array()[0::3]', cb1.get_array()[0::3]



