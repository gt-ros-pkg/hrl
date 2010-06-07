#!/usr/bin/python
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
#  \author Cressel Anderson (Healthcare Robotics Lab, Georgia Tech.)
#  \author Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)
import roslib; roslib.load_manifest('force_torque')
import rospy
import hrl_lib.rutils as ru
import hrl_lib.util as ut
from hrl_lib.msg import FloatArray
import numpy as np

##
# Corresponding client class
class FTClient(ru.GenericListener):
    def __init__(self, topic_name, should_log=False, size_limit=100*60*60):
        def msg_converter(msg):
            if msg.__class__ == ().__class__:  # Weird change in new version of ROS.  This allows backward compatibility (Travis 6/7/2010)
                msg = msg[0]
            m = np.matrix(msg.data, 'f').T
            msg_time = msg.header.stamp.to_time()
            if self.should_log:
                self.log.append(m)
                self.log_time.append(msg_time)
                if len(self.log) > self.size_limit:
                    self.log = self.log[-size_limit:]
                    self.log_time = self.log_time[-size_limit:]
            return m, msg_time

        self.should_log = should_log
        self.size_limit = size_limit
        self.clear_log()
        ru.GenericListener.__init__(self, 'FTClient', FloatArray, topic_name, 15.0, message_extractor = msg_converter)
        self.bias_val = np.matrix([0,0,0, 0,0,0.0]).T

    def set_logging(self, should_log):
        self.should_log = should_log
        self.clear_log()

    def get_log(self):
        return list(self.log), list(self.log_time)

    def clear_log(self):
        self.log = []
        self.log_time = []

    ##
    # Read a force torque value
    # @param avg how many force torque value to average
    # @param without_bias
    # @param fresh
    # @param with_time_stamp 
    # @return an averaged force torque value (6x1 matrix)
    def read(self, avg=1, without_bias=False, fresh=False, with_time_stamp=False):
        assert(avg > 0)
        if avg > 1 and with_time_stamp:
            raise RuntimeError('Can\'t request averaging and timestamping at the same time')

        rs = []
        for i in range(avg):
            if fresh:
                r, msg_time = ru.GenericListener.read(self, allow_duplication=False, willing_to_wait=True) 
            else:
                r, msg_time = ru.GenericListener.read(self, allow_duplication=True, willing_to_wait=False) 
            rs.append(r)
        readings = ut.list_mat_to_mat(rs, axis=1)

        if not without_bias:
            #print 'readiings.mean(1)', readings.mean(1)
            #print 'self.bias_val', self.bias_val

            ret = readings.mean(1) - self.bias_val
        else:
            ret = readings.mean(1)

        if with_time_stamp:
            return ret, msg_time
        else:
            return ret

    def bias(self):
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        print 'BIASING FT'
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
        r, msg_time = ru.GenericListener.read(self, allow_duplication=False, willing_to_wait=True) 
        if r != None:
            self.bias_val = r

if __name__ == '__main__':
    import optparse
    import time
    p = optparse.OptionParser()
    p.add_option('-t', action='store', default='force_torque_ft1', type='string', 
                 dest='topic', help='which topic to listen to (default force_torque_ft1)')
    opt, args = p.parse_args()

    client = FTClient(opt.topic)
    #client.bias()
    while not rospy.is_shutdown():
        el = client.read()
        if el != None:
            print np.linalg.norm(el.T)
        time.sleep(1/1000.0)












