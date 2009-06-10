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

import roslib; roslib.update_path('force_torque')
import rospy
import hrl_lib.rutils as ru
import hrl_lib.util as ut
from hrl_lib.msg import FloatArray

from FTSensor import FTSensor

import time
#import threading as tr
import numpy as np


class FTServer:
    def __init__(self, device, id):
        try:
            rospy.init_node('FTServer' + str(id))
            print 'FTServer: ros is up!'
        except rospy.ROSException:
            pass

        self.name = 'force_torque_' + str(id) 
        self.publisher = rospy.Publisher(self.name, FloatArray, tcp_nodelay=True)
        self.reader    = FTSensor(device)
        self.broadcast()

    def broadcast(self):
        #import pylab as pl
        print 'FTServer: started!'
        #import pdb
        #tlist = []
        #for i in range(200):
        while not rospy.is_shutdown():
            #t0 = time.time()
            #reading = self.reader.read()
            #pdb.set_trace()
            #self.publisher.publish(FloatArray(None, reading))
            self.publisher.publish(FloatArray(None, [1.0]))
            #t1 = time.time()
            #tlist.append(t1 - t0)

        #pl.plot(tlist)
        #pl.show()
        #exit()


class FTClient(ru.FloatArrayListener):
    def __init__(self, id):
        ru.FloatArrayListener.__init__(self, 'FTClient', 'force_torque_' + str(id), 100.0)
        self.bias_val = np.matrix([0,0,0, 0,0,0.0]).T

    def read(self, avg=1, without_bias=False, fresh=False):
        assert(avg > 0)
        rs = []
        for i in range(avg):
            r = ru.FloatArrayListener.read(self, fresh) 
            if r == None:
                return None
            rs.append(r)
        readings = ut.list_mat_to_mat(rs, axis=1)
        if not without_bias:
            return readings.mean(1) - self.bias_val
        else:
            return readings.mean(1)

    def bias(self, fresh=True):
        r = ru.FloatArrayListener.read(self, fresh)
        if r != None:
            self.bias_val = r




