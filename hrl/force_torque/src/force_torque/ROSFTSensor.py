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
import threading as tr
import numpy as np

##
# Used for continuously polling FT sensor, this is needed as any slow down in
# reading FT values will result in corrupt data
class ReaderThreads(tr.Thread):
    def __init__(self, reader):
        tr.Thread.__init__(self)
        self.lock = tr.RLock()
        self.reader     = reader
        self.should_run = True
        self.reading    = None, -1
        self.reset_reader = False
        self.start()

    def run(self): 
       tstart = time.time() 
       while self.should_run:
           #try:
           if self.reset_reader:
               self.reset_reader = False
               self.reader.reset()
           self.reading   = self.reader.read(), time.time()

           tcurrent = time.time()
           rate     = 1.0 / (tcurrent - tstart)
           tstart   = tcurrent
           #print '%.2f'% (1000.0/rate)
           if rate < 10:
               print 'ReaderThreads: WARNING rate data being read is less than 10 hz (', rate, 'hz )'
           #except Exception, e:
           #    print 'Caught exception', e
           #    import sys
           #    sys.exit(-1)
 
    def stop(self):
       self.should_run = False 
       self.join(3) 
       if (self.isAlive()): 
           raise RuntimeError("ReaderThreads: unable to stop thread")


##
# Original code created by Cressel Anderson.  ROS wrapped by Hai Nguyen.
# Broadcasts only fresh readings
class FTServer:
    def __init__(self, device, id):
        try:
            rospy.init_node('FTServer' + str(id))
            print 'FTServer: ros is up!'
        except rospy.ROSException:
            pass

        self.name = 'force_torque_' + str(id)
        print 'FTServer: publishing', self.name, 'with type FloatArray.'
        self.channel = rospy.Publisher(self.name, FloatArray, tcp_nodelay=True)
        print 'FTServer: instantiating FTSensor class'
        self.reader   = FTSensor(device)
        self.thread   = ReaderThreads(self.reader)
        self.broadcast()

    def broadcast(self):
        print 'FTServer: started!'
        time_sent = 0
        tstart = time.time()

        times = []
        while len(times) < 201:
        #while not rospy.is_shutdown():
            #time.sleep(1.0/4000.0)
            reading, time_read = self.thread.reading
            if reading != None and time_read > time_sent:
                #Record time of last message sent to ensure freshness
                time_sent = time_read 
                self.channel.publish(FloatArray(None, reading))

                tcurrent = time.time()
                diff     = tcurrent - tstart
                rate     = 1.0 / diff
                tstart   = tcurrent
                times.append(diff)
                print diff * 1000.0
                if rate < 10:
                    print 'FTServer: WARNING rate data being sent out is less than 10 hz (', rate, 'hz )'
                    self.thread.reset_reader = True

        import pylab as pl
        pl.plot(times)
        pl.show()

##
# Corresponding client class
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

if __name__ == '__main__':    
    FINGER1 = '/dev/robot/fingerFT1'
    ftserver = FTServer(FINGER1, 1)
    exit()





