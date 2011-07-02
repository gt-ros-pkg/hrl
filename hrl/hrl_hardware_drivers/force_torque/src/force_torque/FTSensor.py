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

import time, serial
import serial

class FTSensor:
    """
        Instantiate to poll the force/torque sensor of your choice.
        Must continually poll, running anything else in the same
        process will result in many failed reads and/or incorrect 
        values!
    """
    def __init__(self, dev, baudrate=19200):
        print 'FTSensor: opening serial port (baudrate =', baudrate, ')'
        self.ftcon = serial.Serial(dev,timeout=0.1)
        print 'FTSensor: setting properties'
        self.ftcon.setBaudrate(baudrate)
        self.ftcon.setParity('N')
        self.ftcon.setStopbits(1)
        self.ftcon.open()

        #self.RESET_INTERVAL=1.0 #seconds until transmittion is reset to fix bug
        count = 0
        self.reset_time = None
        self.reset()
        while not self._start_query():
            rospy.logwarn('FTSensor: resetting')
            count = count + 1
            self.reset()
            if count == 10:
                rospy.logfatal('FTSensor.__init__: Too many resets.  Try manually restarting FT sensors.')

        #self.last_access = time.time()
        self.last_value = []

    def read(self):
        '''
            Try to return a fresh reading *always*
        '''
        current_value = self.last_value
        while current_value == self.last_value:
            t = self.ftcon.read(19)
            #if we get an error message, restart
            if len(t) < 1 or ord(t[0]) != 0 or len(t) < 10:
                #pass
                #self.reset()
                print 'exiting due to if len(t) < 1 or ord(t[0]) != 0 or len(t) < 10'
                print 't is', t
                exit()
                while not self._start_query():
                    #self.reset()
                    print 'exiting due to not self._start_query()'
                    print 't is', t
                    exit()
                break
            else:
                current_value = t
                #current_value = binary_to_ft(t)

        self.last_value = current_value
        return current_value

    def read_ft(self):
        f = binary_to_ft(self.read())

        if abs(f[0]) > 500.0:
            exit()
        return f

    #def binary_to_ft( self, raw_binary ):
    #    counts_per_force  = 192
    #    counts_per_torque = 10560

    #    #raw_binary[0] = error value
    #    #TODO: this error is a checksum byte that we should watch for
    #    #corrupted transmission
    #    Fx = ord(raw_binary[1])*65536+ord(raw_binary[2])*256+ord(raw_binary[3])
    #    Fy = ord(raw_binary[4])*65536+ord(raw_binary[5])*256+ord(raw_binary[6])
    #    Fz = ord(raw_binary[7])*65536+ord(raw_binary[8])*256+ord(raw_binary[9])
    #    Tx = ord(raw_binary[1])*65536+ord(raw_binary[2])*256+ord(raw_binary[3])
    #    Ty = ord(raw_binary[4])*65536+ord(raw_binary[5])*256+ord(raw_binary[6])
    #    Tz = ord(raw_binary[7])*65536+ord(raw_binary[8])*256+ord(raw_binary[9])
    #    
    #    _temp_val = []
    #    for c,list in zip([counts_per_force, counts_per_torque], [[Fx,Fy,Fz], [Tx,Ty,Tz]]):
    #        for val in list:
    #            if val > 8388607: #Adjust for two's complement
    #                val -= 16777216
    #            val /= float(c) #scale so the values are in N and Nm
    #            _temp_val.append(val)

    #    Fx = _temp_val[0]
    #    Fy = _temp_val[1]
    #    Fz = _temp_val[2]
    #    Tx = _temp_val[3]
    #    Ty = _temp_val[4]
    #    Tz = _temp_val[5]

    #    return [Fx,Fy,Fz,Tx,Ty,Tz]

    def _start_query( self ):
        self.ftcon.write('QS\r')
        t = self.ftcon.read(5) # echo
        if 5 == len(t):
            st = 'QS: '
            for i in range(5):
                st += str(ord(t[i])) + '\t'
            return True
        else:
            return False


    def _stop_query(self):
        self.ftcon.write('\r')
        self.ftcon.flushInput()
        self.ftcon.flush()
        t = self.ftcon.readlines(3) # flush doesnt seem to work
        #print t

    def reset(self):
        ctime = time.time()
        if self.reset_time != None:
            if ctime - self.reset_time < 2:
                self.reset_time = ctime
                return
        self.reset_time = ctime
        print 'FTSensor.reset(): Resetting at time', time.ctime()

        self._stop_query()
        self.ftcon.write('CD B\r')
        t = self.ftcon.readlines(2) # echo
        #print t
        self.ftcon.write('SA 16\r')
        t = self.ftcon.readlines(2) # echo
        #print t
        self.ftcon.write('SF\r')
        t = self.ftcon.readlines(2) # echo
        #print t
        self.ftcon.write('CF\r')
        t = self.ftcon.readlines(2) # echo
        #print t
        
        # we might should consider using CD R or resolved gauge
        self.ftcon.write('CD R\r')
        t = self.ftcon.readlines(2) # echo
        #print t
        self.ftcon.write('CF 1\r')
        t = self.ftcon.readlines(2) # echo
        #print t
        self.ftcon.write('SF\r')
        t = self.ftcon.readlines(2) # echo
        #print t
        self._zero_bias()
        self.ftcon.write('SZ\r')
        t = self.ftcon.readlines(2) # echo
        #print t

        #self.last_reset=time.time()

    def _zero_bias(self):
        """Don't use, now biased in software"""
        self.ftcon.write('SZ\r')
        t = self.ftcon.readlines(4) # echo
        #print t


def binary_to_ft( raw_binary ):
    counts_per_force  = 192
    counts_per_torque = 10560

    #raw_binary[0] = error value
    #TODO: this error is a checksum byte that we should watch for
    #corrupted transmission
    Fx = ord(raw_binary[1])*65536+ord(raw_binary[2])*256+ord(raw_binary[3])
    Fy = ord(raw_binary[4])*65536+ord(raw_binary[5])*256+ord(raw_binary[6])
    Fz = ord(raw_binary[7])*65536+ord(raw_binary[8])*256+ord(raw_binary[9])
    Tx = ord(raw_binary[10])*65536+ord(raw_binary[11])*256+ord(raw_binary[12])
    Ty = ord(raw_binary[13])*65536+ord(raw_binary[14])*256+ord(raw_binary[15])
    Tz = ord(raw_binary[16])*65536+ord(raw_binary[17])*256+ord(raw_binary[18])
    
    _temp_val = []
    for c,list in zip([counts_per_force, counts_per_torque], [[Fx,Fy,Fz], [Tx,Ty,Tz]]):
        for val in list:
            if val > 8388607: #Adjust for two's complement
                val -= 16777216
            val /= float(c) #scale so the values are in N and Nm
            _temp_val.append(val)

    Fx = _temp_val[0]
    Fy = _temp_val[1]
    Fz = -_temp_val[2]
    Tx = _temp_val[3]
    Ty = _temp_val[4]
    Tz = _temp_val[5]

    return [Fx,Fy,Fz,Tx,Ty,Tz]


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--mode', action='store', default='run', type='string', 
                 dest='mode', help='either "test" or "run"')
    p.add_option('--name', action='store', default='ft1', type='string', 
                 dest='name', help='name of sensor')
    p.add_option('--path', action='store', default='/dev/robot/fingerFT1', type = 'string',
                 dest='path', help='path to force torque device in (linux)')
    p.add_option('--baudrate', action='store', default=19200, type = 'int', 
                 dest='baudrate', 
                 help='baudrate that force torque boxes are running at (VERY IMPORTANT!)')
    opt, args = p.parse_args()
    print 'FTSensor: Reading from', opt.path

    if opt.mode == 'run':
        import roslib; roslib.load_manifest('force_torque')
        import rospy
        import hrl_lib.rutils as ru
        from force_torque.srv import *
        import numpy as np
        import time

        node_name = 'FT_poller_' + opt.name
        service_name = 'FTRelay_' + opt.name + '_set_ft'

        rospy.init_node(node_name)
        print node_name + ': waiting for service', service_name
        rospy.wait_for_service(service_name)
        ft_server_set_ft = rospy.ServiceProxy(service_name, StringService)
        ftsensor = FTSensor(opt.path, baudrate=opt.baudrate)

        times = []
        print node_name + ': Retrieving sensor data and sending to FTRelay on service', service_name
        #for i in range(2000):
        while not rospy.is_shutdown():
            v = ftsensor.read()
            #print repr(v), v.__class__
            
            t = rospy.get_time()
            #times.append(time.time())
            try:
                ft_server_set_ft(v, t)
            except rospy.ServiceException, e:
                print "Service call failed %s" % e

        print 'rospy is not shutdown', not rospy.is_shutdown()
        #a = np.array(times)
        #diffs = a[1:] - a[:-1]
        #print 1.0/diffs.mean(), 1.0/np.std(diffs)
        #pl.plot(diffs, '.') 
        #pl.show()

    if opt.mode == 'test':
        print 'testing...'
        import numpy as np
        #import pylab as pl

        print 'Testing', opt.path
        sensor1 = FTSensor(opt.path, opt.baudrate)

        tlist = []
        for i in range(200):
            t0 = time.time()
            v1 = sensor1.read_ft()
            t1 = time.time()
            tlist.append(t1 - t0)
            print np.linalg.norm(v1)

        #pl.plot(tlist)
        #pl.show()

