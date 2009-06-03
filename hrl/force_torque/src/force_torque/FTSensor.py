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
    def __init__(self, dev):
        print 'FTSensor: opening serial port...'
        self.ftcon = serial.Serial(dev,timeout=0.1)
        print 'FTSensor: setting properties'
        self.ftcon.setBaudrate(115200)
        self.ftcon.setParity('N')
        self.ftcon.setStopbits(1)
        self.ftcon.open()

        #self.RESET_INTERVAL=1.0 #seconds until transmittion is reset to fix bug
        count = 0
        self.reset_time = None
        self.reset()
        while not self._start_query():
            print 'FTSensor: resetting'
            count = count + 1
            self.reset()
            if count == 10:
                print 'FTSensor.__init__: Exiting, too many resets.  Try manually restarting FT sensors.'

        #self.last_access = time.time()
        self.last_value = []

    def read(self):
        '''
            Try to return a fresh reading *always*
        '''
        current_value = self.last_value
        while current_value == self.last_value:
            t = self.ftcon.read(19)
            #if we get an error message restart
            if len(t) < 1 or ord(t[0]) != 0 or len(t) < 10:
                self.reset()
                while not self._start_query():
                    self.reset()
                break
            else:
                current_value = self.binary_to_ft(t)

        self.last_value = current_value
        return current_value


    def binary_to_ft( self, raw_binary ):
        counts_per_force  = 192
        counts_per_torque = 10560

        #raw_binary[0] = error value
        #TODO: this error is a checksum byte that we should watch for
        #corrupted transmission
        Fx = ord(raw_binary[1])*65536+ord(raw_binary[2])*256+ord(raw_binary[3])
        Fy = ord(raw_binary[4])*65536+ord(raw_binary[5])*256+ord(raw_binary[6])
        Fz = ord(raw_binary[7])*65536+ord(raw_binary[8])*256+ord(raw_binary[9])
        Tx = ord(raw_binary[1])*65536+ord(raw_binary[2])*256+ord(raw_binary[3])
        Ty = ord(raw_binary[4])*65536+ord(raw_binary[5])*256+ord(raw_binary[6])
        Tz = ord(raw_binary[7])*65536+ord(raw_binary[8])*256+ord(raw_binary[9])
        
        _temp_val = []
        for c,list in zip([counts_per_force, counts_per_torque], [[Fx,Fy,Fz], [Tx,Ty,Tz]]):
            for val in list:
                if val > 8388607: #Adjust for two's complement
                    val -= 16777216
                val /= float(c) #scale so the values are in N and Nm
                _temp_val.append(val)

        Fx = _temp_val[0]
        Fy = _temp_val[1]
        Fz = _temp_val[2]
        Tx = _temp_val[3]
        Ty = _temp_val[4]
        Tz = _temp_val[5]

        return [Fx,Fy,Fz,Tx,Ty,Tz]

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

if __name__ == '__main__':
    import numpy as np

    print 'FTSensor: testing FT sensor 1'
    print 'FTSensor: testing FT sensor 2'
    sensor1 = FTSensor('/dev/robot/fingerFT1')
    #sensor2 = FTSensor('/dev/robot/fingerFT2')

    while True:
        v1 = sensor1.read()
        #v2 = sensor2.read()
        print np.linalg.norm(v1)
        #print np.linalg.norm(v2)
    #def _bias_sensor(self):
    #    """Don't use, now biased in software"""
    #    self.ftcon.write('SB\r')
    #    t = self.ftcon.readlines(2) # echo
    #    print t

    #def _check_time(self,message='timeout'):
    #    if time.time() - self.last_access > 0.03:
    #        print time.time() - self.last_access 
    #        print 'long delay'
    #    self.last_access = time.time()        
        #self.FT_lock.acquire()
        
        #this storage is faster and more reliable than
        #using numpy matrix operations
        #self.data = [Fx,Fy,Fz,Tx,Ty,Tz]

        #self.FT_lock.release()  

#class FT_Middleware:
#    def __init__( self, cpf=192, cpt=10560, sensor='left' ):
#        self.FT_lock = RLock()
#        self.bias_lock = RLock()
#
#        self.cpf = cpf
#        self.cpt = cpt
#        self.bias_data = None
#        self.sensor = sensor
#
#        self.reset()
#        self.last_data = None
#        
#    def reset(self):
#        self.FT_lock.acquire()
#        self.data = None
#        self.FT_lock.release()
#        
#        self.bias_lock.acquire()
#        self.bias_data = None
#        self.bias_lock.release()

#    def set_FT( self, raw_binary ):
#        #raw_binary[0] = error value
#        #TODO: this error is a checksum byte that we should watch for
#        #corrupted transmission
#        Fx = ord(raw_binary[1])*65536+ord(raw_binary[2])*256+ord(raw_binary[3])
#        Fy = ord(raw_binary[4])*65536+ord(raw_binary[5])*256+ord(raw_binary[6])
#        Fz = ord(raw_binary[7])*65536+ord(raw_binary[8])*256+ord(raw_binary[9])
#        Tx = ord(raw_binary[1])*65536+ord(raw_binary[2])*256+ord(raw_binary[3])
#        Ty = ord(raw_binary[4])*65536+ord(raw_binary[5])*256+ord(raw_binary[6])
#        Tz = ord(raw_binary[7])*65536+ord(raw_binary[8])*256+ord(raw_binary[9])
#        
#        _temp_val = []
#        for c,list in zip([self.cpf,self.cpt],[[Fx,Fy,Fz],[Tx,Ty,Tz]]):
#            for val in list:
#                if val > 8388607: #Adjust for two's complement
#                    val -= 16777216
#                val /= float(c) #scale so the values are in N and Nm
#                _temp_val.append(val)
#
#        Fx = _temp_val[0]
#        Fy = _temp_val[1]
#        Fz = _temp_val[2]
#        Tx = _temp_val[3]
#        Ty = _temp_val[4]
#        Tz = _temp_val[5]
#
#        self.FT_lock.acquire()
#        
#        #this storage is faster and more reliable than
#        #using numpy matrix operations
#        self.data = [Fx,Fy,Fz,Tx,Ty,Tz]
#
#        self.FT_lock.release()  
        
#    def get_FT( self, avoid_duplicate=False, without_bias=False):
#        """ get_FT( n=1 ), retreives the n last right FT measurements
#        may be extremely slow for large n when biased
#        """
#        for i in range(200):
#            if (self.last_data != self.data or avoid_duplicate==False):
##                print 'data:', self.data
#                self.last_data = self.data
#                break
#        self.FT_lock.acquire()
#        vals = np.matrix(self.data)
#        self.FT_lock.release()  
#
#        #Bias the sensor values if required
#        self.bias_lock.acquire()
#        if self.bias_data != None and without_bias==False:
#            ret_val = vals - self.bias_data
#            vals = ret_val
#        self.bias_lock.release()
#
#        return vals
#
    #def bias( self ):
    #    print 'biasing'

    #    self.FT_lock.acquire()
    #    vals = np.matrix(self.data)
    #    self.FT_lock.release()  

    #    self.bias_lock.acquire()
    #    self.bias_data = vals
    #    self.bias_lock.release()
    #
    #def zero_bias( self ):
    #    print 'zeroing bias'
    #    self.bias_lock.acquire()
    #    self.bias_data = None
    #    self.bias_lock.release()




    
    
