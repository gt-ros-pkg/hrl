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

import time, os, copy
import serial, numpy as np
import threading
from threading import RLock

class AMTI_force:
    def __init__(self, dev_name="/dev/robot/force_plate0", broadcast=True):
        self.dev_name   = dev_name
        self.serial_dev = open_serial(dev_name, 57600)
        self.offset     = None
        if not self._reset():
            while not self._reset():
                time.sleep(50/1000.0)
        self.last_value = self._read_once()

    def read(self):
        reading = self._read_once()
        while (reading == self.last_value).all():
            time.sleep(8/1000.0)
            reading = self._read_once()
        return reading

    def _reset(self):
        self.serial_dev.write('R')
        self.serial_dev.flushInput()
        self.serial_dev.flushOutput()
        self.serial_dev.write('Q')
        x = self.serial_dev.read(1)
        print 'AMTI_force._reset: resetting.'
        return x == 'U'

    def _read_once(self):
        #Read until succeeds
        while True:
            x = self.serial_dev.read(24)
            if len(x) > 0:
                ret, error = stream_to_values(x)
                ret[0,0] = 4.448221628254617 * ret[0,0]
                ret[1,0] = 4.448221628254617 * ret[1,0]
                ret[2,0] = 4.448221628254617 * ret[2,0]
                ret[3,0] = 4.448221628254617 * 0.0254 * ret[3,0]
                ret[4,0] = 4.448221628254617 * 0.0254 * ret[4,0]
                ret[5,0] = 4.448221628254617 * 0.0254 * ret[5,0]
                if error:
                    time.sleep(50/1000.0)
                    self._reset()
                else:
                    ft_vector = ret
                    ft_vector[0,0] = -ft_vector[0,0] * 1.8
                    ft_vector[1,0] =  ft_vector[1,0] * 1.7
                    ft_vector[2,0] = -ft_vector[2,0]
                    ft_vector[3,0] = -ft_vector[3,0]
                    ft_vector[5,0] = -ft_vector[5,0]
                    return ft_vector


COEFF_MAT = np.matrix([[ 0.0032206,-0.0000321,0.0003082,-0.0032960,-0.0000117,-0.0002678,0.0032446,0.0000940,0.0001793,-0.0031230,-0.0000715,-0.0002365],
                       [ -0.0001470,0.0032134,0.0004389,0.0000222,0.0032134,0.0003946,0.0000684,-0.0031486,0.0000523,-0.0000209,-0.0031797,-0.0001470],
                       [ -0.0006416,-0.0005812,-0.0087376,-0.0006207,-0.0005215,-0.0086779,-0.0006731,-0.0008607,-0.0087900,-0.0005766,-0.0007237,-0.0084300],
                       [ 0.0000000,0.0000000,-0.0279669,0.0000000,0.0000000,-0.0269454,0.0000000,0.0000000,0.0281477,0.0000000,0.0000000,0.0268347],
                       [ 0.0000000,0.0000000,0.0273877,0.0000000,0.0000000,-0.0269034,0.0000000,0.0000000,0.0278664,0.0000000,0.0000000,-0.0272117],
                       [ -0.0123918,0.0124854,0.0000917,0.0126818,-0.0124854,0.0000911,0.0124843,-0.0122338,0.0000923,-0.0120165,0.0123544,0.0000885]])

def open_serial(dev_name, baudrate):
    serial_dev = serial.Serial(dev_name, timeout=4.)
    serial_dev.setBaudrate(baudrate)
    serial_dev.setParity('N')
    serial_dev.setStopbits(1)
    serial_dev.open()

    serial_dev.flushOutput()
    serial_dev.flushInput()
    serial_dev.write('R')
    time.sleep(1.)
    if(serial_dev == None):
            raise RuntimeError("AMTI_force: Serial port not found!\n")
    return serial_dev

def stream_to_values(data):
    val = np.matrix(np.zeros((12,1)))
    val[0,0]  = ord(data[0])  + 256*(ord(data[1])%16)
    val[1,0]  = ord(data[2])  + 256*(ord(data[3])%16)
    val[2,0]  = ord(data[4])  + 256*(ord(data[5])%16)
    val[3,0]  = ord(data[6])  + 256*(ord(data[7])%16)
    val[4,0]  = ord(data[8])  + 256*(ord(data[9])%16)
    val[5,0]  = ord(data[10]) + 256*(ord(data[11])%16)
    val[6,0]  = ord(data[12]) + 256*(ord(data[13])%16)
    val[7,0]  = ord(data[14]) + 256*(ord(data[15])%16)
    val[8,0]  = ord(data[16]) + 256*(ord(data[17])%16)
    val[9,0]  = ord(data[18]) + 256*(ord(data[19])%16)
    val[10,0] = ord(data[20]) + 256*(ord(data[21])%16)
    val[11,0] = ord(data[22]) + 256*(ord(data[23])%16)
    error = ord(data[1])/16 != 0 or ord(data[3])/16 != 1 or ord(data[5])/16 != 2 or \
            ord(data[7])/16 != 3 or ord(data[9])/16 != 4 or ord(data[11])/16 != 5 or\
            ord(data[13])/16 != 6 or ord(data[15])/16 != 7 or ord(data[17])/16 != 8 or \
            ord(data[19])/16 != 9 or ord(data[21])/16 != 10 or ord(data[23])/16 != 11

    return COEFF_MAT * val, error

