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

#  Discover all Robotis Dynamixel servos attached to a single USB2Dynamixel adaptor.
#  \author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)

import serial
import robotis_servo as rs
import optparse

p = optparse.OptionParser()
p.add_option('-d', action='store', type='string', dest='device',
             help='Which linux device to search (i.e. /dev/ttyUSB0).')
opt, args = p.parse_args()

if opt.device:
    for i in xrange(255):
        try:
            s = rs.robotis_servo( opt.device, i, timeout = 0.1)
            print '\n FOUND A SERVO @ ID %d\n' % i
        except:
            pass


# import time
# s = rs.robotis_servo( opt.device, 12, timeout = 0.1)
# t0 = time.time()
# for i in xrange(1000):
#     s.read_angle()
# t1 = time.time()
# print 'Time: ', t1-t0
# print 'Time / Read: ', (t1 - t0) / 1000
