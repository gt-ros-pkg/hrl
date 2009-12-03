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

import roslib; roslib.load_manifest('pan_tilt_robotis')

import time
import sys, optparse

import numpy as np, math

import hrl_lib.util as ut
import robotis.robotis_servo as rs


class PanTilt():
    
    ## Assumes that both pan and tilt servos are controlled using the
    # same usb2dynamixel adaptor.
    # @param dev_name - name of serial device of the servo controller (e.g. '/dev/robot/servo0')
    # @param pan_id -  servo id for the pan Robotis servo.
    # @param pan_id -  servo id for the tilt Robotis servo.
    # @param baudrate - for the servo controller (usb2dynamixel)
    # @param pan_speed - max pan speed (radians/sec)
    # @param tilt_speed - max tilt speed (radians/sec)
    def __init__(self, dev_name, pan_id, tilt_id, baudrate=57600,
                 pan_speed = math.radians(180),
                 tilt_speed = math.radians(180)):
        self.pan_servo = rs.robotis_servo(dev_name,pan_id,baudrate,
                                          max_speed = pan_speed)
        self.tilt_servo = rs.robotis_servo(dev_name,tilt_id,baudrate,
                                           max_speed = tilt_speed)

        self.max_pan = self.pan_servo.max_ang
        self.min_pan = self.pan_servo.min_ang

        self.max_tilt = self.tilt_servo.max_ang
        self.min_tilt = self.tilt_servo.min_ang


    ## return (pan,tilt) angles in RADIANS.
    def get_pan_tilt(self):
        pan = self.pan_servo.read_angle()
        tilt = self.tilt_servo.read_angle()
        return pan, -tilt

    ## set (pan,tilt) angles in RADIANS.
    # blocks until the pan and tilt angles are attained.
    # @param pan - pan angle (RADIANS)
    # @param tilt - tilt angle (RADIANS)
    def set_pan_tilt(self, pan, tilt, speed=math.radians(180)):
        self.pan_servo.move_angle(pan, angvel=speed, blocking=False)
        self.tilt_servo.move_angle(tilt, angvel=speed, blocking=True)
        self.pan_servo.move_angle(pan, angvel=speed, blocking=True)


    ## new pan,tilt = current pan,tilt + pan_d,tilt_d
    # blocks until the pan and tilt angles are attained.
    # @param pan - pan angle (RADIANS)
    # @param tilt - tilt angle (RADIANS)
    def set_pan_tilt_delta(self,pan_d,tilt_d):
        p,t = self.get_pan_tilt()
        self.set_pan_tilt(p+pan_d,t+tilt_d)

    def set_ptz_angles_rad(self, pan, tilt):
        print 'pan_tilt.set_ptz_angles_rad: WARNING this function has been deprecated. use set_pan_tilt'
        self.set_pan_tilt(pan, tilt)

    def set_ptz_values(self, pan, tilt, blocking=True):
        print 'pan_tilt.set_ptz_values: WARNING this function has been deprecated. use set_pan_tilt'
        self.set_pan_tilt(pan, tilt)

    def get_ptz_angles_rad(self):
        print 'pan_tilt.get_ptz_angles_rad: WARNING this function has been deprecated. use set_pan_tilt'
        return self.get_pan_tilt()

    def get_ptz_values(self):
        print 'pan_tilt.get_ptz_values: WARNING this function has been deprecated. use set_pan_tilt'
        p, t = self.get_pan_tilt()
        #return p, t
        return math.degrees(p), math.degrees(t)


if __name__ == '__main__':

    p = optparse.OptionParser()
    p.add_option('-d', action='store', type='string', dest='servo_dev_name',
                 default='/dev/robot/pan_tilt0', help='servo device string. [default= /dev/robot/pan_tilt0]')
    p.add_option('--pan_id', action='store', type='int', dest='pan_id',
                 help='id of the pan servo',default=None)
    p.add_option('--tilt_id', action='store', type='int', dest='tilt_id',
                 help='id of the tilt servo',default=None)
    p.add_option('--pan', action='store', type='float', dest='pan',
                 help='pan angle (degrees).',default=None)
    p.add_option('--tilt', action='store', type='float', dest='tilt',
                 help='tilt angle (degrees).',default=None)

    opt, args = p.parse_args()

    servo_dev_name = opt.servo_dev_name
    pan_id = opt.pan_id
    tilt_id = opt.tilt_id

    pan = opt.pan
    tilt = opt.tilt

    if pan_id == None:
        print 'Please provide a pan_id'
        print 'Exiting...'
        sys.exit()
    if tilt_id == None:
        print 'Please provide a tilt_id'
        print 'Exiting...'
        sys.exit()
    if pan == None:
        print 'Please provide a pan (angle)'
        print 'Exiting...'
        sys.exit()
    if tilt == None:
        print 'Please provide a tilt (angle)'
        print 'Exiting...'
        sys.exit()

    ptu = PanTilt(servo_dev_name,pan_id,tilt_id)
    ptu.set_pan_tilt(math.radians(pan),math.radians(tilt))

# For EL-E:
# python pan_tilt.py -d /dev/robot/servos_pan_tilt_hat  --pan_id=6 --tilt_id=18 --pan=0 --tilt=0



