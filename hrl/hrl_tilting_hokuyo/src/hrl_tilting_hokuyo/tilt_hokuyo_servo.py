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

import roslib; roslib.load_manifest('hrl_tilting_hokuyo')

import time
import sys, optparse

import numpy as np, math

import hrl_lib.util as ut
import robotis.robotis_servo as rs
import hrl_hokuyo.hokuyo_scan as hs


class tilt_hokuyo():
    def __init__(self, dev_name, servo_id, hokuyo, baudrate=57600, l1=0.,l2=0.035, camera_name=None):
        ''' dev_name - name of serial device of the servo controller (e.g. '/dev/robot/servo0')
            servo_id - 2,3,4 ... (2 to 253)
            hokuyo - Hokuyo object.
            baudrate - for the servo controller.
            camera_name - name of the 
        '''
        self.servo = rs.robotis_servo(dev_name,servo_id,baudrate)

        self.hokuyo = hokuyo
        self.l1 = l1
        self.l2 = l2
        
    def scan(self, range, speed, save_scan=False,avg=1, _max_retries=0):
        ''' range - (start,end) in radians
            speed - scan speed in radians/sec
            save_scan - save a dict of pos_list,scan_list,l1,l2
            avg - average scans from the hokuyo.
            returns pos_list,scan_list. list of angles and HokuyoScans
        '''

        ramp_up_angle = math.radians(5)
        if abs(range[0])+ramp_up_angle > math.radians(95) or \
           abs(range[1])+ramp_up_angle > math.radians(95):
            print 'tilt_hokuyo_servo.scan:bad angles- ',math.degrees(range[0]),math.degrees(range[1])

        min_angle = min(range[0],range[1])
        max_angle = max(range[0],range[1])
#        if max_angle>math.radians(60.5):
#            print 'tilt_hokuyo_servo.scan: maximum angle is too high, will graze bottom plate of mount. angle:', math.degrees(max_angle)
#            sys.exit()

        self.servo.move_angle(range[0]+np.sign(range[0])*ramp_up_angle)
#        time.sleep(0.05)
#        while(self.servo.is_moving()):
#            continue

        self.servo.move_angle(range[1]+np.sign(range[1])*ramp_up_angle,speed,blocking=False)
        #self.servo.move_angle(range[1], speed)
        time.sleep(0.05)
        t1 = time.time()

        pos_list = []
        scan_list = []
        while self.servo.is_moving():
            pos = self.servo.read_angle()
            #print 'h6', pos
            if pos < min_angle or pos > max_angle:
                continue
            pos_list.append(pos)
            plane_scan = self.hokuyo.get_scan(avoid_duplicate=True,remove_graze=True,avg=avg)
            scan_list.append(plane_scan)
        t2 = time.time()

        self.servo.move_angle(0)
        if save_scan:
            date_name = ut.formatted_time()
            dict = {'pos_list': pos_list,'scan_list': scan_list,
                    'l1': self.l1, 'l2': self.l2}
            ut.save_pickle(dict,date_name+'_dict.pkl')

        runtime = t2 - t1 
        expected_number_scans = 19.0 * runtime * (1.0/avg)
        scan_threshold = expected_number_scans - expected_number_scans*.2
        if len(scan_list) < scan_threshold:
            print 'tilt_hokuyo_servo.scan: WARNING! Expected at least %d scans but got only %d scans.' % (expected_number_scans, len(scan_list))
            print 'tilt_hokuyo_servo.scan: trying again.. retries:', _max_retries
            if _max_retries > 0:
                return self.scan(range, speed, save_scan, avg, _max_retries = _max_retries-1)
            else:
                print 'tilt_hokuyo_servo.scan: returning anyway'

        print 'tilt_hokuyo_servo.scan: got %d scans over range %f with speed %f.' % (len(scan_list), (max_angle - min_angle), speed)
        return pos_list,scan_list
    
    def scan_around_pt(self,pt,speed=math.radians(5)):
        ''' pt - in thok coord frame.
            this function scans in a fixed range.
            returns pos_lit,scan_list
        '''
        ang1 = math.radians(40)
        ang2 = math.radians(0)

        tilt_angles = (ang1,ang2)
        pos_list,scan_list = self.scan(tilt_angles,speed=speed)
        return pos_list,scan_list


if __name__ == '__main__':

# urg mount - l1=0.06, l2=0.05
# utm - l1 = 0.0, l2 = 0.035

    p = optparse.OptionParser()
    p.add_option('-d', action='store', type='string', dest='servo_dev_name',
                 default='/dev/robot/servo0', help='servo device string. [default= /dev/robot/servo0]')
    p.add_option('-t', action='store', type='string', dest='hokuyo_type',default='utm',
                 help='hokuyo_type. urg or utm [default=utm]')
    p.add_option('-n', action='store', type='int', dest='hokuyo_number', default=0,
                 help='hokuyo number. 0,1,2 ... [default=0]')
    p.add_option('--save_scan',action='store_true',dest='save_scan',
                 help='save the scan [dict and cloud]')
    p.add_option('--speed', action='store', type='float', dest='scan_speed',
                 help='scan speed in deg/s.[default=5]',default=5.)
    p.add_option('--ang0', action='store', type='float', dest='ang0',
                 help='starting tilt angle for scan (degrees). [default=20]', default=20.0)
    p.add_option('--ang1', action='store', type='float', dest='ang1',
                 help='ending tilt angle for scan (degrees). default=[-20]', default=-20.0)
    p.add_option('--id', action='store', type='int', dest='servo_id', default=2,
                 help='servo id 1,2 ... [default=2]')
    p.add_option('--l2', action='store', type='float', dest='l2', help='l2 (in meters) [0.035 for ElE, -0.055 for mekabot]')
    p.add_option('--flip', action='store_true', dest='flip',help='flip the hokuyo scan')


    opt, args = p.parse_args()
    hokuyo_type = opt.hokuyo_type
    hokuyo_number = opt.hokuyo_number
    servo_dev_name = opt.servo_dev_name

    save_scan = opt.save_scan
    scan_speed = math.radians(opt.scan_speed)

    ang0 = opt.ang0
    ang1 = opt.ang1

    servo_id = opt.servo_id
    l2 = opt.l2
    if l2==None:
        print 'please specify l2. do -h for details.'
        print 'Exiting...'
        sys.exit()

    flip = opt.flip

    if hokuyo_type == 'utm':
        h = hs.Hokuyo('utm',hokuyo_number,flip=flip)
    elif hokuyo_type == 'urg':
        h = hs.Hokuyo('urg',hokuyo_number,flip=flip)
    else:
        print 'unknown hokuyo type: ', hokuyo_type

    thok = tilt_hokuyo(servo_dev_name,servo_id,h,l1=0.,l2=l2)

    tilt_angles = (math.radians(ang0),math.radians(ang1))
    pos_list,scan_list = thok.scan(tilt_angles,speed=scan_speed,save_scan=save_scan)

    sys.exit() # to kill the hokuyo thread.


