#! /usr/bin/python
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
#  \author Chih-Hung King (Healthcare Robotics Lab, Georgia Tech.)
#   platform_node2.py controls the robotis on the hokuyo


import roslib; roslib.load_manifest('tele_mobile')
import math
import time
import sys

import rospy
import zenither.zenither as zenither
import robotis.robotis_servo as rs
import segway_omni.segway as segway

from tele_mobile.msg import direction


class Ctrl():
    def __init__(self,topic="tmobile"):
        self.sub = rospy.Subscriber(topic, direction,self.callback, None, 1)
        print 'start subscribing to topic', topic
        try:
            print 'initialize platform node'
            rospy.init_node("platform", anonymous=False)
        except rospy.ROSException, e:
            pass

#		Robotis servo
        self.tilt = rs.robotis_servo('/dev/robot/servo0', 5, baudrate=57600)
        self.pan = rs.robotis_servo('/dev/robot/servo0', 25, baudrate=57600)
#        self.tilt = rs.robotis_servo('/dev/robot/servo0', 26, baudrate=57600)
        self.angle1 = 0
        self.angle2 = 0
        self.reset = 0

#		Zenither
        self.z = zenither.Zenither(robot='HRL2')
        self.z_dir = 0
        self.init_height = 1.   #initial zenither height = 1m

#		Segway Omni
        self.mec = segway.Mecanum(init_ros_node=False)
        self.xvel = 0.
        self.yvel = 0.
        self.avel = 0.
        self.lock = 0.


#	Callback funtion for rospy
    def callback(self, cmd):
        max_ang = 75
        min_ang = -75
        delta_ang = 2
        self.z_dir = cmd.zen
        self.xvel = cmd.xvel
        self.yvel = cmd.yvel
        self.avel = cmd.avel
        self.reset = cmd.reset
        self.zen_reset = cmd.zen_reset
        self.lock = cmd.lock

        if cmd.y == -1.0:
            if self.angle1 < math.radians(max_ang):
                self.angle1 = self.angle1 + math.radians(delta_ang)
        elif cmd.y == 1.0:
            if self.angle1 > math.radians(min_ang):
                self.angle1 = self.angle1 - math.radians(delta_ang)
        if cmd.x == -1.0:
            if self.angle2 < math.radians(max_ang):
                self.angle2 = self.angle2 + math.radians(delta_ang)
        elif cmd.x == 1.0:
            if self.angle2 > math.radians(min_ang):
                self.angle2 = self.angle2 - math.radians(delta_ang)    
        if cmd.reset == 1:      
            self.angle1 = 0
            self.angle2 = 0


    def set_servo(self):
        if self.reset == 1:
            print 'set servos to default position'
            self.pan.move_angle(0)
            self.tilt.move_angle(0)

        if (self.angle1 != 0 or self.angle2 != 0):
            print "tilt angle =", math.degrees(self.angle1),"pan angle=", math.degrees(self.angle2)
            self.pan.move_angle(self.angle2)
            self.tilt.move_angle(self.angle1)


    def set_zenither(self):
        self.move_zenither_flag = False
        if self.zen_reset == 1:
            curr_height = self.z.get_position_meters()    #get zenither height
            if curr_height == self.init_height:
                print 'zenither height is at ',self.init_height,'m'                
            else:
                self.z.torque_move_position(self.init_height,speed='slow')
                torque = self.z.calib['zero_vel_torque']
                print 'Reset zenither position to ',self.init_height,'m'

        if self.z_dir == 1.0:         
            curr_height = self.z.get_position_meters()    #get zenither height
            if curr_height <= (self.z.calib['max_height'] - 0.1):
                self.z.nadir(self.z.calib['up_slow_torque'])
                self.move_zenither_flag = True
                print "zenither moving up to", curr_height
            else:
                print 'max height threshold reached: ', curr_height

        if self.z_dir == -1.0:
            curr_height = self.z.get_position_meters()    #get zenither height
            if curr_height >= (self.z.calib['min_height'] + 0.40):
                self.z.nadir(self.z.calib['down_snail_torque'])
                self.move_zenither_flag = True
                print "zenither moving down to", curr_height
            else:
                print 'min height threshold reached: ', curr_height

        if not self.move_zenither_flag:
            self.z.estop()
#            torque = self.z.calib['zero_vel_torque']
#            print "zenither estop"


#    def set_segway(self):
#        if self.lock == 1.:
#            print 'segway control is locked'
#        if self.lock == 0.:
#            self.mec.set_velocity(self.xvel, self.yvel, self.avel)


    def stop(self):
        print 'stop servos'
        self.tilt.disable_torque()
        time.sleep(0.1)
        self.pan.disable_torque()
        print 'estop zenither'
        self.z.estop()
        print 'stop segway'
        self.mec.set_velocity(0., 0., 0.)
        print 'stopping platform...'
        sys.exit()
  

if __name__ == '__main__':

    platform = Ctrl()

    while not rospy.is_shutdown():
        platform.set_servo()
        platform.set_zenither()
        if platform.lock == 0.:
            print 'segway is locked'
        else:
            print 'segway is ready'
        platform.mec.set_velocity(platform.xvel,platform.yvel,platform.avel)

    platform.stop()
