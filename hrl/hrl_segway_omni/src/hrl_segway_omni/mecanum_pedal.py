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

#  \author Cressel Anderson (Healthcare Robotics Lab, Georgia Tech.)
#  \author Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)

import pygame
import pygame.joystick
import sys, optparse
import segway_command as sc
import time as time
import roslib
roslib.load_manifest('segway_omni')
import rospy

import hrl_lib.util as ut
import numpy as np

from pygame.locals import *


if __name__=='__main__':
    p = optparse.OptionParser()

    p.add_option('-z', action='store_true', dest='zenither',
                 help='control the zenither also')

    opt, args = p.parse_args()
    zenither_flag = opt.zenither

    if zenither_flag:
        import zenither.zenither as zenither
        z = zenither.Zenither(robot='HRL2')

    cmd_node = sc.SegwayCommand()

    max_xvel = 0.18 
    max_yvel = 0.15
    max_speed = 0.18 # don't exceed 0.18 under any condition.
    max_avel = 0.18

    xvel = 0.0
    yvel = 0.0
    avel = 0.0

    #init pygame
    pygame.init()

    #joystick_status
    joystick_count = pygame.joystick.get_count()

    print "Joysticks found: %d\n" % joystick_count

    try:
        js = pygame.joystick.Joystick(0)
        js.init()
    except pygame.error:
        print "joystick error"
        js = None

    js_status = js.get_init()
    print js_status

    screen = pygame.display.set_mode((320,80))
    pygame.display.set_caption('Snozzjoy')

    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((250,250,250))

    x=0.
    y=0.
    a=0.

    len = 5
    xvel_history = np.matrix(np.zeros([len,1]))

    connected = False
    while not rospy.is_shutdown():

        move_zenither_flag=False
        for event in pygame.event.get():
            
            if (event.type == JOYAXISMOTION):
                if event.axis == 0:   #left pedal
                    x = -0.5 * event.value - 0.5
                elif event.axis == 1 and x == 0:   #right pedal
                    x = 0.5 * event.value + 0.5
                if event.axis == 2:
                    a = event.value
                print 'event.axis: ',event.axis,'event.value: ',event.value

#            elif (event.type == JOYBUTTONDOWN):
#                if zenither_flag:
#                    if(event.button == 0):
#                        z.zenith(z.calib['up_slow_torque'])
#                        move_zenither_flag=True
#                    if(event.button == 1):
#                        z.nadir(z.calib['down_snail_torque'])
#                        move_zenither_flag=True


        if x == 0 and y == 0 and a ==0:
            connected = True

        # detect a joystick disconnect
        try:
            js.quit()
            js.init()
        except pygame.error:
            print "joystick error"
            rospy.signal_shutdown()

#        if zenither_flag and (move_zenither_flag == False):
#            z.estop()

        #send segway commands
        if connected:
            xvel = x*max_xvel
#            xvel_history[0:(len-1)] = xvel_history[1:len]
#            xvel_history[(len-1)] = xvel
#            xvel = np.mean(xvel_history)

            yvel = y*max_yvel
            avel = a*max_avel

            vel_vec = np.matrix([xvel,yvel]).T
            vel_mag = np.linalg.norm(vel_vec)
            speed = ut.bound(vel_mag,max_speed,0.)
            if speed >= 0.05:
                vel_vec = vel_vec*speed/vel_mag
            
            xvel,yvel = vel_vec[0,0],vel_vec[1,0]
            cmd_node.set_velocity(xvel,yvel,avel)               
	    print '*******xvel=',xvel,'; avel=',avel		
    # stop the segway
    cmd_node.set_velocity(0.,0.,0.)



