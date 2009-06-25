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
        zenith = False
        nadir = False

    cmd_node = sc.SegwayCommand()

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

    done = False

    x=0.
    y=0.
    a=0.

    connected = False

    dead_zone = 0.0
    scale = .35# for data collect
    def g( u ):
        if u != 0:
            sgn = u/abs(u)
        else:
            sgn = 1
        return  scale*sgn*(u)**2+sgn*dead_zone

    start = time.time()
    lastcmd = time.time()
    while not done:

        for event in pygame.event.get():
            
            if (event.type == JOYAXISMOTION):
                if event.axis == 0:
                    y = -event.value
                if event.axis == 1:
                    x = -event.value
                if event.axis == 2:
                    a = -event.value
                    
                print 'event.axis: ',event.axis
                print 'event.value: ',event.value

            elif (event.type == JOYBUTTONDOWN):
                print 'event.button = ',event.button

                if zenither_flag:
                    if(event.button == 0):
                        z.zenith(z.calib['up_slow_torque'])
                    if(event.button == 1):
                        z.nadir(z.calib['down_snail_torque'])
                    if(event.button == 2):
                        z.estop()

            else:
                print 'Unusual event: '
                print 'event.type: ',event.type


        if x == 0 and y == 0 and a ==0:
            connected = True

        # detect a joystick disconnect
        try:
            js.quit()
            js.init()
        except pygame.error:
            print "joystick error"
            done=True

        #send segway commands
        if connected:
            cmd_node.set_velocity(2*g(x),2*g(y),a*0.5)

    # stop the segway
    cmd_node.set_velocity(0.,0.,0.)



