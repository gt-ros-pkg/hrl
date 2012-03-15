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

import math
import numpy as np
import pygame
import pygame.joystick
import time

import roslib; roslib.load_manifest('tele_mobile')
import rospy

from pygame.locals import *
from tele_mobile.msg import direction


class JoystickCommand:
    def __init__(self, topic='tmobile', name='joystick'):
        self.pub = rospy.Publisher('tmobile', direction)
        try:
            rospy.init_node(name, anonymous=True)
        except rospy.ROSException, e:
            pass


    def set_platform(self, x, y,reset, zen, xvel, yvel, avel,zen_reset, lock):
        cmd = direction(x, y, reset, zen, xvel, yvel, avel,zen_reset, lock)
        self.pub.publish(cmd)
        print "publishing:", cmd
        time.sleep(0.05)
	

if __name__ == '__main__':

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

	screen = pygame.display.set_mode((320, 80))
	pygame.display.set_caption('Snozzjoy')

	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((250, 250, 250))

	s = JoystickCommand()

	max_xvel = 0.18 
	max_yvel = 0.15
	max_speed = 0.18 	# don't exceed 0.18 under any condition.
	max_avel = 0.18

	x = 0.
	y = 0.
	reset = 0.
	lock = 0.
	zen = 0.
	seg_x = 0.
	seg_y = 0.
	seg_a = 0.
	xvel = 0.
	yvel = 0.
	avel = 0.
	zen_reset = 0.
	connected = False

	while not rospy.is_shutdown():
		for event in pygame.event.get():
#			print event
			if (event.type == JOYHATMOTION):
				x = event.value[0]
				y = event.value[1]
			if (event.type == JOYBUTTONDOWN):
				if(event.button == 0):
					reset = 1
#				if(event.button == 1):
#					lock = 1
				if(event.button == 9):
					zen_reset = 1
				if(event.button == 4 or event.button == 5 ):
					zen = 1
				if(event.button == 2 or event.button == 3 ):
					zen = -1
				if(event.button == 2 or event.button == 3 ):
					zen = -1		
			if (event.type == JOYBUTTONUP):
				if(event.button == 0):
					reset = 0
#				if(event.button == 1):
#					lock = 0
				if(event.button == 9):
					zen_reset = 0
				if(event.button == 2 or event.button == 3 or event.button == 4 or event.button == 5):
					zen = 0
			if (event.type == JOYAXISMOTION):
				if event.axis == 0:
					seg_y = -event.value
				if event.axis == 1:
					seg_x = -event.value
				if event.axis == 2:
					seg_a = -event.value
				if event.axis == 3:
					lock = (1 - event.value) / 2
		if seg_x == 0 and seg_y == 0 and seg_a ==0:
			connected = True

        # detect a joystick disconnect
#        try:
#            js.quit()
#            js.init()
#        except pygame.error:
#            print "joystick error"
#            rospy.signal_shutdown()

		if connected:
			xvel = seg_x * max_xvel * lock
			yvel = seg_y * max_yvel * lock
			avel = seg_a * max_avel * lock
			s.set_platform(x, y, reset, zen, xvel, yvel, avel, zen_reset, lock) 

	s.set_platform(0, 0, 0, 0, 0, 0, 0, 0, 0)
