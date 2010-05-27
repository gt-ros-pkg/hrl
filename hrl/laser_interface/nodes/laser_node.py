#!/usr/bin/env python
# A port of Hai's UI to pygame.  Seems to handle mouse events better.

## @author Travis Deyle / tdeyle@gatech.edu
from pkg import *
from std_msgs.msg import String

import time as time
import pygame
from pygame.locals import *

#init pygame
pygame.init()

screen = pygame.display.set_mode((640,480))
pygame.display.set_caption('Laser Pointer')

background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250,250,250))

done = False

mouse_click_pub = rospy.Publisher(MOUSE_CLICK_TOPIC, String).publish
laser_mode_pub  = rospy.Publisher(LASER_MODE_TOPIC, String).publish
rospy.init_node('laser_pointer_interface')

instr = ['g - debug',
         'd - display',
         'v - verbose',
         'p - positive',
         'c - clear',
         'space - rebuild']
         

def set_text( text ):
    background.fill((250,250,250))
    # Display some text
    font = pygame.font.Font(None, 36)
    text = font.render(text, 1, (10, 10, 10))
    textpos = text.get_rect()
    textpos.centerx = background.get_rect().centerx
    background.blit(text, textpos)

    for i, t in enumerate(instr):
        text = font.render(t, 1, (10, 10, 10))
        textpos = pygame.rect.Rect( 0, i * 30 + 40, 0, 0)
        background.blit(text, textpos)
    
    
def key_to_command(key):
    ret = None
    if key == K_g:
        ret = 'debug'
    if key == K_d:
        ret = 'display'
    if key == K_v:
        ret = 'verbose'
    if key == K_SPACE:
        ret = 'rebuild'
    if key == K_p:
        ret = 'positive'
    if key == K_c:
        ret = 'clear'
    return ret


set_text(' ')
while not done:
    for event in pygame.event.get():
        if event.type == MOUSEBUTTONDOWN:
            if event.button == 1:
                mouse_click_pub('True')
                set_text('cli...')
        if event.type == MOUSEBUTTONUP:
            if event.button == 1:
                mouse_click_pub('False')
                set_text('...cked')
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                done = True
                set_text( 'Shutting down...' )
            else:
                msg = key_to_command( event.key )
                if msg == None:
                    set_text( event.unicode )
                else:
                    set_text( msg )
                    laser_mode_pub( msg )
            
    screen.blit(background, (0, 0))
    pygame.display.flip()


