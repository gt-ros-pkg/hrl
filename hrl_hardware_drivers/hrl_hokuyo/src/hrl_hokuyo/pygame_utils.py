import pygame
import pygame.locals
import numpy as np
import time
import copy

#------ Slider class is code copied from the internet. (http://www.pygame.org/project/668/)
class Slider(object):

    #Constructs the object
    def __init__(self, pos, value=0):
        self.pos = pos
        self.size = (275,15)

        self.bar = pygame.Surface((275, 15))
        self.bar.fill((200, 200, 200))
        self.slider = pygame.Surface((20, 15))
        self.slider.fill((230, 230, 230))
        pygame.draw.rect(self.bar, (0, 0, 0), (0, 0, 275, 15), 2)
        pygame.draw.rect(self.slider, (0, 0, 0), (-1, -1, 20, 15), 2)
        self.slider.set_at((19, 14), (0, 0, 0))
        self.brect = self.bar.get_rect(topleft = pos)
        self.srect = self.slider.get_rect(topleft = pos)
        self.srect.left = value+pos[0]
        self.clicked = False
        self.value = value
        self.font_size = 15
        self.font = pygame.font.SysFont("Times New Roman", self.font_size)
        self.text = ''

    def set_text(self, text):
        ''' set the text to be displayed below the slider
        '''
        self.text = text

    #Called once every frame
    def update(self):
        mousebutton = pygame.mouse.get_pressed()
        cursor = pygame.locals.Rect(pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1], 1, 1)
        if cursor.colliderect(self.brect):
            if mousebutton[0]:
                self.clicked = True
            else:
                self.clicked = False
        if not mousebutton[0]:
            self.clicked = False
        if self.clicked:
            self.srect.center = cursor.center
        self.srect.clamp_ip(self.brect)
        self.value = self.srect.left - self.brect.left

    #Draws the slider
    def render(self, surface):
        surface.blit(self.bar, self.brect)
        surface.blit(self.slider, self.srect)
        ren = self.font.render(self.text,1,(0,0,0))
        surface.blit(ren, (self.pos[0], self.pos[1]+self.font_size+2))


