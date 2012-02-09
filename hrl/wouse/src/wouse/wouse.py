#!/usr/bin/env python

#xinput polling
import subprocess as sp
import shlex
import re

#Xlib polling
from Xlib import display

#Pygame polling
import pygame as pg
import numpy as np

#General
import roslib; roslib.load_manifest('wouse')
import rospy

from run_stop_util import RunStopUtil

class ThreshWouse(object):
    def __init__(self):
       
        self.disp_root = display.Display().screen().root

        #pygame_polling
        #pg.init()
        #screen = pg.display.set_mode()
        #pg.mouse.set_visible(1)
        #bg = pg.Surface(screen.get_size())

        filt_pos = None
        stop_cnt = 0
        stop_time = rospy.get_time()

        runstop = RunStopUtil()
        
    def poll(self):
        #return self.xinput_polling()
        #return self.pygame_polling()
        return self.Xlib_polling()

    def Xlib_polling(self):
        curr_point = self.disp_root.query_pointer()
        return (curr_point._data["root_x"], curr_point._data["root_y"])


    def xinput_polling(self):
        """An implementation which polls xinput from the commandline to get mouse position"""
        xid = rospy.get_param('~xinput_id', "11")
        proc = sp.Popen(["xinput query-state "+xid],
                        shell=True, 
                        stdout=sp.PIPE)
        (out, err) = proc.communicate()
        if not err:
            mouse_pos = []
            state_fields = shlex.split(out)
            for field in state_fields[-2:]:
                eq_ind = field.find('=')
                mouse_pos.append(float(field[(eq_ind+1):]))
            return tuple(mouse_pos)

    def pygame_polling(self):
        pg.event.get()
        return pg.mouse.get_pos()

    def threshold(self,x,y):
        if filt_pos == None:
            filt_pos = pos
        else:
            alpha = 0.8
            filt_pos = (alpha * filt_pos) + ((1.0 - alpha) * pos)
        diff = pos - filt_pos
        x_thresh = 1.5
        y_thresh = 1.5
        stop_wait = rospy.Duration(1.0) #seconds
        stop_time_diff = rospy.get_time() - stop_time
        if ((diff[0] > x_thresh) and
            (diff[1] > y_thresh) and
            (stop_time_diff > stop_wait)):
            
            runstop.run_stop()

            stop_cnt = stop_cnt + 1
            stop_time = rospy.get_time()
            print '*****************'
            print 'STOP %d!' % stop_cnt
            print 'seconds since last STOP = %f' % stop_time_diff.to_sec()
            print 'diff =', diff
            print '*****************'


if __name__=='__main__':
    rospy.init_node('wouse_node')
    wouse = ThreshWouse()
    polling_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        (x,y) = wouse.poll()
        print "X: ",x," Y: ",y
        polling_rate.sleep()
