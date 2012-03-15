#!/usr/bin/env python

from threading import Thread, Lock
import io

class MouseEvent(object):
    """An object describing basic mouse events from PS/2 protocol"""
    def __init__(self):
        self.rel_x = 0
        self.rel_y = 0
        self.x_overflow = False
        self.y_overflow = False

Y_OVERFLOW = 128
X_OVERFLOW = 64
Y_SIGN = 32
X_SIGN = 16
CHECK = 8
MIDDLE_BUTTON = 4
RIGHT_BUTTON = 2
LEFT_BUTTON = 1

class MouseListener(Thread):
    """A thread for reading a PS/2 mouse character device file"""
    def __init__(self, event, condition, device_file="/dev/input/mouse1"):
        Thread.__init__(self, name="mouse_listener_thread")
        self.daemon = True
        self.event = event
        self.condition = condition
        self.device_file = device_file

    def run(self):
        """Open device file, read mouse events, and place data in MouseEvent"""
        with io.open(self.device_file,'rb',0) as f:
            while True:
                """Read and interpret mouse events"""
                data = f.read(3)
                bits = ord(data[0])
                if not bits & CHECK:
                   print "[wouse_listener]: Bit Check Failed"
                   return
                
                del_x = ord(data[1])
                del_y = ord(data[2])
                if bits & Y_SIGN:
                    del_y = -(256-del_y)
                if bits & X_SIGN:
                    del_x = -(256-del_x)
               
                with self.condition:
                    self.event.rel_x = del_x
                    self.event.rel_y = del_y
                    self.event.x_overflow = bits & X_OVERFLOW
                    self.event.y_overflow = bits & Y_OVERFLOW
                    self.condition.notifyAll()
                #print "Thread: (x,y) = (%s,%s)" %(del_x,del_y)
