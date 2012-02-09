#!/usr/bin/env python

import io

Y_OVERFLOW = 128
X_OVERFLOW = 64
Y_SIGN = 32
X_SIGN = 16
CHECK = 8
MIDDLE_BUTTON = 4
RIGHT_BUTTON = 2
LEFT_BUTTON = 1

while True:
    with io.open('/dev/input/mouse2','rb',0) as f:
        data = f.read(3)
        bits = ord(data[0])
        del_x = -ord(data[1])
        del_y = -ord(data[2])

        if not bits & CHECK:
            print "Check Failed"
            break
        if bits & Y_OVERFLOW:
            print "Y Overflow"
        if bits & X_OVERFLOW:
            print "X Overflow"
        if bits & Y_SIGN:
            del_y = -del_y
        if bits & X_SIGN:
            del_x = -del_x
        if bits & MIDDLE_BUTTON:
            print "Middle Button"
        if bits & RIGHT_BUTTON:
            print "RIGHT BUTTON"
        if bits & LEFT_BUTTON:
            print "Left Button"

        print "X: ", del_x, " Y: ", del_y
