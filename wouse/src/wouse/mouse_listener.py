#!/usr/bin/env python
import os, sys

import roslib; roslib.load_manifest('wouse')
import rospy
from geometry_msgs.msg import Vector3Stamped

Y_OVERFLOW = 128
X_OVERFLOW = 64
Y_SIGN = 32
X_SIGN = 16
CHECK = 8
MIDDLE_BUTTON = 4
RIGHT_BUTTON = 2
LEFT_BUTTON = 1

class MouseListener(object):
    """A node for reading a PS/2 mouse linux character device file"""
    def __init__(self, device_file):
        self.f = os.open(device_file, os.O_RDONLY | os.O_NONBLOCK)
        self.vec_pub = rospy.Publisher('wouse_movement', Vector3Stamped)
        rospy.loginfo("[wouse_listener]: Ready")

    def poll(self):
        """Tries to read data from the mouse device, then parse and publish."""
        try:
            data = os.read(self.f, 3)
        except OSError as (errno, strerr):
            #Ignore 'resource not available' err 
            #caused by non-blocking read on empty file.
            if errno == 11: 
                rate.sleep()
                return
            else:
                raise
        bits = ord(data[0])
        if not bits & CHECK:
           rospy.logwarn("[mouse_listener]: Bit Check Failed")
           return
        del_x = ord(data[1])
        del_y = ord(data[2])
        if bits & Y_SIGN:
            del_y = -(256-del_y)
        if bits & X_SIGN:
            del_x = -(256-del_x)

        v3st = Vector3Stamped() 
        v3st.header.stamp = rospy.Time.now()
        v3st.vector.x = del_x
        v3st.vector.y = del_y
        if (bits & X_OVERFLOW or bits & Y_OVERFLOW):
            v3st.vector.z = 1
        self.vec_pub.publish(v3st)

if __name__=='__main__':
    rospy.init_node('mouse_listener')
    ml =  MouseListener(sys.argv[1])
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        ml.poll()
        rate.sleep()
    os.close(ml.f)
