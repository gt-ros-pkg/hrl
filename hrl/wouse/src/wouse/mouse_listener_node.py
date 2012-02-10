#!/usr/bin/env python

import io

import roslib; roslib.load_manifest('wouse')
import rospy
from geometry_msgs.msg import PointStamped

Y_OVERFLOW = 128
X_OVERFLOW = 64
Y_SIGN = 32
X_SIGN = 16
CHECK = 8
MIDDLE_BUTTON = 4
RIGHT_BUTTON = 2
LEFT_BUTTON = 1

def poll_mouse(ptst):
    """Read and interpret mouse events"""
    data = f.read(3)
    ptst.header.stamp = rospy.Time.now()
    bits = ord(data[0])
    del_x = ord(data[1])
    del_y = ord(data[2])

    if not bits & CHECK:
        rospy.logerr("[wouse_listener]: Bit Check Failed")
        return
    if bits & Y_OVERFLOW:
        rospy.logwarn("[wouse_listener]: Y Overflow")
        ptst.point.z += 1
    if bits & X_OVERFLOW:
        rospy.logwarn("[wouse_listener]: X Overflow")
        ptst.point.z += 1
    if bits & Y_SIGN:
        del_y = -(256-del_y)
    if bits & X_SIGN:
        del_x = -(256-del_x)
    ptst.point.x = del_x
    ptst.point.y = del_y
    pt_pub.publish(ptst)

  # if bits & MIDDLE_BUTTON:
  #     print "Middle Button"
  # if bits & RIGHT_BUTTON:
  #     print "RIGHT BUTTON"
  # if bits & LEFT_BUTTON:
  #     print "Left Button"

if __name__=='__main__':
    rospy.init_node('wouse_listener')
    dev_file = rospy.get_param('~wouse_dev_file', '/dev/input/mouse1')
    with io.open(dev_file,'rb',0) as f:
        ptst = PointStamped()
        pt_pub = rospy.Publisher('wouse_movement', PointStamped)
        while not rospy.is_shutdown():
            poll_mouse(ptst)
