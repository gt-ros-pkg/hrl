#!/usr/bin/python

import numpy as np, math
import copy
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_door_opening')
import rospy

from hrl_msgs.msg import FloatArray
from geometry_msgs.msg import Twist

def ft_cb(data):
    lock.acquire()
    ft_val[0] = data.linear.x
    ft_val[1] = data.linear.y
    ft_val[2] = data.linear.z
    ft_val[3] = data.angular.x
    ft_val[4] = data.angular.y
    ft_val[5] = data.angular.z
    lock.release()

if __name__ == '__main__':
    lock = RLock()
    ft_val = [0.] * 6
    pub = rospy.Subscriber('/r_cart/state/wrench', Twist, ft_cb)
    pub = rospy.Publisher('/force_torque_ft2_estimate', FloatArray)
    rospy.init_node('ati_ft_emulator')
    rospy.loginfo('Started the ATI FT emulator.')
    rt = rospy.Rate(100)
    while not rospy.is_shutdown():
        lock.acquire()
        send_ft_val = copy.copy(ft_val)
        lock.release()
        fa = FloatArray(rospy.Header(stamp=rospy.Time.now()), send_ft_val)
        pub.publish(fa)
        rt.sleep()


