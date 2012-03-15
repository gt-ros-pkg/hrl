import roslib; roslib.load_manifest('hrl_lib')
import rospy
import std_srvs.srv as srv
from hrl_msgs.msg import FloatArray

import time

def cb( msg ):
    print 'Received msg: ', msg, '\n\tType: ', msg.__class__
    return

x = 3.0
rospy.init_node('trial_subscriber')
sub = rospy.Subscriber('trial_pub', FloatArray, cb)

while not rospy.is_shutdown():
    rospy.spin()
