import roslib; roslib.load_manifest('hrl_lib')
import rospy
import std_srvs.srv as srv
from hrl_msgs.msg import FloatArray

import time

x = 3.0
rospy.init_node('trial_publisher')
pub = rospy.Publisher( 'trial_pub', FloatArray)
while not rospy.is_shutdown():
    x += 0.1
    fa = FloatArray()
    fa.data = [ x, x+1.0, x+2.0 ]
    pub.publish( fa )
    time.sleep( 0.3 )
