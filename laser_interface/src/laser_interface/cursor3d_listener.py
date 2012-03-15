from pkg import *
from geometry_msgs.msg import Point
import sys

class Echo:
    def echo(self, point):
        print 'x y z', point.x, point.y, point.z

rospy.Publisher(CURSOR_TOPIC, Point, Echo().echo)
rospy.init_node('cursor3d_listener')
rospy.spin()





