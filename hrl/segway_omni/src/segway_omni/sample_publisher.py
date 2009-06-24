#! /usr/bin/python
import roslib; roslib.load_manifest('segway_omni')
from segway_omni.msg import PlanarBaseVel
import rospy

pub = rospy.Publisher('base', PlanarBaseVel)
rospy.init_node('joystick', anonymous=True)
r = rospy.Rate(100) # 10hz
while not rospy.is_shutdown():
    cmd = PlanarBaseVel(None, .01, 0, 0)
    pub.publish(cmd)
    r.sleep()


