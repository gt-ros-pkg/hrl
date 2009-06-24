#! /usr/bin/python
import roslib; roslib.load_manifest('segway_omni')
from segway_omni.msg import PlanarBaseVel
import rospy
import segway

mecanum = segway.Mecanum()
def callback(cmd):
    #print 'segway_node:', cmd.header.seq, cmd.header.stamp, cmd.xvel, cmd.yvel, cmd.angular_velocity
    mecanum.set_platform_velocity(cmd.xvel, cmd.yvel, cmd.angular_velocity)

rospy.init_node("segway_node", anonymous=False)
rospy.Subscriber("base", PlanarBaseVel, callback)
rospy.spin()

