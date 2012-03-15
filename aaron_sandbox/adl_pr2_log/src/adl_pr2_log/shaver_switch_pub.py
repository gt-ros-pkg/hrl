#!/usr/bin/python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('std_msgs')
import rospy, optparse, math, time
import numpy as np
import serial
from std_msgs.msg import Bool


class shaver_pub():
	def __init__(self):
		self.pub = rospy.Publisher('/ros_switch', Bool)
		rospy.init_node('shaver_pwr_pub', anonymous = True)
		rospy.logout('shaver_pwr_pub node publishing..')
		self.state = False
		self.pwr_on = 'Power is on'
		self.pwr_off = 'Power is off'


	def input_state(self):
		raw_input("\nPress Enter to Toggle")
		self.state = not self.state
		self.pub.publish(Bool(self.state))
		if self.state:
			print self.pwr_on
		else:
			print self.pwr_off

if __name__ == '__main__':
	pwr = shaver_pub()
	while not rospy.is_shutdown():
		pwr.input_state()

