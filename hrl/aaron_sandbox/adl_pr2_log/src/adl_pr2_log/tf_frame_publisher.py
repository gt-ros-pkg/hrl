#!/usr/bin/python

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('hrl_lib')
import rospy, optparse, math, time
import numpy as np
import tf
import tf.transformations as tr
import cPickle as pkl
import hrl_lib.transforms as hrl_tr
import hrl_lib.util as ut

from geometry_msgs.msg import TransformStamped

def log_parse():
	parser = optparse.OptionParser('Input the source frame name \
		and the target frame name')

	parser.add_option("-s", "--source", action="store", type="string",\
		dest="source_frame", default="l_gripper_tool_frame")
	parser.add_option("-t", "--target" , action="store", type="string",\
		dest="target_frame",default="base_link")
	(options, args) = parser.parse_args()

	return options.source_frame, options.target_frame


class tf_frame_publisher():
	def __init__(self):
		self.source_frame, self.target_frame = log_parse()
		self.pub = rospy.Publisher('/frame/'+self.source_frame,\
				TransformStamped)
		rospy.init_node('pub_tf_'+self.source_frame, anonymous = True)
		self.tflistener = tf.TransformListener()
		self.pos = np.matrix([0.,0.,0.]).T
		self.rot = np.matrix([0.,0.,0.]).T
		self.init_rot = np.matrix([0.,0.,0.]).T
		self.quat = [0.,0.,0.,0.]
		self.tf = TransformStamped()


	def listen_pub(self):
		while not rospy.is_shutdown():
			p, q = self.tflistener.lookupTransform(self.target_frame,\
					self.source_frame, rospy.Time(0))
			self.tf.header.frame_id = '/'+self.target_frame
			self.tf.header.stamp = rospy.Time.now()
			self.tf.child_frame_id = '/'+self.source_frame		
			self.tf.transform.translation.x = p[0]
			self.tf.transform.translation.y = p[1]
			self.tf.transform.translation.z = p[2]
			self.tf.transform.rotation.x = q[0]
			self.tf.transform.rotation.y = q[1]
			self.tf.transform.rotation.z = q[2]
			self.tf.transform.rotation.w = q[3]
			self.pub.publish(self.tf)
			rospy.sleep(1/100.)

if __name__ == '__main__':
	frame = tf_frame_publisher()
	rospy.sleep(1)
	try:
		frame.listen_pub()
	except rospy.ROSInterruptException: pass

