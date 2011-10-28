#!/usr/bin/python

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('rospy')
roslib.load_manifest('adl_pr2_log')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('hrl_lib')
import rospy, optparse, math, time
import numpy as np
import tf
import tf.transformations as tr
import cPickle as pkl
import hrl_lib.transforms as hrl_tr
import hrl_lib.util as ut
from adl_pr2_log.msg import WrenchPoseArrayStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import WrenchStamped


def log_parse():
	parser = optparse.OptionParser('Input the source frame name \
		and the target frame name')

	parser.add_option("-t", "--tool", action="store", type="string",\
		dest="tool_frame", default="l_gripper_tool_frame")
	(options, args) = parser.parse_args()

	return options.tool_frame


class posearray_wrench_publisher():
	def __init__(self):
		self.tool_frame = '/'+log_parse()
		self.head_frame = '/ellipse_frame'
		self.torso_frame = '/torso_lift_link'
		self.base_frame = '/base_link'
		ft_topic = '/netft_gravity_zeroing/wrench_zeroed'

		rospy.init_node('adl_poses_wrench', anonymous = True)
		self.pub = rospy.Publisher('/adl_wrench_posearray',\
				WrenchPoseArrayStamped)
		self.tflistener = tf.TransformListener()
		self.force_sub = rospy.Subscriber(ft_topic, WrenchStamped, self.listen_cb)
		self.msg = WrenchPoseArrayStamped()

		self.tool_p = [0.,0.,0.]
		self.tool_q = [0.,0.,0.,0.]
		self.head_p = [0.,0.,0.]
		self.head_q = [0.,0.,0.,0.]
		self.torso_p = [0.,0.,0.]
		self.torso_q = [0.,0.,0.,0.]


	def listen_cb(self, f_msg):
		self.tool_p, self.tool_q = self.tflistener.lookupTransform\
			(self.torso_frame, self.tool_frame, rospy.Time(0))
		self.head_p, self.head_q = self.tflistener.lookupTransform\
			(self.torso_frame, self.head_frame, rospy.Time(0))
		self.torso_p, self.head_q = self.tflistener.lookupTransform\
			(self.base_frame, self.torso_frame, rospy.Time(0))
		self.msg.header.frame_id = '/'+self.target_frame
		self.msg.header.stamp = rospy.Time.now()
		self.msg.wrench = f_msg.wrench

#	poses[0] is the tool frame
		self.msg.poses[0].translation.x = tool_p[0]
		self.msg.poses[0].translation.y = tool_p[1]
		self.msg.poses[0].translation.z = tool_p[2]
		self.msg.poses[0].rotation.x = tool_q[0]
		self.msg.poses[0].rotation.y = tool_q[1]
		self.msg.poses[0].rotation.z = tool_q[2]
		self.msg.poses[0].rotation.w = tool_q[3]
#	poses[1] is the head frame
		self.msg.poses[0].translation.x = head_p[0]
		self.msg.poses[0].translation.y = head_p[1]
		self.msg.poses[0].translation.z = head_p[2]
		self.msg.poses[0].rotation.x = head_q[0]
		self.msg.poses[0].rotation.y = head_q[1]
		self.msg.poses[0].rotation.z = head_q[2]
		self.msg.poses[0].rotation.w = head_q[3]
#	poses[2] is the tool frame
		self.msg.poses[0].translation.x = head_p[0]
		self.msg.poses[0].translation.y = head_p[1]
		self.msg.poses[0].translation.z = head_p[2]
		self.msg.poses[0].rotation.x = head_q[0]
		self.msg.poses[0].rotation.y = head_q[1]
		self.msg.poses[0].rotation.z = head_q[2]
		self.msg.poses[0].rotation.w = head_q[3]


if __name__ == '__main__':
	data = posearray_wrench_publisher()
	rospy.sleep(1)
	while not rospy.is_shutdown():
		data.pub.publish(data.msg)
		rospy.sleep(1/100.)
