#!/usr/bin/python

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('rospy')
roslib.load_manifest('adl_pr2_log')
roslib.load_manifest('geometry_msgs')
import rospy, optparse, math, time
import numpy as np
import tf
from hrl_lib.msg import WrenchPoseArrayStamped
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Pose



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
#		self.head_frame = '/'+log_parse()
		self.torso_frame = '/torso_lift_link'
		self.base_frame = '/base_link'
		ft_topic = '/netft_gravity_zeroing/wrench_zeroed'

		rospy.init_node('adl_poses_wrench', anonymous = True)
		self.pub = rospy.Publisher('/adl_wrench_posearray',\
				WrenchPoseArrayStamped)
		self.tflistener = tf.TransformListener()
		self.force_sub = rospy.Subscriber(ft_topic, WrenchStamped, self.force_cb)
		self.msg = WrenchPoseArrayStamped()
		self.tool_pose = Pose()
		self.head_pose = Pose()
		self.torso_pose = Pose()


	def force_cb(self, f_msg):
		self.msg.wrench = f_msg.wrench		


	def pose_wrench_pub(self):
		while not rospy.is_shutdown():
			self.tool_p, self.tool_q = self.tflistener.lookupTransform\
				(self.base_frame, self.tool_frame, rospy.Time(0))
			self.head_p, self.head_q = self.tflistener.lookupTransform\
				(self.base_frame, self.head_frame, rospy.Time(0))
			self.torso_p, self.torso_q = self.tflistener.lookupTransform\
				(self.base_frame, self.torso_frame, rospy.Time(0))
			self.msg.header.stamp = rospy.Time.now()
			self.msg.header.frame_id = self.base_frame
			self.msg.poses = []

#		poses[0] is the tool frame
			self.tool_pose.position.x = self.tool_p[0]
			self.tool_pose.position.y = self.tool_p[1]
			self.tool_pose.position.z = self.tool_p[2]
			self.tool_pose.orientation.x = self.tool_q[0]
			self.tool_pose.orientation.y = self.tool_q[1]
			self.tool_pose.orientation.z = self.tool_q[2]
			self.tool_pose.orientation.w = self.tool_q[3]
			self.msg.poses.append(self.tool_pose)
#		poses[1] is the head frame
			self.head_pose.position.x = self.head_p[0]
			self.head_pose.position.y = self.head_p[1]
			self.head_pose.position.z = self.head_p[2]
			self.head_pose.orientation.x = self.head_q[0]
			self.head_pose.orientation.y = self.head_q[1]
			self.head_pose.orientation.z = self.head_q[2]
			self.head_pose.orientation.w = self.head_q[3]
			self.msg.poses.append(self.head_pose)
#		poses[2] is the tool frame
			self.torso_pose.position.x = self.torso_p[0]
			self.torso_pose.position.y = self.torso_p[1]
			self.torso_pose.position.z = self.torso_p[2]
			self.torso_pose.orientation.x = self.torso_q[0]
			self.torso_pose.orientation.y = self.torso_q[1]
			self.torso_pose.orientation.z = self.torso_q[2]
			self.torso_pose.orientation.w = self.torso_q[3]
			self.msg.poses.append(self.torso_pose)

			self.pub.publish(self.msg)
#			print '\nwrench:\n ', self.msg.wrench
#			print '\ntool_pose:\n ', self.msg.poses[0]
			rospy.sleep(1/100.)


if __name__ == '__main__':
	data = posearray_wrench_publisher()
	rospy.sleep(1)
	try:
		data.pose_wrench_pub()
	except rospy.ROSInterruptException: pass

