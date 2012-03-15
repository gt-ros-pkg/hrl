#! /usr/bin/python
# Directly stolen from Tiffany.  Thanks!  ;-)

import roslib
roslib.load_manifest('hrl_pr2_lib')
import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np
import dynamic_reconfigure.client 	#to turn on/off projector

try:
	rospy.init_node('tmp_headmove')
except:
	pass

class Head():
	def __init__(self ):
	#	rospy.init_node('move_the_head', anonymous=True)
		self.client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
		self.client.wait_for_server()
		self.goal = PointHeadGoal()

	def set_pose(self, pos):
		self.goal.target.header.frame_id = 'torso_lift_link'
		self.goal.target.point.x = pos[0]
		self.goal.target.point.y = pos[1]
		self.goal.target.point.z = pos[2]
		self.goal.min_duration = rospy.Duration(1.0)

		self.client.send_goal(self.goal)
		self.client.wait_for_result()

		if self.client.get_state() == GoalStatus.SUCCEEDED:
			print "Succeeded"
		else:
			print "Failed"
	
	#Borrowed from Kelsey's hrl_pr2_lib/simple_grasp_learner.py
	def change_projector_mode(self, on):
		client = dynamic_reconfigure.client.Client("camera_synchronizer_node")
		node_config = client.get_configuration()
		node_config["projector_mode"] = 2
		if on:
			node_config["narrow_stereo_trig_mode"] = 3
		else:
			node_config["narrow_stereo_trig_mode"] = 4
		client.update_configuration(node_config)


if __name__ == '__main__':
	head = Head()
 	pos = np.matrix([.54, 0.0, 0.9]).T
	head.set_pose(pos)
	#rospy.sleep(5.0)
	#rospy.logout('Moved the head, done pausing.')
	head.change_projector_mode(False)
