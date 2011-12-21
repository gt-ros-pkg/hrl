#!/usr/bin/python
import roslib
roslib.load_manifest("hrl_pr2_arms")
roslib.load_manifest("hrl_generic_arms")
roslib.load_manifest("hrl_lib")
roslib.load_manifest("ar_pose")

import math, time, copy
import numpy as np
import tf, rospy, actionlib
import hrl_lib.transforms as hrl_tr
import hrl_pr2_arms.pr2_controller_switcher as pr2cs
import hrl_pr2_arms.pr2_arm as pr2arm
import hrl_generic_arms.ep_trajectory_controller as eptc

from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from ar_pose.msg import ARMarkers
from std_msgs.msg import String


class head():
	def __init__(self):
	#	rospy.init_node('move_the_head', anonymous=True)
		self.client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', 
			PointHeadAction)
		self.client.wait_for_server()
		self.goal = PointHeadGoal()

	def set_pose(self, pos):
		self.goal.target.header.frame_id = 'torso_lift_link'
		self.goal.target.point.x = pos[0]
		self.goal.target.point.y = pos[1]
		self.goal.target.point.z = pos[2]
		self.goal.min_duration = rospy.Duration(3.)
		rospy.logout('Sending Head Goal')
		self.client.send_goal(self.goal)
		self.client.wait_for_result()

		if self.client.get_state() == GoalStatus.SUCCEEDED:
			print "Succeeded"
		else:
			print "Failed"

class torso():
	def __init__(self):
		self.client = actionlib.SimpleActionClient('/torso_controller/position_joint_action',
			SingleJointPositionAction)
		self.client.wait_for_server()
		self.pos = SingleJointPositionGoal()

	def down(self):
		self.pos.position = 0.01
		self.pos.min_duration = rospy.Duration(2.)
		self.pos.max_velocity = 1.
		rospy.logout('Sending torso down')
		self.client.send_goal(self.pos)
		self.client.wait_for_result()

		if self.client.get_state() == GoalStatus.SUCCEEDED:
			print "Succeeded"
		else:
			print "Failed"

class gripper():
	def __init__(self):
		self.client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action',
			Pr2GripperCommandAction)
		self.client.wait_for_server()
		self.state = Pr2GripperCommandGoal()


	def Open(self):
		self.state.command.position = .08
		self.state.command.max_effort = -1.
		rospy.logout('Open the gripper')
		self.client.send_goal(self.state)
		self.client.wait_for_result()

		if self.client.get_state() == GoalStatus.SUCCEEDED:
			print "Succeeded"
		else:
			print "Failed"


	def Close(self):	
		self.state.command.position = 0.
		self.state.command.max_effort = 50.
		rospy.logout('Close the gripper')
		self.client.send_goal(self.state)
		self.client.wait_for_result()

		if self.client.get_state() == GoalStatus.SUCCEEDED:
			print "Succeeded"
		else:
			print "Failed"


class ar_manipulation():
	def __init__(self):
		rospy.init_node("ar_manipulation")
#		rospy.Subscriber("/ar_pose_markers", ARMarkers, self.read_markers_cb)
		rospy.Subscriber("/ar_object_name", String, self.marker_lookup_cb)
		rospy.Subscriber("/put_back_tool", String, self.put_back_tool_cb)

		self.pub_rate = rospy.Rate(10)
		self.torso = torso()
		self.head = head()
		self.gripper = gripper()
		self.tf_listener = tf.TransformListener()
		self.cs = pr2cs.ControllerSwitcher()
		self.pr2_init = False
		self.search_tag = False
		self.found_tag = False

#		Load JTcontroller
		self.r_arm_cart = pr2arm.create_pr2_arm('r', pr2arm.PR2ArmJTranspose)
		self.l_arm_cart = pr2arm.create_pr2_arm('l', pr2arm.PR2ArmJTranspose)
# 		Load Joint space controller
		self.r_arm = pr2arm.create_pr2_arm('r', pr2arm.PR2ArmJointTrajectory)
		self.l_arm = pr2arm.create_pr2_arm('l', pr2arm.PR2ArmJointTrajectory)

		self.epc = eptc.EPC('linear_move')
		self.time_step = 1/20.


	def marker_lookup_cb(self,msg):
		self.tool = msg.data
		if self.tool == 'shaver' or self.tool == 'scratcher':
			rospy.logout('Receive request to find tag for '+self.tool)
			self.marker_frame = '/'+msg.data+'_ar_marker'
			self.search_tag = True
		else:
			print 'no valid marker found'
#			self.marker_frame = 'N/A'


	def get_angles(self):
		self.r_arm.reset_ep()
		self.l_arm.reset_ep()
		self.r_angle = self.r_arm.get_joint_angles()
		self.l_angle = self.l_arm.get_joint_angles()
#		print 'r_arm: ', self.r_angle, '\nl_arm: ', self.l_angle


	def epc_move_arm(self, arm, ep1, ep2, duration=5.):
		self.t_vals = eptc.min_jerk_traj(duration/self.time_step)
		traj = arm.interpolate_ep(ep1, ep2, self.t_vals)
		tc = eptc.EPTrajectoryControl(arm, traj)
		self.epc.epc_motion(tc, self.time_step)


	def setup_pr2_init_pose(self):
		rospy.logout('Initializing the Robot..'+self.tool)
		self.head.set_pose([0.15,0.,0.])
		self.torso.down()
		self.get_angles()
		duration=5.
		self.t_vals = eptc.min_jerk_traj(duration/self.time_step)
		self.r_ep =np.array([-1.397, 0.375, -1.740, -2.122, -1.966, -1.680, -2.491])
		self.l_ep =np.array([1.397, 0.375, 1.740, -2.122, 1.966, -1.680, -3.926])

#		self.r_ep =np.array([-1.397, 0.375, -1.740, -2.122, -1.966, -1.680, .651])
#		self.l_ep =np.array([1.397, 0.375, 1.740, -2.122, 1.966, -1.680, -.784])


		self.cs.carefree_switch('r', '%s_arm_controller')
		self.cs.carefree_switch('l', '%s_arm_controller')
		self.epc_move_arm(self.r_arm, self.r_angle, self.r_ep, duration)
		self.epc_move_arm(self.l_arm, self.l_angle, self.l_ep, duration)
		self.pr2_init = True

		
	def detect_artag(self):
		try:
			rospy.logout("Finding the AR tag..")
			self.pub_rate.sleep()
			(self.ar_pos, rot) = self.tf_listener.lookupTransform("/torso_lift_link",
                	self.marker_frame, rospy.Time(0))
			self.pub_rate.sleep()
			(pos, gripper_rot) = self.tf_listener.lookupTransform("/torso_lift_link",
                	"/l_gripper_tool_frame", rospy.Time(0))

			self.ar_rot = hrl_tr.quaternion_to_matrix(rot)*hrl_tr.quaternion_to_matrix(gripper_rot)

			print "Found tag at Position: ",pplist(self.ar_pos),\
				"\nRotation: ",pplist(rot)
			self.ar_ep = []
			self.ar_ep.append(np.matrix(self.ar_pos).T)
			self.ar_ep.append(self.ar_rot)
#			rospy.logout("Found tag at \nPosition: ",self.ar_pos,"\nRotation: ",self.ar_rot)
			self.found_tag = True
		except:
			rospy.logout('AARtagDetect: Transform failed for '+self.tool)
			return False


	def fetch_tool(self, arm='left_arm', duration=5.):
		rospy.logout("Moving the arm to fetch the object")
		if arm == 'left_arm':
			side = 'l'
			arm_controller = self.l_arm_cart
		elif arm == 'right_arm':
			side = 'r'
			arm_controller = self.r_arm_cart	

		self.cs.carefree_switch(side, side+'_cart', 
#			"$(find hrl_pr2_arms)/params/j_transpose_params_low.yaml")
			"$(find hrl_pr2_arms)/params/j_transpose_params_high.yaml")
		arm_controller.reset_ep()
		ep_cur = arm_controller.get_ep()
		ep1 = copy.deepcopy(self.ar_ep)
		ep1[0][2]=ep_cur[0][2]+.1

		self.epc_move_arm(arm_controller, ep_cur, ep1, duration)
		self.gripper.Open()
		time.sleep(2.)
		
#		arm_controller.reset_ep()
#		ep_cur = arm_controller.get_ep()
#		self.tool_ep = copy.deepcopy(ep_cur)

		self.tool_ep = copy.deepcopy(self.ar_ep)
		self.tool_ep[0][0]=self.ar_pos[0]-.05
#		self.tool_ep[0][1]=self.ar_pos[1]
		self.tool_ep[0][2]=self.ar_pos[2]-.03
#		self.tool_ep[0][2]=self.ar_pos[2]-.05

#		self.epc_move_arm(arm_controller, ep_cur, self.tool_ep, duration)
		self.epc_move_arm(arm_controller, ep1, self.tool_ep, duration)
		self.gripper.Close()
		time.sleep(2.)
		self.epc_move_arm(arm_controller, self.tool_ep, ep1, duration)

		self.found_tag = False
		self.search_tag = False
		self.pr2_init = False


	def put_back_tool_cb(self,duration=5.):
		rospy.logout("Putting back the object")
		self.cs.carefree_switch('l', '%s_cart', 
#			"$(find hrl_pr2_arms)/params/j_transpose_params_low.yaml")
			"$(find hrl_pr2_arms)/params/j_transpose_params_high.yaml")
		self.l_arm_cart.reset_ep()
		ep_cur = self.l_arm_cart.get_ep()
		ep1 = copy.deepcopy(self.tool_ep)
		ep1[0][2] = self.tool_ep[0][2]+.15
		self.epc_move_arm(self.l_arm_cart, ep_cur, ep1, duration)
		self.epc_move_arm(self.l_arm_cart, ep1, self.tool_ep, duration)
		self.gripper.Open()
		self.epc_move_arm(self.l_arm_cart, self.tool_ep, ep1, duration)


def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


if __name__ == "__main__":
	arm = ar_manipulation()
#	arm.get_angles()
	while not rospy.is_shutdown():
		if arm.search_tag:
			if not arm.pr2_init:
				arm.setup_pr2_init_pose()
			arm.detect_artag()
			if arm.found_tag:
				arm.fetch_tool()





