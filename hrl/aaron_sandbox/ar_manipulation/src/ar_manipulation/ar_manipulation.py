import roslib
roslib.load_manifest("hrl_pr2_arms")
roslib.load_manifest("hrl_generic_arms")
roslib.load_manifest("hrl_lib")
roslib.load_manifest("ar_pose")

import math, time, copy
import numpy as np
import tf, rospy, actionlib
import hrl_lib 
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
		self.client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
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
		self.client = actionlib.SimpleActionClient('/torso_controller/position_joint_action', 				SingleJointPositionAction)
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


class ar_manipulation():
	def __init__(self):
		rospy.init_node("ar_manipulation")
#		rospy.Subscriber("/ar_pose_markers", ARMarkers, self.read_markers_cb)
		rospy.Subscriber("/adl_tool", String, self.marker_lookup_cb)

		self.pub_rate = rospy.Rate(10)
		self.found_tag = False
		self.torso = torso()
		self.head = head()
		self.tf_listener = tf.TransformListener()
		self.cs = pr2cs.ControllerSwitcher()
#	Load JTcontroller
		self.r_arm_cart = pr2arm.create_pr2_arm('r', pr2arm.PR2ArmJTranspose)
		self.l_arm_cart = pr2arm.create_pr2_arm('l', pr2arm.PR2ArmJTranspose)
# 	Load Joint space controller
		self.r_arm = pr2arm.create_pr2_arm('r', pr2arm.PR2ArmJointTrajectory)
		self.l_arm = pr2arm.create_pr2_arm('l', pr2arm.PR2ArmJointTrajectory)

		self.epc = eptc.EPC('linear_move')
		self.time_step = 1/20.


	def marker_lookup_cb(self,msg):
		self.tool = msg.data
		if self.tool == 'shaver' or self.tool == 'scratcher':
			rospy.logout('Finding tag for '+self.tool)
			self.marker_frame = msg.data+'_ar_marker'
		else:
			print 'no valid marker found'
			self.marker_frame = 'N/A'

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
		self.head.set_pose([0.15,0.,0.])
		self.torso.down()
		self.get_angles()
		self.t_vals = eptc.min_jerk_traj(duration/self.time_step)
		self.r_ep =np.array([-1.397, 0.375, -1.740, -2.122, -1.966, -1.680, -2.491])
		self.l_ep =np.array([1.397, 0.375, 1.740, -2.122, 1.966, -1.680, -3.926])

		duration=5.
		self.cs.carefree_switch('r', '%s_arm_controller')
		self.cs.carefree_switch('l', '%s_arm_controller')
		self.epc_move_arm(self.r_arm, self.r_angle, self.r_ep, duration)
		self.epc_move_arm(self.l_arm, self.l_angle, self.l_ep, duration)
		
	def detect_artag(self):
		try:
#			(self.ar_pos, self.ar_rot) = self.tf_listener.lookupTransform("l_gripper_tool_frame",
			(self.ar_pos, self.ar_rot) = self.tf_listener.lookupTransform("torso_lift_link",
                	self.marker_frame,rospy.Time(0))
			self.pub_rate.sleep()
			print "Position: ",pplist(self.ar_pos),\
				"Rotation: ",pplist(self.ar_rot)		
			self.found_tag = True
		except:
			rospy.logout('AARtagDetect: Transform failed for '+self.tool)
			return False


	def move_arm_to_tag(self,duration=5.):
		self.cs.carefree_switch('l', '%s_cart', 
			"$(find hrl_pr2_arms)/params/j_transpose_params_low.yaml")
		self.l_arm_cart.reset_ep()	
		ep_cur = self.l_arm_cart.get_ep()
		ep2 = copy.deepcopy(ep_cur)
		ep2[0][0]=self.ar_pos[0]
		ep2[0][1]=self.ar_pos[1]
		
		self.epc_move_arm(self.l_arm_cart, ep_cur, ep2, duration)
		time.sleep(2.)

		self.l_arm_cart.reset_ep()
		ep_cur = self.l_arm_cart.get_ep()
		ep2 = copy.deepcopy(ep_cur)
		ep2[0][0]=self.ar_pos[0]-.05
		ep2[0][1]=self.ar_pos[1]
		ep2[0][2]=self.ar_pos[2]-.03
		self.epc_move_arm(self.l_arm_cart, ep_cur, ep2, duration)
		self.found_tag = False

def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


if __name__ == "__main__":
	arm = ar_manipulation()
#	arm.get_angles()
	arm.setup_pr2_init_pose()
#	raw_input("press a key to continue")
	arm.detect_artag()
#	raw_input("press a key to continue")
	if arm.found_tag:
		arm.move_arm_to_tag()

#    while not rospy.is_shutdown():



