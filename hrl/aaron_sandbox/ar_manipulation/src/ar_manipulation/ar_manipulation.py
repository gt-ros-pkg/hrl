import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("tf")
roslib.load_manifest("hrl_pr2_arms")
roslib.load_manifest("hrl_generic_arms")
roslib.load_manifest("hrl_lib")
roslib.load_manifest("pr2_controllers_msgs")
roslib.load_manifest("std_msgs")
#roslib.load_manifest("actionlib")

import math, time, copy
import numpy as np
import hrl_lib, tf, rospy
import hrl_pr2_arms.pr2_controller_switcher as pr2cs
import hrl_pr2_arms.pr2_arm as pr2arm
import hrl_generic_arms.ep_trajectory_controller as eptc
#import actionlib.simple_action_client as sac
from pr2_controllers_msgs.msg import SingleJointPositionActionGoal
from ar_pose.msg import ARMarkers
from std_msgs.msg import String


class torso():
	def __init__(self):
		self.pub = rospy.Publisher('/torso_controller/position_joint_action/goal',
			SingleJointPositionActionGoal)

	def down(self):
		self.down = SingleJointPositionActionGoal()
		self.down.goal_id.id = 'torso_zero'
		self.down.goal.position = 0.
		self.down.goal.min_duration = 2.		
		self.down.goal.max_velocity = 1.
		print 'moving torso...'
		self.pub.publish(self.down)


class ar_manipulation():
	def __init__(self):
		rospy.init_node("ar_manipulation")
#		rospy.Subscriber("/ar_pose_markers", ARMarkers, self.read_markers_cb)
		rospy.Subscriber("/adl_tool", String, self.marker_lookup_cb)

		self.pub_rate = rospy.Rate(10)
		self.marker_frame = 'N/A'
		self.torso = torso()
        self.tf_listener = tf.TransformListener()
		self.cs = pr2cs.ControllerSwitcher()
#	Load JTcontroller
		self.cs.carefree_switch('r', '%s_cart', 
			"$(find hrl_pr2_arms)/params/j_transpose_params_low.yaml")
		self.r_arm_cart = pr2arm.create_pr2_arm('r', pr2arm.PR2ArmJTranspose)
		self.cs.carefree_switch('r', '%s_cart', 
			"$(find hrl_pr2_arms)/params/j_transpose_params_low.yaml")
		self.l_arm_cart = pr2arm.create_pr2_arm('r', pr2arm.PR2ArmJTranspose)

# 	Load Joint space controller
		self.cs.carefree_switch('r', '%s_arm_controller')
		self.r_arm = pr2arm.create_pr2_arm('r', pr2arm.PR2ArmJointTrajectory)
		self.cs.carefree_switch('l', '%s_arm_controller')
		self.l_arm = pr2arm.create_pr2_arm('l', pr2arm.PR2ArmJointTrajectory)

		self.epc = eptc.EPC('linear_move')
		self.time_step = 1/20.

#	def read_markers_cb(self,msg):
#		len(msg.markers)


	def marker_lookup_cb(self,msg):
		if msg == 'shaver' or msg == 'scratcher':
			self.marker_frame = msg+'_ar_marker'
		else:
			print 'no valid marker found'
			self.marker_frame = 'N/A'



	def get_angles(self):
		self.r_arm.reset_ep()
		self.l_arm.reset_ep()
		self.r_angle = self.r_arm.get_joint_angles()
		self.l_angle = self.l_arm.get_joint_angles()
#		print 'r_arm: ', self.r_angle, '\nl_arm: ', self.l_angle


	def move_arm_away(self,duration=5.):
		self.torso.down()
		self.get_angles()
		self.t_vals = eptc.min_jerk_traj(duration/self.time_step)
		self.r_ep =np.array([-1.96938757, 1.04533946, -1.87194269,
			-1.92571321, -2.72870494, -1.52796493, -3.32591939])
		self.l_ep = np.array([2.13760509, 0.98891465, 2.03678745,
			-1.87417484, 2.5775505, -1.69534375, -3.07018065])

		r_traj = self.r_arm.interpolate_ep(self.r_angle, self.r_ep, self.t_vals)
		l_traj = self.l_arm.interpolate_ep(self.l_angle, self.l_ep, self.t_vals)
		l_tc = eptc.EPTrajectoryControl(self.l_arm, l_traj)
		r_tc = eptc.EPTrajectoryControl(self.r_arm, r_traj)
		raw_input("press a key to continue")
		self.epc.epc_motion(l_tc, self.time_step)
		self.epc.epc_motion(r_tc, self.time_step)

		
#	def read_artag(self,marker_frame='shaver_ar_marker'):
	def read_artag(self):
		if self.marker_frame != 'N/A':
			(pos, rot) = self.tf_listener.lookupTransform("l_gripper_tool_frame",
                	self.marker_frame,rospy.Time(0))
			self.pub_rate.sleep()
			print pplist(pos), pplist(rot)		
		else:
			print 'no tag is read or selected'


def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


if __name__ == "__main__":
	tg = ar_manipulation()
	tg.move_arm_away()
	raw_input("press a key to continue")
	tg.read_artag()

#    while not rospy.is_shutdown():



