#!/usr/bin/python

import roslib
roslib.load_manifest('pr2_controllers_msgs')
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

from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped, WrenchStamped
from std_msgs.msg import Bool
from pr2_controllers_msgs.msg import JointTrajectoryControllerState


def log_parse():
	parser = optparse.OptionParser('Input the Pose node name and the ft sensor node name')

	parser.add_option("-t", "--tracker", action="store", type="string",\
		dest="tracker_name", default="adl2")
	parser.add_option("-f", "--force" , action="store", type="string",\
		dest="ft_sensor_name",default="/netft_gravity_zeroing")

	(options, args) = parser.parse_args()

	return options.tracker_name, options.ft_sensor_name 


class PR2_tool_pose():
	def __init__(self,tool_frame,tflistener):
		self.tool_frame = tool_frame
		self.tflistener = tflistener
		self.arm_controller_state_name = '/l_arm_controller/state'
		self.pr2_jt_sub = rospy.Subscriber(self.arm_controller_state_name,\
			JointTrajectoryControllerState, self.pr2_jt_cb )

		self.time = 0.
		self.init_time = 0.
		self.counter = 0
		self.counter_prev= 0
		self.pos = np.matrix([0.,0.,0.]).T
		self.init_pos = np.matrix([0.,0.,0.]).T
		self.rot = np.matrix([0.,0.,0.]).T
		self.init_rot = np.matrix([0.,0.,0.]).T
		self.quat = [0.,0.,0.,0.]
		self.jt_name = 0.
		self.jt_pos = 0.
		self.jt_vel = 0.
		self.jt_time = 0.

		self.time_data = []
		self.pos_data = []
		self.rot_data = []
		self.quat_data = []
		self.jt_name_data = []
		self.jt_pos_data = []
		self.jt_vel_data = []
		self.jt_time_data = []

	def set_origin(self):
		print 'Set origin of position:', self.tool_frame
		print '!!!!!!!!!!!!!!!!!!!!'
		b_list = []
		r_list = []
		for i in range(5):
			self.read_tf_pose()
			b_list.append(self.pos)
			r_list.append(self.rot)
			rospy.sleep(1/10.)
		if b_list[0] != None:
			self.init_pos  = np.mean(np.column_stack(b_list),1) 
		if r_list[0] != None:
			self.init_rot  = np.mean(np.column_stack(r_list),1) 
		print 'Origin of position: ', self.init_pos
		print 'initial orientation: ', math.degrees(self.init_rot[0,0]),\
					math.degrees(self.init_rot[1,0]),\
					math.degrees(self.init_rot[2,0])


	def read_tf_pose(self):
		p, self.quat = self.tflistener.lookupTransform('/base_link',
					'/'+self.tool_frame, rospy.Time(0))
		self.time = rospy.get_time()
		self.pos = np.matrix(p).T
		self.rot = np.matrix(tr.euler_from_quaternion(self.quat)).T
#		self.tool_rot_mat = hrl_tr.quaternion_to_matrix(quat)
		self.counter = self.time

	def pr2_jt_cb(self,msg):
		self.jt_time = msg.header.stamp.to_time()
		self.jt_pos = msg.actual.positions
		self.jt_vel = msg.actual.velocities
		self.jt_name = msg.joint_names

	def log(self, log):
		self.read_tf_pose()
		self.delta_pos = self.pos - self.init_pos
		if self.counter > self.counter_prev:
			self.counter_prev= self.counter
			time_int = self.time-self.init_time

			print >> log, time_int, self.counter,\
				self.delta_pos[0,0],self.delta_pos[1,0],\
				self.delta_pos[2,0],\
				self.rot[0,0],self.rot[1,0],self.rot[2,0]

			self.rot_data.append(self.rot)
			self.quat_data.append(self.quat)
			self.pos_data.append(self.delta_pos)
			self.time_data.append(self.time)

			self.jt_name_data.append(self.jt_name)
			self.jt_pos_data.append(self.jt_pos)
			self.jt_vel_data.append(self.jt_vel)
			self.jt_time_data.append(self.jt_time)


class tool_ft():
	def __init__(self,ft_sensor_node_name):
		self.init_time = 0.
		self.counter = 0
		self.counter_prev = 0
		self.force = np.matrix([0.,0.,0.]).T
		self.force_raw = np.matrix([0.,0.,0.]).T
		self.torque = np.matrix([0.,0.,0.]).T
		self.torque_raw = np.matrix([0.,0.,0.]).T
		self.torque_bias = np.matrix([0.,0.,0.]).T

		self.time_data = []
		self.force_data = []
		self.force_raw_data = []
		self.torque_data = []
		self.torque_raw_data = []

		#capture the force on the tool tip	
		self.force_sub = rospy.Subscriber(ft_sensor_node_name+\
			'/wrench_zeroed', WrenchStamped, self.force_cb)
		#raw ft values from the NetFT
		self.force_raw_sub = rospy.Subscriber('pr2_netft/wrench_raw',\
			WrenchStamped, self.force_raw_cb)
		self.force_zero = rospy.Publisher(ft_sensor_node_name+\
			'/rezero_wrench', Bool)
		rospy.logout('Done subscribing to '+ft_sensor_node_name+\
			'/rezero_wrench topic')


	def force_cb(self, msg):
		self.time = msg.header.stamp.to_time()
		self.force = np.matrix([msg.wrench.force.x, 
					msg.wrench.force.y,
					msg.wrench.force.z]).T
		self.torque = np.matrix([msg.wrench.torque.x, 
					msg.wrench.torque.y,
					msg.wrench.torque.z]).T
		self.counter += 1


	def force_raw_cb(self, msg):
		self.force_raw = np.matrix([msg.wrench.force.x, 
					msg.wrench.force.y,
					msg.wrench.force.z]).T
		self.torque_raw = np.matrix([msg.wrench.torque.x, 
					msg.wrench.torque.y,
					msg.wrench.torque.z]).T


	def reset(self):
		self.force_zero.publish(Bool(True))
	

	def log(self, log_file):
		if self.counter > self.counter_prev:
			self.counter_prev = self.counter
			time_int = self.time-self.init_time
#			print >> log_file,time_int,\
			print >> log_file, time_int, self.counter,\
				self.force[0,0],self.force[1,0],\
				self.force[2,0],\
				self.force_raw[0,0],self.force_raw[1,0],\
				self.force_raw[2,0],\
				self.torque[0,0],self.torque[1,0],\
				self.torque[2,0],\
				self.torque_raw[0,0],self.torque_raw[1,0],\
				self.torque_raw[2,0]

			self.force_data.append(self.force)
			self.force_raw_data.append(self.force_raw)
			self.torque_data.append(self.torque)
			self.torque_raw_data.append(self.torque_raw)
			self.time_data.append(self.time)


	def static_bias(self):
		print '!!!!!!!!!!!!!!!!!!!!'
		print 'BIASING FT'
		print '!!!!!!!!!!!!!!!!!!!!'
		f_list = []
		t_list = []
		for i in range(20):
			f_list.append(self.force)
			t_list.append(self.torque)
			rospy.sleep(2/100.)
		if f_list[0] != None and t_list[0] !=None:
			self.force_bias = np.mean(np.column_stack(f_list),1)
			self.torque_bias = np.mean(np.column_stack(t_list),1)
			print self.gravity
			print '!!!!!!!!!!!!!!!!!!!!'
			print 'DONE Biasing ft'
			print '!!!!!!!!!!!!!!!!!!!!'
		else:
			print 'Biasing Failed!'



class ADL_PR2_log():
	def __init__(self):
		self.init_time = 0.
		rospy.init_node('ADLs_log', anonymous = True)
		self.tflistener = tf.TransformListener()
		tool_tracker_name, self.ft_sensor_node_name = log_parse()
		rospy.logout('ADLs_log node subscribing..')


	def tool_cmd_input(self):
		confirm = False
		while not confirm:
			valid = True
			self.sub_name=raw_input("Enter subject's name: ")
			num=raw_input("Enter the number for the choice of tool:"+\
					"\n1) scratcher\n2) shaver\n3) wipe"+\
					"\n4) spoon\n5) tooth brush\n6) comb"+\
					"\n7) brush\n0) gripper\n: ")
			if num == '1':
				self.tool_name = 'scratcher'
				self.tool_frame_name = 'scratcher'
			elif num == '2':
				self.tool_name = 'shaver'
				self.tool_frame_name = 'l_gripper_shaver45_frame'
			elif num == '3':
				self.tool_name = 'wipe_finger'
				self.tool_frame_name = 'wipe_finger'
			elif num == '4':
				self.tool_name = 'spoon'
				self.tool_frame_name = 'spoon'
			elif num == '5':
				self.tool_name = 'tooth_brush'
				self.tool_frame_name = 'tooth_brush'
			elif num == '6':
				self.tool_name = 'comb'
				self.tool_frame_name = 'comb'
			elif num == '7':
				self.tool_name = 'brush'
				self.tool_frame_name = 'brush'
			elif num == '0':
				self.tool_name = 'gripper'
				self.tool_frame_name = 'l_gripper_tool_frame'
			else:
				print '\n!!!!!Invalid choice of tool!!!!!\n'
				valid = False

			if valid:
				num=raw_input("Select body:\n1)PR2\n2) caregiver\n: ")
				if num == '1':
					self.body = 'PR2'
				elif num == '2':
					self.body = 'caregiver'
				else:
					print '\n!!!!!Invalid choice of body!!!!!\n'
					valid = False
			if valid:
				self.trial_name=raw_input("Enter trial's name (e.g. arm1, arm2): ")
				self.file_name = self.sub_name+'_'+self.tool_name+'_'+self.body+'_'+self.trial_name			
				ans=raw_input("Enter y to confirm that log file is:  "+\
					self.file_name+"\n: ")
				if ans == 'y':
					confirm = True


	def init_log_file(self):	
		self.tool_cmd_input()
		self.tooltip = PR2_tool_pose(self.tool_frame_name,self.tflistener)
		self.ft = tool_ft(self.ft_sensor_node_name)
		self.ft_log_file = open(self.file_name+'_ft.log','w')
		self.tooltip_log_file = open(self.file_name+'_tool.log','w')
		self.gen_log_file = open(self.file_name+'_gen.log','w')
		self.pkl = open(self.file_name+'.pkl','w')
		head_p, head_q = self.tflistener.lookupTransform('/base_link',
					'/ellipse_frame', rospy.Time(0))
		torso_p, torso_q = self.tflistener.lookupTransform('/base_link',
					'/torso_lift_link', rospy.Time(0))

		raw_input('press Enter to set origin')
		self.tooltip.set_origin()
#		self.ft.reset()		#rezero the ft sensor
			
		raw_input('press Enter to begin the test')
		self.init_time = rospy.get_time()
		self.tooltip.init_time = self.init_time
		self.ft.init_time = self.init_time

		print >> self.gen_log_file,'Begin_time',self.init_time,\
			'\ntooltip_init_pos\n X Y Z\n',\
				self.tooltip.init_pos[0,0],\
				self.tooltip.init_pos[1,0],\
				self.tooltip.init_pos[2,0],\
			'\ntooltip_init_rot\n Rotx Roty Rotz\n',\
				self.tooltip.init_rot[0,0],\
				self.tooltip.init_rot[1,0],\
				self.tooltip.init_rot[2,0],\
			'\nFx Fy Fz Fx_raw Fy_raw Fz_raw\n',\
				self.ft.force[0,0],\
				self.ft.force[1,0],\
				self.ft.force[2,0],\
				self.ft.force_raw [0,0],\
				self.ft.force_raw [1,0],\
				self.ft.force_raw [2,0],\
			'\nTx Ty Tz Tx_raw Ty_raw Tz_raw\n',\
				self.ft.torque[0,0],\
				self.ft.torque[1,0],\
				self.ft.torque[2,0],\
				self.ft.torque_raw [0,0],\
				self.ft.torque_raw [1,0],\
				self.ft.torque_raw [2,0],\
			'\nhead_pos\n X Y Z\n',\
				head_p[0],head_p[1],head_p[2],\
			'\nhead_quat\n Qx Qy Qz Qw\n',\
				head_q[0],head_q[1],head_q[2],head_q[3],\
			'\ntorso_pos\n X Y Z\n',\
				torso_p[0],torso_p[1],torso_p[2],\
			'\ntorso_quat\n Qx Qy Qz Qw\n',\
				torso_q[0],torso_q[1],torso_q[2],torso_q[3],\


	def log_state(self):
		self.tooltip.log(self.tooltip_log_file)
		self.ft.log(self.ft_log_file)
		print '\nTool_Pos\t\tForce',\
			'\nX: ', self.tooltip.delta_pos[0,0],'\t',\
				self.ft.force[0,0],'\t',\
			'\nY: ', self.tooltip.delta_pos[1,0],'\t',\
				self.ft.force[1,0],'\t',\
			'\nZ: ', self.tooltip.delta_pos[2,0],'\t',\
				self.ft.force[2,0],'\t'


	def close_log_file(self):
		dict = {}
		dict['init_time'] = self.init_time
		dict['init_pos'] = self.tooltip.init_pos
		dict['pos'] = self.tooltip.pos_data
		dict['quat'] = self.tooltip.quat_data
		dict['rot_data'] = self.tooltip.rot_data
		dict['ptime'] = self.tooltip.time_data

		dict['jt_name']=self.tooltip.jt_name_data
		dict['jt_pos']=self.tooltip.jt_pos_data
		dict['jt_vel']=self.tooltip.jt_vel_data
		dict['jt_time']=self.tooltip.jt_time_data

		dict['force'] = self.ft.force_data
		dict['force_raw'] = self.ft.force_raw_data
		dict['torque'] = self.ft.torque_data
		dict['torque_raw'] = self.ft.torque_raw_data
		dict['ftime'] = self.ft.time_data

		pkl.dump(dict, self.pkl)
		self.pkl.close()

		self.ft_log_file.close()
		self.tooltip_log_file.close()
		self.gen_log_file.close()
		print 'Closing..  log files have saved..'


if __name__ == '__main__':
	log = ADL_PR2_log()
	log.init_log_file()
	
	while not rospy.is_shutdown():
		log.log_state()
		rospy.sleep(1/1000.)

	log.close_log_file()
