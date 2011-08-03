#!/usr/bin/python

import roslib
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('hrl_lib')
import rospy, optparse, math, time, tf
import numpy as np
import tf.transformations as tr
import cPickle as pkl
import hrl_lib.transforms as hrl_tr
import hrl_lib.util as ut

from tf.msg import tfMessage
from geometry_msgs.msg import WrenchStamped
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

def log_parse():
	parser = optparse.OptionParser('Input the Pose node name and the ft sensor node name')

	parser.add_option("-f", "--force" , action="store", type="string",\
		dest="ft_sensor_name",default="l_wrist_ft_raw")

	(options, args) = parser.parse_args()

	return options.ft_sensor_name 


class PR2_ADL_log():
	def __init__(self):
		self.init_time = 0.
		self.p_count = 0
		self.f_count = 0
		self.jt_count = 0
		
		self.force = np.matrix([0.,0.,0.]).T
		self.force_raw = np.matrix([0.,0.,0.]).T
		self.force_bias = np.matrix([0.,0.,0.]).T
		self.torque = np.matrix([0.,0.,0.]).T
		self.torque_raw = np.matrix([0.,0.,0.]).T
		self.torque_bias = np.matrix([0.,0.,0.]).T

		self.pos = np.matrix([0.,0.,0.]).T
		self.init_pos = np.matrix([0.,0.,0.]).T
		self.quat = [0.,0.,0.,0.]
		self.rot = np.matrix([0.,0.,0.]).T
		self.rot_mat = np.identity(3,float)

		self.pos_data = []
		self.rot_data = []
		self.quat_data = []
		self.p_time_data = []
		
		self.jt_name_data = []
		self.jt_pos_data = []
		self.jt_vel_data = []
		self.jt_time_data = []

		self.force_data = []
		self.force_raw_data = []
		self.torque_data = []
		self.torque_raw_data = []
		self.f_time_data = []

		self.ft_sensor_node_name = log_parse()
		self.tool_input()
		self.select_robot_arm()
		
		rospy.init_node('PR2_ft_log', anonymous = True)
		rospy.logout('ADLs_log node subscribing..')

		self.tflistener = tf.TransformListener()
		self.pr2_jt_sub = rospy.Subscriber(self.arm_controller_state_name,\
			JointTrajectoryControllerState, self.pr2_jt_cb )
		rospy.logout('Done subscribing to '+self.arm_controller_state_name+' topic')
		self.force_sub = rospy.Subscriber(self.ft_sensor_node_name,\
			WrenchStamped , self.force_cb )	#using new NetFT box

		rospy.logout('Done subscribing to '+self.ft_sensor_node_name+' topic')
		self.ft_pub = rospy.Publisher("l_wrist_ft",WrenchStamped)
		self.m = WrenchStamped()


#	Callback Function for the NetFT Subscriber node
	def force_cb(self, msg):
		self.force_time = msg.header.stamp.to_time()
#		Transform the force coordinate frame to match with the /optitrak frame		
		self.force_raw = np.matrix([-msg.wrench.force.z, 
					msg.wrench.force.y,
					-msg.wrench.force.x]).T
		self.torque_raw = np.matrix([-msg.wrench.torque.z, 
					msg.wrench.torque.y,
					-msg.wrench.torque.x]).T
		self.f_count += 1

	
	def pr2_jt_cb(self,msg):
		self.jt_time = msg.header.stamp.to_time()
		self.jt_pos = msg.actual.positions
		self.jt_vel = msg.actual.velocities
		self.jt_name = msg.joint_names
#		print self.jt_pos, '\n', self.jt_vel
		self.jt_count += 1


	def select_robot_arm(self):
		select=False
		while not select:
			arm=raw_input('Enter l for left arm, r for right arm: ')
			if arm == 'r' or arm == 'l':
				self.tool_frame_name = '/'+arm+'_'+self.tool+'_tip_adl'
				self.arm_controller_state_name = arm+'_arm_controller/state'
				select = True
			else:
				print 'How hard is it??!!! l or r'


	def static_gravity(self):
		print '!!!!!!!!!!!!!!!!!!!!'
		print 'BIASING FT'
		print '!!!!!!!!!!!!!!!!!!!!'
		f_list = []
		t_list = []
		for i in range(20):
			f_list.append(self.force_raw)
			t_list.append(self.torque_raw)
			rospy.sleep(2/100.)
		if f_list[0] != None and t_list[0] !=None:
			self.torque_bias = np.mean(np.column_stack(t_list),1)
#			self.gravity = np.mean(np.column_stack(f_list),1)
#			print self.gravity
			self.force_bias = np.mean(np.column_stack(f_list),1)
			print self.force_bias
			print '!!!!!!!!!!!!!!!!!!!!'
			print 'DONE Biasing ft'
			print '!!!!!!!!!!!!!!!!!!!!'
		else:
			print 'Biasing Failed!'


	def set_origin(self):
		print 'Set origin of position'
		print '!!!!!!!!!!!!!!!!!!!!'
		b_list = []
		for i in range(20):
			self.read_pose()
			b_list.append(self.pos)
			rospy.sleep(2/100.)
		if b_list[0] != None:
			self.init_pos  = np.mean(np.column_stack(b_list),1) 
		print 'Origin of position: ', self.init_pos


	def read_pose(self):
		p, self.quat = self.tflistener.lookupTransform('/base_link',self.tool_frame_name, rospy.Time(0))
		self.pose_time = rospy.get_time()
		self.rot_mat = hrl_tr.quaternion_to_matrix(self.quat)
		self.pos = np.matrix(p).T
		self.rot = np.matrix(tr.euler_from_quaternion(self.quat)).T
		self.p_count = self.pose_time

	
	def read_ft(self, bias=True):
		p, self.ft_quat = self.tflistener.lookupTransform('/base_link',self.ft_frame_name, rospy.Time(0))
		self.ft_rot_mat = hrl_tr.quaternion_to_matrix(self.ft_quat)
		if bias:
#			self.force = self.force_raw - self.force_bias
#			self.force = self.rot_mat*(self.force_raw - self.force_bias) - self.gravity
			self.force = self.ft_rot_mat*(self.force_raw - self.force_bias)
			self.torque = self.torque_raw - self.torque_bias
		else:
			self.force = self.force_raw
			self.torque = self.torque_raw
#		Transform the force coordinate frame to match with the /optitrak frame		
#		self.force = np.transpose(self.rot_mat) * self.force
		self.torque = np.transpose(self.rot_mat) * self.torque
		self.publish_ft()



	def publish_ft(self):
		self.m.header.frame_id = "l_wrist_roll_link"
		self.m.header.stamp = rospy.Time.now()
		self.m.wrench.force.x = self.force[0,0]
		self.m.wrench.force.y = self.force[1,0]
		self.m.wrench.force.z = self.force[2,0]
		self.m.wrench.torque.x = self.torque[0,0]
		self.m.wrench.torque.y = self.torque[1,0]
		self.m.wrench.torque.z = self.torque[2,0]
		self.ft_pub.publish(self.m)


	def tool_input(self):
		confirm = False
		while not confirm:
			valid = True
			self.sub_name=raw_input("Enter subject's name: ")
			num=raw_input("Enter the number for the choice of tool:\n1) scratcher\n2) shaver\n: ")
			if num == '1':
				self.tool = 'scratcher'
			elif num == '2':
				self.tool = 'shaver'
			else:
				print '\n!!!!!Invalid choice of tool!!!!!\n'
				valid = False

			if valid:
				self.trial_name=raw_input("Enter trial's name (e.g. arm1, arm2): ")
				self.file_name = self.sub_name+'_'+self.tool+'_'+self.trial_name			
				ans=raw_input("Enter y to confirm that log file is:  "+self.file_name+"\n: ")
				if ans == 'y':
					confirm = True


	def init_log(self): 			
		self.ft_log = open(self.file_name+'_ft.log','w')
		self.pos_log = open(self.file_name+'_pos.log','w')
		self.pr2_jt_log = open(self.file_name+'_pr2_jt.log','w')
		self.gen_log = open(self.file_name+'_gen.log','w')
		self.pkl = open(self.file_name+'.pkl','w')	

#		d=ut.load_pickle('sensor_calibrate.pkl')
#		self.force_bias = d['mean_force_raw']
#		self.force_bias[0,0] = self.force_bias[0,0]+1.34
#		self.torque_bias = d['mean_torque_raw']
		raw_input('press Enter to set origin')
		self.set_origin()
		self.static_gravity()
			
		raw_input('press Enter to begin the test')
		self.init_time = rospy.get_time()
		print >> self.gen_log,'Begin_time',self.init_time,\
			'\ninit_pos',self.init_pos[0,0],self.init_pos[1,0],self.init_pos[2,0],\
			'\nforce_bias', self.force_bias[0,0], self.force_bias[1,0], self.force_bias[2,0],\
			'\n\n *_pos.log:\nTime X Y Z Rotx Roty Rotz',\
			'\n\n *_ft.log:\nTime Fx Fy Fz Fx_raw Fy_raw Fz_raw \
				Tx Ty Tz Tx_raw Ty_raw Tz_raw',\
			'\n\n *_pr2_jt.log:',\
			'\nTime ', self.jt_name[0]+'_pos', self.jt_name[1]+'_pos',self.jt_name[2]+'_pos',\
				self.jt_name[3]+'_pos',self.jt_name[4]+'_pos',self.jt_name[5]+'_pos',\
				self.jt_name[6]+'_pos',self.jt_name[0]+'_vel',self.jt_name[1]+'_vel',\
				self.jt_name[2]+'_vel',self.jt_name[3]+'_vel',self.jt_name[4]+'_vel',\
				self.jt_name[5]+'_vel',self.jt_name[6]+'_vel'
		self.f_count_pre = self.f_count
		self.p_count_pre = self.init_time
		self.jt_count_pre = self.jt_count


	def log_state(self, bias=True):
		if self.f_count > self.f_count_pre:
			self.f_count_pre = self.f_count
			time_int = self.force_time-self.init_time
			self.read_ft(bias)
#			print >> self.ft_log,time_int,\
			print >> self.ft_log, time_int, self.f_count,\
				self.force[0,0],self.force[1,0],self.force[2,0],\
				self.force_raw[0,0],self.force_raw[1,0],self.force_raw[2,0],\
				self.torque[0,0],self.torque[1,0],self.torque[2,0],\
				self.torque_raw[0,0],self.torque_raw[1,0],self.torque_raw[2,0]
	
			self.force_data.append(self.force)
			self.force_raw_data.append(self.force_raw)
			self.torque_data.append(self.torque)
			self.torque_raw_data.append(self.torque_raw)
			self.f_time_data.append(self.force_time)

		self.read_pose()
		pos = self.pos - self.init_pos
		if self.p_count > self.p_count_pre:
			self.p_count_pre = self.p_count
			time_int = self.pose_time-self.init_time
#			print >> self.pos_log,time_int,\
			print >> self.pos_log, time_int, self.p_count,\
				pos[0,0],pos[1,0],pos[2,0],\
				self.rot[0,0],self.rot[1,0],self.rot[2,0]

			self.rot_data.append(self.rot)
			self.quat_data.append(self.quat)
			self.pos_data.append(pos)
			self.p_time_data.append(self.pose_time)

		if self.jt_count > self.jt_count_pre:
			self.jt_count_pre = self.jt_count
			time_int = self.jt_time-self.init_time
			print >> self.pr2_jt_log, time_int, self.jt_count,\
				self.jt_pos[0],self.jt_pos[1],self.jt_pos[2],self.jt_pos[3],\
				self.jt_pos[4],self.jt_pos[5],self.jt_pos[6],self.jt_vel[0],\
				self.jt_vel[1],self.jt_vel[2],self.jt_vel[3],self.jt_vel[4],\
				self.jt_vel[5],self.jt_vel[6]

			self.jt_name_data = self.jt_name
			self.jt_pos_data.append(self.jt_pos)
			self.jt_vel_data.append(self.jt_vel)
			self.jt_time_data.append(self.jt_time)

#			print self.jt_pos, '\n', self.jt_vel

		print '\nPosition\t\tRotation\t\tForce:',\
			'\nX: ', pos[0,0],'\t', math.degrees(self.rot[0,0]), '\t\t', self.force[0,0],\
			'\nY: ', pos[1,0],'\t', math.degrees(self.rot[1,0]), '\t\t', self.force[1,0],\
			'\nZ: ', pos[2,0],'\t', math.degrees(self.rot[2,0]), '\t\t', self.force[2,0]


	def close_log(self):
		dict = {}
		dict['init_pos'] = self.init_pos
		dict['init_time'] = self.init_time
		dict['force'] = self.force_data
		dict['force_raw'] = self.force_raw_data
		dict['torque'] = self.torque_data
		dict['torque_raw'] = self.torque_raw_data
		dict['rot_data'] = self.rot_data
		dict['pos'] = self.pos_data
		dict['quat'] = self.quat_data
		dict['ptime'] = self.p_time_data
		dict['ftime'] = self.f_time_data
		dict['pr2_jt_name'] = self.jt_name_data
		dict['pr2_jt_time'] = self.jt_time_data
		dict['pr2_jt_vel'] = self.jt_vel_data
		dict['pr2_jt_pos'] = self.jt_pos_data
		pkl.dump(dict, self.pkl)
		self.pkl.close()

		self.ft_log.close()
		self.pos_log.close()
		self.gen_log.close()
		self.pr2_jt_log.close()
		print 'Closing..  log files have saved..'


if __name__ == '__main__':
	log = PR2_ADL_log()
	log.init_log()
	
	while not rospy.is_shutdown():
#		log.log_state(bias=False)
		log.log_state()
		rospy.sleep(1/1000.)

	log.close_log()

