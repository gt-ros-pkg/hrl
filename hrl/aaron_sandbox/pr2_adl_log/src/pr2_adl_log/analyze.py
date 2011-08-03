#!/usr/bin/python

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('hrl_lib')
import math, time, optparse
import numpy as np
import tf.transformations as tr
import cPickle as pkl
import scipy.stats as st
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import scipy.optimize as so
import hrl_lib.util as ut
import hrl_lib.transforms as hrl_tr

def parse():
	parser = optparse.OptionParser('Input the Pose node name and the ft sensor node name')
	parser.add_option("-n", "--name", action="store", type="string",\
		dest="file_name", default="test")
	(options, args) = parser.parse_args()
	print 'Opening file name: ',options.file_name

	return options.file_name

def compare(file1,file2):
	f1=ut.load_pickle(file1+'.pkl')
	f2=ut.load_pickle(file2+'.pkl')
	force1 = f1['force']
	force2 = f2['force']
	fmag1 = []
	fmag2 = []

	for i in range(len(force1)):
		fmag1.append(np.linalg.norm(force1[i]))
	for i in range(len(force2)):
		fmag2.append(np.linalg.norm(force2[i]))
	res = st.ttest_ind(fmag1,fmag2)
	print res


if __name__ == '__main__':
	file_name = parse()
	d = ut.load_pickle(file_name+'.pkl')
	log = open('result_'+file_name+'.log','w')
	force = d['force']
	force_raw = d['force_raw']
	rot = d['rot_data']
	quat = d['quat']
	pos = d['pos']
	ptime = np.array(d['ptime'])-d['init_time']
	ftime = np.array(d['ftime'])-d['init_time']
	init_time = d['init_time']
	ptime2 = []
	force_mag = []
	length = []
	accel = []
	accel_mag = []
	v = []
	fx = []
	fy = []
	fz = []
	x = []
	y = []
	z = []
	vx = []
	vy = []
	vz = []
	rotx = []
	roty = []
	rotz = []
	time_diff = []
	vel = np.matrix([0.,0.,0.]).T

	for i in range(len(pos)):
		if i < len(pos)-1:
			if ptime[i+1]-ptime[i] > .02:
				vel=(pos[i+1]-pos[i])/(ptime[i+1]-ptime[i])
				length.append(np.linalg.norm(vel)*(ptime[i+1]-ptime[i]))
				ptime2.append(ptime[i])
				v.append(np.linalg.norm(vel))
				vx.append(vel[0,0])
				vy.append(vel[1,0])
				vz.append(vel[2,0])
		
		x.append(pos[i][0,0])
		y.append(pos[i][1,0])
		z.append(pos[i][2,0])
		
		rotx.append(math.degrees(rot[i][0,0]))
		roty.append(math.degrees(rot[i][1,0]))
		rotz.append(math.degrees(rot[i][2,0]))

	for i in range(len(force)):
		force_mag.append(np.linalg.norm(force[i]))
		fx.append(force[i][0,0])
		fy.append(force[i][1,0])
		fz.append(force[i][2,0])

		
		
#	for i in range(len(v)-1):
#		a = (v[i+1]-v[i])/.01
#		accel.append(a)
#		accel_mag.append(np.linalg.norm(a))
	
	print 'time: ', max(ptime)
	print 'max speed: ', max(v)
	print 'min speed: ', min(v)
		
	path_length = sum(length)
	print 'Path Length: ', path_length

	print 'max vel: ', max(vx),max(vy),max(vz)
	print 'min vel: ', min(vx),min(vy),min(vz)
	print 'ave vel: ', np.mean(vx),np.mean(vx),np.mean(vz)

	print 'max force: ', max(fx), max(fy), max(fz)
	print 'min force: ', min(fx), min(fy), min(fz)
	print 'ave force: ', np.mean(fx),np.mean(fx),np.mean(fz)

	print 'max force_mag: ', max(force_mag)
	print 'min force_mag: ', min(force_mag)
	print 'ave force_mag: ', np.mean(force_mag)
	print 'std_force_mag: ', np.std(force_mag)
	print 'integration of force (N*s): ',sum(np.array(force_mag)*.01)

	print >> log, 'Categories x-axis y-axis z-axis'
	print >> log, 'max_vel', max(vx),max(vy),max(vz)
	print >> log, 'min_vel', min(vx),min(vy),min(vz)
	print >> log, 'ave_vel', np.mean(vx),np.mean(vx),np.mean(vz)

	print >> log, 'max_force', max(fx), max(fy), max(fz)
	print >> log, 'min_force', min(fx), min(fy), min(fz)
	print >> log, 'ave_force', np.mean(fx),np.mean(fx),np.mean(fz)


	print >> log, 'time', max(ptime)
	print >> log, 'path_length', sum(length)
	print >> log, 'max_force_mag', max(force_mag)
	print >> log, 'min_force_mag', min(force_mag)
	print >> log, 'ave_force_mag', np.mean(force_mag)
	print >> log, 'std_force_mag', np.std(force_mag)
	print >> log, 'int_force_mag',sum(np.array(force_mag)*.01)


	fig = plt.figure()
#	ax = Axes3D(fig)
#	ax.scatter(x,y,z,zdir = 'z')
	fig.suptitle(file_name+'_Position')
	ax = fig.add_subplot(3,1,1)
	ax.plot(ptime,x)
	ax.grid(True)
	ax.set_ylabel('x (m)')
	ax = fig.add_subplot(3,1,2)
	ax.plot(ptime,y)
	ax.grid(True)
	ax.set_ylabel('y (m)')
	ax = fig.add_subplot(3,1,3)
	ax.plot(ptime,z)
	ax.grid(True)
	ax.set_ylabel('z (m)')

	fig2 = plt.figure()
	fig2.suptitle(file_name+'_Force')
	ax = fig2.add_subplot(3,1,1)
	ax.plot(ftime,fx)
	ax.grid(True)
	ax.set_ylabel('Fx (N)')
	ax = fig2.add_subplot(3,1,2)
	ax.plot(ftime,fy)
	ax.grid(True)
	ax.set_ylabel('Fy (N)')
	ax = fig2.add_subplot(3,1,3)
	ax.plot(ftime,fz)
	ax.grid(True)
	ax.set_ylabel('Fz (N)')


	fig2b = plt.figure()
	fig2b.suptitle(file_name+' 3D Tra')
	ax = Axes3D(fig2b)
	ax.plot3D(x,y,z)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	

	fig2c = plt.figure()
	fig2c.suptitle(file_name+' Force Magnitute')
	ax = fig2c.gca()
	ax.plot(ftime,force_mag)
	ax.set_ylabel('Force (N)')
	ax.set_xlabel('Time (s)')

	fig3 = plt.figure()
	fig3.suptitle(file_name+'_Velocity')
	ax = fig3.add_subplot(3,1,1)
	ax.plot(ptime2,vx)
	ax.grid(True)
	ax.set_ylabel('Vx (m/s)')
	ax = fig3.add_subplot(3,1,2)
	ax.plot(ptime2,vy)
	ax.grid(True)
	ax.set_ylabel('Vy (m/s)')
	ax = fig3.add_subplot(3,1,3)
	ax.plot(ptime2,vz)
	ax.grid(True)
	ax.set_ylabel('Vz (m/s)')

	fig3a = plt.figure()
	fig3a.suptitle(file_name+' Speed')
	ax = fig3a.gca()
	ax.plot(ptime2,v)
	ax.set_ylabel('Speed (m/s)')
	ax.set_xlabel('Time (s)')


	fig4 = plt.figure()
	fig4.suptitle(file_name+'_rotation')
	ax = fig4.add_subplot(3,1,1)
	ax.plot(ptime,rotx)
	ax.grid(True)
	ax.set_ylabel('angle (deg)')
	ax = fig4.add_subplot(3,1,2)
	ax.plot(ptime,roty)
	ax.grid(True)
	ax.set_ylabel('angle (deg)')
	ax = fig4.add_subplot(3,1,3)
	ax.plot(ptime,rotz)
	ax.grid(True)
	ax.set_ylabel('angle (deg)')


	plt.show()
	log.close()

#	compare('aa_scratch_arm','aa_scratch_face')
