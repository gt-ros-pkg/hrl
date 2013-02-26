#   Copyright 2013 Georgia Tech Research Corporation
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#  http://healthcare-robotics.com/

# Author: Marc Killpack, Advait Jain, Philip Grice

import numpy as np, math
from threading import RLock, Timer
import sys, copy

import roslib; roslib.load_manifest('hrl_haptic_mpc')
import rospy
import tf

from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Empty

from equilibrium_point_control.hrl_arm import HRLArm
from pykdl_utils.kdl_kinematics import create_kdl_kin
from hrl_msgs.msg import FloatArrayBare
import hrl_lib.viz as hv

class URDFArm(HRLArm):

    def __init__(self, arm, tf_listener=None, base_link=None, end_link=None):
	if not arm in ['l', 'r']:
            raise Exception, 'Arm should be "l" or "r"'
        if base_link is None:
            self.base_link = '/torso_lift_link'
	else: 
	    self.base_link = base_link
        if end_link is None:
            self.end_link = arm + '_gripper_tool_frame'
	else:
	    self.end_link = end_link
	kinematics = create_kdl_kin(self.base_link, self.end_link)
        HRLArm.__init__(self, kinematics)
        self.joint_names_list = kinematics.get_joint_names()
        self.torso_position = None
        self.arm_efforts = None
        self.delta_jep = None
	print "LENGTH: ",len(self.joint_names_list)
	try:
	    if len(self.joint_names_list) == 9:
	        self.kp = [rospy.get_param('crona_torso_traj_controller/gains/'+nm+'/p') for nm in self.joint_names_list[:3]]
	        self.kp += [rospy.get_param(arm+'_arm_controller/gains/'+nm+'/p') for nm in self.joint_names_list[3:]] # testing
	    else:
  	 	self.kp = [rospy.get_param(arm+'_arm_controller/gains/'+nm+'/p') for nm in self.joint_names_list]
	        
        except:
            print "kp is not on param server ... exiting"
            assert(False)
#        max_kp =  np.max(self.kp)
        self.kp[-1] = 5. #This smells like a Hack. # testing
        self.kp[-2] = 50.
        self.kp[-3] = 50.
	self.kp = [50. for i in range(len(self.kp))]

        try:
	    if len(self.joint_names_list) == 9:
	        self.kd = [rospy.get_param('crona_torso_traj_controller/gains/'+nm+'/d') for nm in self.joint_names_list[:3]]
                self.kd += [rospy.get_param(arm+'_arm_controller/gains/'+nm+'/d') for nm in self.joint_names_list[3:]] # testing
	    else:
 		self.kd = [rospy.get_param(arm+'_arm_controller/gains/'+nm+'/d') for nm in self.joint_names_list]
        except:
            print "kd is not on param server ... exiting"
            assert(False)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)

        # Set desired joint angle - either through a delta from the current position, or as an absolute value.
        rospy.Subscriber ("/haptic_mpc/q_des", FloatArrayBare, self.set_ep_callback)
        rospy.Subscriber ("/haptic_mpc/delta_q_des", FloatArrayBare, self.set_delta_ep_callback)
        #rospy.Subscriber("/delta_jep_mpc_cvxgen", FloatArrayBare, self.set_ep_callback)

        self.marker_pub = rospy.Publisher(arm+'_arm/viz/markers', Marker)
        self.cep_marker_id = 1

        try:
          if tf_listener == None:
            self.tf_lstnr = tf.TransformListener()
          else:
            self.tf_lstnr = tf_listener
        except rospy.ServiceException, e:
          rospy.loginfo("ServiceException caught while instantiating a TF listener. This seems to be normal.")
          pass

        self.joint_angles_pub = rospy.Publisher(arm+'_arm_controller/command',
                                                JointTrajectory)
	if len(self.joint_names_list) == 9:
	    self.torso_ja_pub = rospy.Publisher('/crona_torso_traj_controller/command',JointTrajectory) # testing
    ##
    # Callback for /joint_states topic. Updates current joint
    # angles and efforts for the arms constantly
    # @param data JointState message recieved from the /joint_states topic
    def joint_states_cb(self, data):
        arm_angles = []
        arm_efforts = []
        arm_vel = []
        jt_idx_list = [0]*len(self.joint_names_list)
        for i, jt_nm in enumerate(self.joint_names_list):
            jt_idx_list[i] = data.name.index(jt_nm)

        for i, idx in enumerate(jt_idx_list):
            if data.name[idx] != self.joint_names_list[i]:
                raise RuntimeError('joint angle name does not match.')
            arm_angles.append(data.position[idx])
            arm_efforts.append(data.effort[idx])
            arm_vel.append(data.velocity[idx])

        with self.lock:
	    self.q = arm_angles
            self.arm_efforts = arm_efforts
            self.qdot = arm_vel
	    if self.base_link == 'torso_chest_link': # if/else statement is here because self.base_link is 'torso_chest_link and not 'torso_chest_joint'
	        torso_idx = data.name.index('torso_chest_joint')
	    elif self.base_link == 'base_link':
	        torso_idx = data.name.index('base_rev_joint') # testing
	    else:
                torso_idx = data.name.index(self.base_link)
            self.torso_position = data.position[torso_idx]

    def set_ep(self, jep, duration=0.15):
	jep = copy.copy(jep)
        if jep is None or len(jep) != len(self.joint_names_list):
            raise RuntimeError("set_jep value is " + str(jep))

        with self.lock:
	    if len(self.joint_names_list) == 9: 
                arm_trajectory = JointTrajectory()
                arm_trajectory.joint_names = self.joint_names_list[3:]
                jtp = JointTrajectoryPoint()
                jtp.positions = jep[3:]
                jtp.time_from_start = rospy.Duration(duration)
                arm_trajectory.points.append(jtp)
                self.joint_angles_pub.publish(arm_trajectory)

		torso_trajectory = JointTrajectory()
	        torso_trajectory.joint_names = self.joint_names_list[:3]
                jtp = JointTrajectoryPoint()
                jtp.positions = jep[:3]
                jtp.time_from_start = rospy.Duration(duration)
                torso_trajectory.points.append(jtp)
                self.torso_ja_pub.publish(torso_trajectory)

                self.ep = jep
	    else:
	        trajectory = JointTrajectory()
                trajectory.joint_names = self.joint_names_list
                jtp = JointTrajectoryPoint()
                jtp.positions = jep
                jtp.time_from_start = rospy.Duration(duration)
                trajectory.points.append(jtp)
                self.joint_angles_pub.publish(trajectory)
                self.ep = jep

    def set_delta_ep_callback(self, msg):
        delta_jep = msg.data
        
        if self.ep == None:
            self.ep = self.get_joint_angles()
        des_jep = (np.array(self.ep) + np.array(delta_jep)).tolist()
        
        self.set_ep(des_jep)
        
#        delta_jep = copy.copy(msg.data)
#        if delta_jep is None or len(delta_jep) != len(self.joint_names_list):
#            raise RuntimeError("set_jep value is " + str(delta_jep))
#
#        with self.lock:
#            if self.ep == None:
#              self.ep = self.get_joint_angles()
#            jep = (np.array(self.ep) + np.array(delta_jep)).tolist()
#            trajectory = JointTrajectory()
#            trajectory.joint_names = self.joint_names_list
#            jtp = JointTrajectoryPoint()
#            jtp.positions = jep
#            jtp.time_from_start = rospy.Duration(0.15)
#            trajectory.points.append(jtp)
#            self.joint_angles_pub.publish(trajectory)
#            self.ep = jep

    def set_ep_callback(self, msg):
        des_jep = msg.data        
        self.set_ep(des_jep)
        
        
#        with self.lock:
#            des_jep = copy.copy(msg.data)
#            if des_jep is None or len(des_jep) != len(self.joint_names_list):
#                raise RuntimeError("set_jep value is " + str(des_jep))
##            self.delta_jep = des_jep
#            jep = (np.array(des_jep)).tolist()
#            trajectory = JointTrajectory()
#            trajectory.joint_names = self.joint_names_list
#            jtp = JointTrajectoryPoint()
#            jtp.positions = jep
#            jtp.time_from_start = rospy.Duration(0.15)
#            trajectory.points.append(jtp)
#            self.joint_angles_pub.publish(trajectory)
#            self.ep = jep

    def wrap_angles(self, q):
        for ind in [4, 6]:
            while q[ind] < -np.pi:
                q[ind] += 2*np.pi
            while q[ind] > np.pi:
                q[ind] -= 2*np.pi
        return q

    def publish_rviz_markers(self):
        # publish the CEP marker.
        o = np.matrix([0.,0.,0.,1.]).T
        jep = self.get_ep()
        cep, r = self.kinematics.FK(jep)
        cep_marker = hv.single_marker(cep, o, 'sphere',
                        self.base_link, color=(0., 0., 1., 1.),
                        scale = (0.02, 0.02, 0.02), duration=0.,
                        m_id=1)
        cep_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(cep_marker)

        q = self.get_joint_angles()
        ee, r = self.kinematics.FK(q)
        ee_marker = hv.single_marker(ee, o, 'sphere',
                        self.base_link, color=(0., 1., 0., 1.),
                        scale = (0.02, 0.02, 0.02), duration=0.,
                        m_id=2)
        ee_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(ee_marker)


if __name__ == '__main__':
    rospy.init_node('pr2_arms_test')
    robot = URDFArm('l')

    if False:
        jep = [0.] * len(robot.joint_names_list) 
        jep = np.radians([-30, 0, -90, -60, 0, 0, 0])
        rospy.loginfo('Going to home location.')
        raw_input('Hit ENTER to go')
        robot.set_ep(jep, duration=2.)

    if True:
        # simple go_jep example
        roslib.load_manifest('equilibrium_point_control')
        import equilibrium_point_control.epc as epc
        epcon = epc.EPC(robot)

        while robot.get_joint_angles() == None:
            rospy.sleep(0.1)

        q = robot.get_joint_angles()
        robot.set_ep(q)

        jep = [0.] * len(robot.joint_names_list)
        #jep = np.radians([30, 0, 90, -60, -180, -30, 0])  # for left arm
        #jep = np.radians([-30, 0, -90, -60, 0, 0, 0]) # for right arm
        epcon.go_jep(jep, speed=math.radians(10.))

    if True:
        while robot.get_joint_angles() == None:
            rospy.sleep(0.1)

        q = robot.get_joint_angles()
        ee, r = robot.kinematics.FK(q)
        print "ee is at :\n", ee
        print "r is :\n", r

