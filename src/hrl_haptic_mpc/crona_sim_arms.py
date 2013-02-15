#!/usr/bin/env python

import numpy as np, math
from threading import RLock, Timer
import sys, copy

import roslib; roslib.load_manifest('sttr_behaviors')
import rospy
import actionlib
import tf

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from hrl_msgs.msg import FloatArrayBare

from equilibrium_point_control.hrl_arm import HRLArm
from pykdl_utils.kdl_kinematics import create_kdl_kin

class CronaArm(HRLArm):
    def __init__(self, arm, tf_listener=None):
	if arm != 'l' and arm != 'r':
            raise Exception, 'Arm should only be "l" or "r"'
        kinematics = create_kdl_kin('/torso_chest_link', arm + '_hand_link')
        HRLArm.__init__(self, kinematics)
        self.joint_names_list = kinematics.get_joint_names()
	self.torso_position = None
        self.arm_efforts = None
        self.delta_jep = None
	
        try:
            self.kp = [rospy.get_param(arm+'_arm_controller/gains/'+nm+'/p') for nm in self.joint_names_list]
        except:
            print "kp is not on param server ... exiting"
            assert(False)
#        max_kp =  np.max(self.kp)
        #self.kp[-1] = 5. #This smells like a Hack.
        #self.kp[-2] = 50.
        #self.kp[-3] = 50.

        try:
            self.kd = [rospy.get_param(arm+'_arm_controller/gains/'+nm+'/d') for nm in self.joint_names_list]
        except:
            print "kd is not on param server ... exiting"
            assert(False)

	rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)

        # Set desired joint angle - either through a delta from the current position, or as an absolute value.
        rospy.Subscriber ("/haptic_mpc/q_des", FloatArrayBare, self.set_ep_ros)
        rospy.Subscriber ("/haptic_mpc/delta_q_des", FloatArrayBare, self.set_delta_ep_ros)
	#rospy.Subscriber("/delta_jep_mpc_cvxgen", FloatArrayBare, self.set_ep_ros)

        #self.marker_pub = rospy.Publisher(arm+'_arm/viz/markers', Marker)
        #self.cep_marker_id = 1

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

            torso_idx = data.name.index('torso_chest_joint')
            self.torso_position = data.position[torso_idx]

    def set_ep(self, jep, duration=0.15):
        jep = copy.copy(jep)
        if jep is None or len(jep) != len(self.joint_names_list):
            raise RuntimeError("set_jep value is " + str(jep))

        with self.lock:
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names_list
            jtp = JointTrajectoryPoint()
            jtp.positions = jep
            jtp.time_from_start = rospy.Duration(duration)
            trajectory.points.append(jtp)
            self.joint_angles_pub.publish(trajectory)
            self.ep = jep

    def set_delta_ep_ros(self, msg):
        delta_jep = copy.copy(msg.data)
        if delta_jep is None or len(delta_jep) != len(self.joint_names_list):
            raise RuntimeError("set_jep value is " + str(delta_jep))

        with self.lock:
            if self.ep == None:
              self.ep = self.get_joint_angles()
            jep = (np.array(self.ep) + np.array(delta_jep)).tolist()
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names_list
            jtp = JointTrajectoryPoint()
            jtp.positions = jep
            jtp.time_from_start = rospy.Duration(0.15)
            trajectory.points.append(jtp)
            self.joint_angles_pub.publish(trajectory)
            self.ep = jep

    def set_ep_ros(self, msg):
        with self.lock:
            des_jep = copy.copy(msg.data)
            if des_jep is None or len(des_jep) != len(self.joint_names_list):
                raise RuntimeError("set_jep value is " + str(des_jep))
#            self.delta_jep = des_jep
            jep = (np.array(des_jep)).tolist()
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names_list
            jtp = JointTrajectoryPoint()
            jtp.positions = jep
            jtp.time_from_start = rospy.Duration(0.15)
            trajectory.points.append(jtp)
            self.joint_angles_pub.publish(trajectory)
            self.ep = jep

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
                        '/torso_lift_link', color=(0., 0., 1., 1.),
                        scale = (0.02, 0.02, 0.02), duration=0.,
                        m_id=1)
        cep_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(cep_marker)

        q = self.get_joint_angles()
        ee, r = self.kinematics.FK(q)
        ee_marker = hv.single_marker(ee, o, 'sphere',
                        '/torso_lift_link', color=(0., 1., 0., 1.),
                        scale = (0.02, 0.02, 0.02), duration=0.,
                        m_id=2)
        ee_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(ee_marker)
	
	
if __name__ == '__main__':
    rospy.init_node('crona_arms_test')
    robot = CronaArm('l')
