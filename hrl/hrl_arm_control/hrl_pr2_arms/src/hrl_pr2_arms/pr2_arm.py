#! /usr/bin/python

# TODO TODO TODO
# Saving an internal equilibrium point is dangerous.  If another application
# uses the controller, the internal state of this controller will not be updated
# causing potentially high velocity actions since this controller does not know
# where the EP actually is.  Every low-level controller should publish its current
# EP.  The JT controller does and this code has been modified to that effect.
# The joint trajectory controller does not and this is a significant issue.
#
# Um, never mind...

import copy
import numpy as np
from threading import Lock

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('object_manipulation_msgs')

import rospy
import actionlib

from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from object_manipulation_msgs.msg import CartesianGains
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
import tf.transformations as tf_trans

from hrl_generic_arms.hrl_arm_template import HRLArm
from hrl_generic_arms.pose_converter import PoseConverter
import kdl_parser_python.kdl_parser as kdlp
from hrl_kdl_arms.kdl_arm_kinematics import KDLArmKinematics


JOINT_NAMES_LIST = ['_shoulder_pan_joint',
                    '_shoulder_lift_joint', '_upper_arm_roll_joint',
                    '_elbow_flex_joint', '_forearm_roll_joint',
                    '_wrist_flex_joint', '_wrist_roll_joint']

class PR2Arm(HRLArm):
    ##
    # Initializes subscribers
    # @param arm 'r' for right, 'l' for left
    def __init__(self, arm, kinematics, controller_name, timeout):
        super(PR2Arm, self).__init__(kinematics)
        self.arm = arm
        if '%s' in controller_name:
            controller_name = controller_name % arm
        self.controller_name = controller_name

        self.joint_names_list = []
        for s in JOINT_NAMES_LIST:
            self.joint_names_list.append(arm + s)
        self.joint_state_inds = None
        self.joint_angles = None
        self.joint_efforts = None

        self.ctrl_state_lock = Lock()
        self.ctrl_state_dict = None

        rospy.Subscriber('joint_states', JointState, self._joint_state_cb)

        if timeout > 0:
            self.wait_for_joint_angles(timeout)
            #self.reset_ep()

    ##
    # Joint angles listener callback
    def _joint_state_cb(self, msg):
        with self.lock:
            if self.joint_state_inds is None:
                self.joint_state_inds = [msg.name.index(joint_name) for 
                                         joint_name in self.joint_names_list]
            self.joint_angles = [msg.position[i] for i in self.joint_state_inds]
            self.joint_efforts = [msg.effort[i] for i in self.joint_state_inds]

    ##
    # Returns the current joint angle positions
    # @param wrapped If False returns the raw encoded positions, if True returns
    #                the angles with the forearm and wrist roll in the range -pi to pi
    def get_joint_angles(self, wrapped=False):
        with self.lock:
            if self.joint_angles is None:
                rospy.logwarn("[pr2_arm_base] Joint angles haven't been filled yet")
                return None
            if wrapped:
                return self.wrap_angles(self.joint_angles)
            else:
                return np.array(self.joint_angles)

    def get_controller_state(self):
        with self.ctrl_state_lock:
            if self.ctrl_state_dict is None:
                rospy.logerror("[pr2_arm_base] get_controller_state NOT IMPLEMENTED!")
            elif len(self.ctrl_state_dict) == 0:
                rospy.logwarn("[pr2_arm_base] Controller state not yet published.")
            return self.ctrl_state_dict

    ##
    # Wait until we have found the current joint angles.
    # @param timeout Time at which we break if we haven't recieved the angles.
    def wait_for_joint_angles(self, timeout=5.):
        start_time = rospy.get_time()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() - start_time < timeout:
            with self.lock:
                if self.joint_angles is not None:
                    return True
            r.sleep()
        if not rospy.is_shutdown():
            rospy.logwarn("[pr2_arm_base] Cannot read joint angles, timing out.")
        return False

    ##
    # Wait until we have the ep from the controller
    # @param timeout Time at which we break if we haven't recieved the EP.
    def wait_for_ep(self, timeout=5.):
        start_time = rospy.get_time()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() - start_time < timeout:
            with self.lock:
                if self.ep is not None:
                    return True
            r.sleep()
        if not rospy.is_shutdown():
            rospy.logwarn("[pr2_arm_base] Cannot read controller state, timing out.")
        return False
            
    ##
    # Returns the current joint efforts (similar to torques)
    def get_joint_efforts(self):
        with self.lock:
            if self.joint_efforts is None:
                rospy.logwarn("[pr2_arm_base] Joint efforts haven't been filled yet")
                return None
            return np.array(self.joint_efforts)

    ##
    # Returns the same angles with the forearm and wrist roll wrapped to the 
    # range -pi to pi
    # @param q Joint angles
    # @return Wrapped joint angles
    def wrap_angles(self, q):
        ret_q = list(q)
        for ind in [4, 6]:
            while ret_q[ind] < -np.pi:
                ret_q[ind] += 2*np.pi
            while ret_q[ind] > np.pi:
                ret_q[ind] -= 2*np.pi
        return np.array(ret_q)

def create_pr2_arm(arm, arm_type=PR2Arm, base_link="torso_lift_link",  
                   end_link="%s_gripper_tool_frame", param="/robot_description",
                   controller_name=None, timeout=5.):
    if "%s" in base_link:
        base_link = base_link % arm
    if "%s" in end_link:
        end_link = end_link % arm
    chain, joint_info = kdlp.chain_from_param(base_link, end_link, param=param)
    kin = KDLArmKinematics(chain, joint_info, base_link, end_link)
    if controller_name is None:
        return arm_type(arm, kin)
    else:
        return arm_type(arm, kin, controller_name=controller_name)

def create_pr2_arm_from_file(arm, arm_type=PR2Arm, base_link="torso_lift_link",  
                             end_link="%s_gripper_tool_frame", 
                             filename="$(find hrl_pr2_arms)/params/pr2_robot_uncalibrated_1.6.0.xml",
                             controller_name=None, timeout=0.):
    if "%s" in base_link:
        base_link = base_link % arm
    if "%s" in end_link:
        end_link = end_link % arm
    chain, joint_info = kdlp.chain_from_file(base_link, end_link, filename)
    kin = KDLArmKinematics(chain, joint_info, base_link, end_link)
    if controller_name is None:
        return arm_type(arm, kin, timeout=timeout)
    else:
        return arm_type(arm, kin, timeout=timeout, controller_name=controller_name)

class PR2ArmJointTrajectory(PR2Arm):
    def __init__(self, arm, kinematics, controller_name='/%s_arm_controller', timeout=5.):
        super(PR2ArmJointTrajectory, self).__init__(arm, kinematics, controller_name, timeout)
        self.joint_action_client = actionlib.SimpleActionClient(
                                       controller_name + '/joint_trajectory_action',
                                       JointTrajectoryAction)

        self.ctrl_state_dict = {}
        rospy.Subscriber(controller_name + '/state', JointTrajectoryControllerState, 
                         self._ctrl_state_cb)
        self.wait_for_ep(timeout)

    def _ctrl_state_cb(self, ctrl_state):
        with self.lock:
            self.ep = np.array(ctrl_state.desired.positions)
        with self.ctrl_state_lock:
            self.ctrl_state_dict["frame"] = ctrl_state.header.frame_id
            self.ctrl_state_dict["x_desi"] = np.array(ctrl_state.desired.positions)
            self.ctrl_state_dict["xd_desi"] = np.array(ctrl_state.desired.velocities)
            self.ctrl_state_dict["xdd_desi"] = np.array(ctrl_state.desired.accelerations)
            self.ctrl_state_dict["x_act"] = np.array(ctrl_state.actual.positions)
            self.ctrl_state_dict["xd_act"] = np.array(ctrl_state.actual.velocities)
            self.ctrl_state_dict["xdd_act"] = np.array(ctrl_state.actual.accelerations)
            self.ctrl_state_dict["x_err"] = np.array(ctrl_state.error.positions)
            self.ctrl_state_dict["xd_err"] = np.array(ctrl_state.error.velocities)
            self.ctrl_state_dict["xdd_err"] = np.array(ctrl_state.error.accelerations)
            
    ##
    # Commands joint angles to a single position
    # @param jep List of 7 joint params to command the the arm to reach
    # @param duration Length of time command should take
    # @param delay Time to wait before starting joint command
    def set_ep(self, jep, duration, delay=0.0):
        if jep is None or len(jep) != 7:
            raise RuntimeError("set_ep value is " + str(jep))
        jtg = JointTrajectoryGoal()
        jtg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(delay)
        jtg.trajectory.joint_names = self.joint_names_list
        jtp = JointTrajectoryPoint()
        jtp.positions = list(jep)
        jtp.time_from_start = rospy.Duration(duration)
        jtg.trajectory.points.append(jtp)
        self.joint_action_client.send_goal(jtg)

    def interpolate_ep(self, ep_a, ep_b, t_vals):
        linspace_list = [[ep_a[i] + (ep_b[i] - ep_a[i]) * t for t in t_vals] for i in range(len(ep_a))]
        return np.dstack(linspace_list)[0]

    def reset_ep(self):
        pass
        #self.ep = self.get_joint_angles(False)

class PR2ArmCartesianBase(PR2Arm):
    def __init__(self, arm, kinematics, controller_name='/%s_cart', timeout=5.):
        super(PR2ArmCartesianBase, self).__init__(arm, kinematics, controller_name, timeout)
        self.command_pose_pub = rospy.Publisher(self.controller_name + '/command_pose', PoseStamped)

        self.ctrl_state_dict = {}
        rospy.Subscriber(controller_name + '/state', JointTrajectoryControllerState, 
                         self._ctrl_state_cb)
        self.wait_for_ep(timeout)

    def _ctrl_state_cb(self, ctrl_state):
        with self.lock:
            self.ep = PoseConverter.to_pos_rot(ctrl_state.x_desi_filtered)
        with self.ctrl_state_lock:
            self.ctrl_state_dict["frame"] = ctrl_state.header.frame_id
            self.ctrl_state_dict["x_desi"] = PoseConverter.to_pos_rot(ctrl_state.x_desi)
            self.ctrl_state_dict["xd_desi"] = PoseConverter.to_pos_rot(ctrl_state.xd_desi)
            self.ctrl_state_dict["x_act"] = PoseConverter.to_pos_rot(ctrl_state.x)
            self.ctrl_state_dict["xd_act"] = PoseConverter.to_pos_rot(ctrl_state.xd)
            self.ctrl_state_dict["x_desi_filt"] = PoseConverter.to_pos_rot(
                                                                ctrl_state.x_desi_filtered)
            self.ctrl_state_dict["x_err"] = PoseConverter.to_pos_rot(ctrl_state.x_err)
            self.ctrl_state_dict["tau_pose"] = np.array(ctrl_state.tau_pose)
            self.ctrl_state_dict["tau_posture"] = np.array(ctrl_state.tau_posture)
            self.ctrl_state_dict["tau"] = np.array(ctrl_state.tau)
            self.ctrl_state_dict["F"] = np.array([ctrl_state.F.force.x, 
                                                  ctrl_state.F.force.y,
                                                  ctrl_state.F.force.z,
                                                  ctrl_state.F.torque.x,
                                                  ctrl_state.F.torque.y,
                                                  ctrl_state.F.torque.z])

    def set_ep(self, cep, duration, delay=0.0):
        cep_pose_stmp = PoseConverter.to_pose_stamped_msg('torso_lift_link', cep)
        cep_pose_stmp.header.stamp = rospy.Time.now()
        self.command_pose_pub.publish(cep_pose_stmp)
        #self.ep = copy.deepcopy(cep)

    ##
    # Returns pairs of positions and rotations linearly interpolated between
    # the start and end position/orientations.  Rotations are found using slerp
    # @return List of (pos, rot) waypoints between start and end.
    def interpolate_ep(self, ep_a, ep_b, t_vals):
        pos_a, rot_a = ep_a
        pos_b, rot_b = ep_b
        num_samps = len(t_vals)
        pos_waypoints = np.array(pos_a) + np.array(np.tile(pos_b - pos_a, (1, num_samps))) * np.array(t_vals)
        pos_waypoints = [np.mat(pos).T for pos in pos_waypoints.T]
        rot_homo_a, rot_homo_b = np.eye(4), np.eye(4)
        rot_homo_a[:3,:3] = rot_a
        rot_homo_b[:3,:3] = rot_b
        quat_a = tf_trans.quaternion_from_matrix(rot_homo_a)
        quat_b = tf_trans.quaternion_from_matrix(rot_homo_b)
        rot_waypoints = []
        for t in t_vals:
            cur_quat = tf_trans.quaternion_slerp(quat_a, quat_b, t)
            rot_waypoints.append(np.mat(tf_trans.quaternion_matrix(cur_quat))[:3,:3])
        return zip(pos_waypoints, rot_waypoints)

    def reset_ep(self):
        pass
        #self.ep = self.kinematics.FK(self.get_joint_angles(False))

    def ep_error(self, ep_actual, ep_desired):
        pos_act, rot_act = ep_actual
        pos_des, rot_des = ep_desired
        err = np.mat(np.zeros((6, 1)))
        err[:3,0] = pos_act - pos_des
        err[3:6,0] = np.mat(tf_trans.euler_from_matrix(rot_des.T * rot_act)).T
        return err

class PR2ArmCartesianPostureBase(PR2ArmCartesianBase):
    def __init__(self, arm, kinematics, controller_name='/%s_cart', timeout=5.):
        super(PR2ArmCartesianPostureBase, self).__init__(arm, kinematics, controller_name, timeout)
        self.command_posture_pub = rospy.Publisher(self.controller_name + '/command_posture', 
                                                   Float64MultiArray)

    def set_posture(self, posture):
        assert len(posture) == 7, "Wrong posture length"
        msg = Float64MultiArray()
        msg.data = list(posture)
        self.command_posture_pub.publish(msg)

class PR2ArmJTranspose(PR2ArmCartesianPostureBase):
    pass

class PR2ArmJInverse(PR2ArmCartesianPostureBase):
    pass

class PR2ArmJTransposeTask(PR2ArmCartesianPostureBase):
    def __init__(self, arm, kinematics, controller_name='/%s_cart', timeout=5.):
        super(PR2ArmJTransposeTask, self).__init__(arm, kinematics, controller_name, timeout)
        self.command_gains_pub = rospy.Publisher(self.controller_name + '/gains', CartesianGains)

    def set_gains(self, p_gains, d_gains, frame=None):
        if frame is None:
            frame = self.kinematics.end_link
        all_gains = list(p_gains) + list(d_gains)
        gains_msg = CartesianGains(Header(0, rospy.Time.now(), frame),
                                   all_gains, [])
        self.command_gains_pub.publish(gains_msg)

