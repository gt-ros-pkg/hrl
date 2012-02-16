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
roslib.load_manifest('robot_mechanism_controllers')
roslib.load_manifest('pr2_manipulation_controllers')

import rospy
import actionlib

from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal 
from pr2_controllers_msgs.msg import JointTrajectoryControllerState 
from pr2_manipulation_controllers.msg import JTTaskControllerState
from robot_mechanism_controllers.msg import JTCartesianControllerState
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

ARM_AT_EP_ERR_THRESH = []

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
    # range 0 to 2 * pi
    # @param q Joint angles
    # @return Wrapped joint angles
    def wrap_angles(self, q):
        q_arr = np.array(q).copy()
        q_mod = np.mod(q_arr, 2 * np.pi)
        q_arr[[4, 6]] = q_mod[[4, 6]]
        return q_arr

    def diff_angles(self, q1, q2):
        diff = np.array(q1) - np.array(q2)
        diff_mod = np.mod(diff, 2 * np.pi)
        diff_alt = diff_mod - 2 * np.pi 
        for i in [4, 6]:
            if diff_mod[i] < -diff_alt[i]:
                diff[i] = diff_mod[i]
            else:
                diff[i] = diff_alt[i]
        return diff

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
        return arm_type(arm, kin, "", timeout=timeout)
    else:
        return arm_type(arm, kin, controller_name=controller_name, timeout=timeout)

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
                                       self.controller_name + '/joint_trajectory_action',
                                       JointTrajectoryAction)

        self.ctrl_state_dict = {}
        rospy.Subscriber(self.controller_name + '/state', JointTrajectoryControllerState, 
                         self._ctrl_state_cb)
        if timeout > 0 and not self.wait_for_ep(timeout):
            rospy.logwarn("[pr2_arm] Timed out waiting for EP.")
        
        if timeout > 0 and not self.joint_action_client.wait_for_server(rospy.Duration(timeout)):
            rospy.logwarn("[pr2_arm] JointTrajectoryAction action server timed out.")

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
    # Returns the current equilibrium point
    # @return equilibrium point
    def get_ep(self, wrapped=False):
        with self.lock:
            ret_ep = copy.copy(self.ep)
        if wrapped:
            return self.wrap_angles(ret_ep)
        else:
            return ret_ep
            
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

    # TODO Implementation int arm_pose_move_controller
    def is_arm_at_ep(self, err_thresh=None):
        if err_thresh is None:
            err_thresh = ARM_AT_EP_ERR_THRESH
        return True

    def interpolate_ep(self, ep_a, ep_b, t_vals):
        linspace_list = [[ep_a[i] + (ep_b[i] - ep_a[i]) * t for t in t_vals] for i in range(len(ep_a))]
        return np.dstack(linspace_list)[0]

    def reset_ep(self):
        pass
        #self.ep = self.get_joint_angles(False)

def extract_twist(msg):
    return np.array([msg.linear.x, msg.linear.y, msg.linear.z, 
                     msg.angular.x, msg.angular.y, msg.angular.z])

class PR2ArmCartesianBase(PR2Arm):
    def __init__(self, arm, kinematics, controller_name='/%s_cart', timeout=5.):
        super(PR2ArmCartesianBase, self).__init__(arm, kinematics, controller_name, timeout)
        self.command_pose_pub = rospy.Publisher(self.controller_name + '/command_pose', PoseStamped)

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

    def set_posture(self, posture=None):
        if posture is None:
            posture = self.get_joint_angles()
        assert len(posture) == 7, "Wrong posture length"
        msg = Float64MultiArray()
        msg.data = list(posture)
        self.command_posture_pub.publish(msg)

class PR2ArmJTranspose(PR2ArmCartesianPostureBase):
    def __init__(self, arm, kinematics, controller_name='/%s_cart', timeout=5.):
        super(PR2ArmJTranspose, self).__init__(arm, kinematics, controller_name, timeout)

        self.ctrl_state_dict = {}
        rospy.Subscriber(self.controller_name + '/state', JTCartesianControllerState, 
                         self._ctrl_state_cb)
        if timeout > 0 and not self.wait_for_ep(timeout):
            rospy.logwarn("[pr2_arm] Timed out waiting for EP.")

    def _ctrl_state_cb(self, ctrl_state):
        with self.lock:
            self.ep = PoseConverter.to_pos_rot(ctrl_state.x_desi_filtered)
        with self.ctrl_state_lock:
            self.ctrl_state_dict["frame"] = ctrl_state.header.frame_id
            self.ctrl_state_dict["x_desi"] = PoseConverter.to_pos_rot(ctrl_state.x_desi)
            self.ctrl_state_dict["xd_desi"] = extract_twist(ctrl_state.xd_desi)
            self.ctrl_state_dict["x_act"] = PoseConverter.to_pos_rot(ctrl_state.x)
            self.ctrl_state_dict["xd_act"] = extract_twist(ctrl_state.xd)
            self.ctrl_state_dict["x_desi_filt"] = PoseConverter.to_pos_rot(
                                                                ctrl_state.x_desi_filtered)
            self.ctrl_state_dict["x_err"] = extract_twist(ctrl_state.x_err)
            self.ctrl_state_dict["tau_pose"] = np.array(ctrl_state.tau_pose)
            self.ctrl_state_dict["tau_posture"] = np.array(ctrl_state.tau_posture)
            self.ctrl_state_dict["tau"] = np.array(ctrl_state.tau)
            self.ctrl_state_dict["F"] = np.array([ctrl_state.F.force.x, 
                                                  ctrl_state.F.force.y,
                                                  ctrl_state.F.force.z,
                                                  ctrl_state.F.torque.x,
                                                  ctrl_state.F.torque.y,
                                                  ctrl_state.F.torque.z])

class PR2ArmJInverse(PR2ArmCartesianPostureBase):
    pass

class PR2ArmJTransposeTask(PR2ArmCartesianPostureBase):
    def __init__(self, arm, kinematics, controller_name='/%s_cart', timeout=5.):
        super(PR2ArmJTransposeTask, self).__init__(arm, kinematics, controller_name, timeout)
        self.command_gains_pub = rospy.Publisher(self.controller_name + '/gains', CartesianGains)

        self.ctrl_state_dict = {}
        rospy.Subscriber(self.controller_name + '/state', JTTaskControllerState, 
                         self._ctrl_state_cb)
        if timeout > 0 and not self.wait_for_ep(timeout):
            rospy.logwarn("[pr2_arm] Timed out waiting for EP.")

    def _ctrl_state_cb(self, ctrl_state):
        with self.lock:
            self.ep = PoseConverter.to_pos_rot(ctrl_state.x_desi_filtered)
        with self.ctrl_state_lock:
            self.ctrl_state_dict["frame"] = ctrl_state.header.frame_id
            self.ctrl_state_dict["x_desi"] = PoseConverter.to_pos_rot(ctrl_state.x_desi)
            self.ctrl_state_dict["xd_desi"] = extract_twist(ctrl_state.xd_desi)
            self.ctrl_state_dict["x_act"] = PoseConverter.to_pos_rot(ctrl_state.x)
            self.ctrl_state_dict["xd_act"] = extract_twist(ctrl_state.xd)
            self.ctrl_state_dict["x_desi_filt"] = PoseConverter.to_pos_rot(
                                                                ctrl_state.x_desi_filtered)
            self.ctrl_state_dict["x_err"] = extract_twist(ctrl_state.x_err)
            self.ctrl_state_dict["tau_pose"] = np.array(ctrl_state.tau_pose)
            self.ctrl_state_dict["tau_posture"] = np.array(ctrl_state.tau_posture)
            self.ctrl_state_dict["tau"] = np.array(ctrl_state.tau)
            self.ctrl_state_dict["F"] = np.array([ctrl_state.F.force.x, 
                                                  ctrl_state.F.force.y,
                                                  ctrl_state.F.force.z,
                                                  ctrl_state.F.torque.x,
                                                  ctrl_state.F.torque.y,
                                                  ctrl_state.F.torque.z])

    def set_gains(self, p_gains, d_gains, frame=None):
        if frame is None:
            frame = self.kinematics.end_link
        all_gains = list(p_gains) + list(d_gains)
        gains_msg = CartesianGains(Header(0, rospy.Time.now(), frame),
                                   all_gains, [])
        self.command_gains_pub.publish(gains_msg)

