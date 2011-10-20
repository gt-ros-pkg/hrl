#! /usr/bin/python

import copy
import numpy as np
import roslib
roslib.load_manifest('hrl_pr2_arms')
import rospy
import actionlib

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
#from teleop_controllers.msg import CartesianGains
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
    def __init__(self, arm, kinematics):
        super(PR2Arm, self).__init__(kinematics)
        self.arm = arm

        rospy.Subscriber('joint_states', JointState, self.joint_state_cb)

        self.joint_names_list = []
        for s in JOINT_NAMES_LIST:
            self.joint_names_list.append(arm + s)
        self.joint_state_inds = None
        self.joint_angles = None

    ##
    # Joint angles listener callback
    def joint_state_cb(self, msg):
        with self.lock:
            if self.joint_state_inds is None:
                self.joint_state_inds = [msg.name.index(joint_name) for joint_name in self.joint_names_list]
            self.joint_angles = [msg.position[i] for i in self.joint_state_inds]

    ##
    # Returns the current joint angle positions
    # @param wrapped If False returns the raw encoded positions, if True returns
    #                the angles with the forearm and wrist roll in the range -pi to pi
    def get_joint_angles(self, wrapped=False):
        with self.lock:
            if self.joint_angles is None:
                rospy.logerr("[pr2_arm_base] Joint angles haven't been filled yet")
                return None
            if wrapped:
                return self.wrap_angles(self.joint_angles)
            else:
                return np.array(self.joint_angles)

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
                   end_link="%s_gripper_tool_frame", param="/robot_description"):
    if "%s" in base_link:
        base_link = base_link % arm
    if "%s" in end_link:
        end_link = end_link % arm
    chain, joint_info = kdlp.chain_from_param(base_link, end_link, param=param)
    kin = KDLArmKinematics(chain, joint_info)
    return arm_type(arm, kin)

class PR2ArmJointTrajectory(PR2Arm):
    def __init__(self, arm, kinematics):
        super(PR2ArmJointTrajectory, self).__init__(arm, kinematics)
        self.joint_action_client = actionlib.SimpleActionClient(
                                       arm + '_arm_controller/joint_trajectory_action',
                                       JointTrajectoryAction)
        rospy.sleep(1)

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
        self.ep = copy.copy(jep)

    def interpolate_ep(self, ep_a, ep_b, num_steps):
        linspace_list = [np.linspace(ep_a[i], ep_b[i], num_steps) for i in range(len(ep_a))]
        return np.dstack(linspace_list)[0]

    def reset_ep(self):
        self.ep = self.get_joint_angles(True)

class PR2ArmCartesianBase(PR2Arm):
    def __init__(self, arm, kinematics):
        super(PR2ArmCartesianBase, self).__init__(arm, kinematics)
        self.command_pose_pub = rospy.Publisher('/' + arm + '_cart/command_pose', PoseStamped)
        rospy.sleep(1)

    def set_ep(self, cep, duration, delay=0.0):
        cep_pose_stmp = PoseConverter.to_pose_stamped_msg('torso_lift_link', cep)
        cep_pose_stmp.header.stamp = rospy.Time.now()
        self.command_pose_pub.publish(cep_pose_stmp)
        self.ep = copy.deepcopy(cep)

    ##
    # Returns pairs of positions and rotations linearly interpolated between
    # the start and end position/orientations.  Rotations are found using slerp
    # @return List of (pos, rot) waypoints between start and end.
    def interpolate_ep(self, ep_a, ep_b, t_vals):
        pos_a, rot_a = ep_a
        pos_b, rot_b = ep_b
        num_samps = len(t_vals)
        pos_waypoints = pos_a + np.tile(pos_b - pos_a, (1, num_samps)) * t_vals
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
        self.ep = self.kinematics.FK(self.get_joint_angles(True))

    def ep_error(self, ep_actual, ep_desired):
        pos_act, rot_act = ep_actual
        pos_des, rot_des = ep_desired
        err = np.mat(np.zeros((6, 1)))
        err[:3,0] = pos_act - pos_des
        err[3:6,0] = np.mat(tf_trans.euler_from_matrix(rot_des.T * rot_act)).T
        return err

class PR2ArmJTranspose(PR2ArmCartesianBase):
    pass

class PR2ArmJInverse(PR2ArmCartesianBase):
    pass

class PR2ArmJTransposeTask(PR2ArmCartesianBase):
    def __init__(self, arm, kinematics):
        super(PR2ArmJTransposeTask, self).__init__(arm, kinematics)
        self.command_gains_pub = rospy.Publisher('/' + arm + '_cart/gains', CartesianGains)
        rospy.sleep(1)

    def set_gains(self, p_gains, d_gains, frame='torso_lift_link'):
        all_gains = list(p_gains) + list(d_gains)
        gains_msg = CartesianGains(Header(0, rospy.Time.now(), frame),
                                   all_gains, [])
        self.command_gains_pub.publish(gains_msg)

