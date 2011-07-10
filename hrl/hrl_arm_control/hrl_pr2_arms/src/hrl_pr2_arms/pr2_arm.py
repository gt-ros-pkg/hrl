#! /usr/bin/python

import copy
import numpy as np
import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy
import actionlib

from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import tf.transformations as tf_trans

from equilibrium_point_control.hrl_arm_template import HRLArm


JOINT_NAMES_LIST = ['_shoulder_pan_joint',
                    '_shoulder_lift_joint', '_upper_arm_roll_joint',
                    '_elbow_flex_joint', '_forearm_roll_joint',
                    '_wrist_flex_joint', '_wrist_roll_joint']
JOINT_STATE_INDS_R = [17, 18, 16, 20, 19, 21, 22]
JOINT_STATE_INDS_L = [29, 30, 28, 32, 31, 33, 34]

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
        if arm == 'r':
            self.JOINT_STATE_INDS = JOINT_STATE_INDS_R
        else:
            self.JOINT_STATE_INDS = JOINT_STATE_INDS_L
        self.joint_angles = None

    ##
    # Joint angles listener callback
    def joint_state_cb(self, msg):
        with self.lock:
            self.joint_angles = [msg.position[i] for i in self.JOINT_STATE_INDS]

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

##
# Returns pairs of positions and rotations linearly interpolated between
# the start and end position/orientations.  Rotations are found using slerp
# @return List of (pos, rot) waypoints between start and end.
def interpolate_linear(start_pos, start_rot, end_pos, end_rot, num_steps):
    pos_waypoints = np.dstack([np.linspace(start_pos[0], end_pos[0], num_steps), 
                               np.linspace(start_pos[1], end_pos[1], num_steps), 
                               np.linspace(start_pos[2], end_pos[2], num_steps)])[0]
    start_quat = tf_trans.matrix_to_quaternion(start_rot)
    end_quat = tf_trans.matrix_to_quaternion(end_rot)
    rot_waypoints = []
    for fraction in np.linspace(0, 1, num_steps):
        cur_quat = tf_trans.quaternion_slerp(start_quat, end_quat, fraction)
        rot_waypoints.append(tf_trans.quaternion_to_matrix(cur_quat))
    return zip(pos_waypoints, rot_waypoints)

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


class PR2ArmJTranspose(PR2Arm):
    def __init__(self):
        pass

    def r_cart_state_cb(self, msg):
        pass

    def set_ep(self, p, rot):
        pass

class PR2ArmJInverse(PR2Arm):
    def __init__(self):
        pass


