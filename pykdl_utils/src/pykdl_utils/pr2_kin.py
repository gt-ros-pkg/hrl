#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest('pykdl_utils')

import rospy
from sensor_msgs.msg import JointState

import urdf_parser_python.urdf_parser as urdf
from kdl_kinematics import KDLKinematics
import kdl_parser_python.kdl_parser as kdlp

def kin_from_param(base_link, end_link, param="/robot_description", timeout=5.):
    chain, joint_info = kdlp.chain_from_param(base_link, end_link, param=param)
    return PR2Kinematics(chain, joint_info, base_link, end_link, timeout)

def kin_from_file(base_link, end_link, filename, timeout=5.):
    chain, joint_info = kdlp.chain_from_file(base_link, end_link, filename)
    return PR2Kinematics(chain, joint_info, base_link, end_link, timeout)

class PR2Kinematics(KDLKinematics):
    def __init__(self, chain, joint_info, base_link="", end_link="", timeout=5.):
        super(PR2Kinematics, self).__init__(chain, joint_info, base_link, end_link)
        self._joint_angles = None
        self._joint_efforts = None
        self._joint_state_inds = None

        rospy.Subscriber('/joint_states', JointState, self._joint_state_cb)

        if timeout > 0:
            self.wait_for_joint_angles(timeout)

    ##
    # Joint angles listener callback
    def _joint_state_cb(self, msg):
        if self._joint_state_inds is None:
            joint_names_list = self.get_joint_names()
            self._joint_state_inds = [msg.name.index(joint_name) for 
                                     joint_name in joint_names_list]
        self._joint_angles = [msg.position[i] for i in self._joint_state_inds]
        self._joint_efforts = [msg.effort[i] for i in self._joint_state_inds]

    ##
    # Wait until we have found the current joint angles.
    # @param timeout Time at which we break if we haven't recieved the angles.
    def wait_for_joint_angles(self, timeout=5.):
        start_time = rospy.get_time()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() - start_time < timeout:
            if self._joint_angles is not None:
                return True
            r.sleep()
        if not rospy.is_shutdown():
            rospy.logwarn("[pr2_kin] Cannot read joint angles, timing out.")
        return False

    ##
    # Returns the current joint angle positions
    # @param wrapped If False returns the raw encoded positions, if True returns
    #                the angles with the forearm and wrist roll in the range -pi to pi
    def get_joint_angles(self, wrapped=False):
        if self._joint_angles is None:
            rospy.logwarn("[pr2_kin] Joint angles haven't been filled yet.")
            return None
        if wrapped:
            return self.wrap_angles(self._joint_angles)
        else:
            return np.array(self._joint_angles)

    def wrap_angles(self, q):
        contins = np.array(self.joint_info["types"]) == urdf.Joint.CONTINUOUS
        return np.where(contins, np.mod(q, 2 * np.pi), q)

    def forward_filled(self, target_segment=None, base_segment=None):
        q = self.get_joint_angles()
        if q is None:
            return None
        return self.forward(q, target_segment, base_segment)

    def get_joint_efforts(self):
        if self._joint_efforts is None:
            rospy.logwarn("[pr2_kin] Joint efforts haven't been filled yet.")
            return None
        return np.array(self._joint_efforts)

