#! /usr/bin/python

import copy
import numpy as np
import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('hrl_netft')
import rospy
import actionlib
from std_msgs.msg import Float64MultiArray, Header, MultiArrayDimension, Bool

from hrl_pr2_arms.pr2_arm import PR2ArmCartesianBase
from hrl_netft.msg import HybridCartesianGains

class PR2ArmHybridForce(PR2ArmCartesianBase):
    def __init__(self, arm, kinematics, controller_name='/%s_cart'):
        super(PR2ArmHybridForce, self).__init__(arm, kinematics)
        if '%s' in controller_name:
            controller_name = controller_name % arm
        self.command_gains_pub = rospy.Publisher(controller_name + '/gains', HybridCartesianGains)
        self.command_force_pub = rospy.Publisher(controller_name + '/command_force', Float64MultiArray)
        self.command_zero_pub = rospy.Publisher(controller_name + '/ft_zero', Bool)
        self.tip_frame = rospy.get_param(controller_name + '/tip_name')
        self.force_selector = 6 * [0]
        self.trans_p_motion_gains = 3 * [rospy.get_param(controller_name + '/cart_gains/trans/p')]
        self.trans_d_motion_gains = 3 * [rospy.get_param(controller_name + '/cart_gains/trans/d')]
        self.rot_p_motion_gains = 3 * [rospy.get_param(controller_name + '/cart_gains/rot/p')]
        self.rot_d_motion_gains = 3 * [rospy.get_param(controller_name + '/cart_gains/rot/d')]
        self.trans_p_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/trans/p')]
        self.trans_d_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/trans/d')]
        self.rot_p_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/rot/p')]
        self.rot_d_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/rot/d')]
        rospy.sleep(1)

    def set_motion_gains(self, p_trans=None, p_rot=None, d_trans=None, d_rot=None):
        local_names = ['trans_p_motion_gains', 'trans_d_motion_gains',
                       'rot_p_motion_gains', 'rot_d_motion_gains']
        vals = [p_trans, p_rot, d_trans, d_rot]
        for local_name, val in zip(local_names, vals):
            self._set_gain(local_name, val)

    def set_force_gains(self, p_trans=None, p_rot=None, d_trans=None, d_rot=None):
        local_names = ['trans_p_force_gains', 'trans_d_force_gains',
                       'rot_p_force_gains', 'rot_d_force_gains']
        vals = [p_trans, p_rot, d_trans, d_rot]
        for local_name, val in zip(local_names, vals):
            self._set_gain(local_name, val)

    def _set_gain(self, local_name, val):
        if val is not None:
            try:
                if len(val) == 3:
                    self.__dict__[local_name] = list(val)
                else:
                    raise Exception()
            except Exception as e:
                if type(val) in [int, float]:
                    self.__dict__[local_name] = 3 * [float(val)]
                else:
                    rospy.logerr("Bad gain parameter (must be single value or array-like of 3)")

    def set_tip_frame(self, tip_frame):
        self.tip_frame = tip_frame

    def set_force_directions(self, directions):
        if len(directions) == 6 and all([direction in [0, 1] for direction in directions]):
            self.force_selector = list(directions)
            return
        self.force_selector = 6 * [0]
        names = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        for direction in directions:
            if direction in names:
                self.force_selector[names.index(direction)] = 1

    def set_mass_params(self, mass, center_of_mass=None):
        params_msg = Float64MultiArray()
        params_msg.data = [mass]
        if center_of_mass is not None:
            params_msg.data += list(center_of_mass)

    def update_gains(self):
        motion_gains = (self.trans_p_motion_gains + self.rot_p_motion_gains + 
                        self.trans_d_motion_gains + self.rot_d_motion_gains)
        force_gains = (self.trans_p_force_gains + self.rot_p_force_gains + 
                       self.trans_d_force_gains + self.rot_d_force_gains)
        gains_msg = HybridCartesianGains(Header(0, rospy.Time.now(), self.tip_frame),
                                         motion_gains, force_gains, self.force_selector)
        self.command_gains_pub.publish(gains_msg)

    def set_force(self, f):
        f_msg = Float64MultiArray()
        dim = MultiArrayDimension("force", 6, 1)
        f_msg.layout.dim.append(dim)
        f_msg.data = f
        self.command_force_pub.publish(f_msg)

    def zero_sensor(self):
        self.command_zero_pub(Bool())
