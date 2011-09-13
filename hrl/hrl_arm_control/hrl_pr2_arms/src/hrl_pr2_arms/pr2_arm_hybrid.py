#! /usr/bin/python

import copy
import numpy as np
import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('hrl_netft')
import rospy
import actionlib
from std_msgs.msg import Float64MultiArray, Header, MultiArrayDimension

from hrl_pr2_arms.pr2_arm import PR2ArmCartesianBase
from hrl_netft.msg import HybridCartesianGains

class PR2ArmHybridForce(PR2ArmCartesianBase):
    def __init__(self, arm, kinematics):
        super(PR2ArmHybridForce, self).__init__(arm, kinematics)
        self.command_gains_pub = rospy.Publisher('/' + arm + '_cart/gains', HybridCartesianGains)
        self.command_force_pub = rospy.Publisher('/' + arm + '_cart/command_force', Float64MultiArray)
        rospy.sleep(1)

    def set_gains(self, p_gains, d_gains, fp_gains, fd_gains, force_selector, frame='%s_gripper_tool_frame'):
        if '%s' in frame:
            frame = frame % self.arm
        all_gains = list(p_gains) + list(d_gains)
        all_fgains = list(fp_gains) + list(fd_gains)
        gains_msg = HybridCartesianGains(Header(0, rospy.Time.now(), frame),
                                         all_gains, all_fgains, force_selector)
        self.command_gains_pub.publish(gains_msg)

    def set_force(self, f):
        f_msg = Float64MultiArray()
        dim = MultiArrayDimension("force", 6, 1)
        f_msg.layout.dim.append(dim)
        f_msg.data = f
        self.command_force_pub.publish(f_msg)


