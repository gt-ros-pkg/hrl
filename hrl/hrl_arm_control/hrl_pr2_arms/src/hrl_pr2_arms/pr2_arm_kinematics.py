#! /usr/bin/python
#
# subscribe to thw joint angles and raw forces topics,  and provide FK
# etc.
#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Author: Kelsey Hawkins and Advait Jain

import numpy as np

import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy

from hrl_pr2_arms.kdl_arm_kinematics import KDLArmKinematics, KDLArmParameters

class PR2ArmKinematics(KDLArmKinematics):

    class PR2RightArmParameters(KDLArmParameters):
        joint_offsets = [[0.1, 0., 0.],
                         [0., 0., 0.],
                         [0.4, 0., 0.],
                         [0., 0., 0.],
                         [0.321, 0., 0.],
                         [0., 0., 0.],
                         [0., 0., 0.]]
        rotation_axes = [2, 1, 0, 1, 0, 1, 0]
        arm_base_pos = np.mat([0., -0.188, 0.]).T
        tool_pos = np.mat([0.18, 0., 0.]).T

    class PR2LeftArmParameters(KDLArmParameters):
        joint_offsets = [[0.1, 0., 0.],
                         [0., 0., 0.],
                         [0.4, 0., 0.],
                         [0., 0., 0.],
                         [0.321, 0., 0.],
                         [0., 0., 0.],
                         [0., 0., 0.]]
        rotation_axes = [2, 1, 0, 1, 0, 1, 0]
        arm_base_pos = np.mat([0., -0.188, 0.]).T
        tool_pos = np.mat([0.18, 0., 0.]).T

    def __init__(self, arm):
        assert arm in ['r', 'l']
        if arm == 'r':
            arm_params = PR2ArmKinematics.PR2RightArmParameters()
        else:
            arm_params = PR2ArmKinematics.PR2LeftArmParameters()
            assert False, "left not implemented yet"
        super(PR2ArmKinematics, self).__init__(arm_params)

def main():
    rospy.init_node('pr2_arm_kinematics_test')

    r_kinematics = PR2ArmKinematics('r')
    print r_kinematics.FK([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    print r_kinematics.IK(np.mat([0.1, 0.1, 0.1]).T, np.eye(3))
    print r_kinematics.jacobian([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    l_kinematics = PR2ArmKinematics('l')
    return
            
if __name__ == '__main__':
    main()

