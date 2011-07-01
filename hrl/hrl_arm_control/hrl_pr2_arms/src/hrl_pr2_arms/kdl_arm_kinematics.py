#
# Provides a generic interface for arm kinematics given kdl parameters.
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
import PyKDL as kdl

from equilibrium_point_control.hrl_arm_template import HRLArmKinematics

class KDLArmParameters(object):
    joint_offsets = None
    rotation_axes = None
    arm_base_pos = np.mat([0.]*3).T
    arm_base_rot = np.eye(3)
    tool_pos = np.mat([0.]*3).T
    tool_rot = np.eye(3)

class KDLArmKinematics(HRLArmKinematics):
    def __init__(self, arm_params):
        super(KDLArmKinematics, self).__init__(len(arm_params.joint_offsets), 
                                               arm_params.arm_base_pos, 
                                               arm_params.arm_base_rot,
                                               arm_params.tool_pos, 
                                               arm_params.tool_rot)
        self.chain = self.create_chain(arm_params.joint_offsets, 
                                       arm_params.rotation_axes)
        self.fk_kdl = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_v_kdl = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_p_kdl = kdl.ChainIkSolverPos_NR(self.chain, self.fk_kdl, self.ik_v_kdl)
        self.jac_kdl = kdl.ChainJntToJacSolver(self.chain)

    def create_chain(self, joint_offsets, rotation_axes):
        assert len(joint_offsets) == len(rotation_axes)
        joint_types = [kdl.Joint.RotX, kdl.Joint.RotY, kdl.Joint.RotZ]
        ch = kdl.Chain()
        for i, row in enumerate(joint_offsets):
            ch.addSegment(kdl.Segment(kdl.Joint(joint_types[rotation_axes[i]]),
                                      kdl.Frame(kdl.Vector(*row))))
        return ch

    def FK_vanilla(self, q, link_number):
        endeffec_frame = kdl.Frame()
        kinematics_status = self.fk_kdl.JntToCart(self.joint_list_to_kdl(q), endeffec_frame,
                                              link_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            pos = np.mat([p.x(), p.y(), p.z()]).T
            M = endeffec_frame.M
            rot = np.mat([[M[0,0], M[0,1], M[0,2]], 
                          [M[1,0], M[1,1], M[1,2]], 
                          [M[2,0], M[2,1], M[2,2]]])
            return pos, rot
        else:
            return None, None

    def IK_vanilla(self, pos, rot, q_guess=None):
        pos_kdl = kdl.Vector(pos[0,0], pos[1,0], pos[2,0])
        rot_kdl = kdl.Rotation(rot[0,0], rot[0,1], rot[0,2],
                               rot[1,0], rot[1,1], rot[1,2],
                               rot[2,0], rot[2,1], rot[2,2])
        frame_kdl = kdl.Frame(rot_kdl, pos_kdl)

        if q_guess == None:
            q_guess = [0.] * self.n_jts

        q_kdl = kdl.JntArray(self.n_jts)
        q_guess_kdl = self.joint_list_to_kdl(q_guess)
        if self.ik_p_kdl.CartToJnt(q_guess_kdl, frame_kdl, q_kdl) >= 0:
            return self.joint_kdl_to_list(q_kdl)
        else:
            return None

    def jacobian_vanilla(self, q, pos=None):
        j_kdl = kdl.Jacobian(self.n_jts)
        q_kdl = self.joint_list_to_kdl(q)
        self.jac_kdl.JntToJac(q_kdl, j_kdl)
        j_mat =  np.zeros((6, self.n_jts))
        for i in range(6):
            for j in range(self.n_jts):
                j_mat[i,j] = j_kdl[i,j]
        return j_mat

    def joint_kdl_to_list(self, q):
        if q == None:
            return None
        return [q[i] for i in range(self.n_jts)]

    def joint_list_to_kdl(self, q):
        if q is None:
            return None
        q_kdl = kdl.JntArray(len(q))
        for i, q_i in enumerate(q):
            q_kdl[i] = q_i
        return q_kdl
