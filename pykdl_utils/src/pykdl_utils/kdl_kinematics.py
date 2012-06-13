#!/usr/bin/env python
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

# Author: Kelsey Hawkins
import numpy as np

import roslib
roslib.load_manifest("pykdl_utils")

import PyKDL as kdl
import rospy
import tf.transformations as tf_trans

import urdf_parser_python.urdf_parser as urdf
from hrl_generic_arms.hrl_arm_template import HRLArmKinematics
from hrl_generic_arms.pose_converter import PoseConverter
import kdl_parser_python.kdl_parser as kdlp

def kin_from_param(base_link, end_link, param="/robot_description"):
    chain, joint_info = kdlp.chain_from_param(base_link, end_link, param=param)
    return KDLKinematics(chain, joint_info, base_link, end_link)

def kin_from_file(base_link, end_link, filename):
    chain, joint_info = kdlp.chain_from_file(base_link, end_link, filename)
    return KDLKinematics(chain, joint_info, base_link, end_link)

class KDLKinematics(object):
    def __init__(self, chain, joint_info, base_link="", end_link=""):
        self._chain = chain
        self.joint_info = joint_info
        self.base_link = base_link
        self.end_link = end_link

        self._fk_kdl = kdl.ChainFkSolverPos_recursive(self._chain)
        self._ik_v_kdl = kdl.ChainIkSolverVel_pinv(self._chain)
        mins_kdl = joint_list_to_kdl(joint_info["safe_mins"])
        maxs_kdl = joint_list_to_kdl(joint_info["safe_maxs"])
        self._ik_p_kdl = kdl.ChainIkSolverPos_NR_JL(self._chain, mins_kdl, maxs_kdl, 
                                                   self._fk_kdl, self._ik_v_kdl)
        self._jac_kdl = kdl.ChainJntToJacSolver(self._chain)
        self._dyn_kdl = kdl.ChainDynParam(self._chain, kdl.Vector.Zero())

    def get_segment_names(self):
        return [self._chain.getSegment(i).getName() for i in range(self._chain.getNrOfSegments())]

    def get_joint_names(self):
        return self.joint_info['names']

    def forward(self, q, target_segment=None, base_segment=None):
        seg_names = self.get_segment_names()
        if target_segment is None:
            target_link = self._chain.getNrOfSegments()
        else:
            if target_segment in seg_names:
                target_link = seg_names.index(target_segment)
            else:
                print "Target segment %s not in KDL chain" % target_segment
                return None
        if base_segment is None:
            base_link = 0
        else:
            if base_segment in seg_names:
                base_link = seg_names.index(base_segment)
            else:
                print "Base segment %s not in KDL chain" % base_segment
                return None
        base_trans = self._do_kdl_fk(q, base_link)
        if base_trans is None:
            print "FK KDL failure on base transformation."
        target_trans = self._do_kdl_fk(q, target_link)
        if target_trans is None:
            print "FK KDL failure on target transformation."
        return base_trans**-1 * target_trans

    def _do_kdl_fk(self, q, link_number):
        endeffec_frame = kdl.Frame()
        kinematics_status = self._fk_kdl.JntToCart(joint_list_to_kdl(q), endeffec_frame,
                                                  link_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            return np.mat([[M[0,0], M[0,1], M[0,2], p.x()], 
                           [M[1,0], M[1,1], M[1,2], p.y()], 
                           [M[2,0], M[2,1], M[2,2], p.z()],
                           [     0,      0,      0,     1]])
        else:
            return None

    def inverse(self, pose, q_guess=None):
        pos, rot = PoseConverter.to_pos_rot(pose)
        pos_kdl = kdl.Vector(pos[0,0], pos[1,0], pos[2,0])
        rot_kdl = kdl.Rotation(rot[0,0], rot[0,1], rot[0,2],
                               rot[1,0], rot[1,1], rot[1,2],
                               rot[2,0], rot[2,1], rot[2,2])
        frame_kdl = kdl.Frame(rot_kdl, pos_kdl)

        if q_guess == None:
            q_guess = [0.] * self.n_jts

        q_kdl = kdl.JntArray(self.n_jts)
        q_guess_kdl = joint_list_to_kdl(q_guess)
        if self._ik_p_kdl.CartToJnt(q_guess_kdl, frame_kdl, q_kdl) >= 0:
            #return self.normalize_angles(joint_kdl_to_list(q_kdl))
            return np.array(joint_kdl_to_list(q_kdl))
        else:
            return None

    def jacobian(self, q, pos=None):
        j_kdl = kdl.Jacobian(self.n_jts)
        q_kdl = joint_list_to_kdl(q)
        self._jac_kdl.JntToJac(q_kdl, j_kdl)
        return kdl_to_mat(j_kdl)

    def inertia(self, q):
        h_kdl = kdl.JntSpaceInertiaMatrix(self.n_jts)
        self._dyn_kdl.JntToMass(joint_list_to_kdl(q), h_kdl)
        return kdl_to_mat(h_kdl)

    def cart_inertia(self, q):
        H = self.inertia(q)
        J = self.jacobian(q)
        return np.linalg.inv(J * np.linalg.inv(H) * J.T)

    def normalize_angles(self, q):
        min_lims = self.joint_info["safe_mins"]
        max_lims = self.joint_info["safe_maxs"]
        q_mod = np.mod(q, 2 * np.pi)
        in_lims_a = np.all([q_mod >= min_lims, q_mod <= max_lims], 0)
        in_lims_b = np.all([(q_mod - 2*np.pi) >= min_lims, (q_mod - 2*np.pi) <= max_lims], 0)
        if np.all(np.any([in_lims_a, in_lims_b],0)):
            return np.where(in_lims_a, q_mod, q_mod - 2 * np.pi)
        return None

    def angles_in_limits(self, q):
        min_lims = self.joint_info["lim_mins"]
        max_lims = self.joint_info["lim_maxs"]
        return np.all([q >= min_lims, q <= max_lims], 0)

    def angles_in_safeties(self, q):
        min_lims = self.joint_info["safe_mins"]
        max_lims = self.joint_info["safe_maxs"]
        return np.all([q >= min_lims, q <= max_lims], 0)

    def random_joint_angles(self):
        min_lims = self.joint_info["safe_mins"]
        max_lims = self.joint_info["safe_maxs"]
        contins = np.array(self.joint_info["types"]) == urdf.Joint.CONTINUOUS
        min_lims = np.where(contins, -np.pi, min_lims)
        max_lims = np.where(contins, np.pi, max_lims)
        zip_lims = zip(min_lims, max_lims)
        return np.array([np.random.uniform(min_lim, max_lim) for min_lim, max_lim in zip_lims])

    def IK_search(self, pos, rot, timeout=1.):
        st_time = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - st_time < timeout:
            q_init = self.random_joint_angles()
            q_ik = self.IK_vanilla(pos, rot, q_init)
            if q_ik is not None:
                return q_ik
        return None

    def IK_biased(self, pos, rot, q_init, q_bias, q_bias_weights, rot_weight=1., 
                  bias_vel=0.01, num_iter=100):
        q_out = np.mat(self.IK_search(pos, rot)).T
        for i in range(num_iter):
            pos_fk, rot_fk = self.FK_vanilla(q_out)
            delta_twist = np.mat(np.zeros((6, 1)))
            pos_delta = pos - pos_fk
            delta_twist[:3,0] = pos_delta
            rot_delta = np.mat(np.eye(4))
            rot_delta[:3,:3] = rot * rot_fk.T
            rot_delta_angles = np.mat(tf_trans.euler_from_matrix(rot_delta)).T
            delta_twist[3:6,0] = rot_delta_angles
            J = self.jacobian_vanilla(q_out)
            J[3:6,:] *= np.sqrt(rot_weight)
            delta_twist[3:6,0] *= np.sqrt(rot_weight)
            J_tinv = np.linalg.inv(J.T * J + np.diag(q_bias_weights) * np.eye(len(q_init))) * J.T
            q_bias_diff = q_bias - q_out
            q_bias_diff_normed = q_bias_diff * bias_vel / np.linalg.norm(q_bias_diff)
            delta_q = q_bias_diff_normed + J_tinv * (delta_twist - J * q_bias_diff_normed)
            q_out += delta_q 
            q_out = np.mat(np.clip(q_out.T.A[0], self.joint_info["safe_mins"], 
                                                 self.joint_info["safe_maxs"])).T
        return q_out

    def IK_biased_search(self, pos, rot, q_bias, q_bias_weights, rot_weight=1., 
                         bias_vel=0.01, num_iter=100, num_search=20):
        q_sol_min = []
        min_val = 1000000.
        for i in range(num_search):
            q_init = self.random_joint_angles()
            q_sol = self.IK_biased(pos, rot, q_init, q_bias, q_bias_weights, rot_weight=1., 
                                   bias_vel=0.01, num_iter=100)
            cur_val = np.linalg.norm(np.diag(q_bias_weights) * (q_sol - q_bias)) 
            if cur_val < min_val:
                min_val = cur_val
                q_sol_min = q_sol
        return q_sol_min
            

        
def kdl_to_mat(m):
    mat =  np.mat(np.zeros((m.rows(), m.columns())))
    for i in range(m.rows()):
        for j in range(m.columns()):
            mat[i,j] = m[i,j]
    return mat

def joint_kdl_to_list(q):
    if q == None:
        return None
    return [q[i] for i in range(q.rows())]

def joint_list_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl

def main():
    kin = kin_from_param("openni_rgb_optical_frame", "r_gripper_tool_frame")
    q = kin.random_joint_angles()
    print kin.get_segment_names()
    backwards = kin.forward(q, 'openni_rgb_frame', 'r_gripper_tool_frame') 
    forwards = kin.forward(q, 'r_gripper_tool_frame', 'openni_rgb_frame')
    print backwards
    print forwards
    print backwards * forwards

if __name__ == "__main__":
    main()
