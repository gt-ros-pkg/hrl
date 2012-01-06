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

# Author: Advait Jain


import math
import numpy as np
import copy
import sys, time, os

import PyKDL as kdl
import create_IK_guess_dict as cgd

import roslib; roslib.load_manifest('epc_core')

import hrl_lib.transforms as tr
import hrl_lib.util as ut
import hrl_lib.kdl_utils as ku

#-------------- TF stuff ---------------
def link_tf_name(arm, link_number):
    if arm == 'right_arm':
        nm = 'r_arm'
    else:
        nm = 'l_arm'

    if link_number == 0:
        nm = 'torso_link'
    elif link_number == 7:
        nm = nm + '_ee'
    else:
        nm = nm + '_' + str(link_number)
    return nm


class M3HrlRobot():
    def __init__(self, end_effector_length):
        # create joint limit dicts
        self.joint_lim_dict = {}
        self.joint_lim_dict['right_arm'] = {'max': np.radians([ 120.00, 122.15, 77.5, 144., 122.,  45.,  45.]),
                                            'min': np.radians([ -47.61,  -20., -77.5,   0., -80., -45., -45.])}

        self.joint_lim_dict['left_arm'] = {'max': np.radians([ 120.00,   20.,  77.5, 144.,   80.,  45.,  45.]),
                                           'min': np.radians([ -47.61, -122.15, -77.5,   0., -122., -45., -45.])}

        end_effector_length += 0.0135 + 0.04318 # add wrist linkange and FT sensor lengths
        self.setup_kdl_mekabot(end_effector_length)
        q_guess_pkl_l = os.environ['HOME']+'/svn/gt-ros-pkg/hrl/equilibrium_point_control/epc_core/src/cody_arms/q_guess_left_dict.pkl'
        q_guess_pkl_r = os.environ['HOME']+'/svn/gt-ros-pkg/hrl/equilibrium_point_control/epc_core/src/cody_arms/q_guess_right_dict.pkl'

        self.q_guess_dict_left = ut.load_pickle(q_guess_pkl_l)
        self.q_guess_dict_right = ut.load_pickle(q_guess_pkl_r)

    # KDL joint array to meka joint list. (7->7)
    # arm - 'left_arm' or 'right_arm'  
    def kdl_angles_to_meka(self, arm, q_jnt_arr):
        if q_jnt_arr == None:
            return None

        q_rad = [0. for i in range(7)]
        q_rad[0] = -q_jnt_arr[0]
        q_rad[1] = -q_jnt_arr[1]
        q_rad[2] = -q_jnt_arr[2]
        q_rad[3] = -q_jnt_arr[3]
        q_rad[4] = -q_jnt_arr[4]
        q_rad[5] = -q_jnt_arr[5]
        q_rad[6] = -q_jnt_arr[6]
        return q_rad

    # meka joint list to KDL joint array (7->7)
    # arm - 'left_arm' or 'right_arm'
    def meka_angles_to_kdl(self,arm,q_list):
        if q_list == None:
            return None

        n_joints = len(q_list)
        q = kdl.JntArray(n_joints)
        q[0] = -q_list[0]
        q[1] = -q_list[1]
        q[2] = -q_list[2]
        if n_joints > 3:
            q[3] = -q_list[3]
        if n_joints == 7:
            q[4] = -q_list[4]
            q[5] = -q_list[5]
            q[6] = -q_list[6]
        return q

    def create_right_chain(self, end_effector_length):
        ch = kdl.Chain()
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,-0.18493,0.))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,-0.03175,0.))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.00635,0.,-0.27795))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,-0.27853))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.,0.,0.))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,-end_effector_length))))
        return ch

    def create_left_chain(self, end_effector_length):
        ch = kdl.Chain()
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.18493,0.))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.03175,0.))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.00635,0.,-0.27795))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,-0.27853))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.,0.,0.))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,-end_effector_length))))
        return ch

    def create_solvers(self, ch):
         fk = kdl.ChainFkSolverPos_recursive(ch)
         ik_v = kdl.ChainIkSolverVel_pinv(ch)
         ik_p = kdl.ChainIkSolverPos_NR(ch, fk, ik_v)
         jac = kdl.ChainJntToJacSolver(ch)
         return fk, ik_v, ik_p, jac

    def setup_kdl_mekabot(self, end_effector_length):
        #right arm
        ch = self.create_right_chain(end_effector_length)
        fk, ik_v, ik_p, jac = self.create_solvers(ch)

        kdl_rightarm = {}
        kdl_rightarm['chain'] = ch
        kdl_rightarm['nJnts'] = ch.getNrOfJoints()
        kdl_rightarm['fk_p'] = fk
        kdl_rightarm['ik_v'] = ik_v
        kdl_rightarm['ik_p'] = ik_p
        kdl_rightarm['jacobian_solver'] = jac

        #left arm
        kdl_leftarm = {}
        ch = self.create_left_chain(end_effector_length)
        fk, ik_v, ik_p, jac = self.create_solvers(ch)

        kdl_leftarm['chain'] = ch
        kdl_leftarm['nJnts'] = ch.getNrOfJoints()
        kdl_leftarm['fk_p'] = fk
        kdl_leftarm['ik_v'] = ik_v
        kdl_leftarm['ik_p'] = ik_p
        kdl_leftarm['jacobian_solver'] = jac

        #Add both chains to dictionary
        self.cody_kdl = {'right_arm':kdl_rightarm,'left_arm':kdl_leftarm}

    def FK_kdl(self, arm, q, link_number):
        fk_solver = self.cody_kdl[arm]['fk_p']
        endeffec_frame = kdl.Frame()
        kinematics_status = fk_solver.JntToCart(q, endeffec_frame,
                                                link_number)
        if kinematics_status >= 0:
#            print 'End effector transformation matrix:', endeffec_frame
            return endeffec_frame
        else:
            print 'Could not compute forward kinematics.'
            return None

    def Jac_kdl(self,arm,q):
        ''' returns the Jacobian, given the joint angles
        '''
        J_kdl = kdl.Jacobian(7)
        self.cody_kdl[arm]['jacobian_solver'].JntToJac(q,J_kdl)

        kdl_jac =  np.matrix([
            [J_kdl[0,0],J_kdl[0,1],J_kdl[0,2],J_kdl[0,3],J_kdl[0,4],J_kdl[0,5],J_kdl[0,6]],
            [J_kdl[1,0],J_kdl[1,1],J_kdl[1,2],J_kdl[1,3],J_kdl[1,4],J_kdl[1,5],J_kdl[1,6]],
            [J_kdl[2,0],J_kdl[2,1],J_kdl[2,2],J_kdl[2,3],J_kdl[2,4],J_kdl[2,5],J_kdl[2,6]],
            [J_kdl[3,0],J_kdl[3,1],J_kdl[3,2],J_kdl[3,3],J_kdl[3,4],J_kdl[3,5],J_kdl[3,6]],
            [J_kdl[4,0],J_kdl[4,1],J_kdl[4,2],J_kdl[4,3],J_kdl[4,4],J_kdl[4,5],J_kdl[4,6]],
            [J_kdl[5,0],J_kdl[5,1],J_kdl[5,2],J_kdl[5,3],J_kdl[5,4],J_kdl[5,5],J_kdl[5,6]],
            ])
        return kdl_jac
        
    def IK_kdl(self,arm,frame, q_init):
        ''' IK, returns jointArray (None if impossible)
            frame - desired frame of the end effector
            q_init - initial guess for the joint angles. (JntArray)
        '''
        nJnts = self.cody_kdl[arm]['nJnts']
        ik_solver = self.cody_kdl[arm]['ik_p']
        q = kdl.JntArray(nJnts)
        if ik_solver.CartToJnt(q_init,frame,q)>=0:
            for i in range(nJnts):
                q[i] = tr.angle_within_mod180(q[i])
            return q
        else:
            if arm == 'right_arm':
                ik_solver = self.cody_kdl[arm]['ik_p_nolim']
                if ik_solver.CartToJnt(q_init,frame,q)>=0:
                    for i in range(nJnts):
                        q[i] = tr.angle_within_mod180(q[i])
                    return q
            print 'Error: could not calculate inverse kinematics'
            return None

    def FK_rot(self, arm, q, link_number = 7):
        pos, rot = self.FK_all(arm, q, link_number)
        return rot

    # @param arm - 'left_arm' or 'right_arm'
    # @param q - list of 7 joint angles (RADIANs)
    # @param link_number - perform FK up to this link. (1-7)
    # @return 3x1 numpy matrix
    def FK(self, arm, q, link_number = 7):
        pos, rot = self.FK_all(arm, q, link_number)
        return pos

    def FK_all(self, arm, q, link_number = 7):
        q = self.meka_angles_to_kdl(arm, q)
        frame = self.FK_kdl(arm, q, link_number)
        pos = frame.p
        pos = ku.kdl_vec_to_np(pos)
        m = frame.M
        rot = ku.kdl_rot_to_np(m)
        return pos, rot

    def Jac(self,arm,q):
        ''' q - list of 7 joint angles (meka axes) in RADIANS.
            arm - 'right_arm' or 'left_arm'
            returns 6x7 numpy matrix.
        '''
        jntarr = self.meka_angles_to_kdl(arm,q)
        kdl_jac = self.Jac_kdl(arm,jntarr)
        meka_jac = -kdl_jac  # the kdl jacobian is the negative of meka jacobian (see kdl_angles_to_meka)
        return meka_jac

    ## compute Jacobian at point pos.
    # p is in the torso_lift_link coord frame.
    def Jacobian(self, arm, q, pos):
        chain = self.cody_kdl[arm]['chain']
        v_list = []
        w_list = []
        for i in range(7):
            p, rot = self.FK_all(arm, q, i)
            r = pos - p
            z_idx = chain.getSegment(i).getJoint().getType() - 1
            z = rot[:, z_idx]
            v_list.append(np.matrix(np.cross(z.A1, r.A1)).T)
            w_list.append(z)

        J = np.row_stack((np.column_stack(v_list), np.column_stack(w_list)))
        #J = -J # the kdl jacobian is the negative of meka jacobian (see kdl_angles_to_meka)
        J = self.Jac(arm, q)
        return J

    ##
    # Inverse Kinematics using KDL.
    # @param p - 3X1 numpy matrix.
    # @param rot - 3X3 numpy matrix. It transforms a  vector in the
    # end effector frame to the torso frame. (or it is the orientation
    # of the end effector wrt the torso)
    # @return list of 7 joint angles, or None if IK soln not found.
    def IK(self,arm,p,rot,q_guess=None):
        p_kdl = ku.np_vec_to_kdl(p)
        rot_kdl = ku.np_rot_to_kdl(rot)
        fr = kdl.Frame(rot_kdl,p_kdl)

        if q_guess == None:
            if arm == 'left_arm':
                q_guess = cgd.find_good_config(p,self.q_guess_dict_left)
            elif arm == 'right_arm':    
                q_guess = cgd.find_good_config(p,self.q_guess_dict_right)

        q_guess = self.meka_angles_to_kdl(arm,q_guess)

        q_res = self.IK_kdl(arm,fr,q_guess)
        q_res = self.kdl_angles_to_meka(arm,q_res)

        if self.within_joint_limits(arm,q_res):
            if arm == 'right_arm':  
                if q_res[1]<0.:
                    q_res[1] = math.radians(10.)
                    qg = self.meka_angles_to_kdl(arm,q_res)
                    q_res = self.IK_kdl(arm,fr,qg)
                    q_res = self.kdl_angles_to_meka(arm,q_res)
                if self.within_joint_limits(arm,q_res):
                    return q_res
                else:
                    return None
            else:
                return q_res
        else:
            return None

    ## clamp joint angles to their physical limits.
    # @param arm - 'left_arm' or 'right_arm'
    # @param q - list of 7 joint angles.
    # The joint limits for IK are larger that the physical limits.
    def clamp_to_joint_limits(self, arm, q, delta_list=[0.,0.,0.,0.,0.,0.,0.]):
        d = self.joint_lim_dict[arm]
        max_arr = d['max']
        min_arr = d['min']
        q_arr = np.array(q)
        d_arr = np.array(delta_list)
        return np.clip(q_arr, min_arr-d_arr, max_arr+d_arr)

    def within_joint_limits(self, arm, q, delta_list=[0.,0.,0.,0.,0.,0.,0.]):
        d = self.joint_lim_dict[arm]
        max_arr = d['max']
        min_arr = d['min']
        q_arr = np.array(q)
        d_arr = np.array(delta_list)
        return np.all((q_arr <= max_arr+d_arr, q_arr >= min_arr-d_arr))




