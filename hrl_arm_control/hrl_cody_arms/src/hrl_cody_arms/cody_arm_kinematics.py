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


import numpy as np, math
#np.set_printoptions(precision=4, linewidth=80)

import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('hrl_cody_arms')
import PyKDL as kdl

from equilibrium_point_control.hrl_arm import HRLArmKinematics
import create_IK_guess_dict as cgd

import hrl_lib.transforms as tr
import hrl_lib.util as ut
import hrl_lib.kdl_utils as ku


class CodyArmKinematics(HRLArmKinematics):
    # arm - 'r' or 'l'
    def __init__(self, arm):
        HRLArmKinematics.__init__(self, n_jts = 7)

        # create joint limit dicts
        if arm == 'r':
            max_lim = np.radians([ 120.00, 122.15, 77.5, 144., 122.,  45.,  45.])
            min_lim = np.radians([ -47.61,  -20., -77.5,   0., -80., -45., -45.])
        else:
            max_lim = np.radians([ 120.00,   20.,  77.5, 144.,   80.,  45.,  45.])
            min_lim = np.radians([ -47.61, -122.15, -77.5,   0., -122., -45., -45.])

        self.joint_lim_dict = {}
        self.joint_lim_dict['max'] = max_lim
        self.joint_lim_dict['min'] = min_lim

        wrist_stub_length = 0.0135 + 0.04318 # wrist linkange and FT sensor lengths
        self.setup_kdl_chains(arm, wrist_stub_length)

        if arm == 'r':
            pkl_nm = 'q_guess_right_dict.pkl'
        else:
            pkl_nm = 'q_guess_left_dict.pkl'

        pth = roslib.rospack.rospackexec(['find', 'hrl_cody_arms'])
        q_guess_pkl = pth + '/src/hrl_cody_arms/'+pkl_nm
        self.q_guess_dict = ut.load_pickle(q_guess_pkl)

    #--------------- KDL stuff ----------------
    # KDL joint array to meka joint list. (7->7)
    def kdl_angles_to_meka(self, q_jnt_arr):
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
    def meka_angles_to_kdl(self, q_list):
        if q_list == None:
            return None

        n_joints = len(q_list)
        q = kdl.JntArray(n_joints)
        q[0] = -q_list[0]
        q[1] = -q_list[1]
        q[2] = -q_list[2]
        q[3] = -q_list[3]
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

    def setup_kdl_chains(self, arm, end_effector_length):
        if arm == 'r':
            #right arm
            ch = self.create_right_chain(end_effector_length)
            fk, ik_v, ik_p, jac = self.create_solvers(ch)
        else:
            #left arm
            ch = self.create_left_chain(end_effector_length)
            fk, ik_v, ik_p, jac = self.create_solvers(ch)

        kdl_chains = {}
        kdl_chains['chain'] = ch
        kdl_chains['nJnts'] = ch.getNrOfJoints()
        kdl_chains['fk_p'] = fk
        kdl_chains['ik_v'] = ik_v
        kdl_chains['ik_p'] = ik_p
        kdl_chains['jacobian_solver'] = jac

        #Add both chains to dictionary
        self.kdl_chains = kdl_chains

    def FK_kdl(self, q, link_number):
        fk_solver = self.kdl_chains['fk_p']
        endeffec_frame = kdl.Frame()
        kinematics_status = fk_solver.JntToCart(q, endeffec_frame,
                                                link_number)
        if kinematics_status >= 0:
#            print 'End effector transformation matrix:', endeffec_frame
            return endeffec_frame
        else:
            print 'Could not compute forward kinematics.'
            return None

    def IK_kdl(self,frame, q_init):
        nJnts = self.kdl_chains['nJnts']
        ik_solver = self.kdl_chains['ik_p']
        q = kdl.JntArray(nJnts)
        if ik_solver.CartToJnt(q_init,frame,q)>=0:
            for i in range(nJnts):
                q[i] = tr.angle_within_mod180(q[i])
            return q
        else:
            return None

    #-------------- implementation of HRLArmKinematics -----------

    def FK_vanilla(self, q, link_number = 7):
        q = self.meka_angles_to_kdl(q)
        frame = self.FK_kdl(q, link_number)
        pos = frame.p
        pos = ku.kdl_vec_to_np(pos)
        m = frame.M
        rot = ku.kdl_rot_to_np(m)
        return pos, rot

    ## compute Jacobian at point pos. 
    # p is in the ground coord frame.
    def Jacobian(self, q, pos=None):
        if pos == None:
            pos = self.FK(q)[0]

        ch = self.kdl_chains['chain']
        v_list = []
        w_list = []

        for i in xrange(self.n_jts):
            p, rot = self.FK_vanilla(q, i)
            r = pos - p
            z_idx = ch.getSegment(i).getJoint().getType() - 1
            z = rot[:, z_idx]

            # this is a nasty trick. The rotation matrix returned by
            # FK_vanilla describes the orientation of a frame of the
            # KDL chain in the base frame. It just so happens that the
            # way Advait defined the KDL chain, the axis of rotation
            # in KDL is the -ve of the axis of rotation on the real
            # robot for every joint.
            # Advait apologizes for this.
            z = -z

            v_list.append(np.matrix(np.cross(z.A1, r.A1)).T)
            w_list.append(z)

        J = np.row_stack((np.column_stack(v_list), np.column_stack(w_list)))
        return J

    def IK_vanilla(self, p, rot, q_guess=None):
        p_kdl = ku.np_vec_to_kdl(p)
        rot_kdl = ku.np_rot_to_kdl(rot)
        fr = kdl.Frame(rot_kdl, p_kdl)

        if q_guess == None:
            q_guess = cgd.find_good_config(p, self.q_guess_dict)

        q_guess = self.meka_angles_to_kdl(q_guess)
        q_res = self.IK_kdl(arm, fr, q_guess)
        q_res = self.kdl_angles_to_meka(arm, q_res)
        return q_res

    #----------- extra functions -----------------

    ## clamp joint angles to their physical limits.
    # @param q - list of 7 joint angles.
    # The joint limits for IK are larger that the physical limits.
    def clamp_to_joint_limits(self, q, delta_list=[0.,0.,0.,0.,0.,0.,0.]):
        min_arr, max_arr = self.get_joint_limits()
        q_arr = np.array(q)
        d_arr = np.array(delta_list)
        return np.clip(q_arr, min_arr-d_arr, max_arr+d_arr)

    def within_joint_limits(self, q, delta_list=[0.,0.,0.,0.,0.,0.,0.]):
        min_arr, max_arr = self.get_joint_limits()
        q_arr = np.array(q)
        d_arr = np.array(delta_list)
        return np.all((q_arr <= max_arr+d_arr, q_arr >= min_arr-d_arr))

    def get_joint_limits(self):
        return self.joint_lim_dict['min'], self.joint_lim_dict['max']


    #------------ 2D functions ------------

    # plot the arm using matplotlib.
    def plot_arm(self, q, color='b', alpha=1.):
        pts = [[0.,0.,0.]]
        for i in range(len(q)):
            p = self.FK(q, i+1)[0]
            pts.append(p.A1.tolist())

        pts_2d = np.array(pts)[:,0:2]
        direc_list = (pts_2d[1:] - pts_2d[:-1]).tolist()

        for i, d in enumerate(direc_list):
            d_vec = np.matrix(d).T
            d_vec = d_vec / np.linalg.norm(d_vec)
            w = np.cross(d_vec.A1, np.array([0., 0., 1.])) * 0.03/2
            x1 = pts_2d[i,0]
            y1 = pts_2d[i,1]
            x2 = pts_2d[i+1,0]
            y2 = pts_2d[i+1,1]

            x_data = [x1+w[0], x1-w[0], x2-w[0], x2+w[0], x1+w[0]]
            y_data = [y1+w[1], y1-w[1], y2-w[1], y2+w[1], y1+w[1]]

            l, = pp.plot(x_data, y_data, color+'-', alpha=alpha)






