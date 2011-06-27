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

# Author: Advait Jain

import numpy as np, math
from threading import RLock, Timer
import sys, copy
import time

import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy
from visualization_msgs.msg import Marker

import PyKDL as kdl

import hrl_lib.transforms as tr
import hrl_lib.viz as hv
import hrl_lib.transforms as tr
import hrl_lib.kdl_utils as ku

from equilibrium_point_control.hrl_arm import HRLArmKinematics


##
# using KDL for pr2 arm kinematics.
class PR2ArmKinematics(HRLArmKinematics):
    def __init__(self, tool_pos=np.matrix([0.]*3).T, tool_rot=np.matrix(np.eye(3))):
        super(PR2ArmKinematics, self).__init__(7, tool_pos, tool_rot)
        self.right_chain = self.create_right_chain()
        fk, ik_v, ik_p, jac = self.create_solvers(self.right_chain)
        self.right_fk = fk
        self.right_ik_v = ik_v
        self.right_ik_p = ik_p
        self.right_jac = jac
        self.right_tooltip = np.matrix([0.,0.,0.]).T

    def create_right_chain(self):
        ch = kdl.Chain()
        self.right_arm_base_offset_from_torso_lift_link = np.matrix([0., -0.188, 0.]).T
        # shoulder pan
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.1,0.,0.))))
        # shoulder lift
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
        # upper arm roll
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.4,0.,0.))))
        # elbox flex
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.0,0.,0.))))
        # forearm roll
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.321,0.,0.))))
        # wrist flex
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
        # wrist roll
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,0.))))
        return ch

    def create_solvers(self, ch):
         fk = kdl.ChainFkSolverPos_recursive(ch)
         ik_v = kdl.ChainIkSolverVel_pinv(ch)
         ik_p = kdl.ChainIkSolverPos_NR(ch, fk, ik_v)
         jac = kdl.ChainJntToJacSolver(ch)
         return fk, ik_v, ik_p, jac

    def FK_vanilla(self, q, link_number):
        fk = self.right_fk
        endeffec_frame = kdl.Frame()
        kinematics_status = fk.JntToCart(self.pr2_to_kdl(q), endeffec_frame,
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
            rospy.loginfo('Could not compute forward kinematics.')
            return None, None

    ## returns point in torso lift link.
    def FK_all(self, arm, q, link_number = 7):
        q = self.pr2_to_kdl(q)
        frame = self.FK_kdl(arm, q, link_number)
        pos = frame.p
        pos = ku.kdl_vec_to_np(pos)
        pos = pos + self.right_arm_base_offset_from_torso_lift_link
        m = frame.M
        rot = ku.kdl_rot_to_np(m)
        if arm == 0:
            tooltip_baseframe = rot * self.right_tooltip
            pos += tooltip_baseframe
        else:
            rospy.logerr('Arm %d is not supported.'%(arm))
            return None
        return pos, rot

    def kdl_to_pr2(self, q):
        if q == None:
            return None
        return [q_i for q_i in q]

    def pr2_to_kdl(self, q):
        if q == None:
            return None
        q_kdl = kdl.JntArray(len(q))
        for i, q_i in enumerate(q):
            q_kdl[i] = q_i
        return q_kdl

    def Jac_kdl(self, arm, q):
        J_kdl = kdl.Jacobian(7)
        if arm != 0:
            rospy.logerr('Unsupported arm: '+ str(arm))
            return None

        self.right_jac.JntToJac(q,J_kdl)

        kdl_jac =  np.matrix([
            [J_kdl[0,0],J_kdl[0,1],J_kdl[0,2],J_kdl[0,3],J_kdl[0,4],J_kdl[0,5],J_kdl[0,6]],
            [J_kdl[1,0],J_kdl[1,1],J_kdl[1,2],J_kdl[1,3],J_kdl[1,4],J_kdl[1,5],J_kdl[1,6]],
            [J_kdl[2,0],J_kdl[2,1],J_kdl[2,2],J_kdl[2,3],J_kdl[2,4],J_kdl[2,5],J_kdl[2,6]],
            [J_kdl[3,0],J_kdl[3,1],J_kdl[3,2],J_kdl[3,3],J_kdl[3,4],J_kdl[3,5],J_kdl[3,6]],
            [J_kdl[4,0],J_kdl[4,1],J_kdl[4,2],J_kdl[4,3],J_kdl[4,4],J_kdl[4,5],J_kdl[4,6]],
            [J_kdl[5,0],J_kdl[5,1],J_kdl[5,2],J_kdl[5,3],J_kdl[5,4],J_kdl[5,5],J_kdl[5,6]],
            ])
        return kdl_jac
        
    ## compute Jacobian (at wrist).
    # @param arm - 0 or 1
    # @param q - list of 7 joint angles.
    # @return 6x7 np matrix
    def Jac(self, arm, q):
        rospy.logerr('Jac only works for getting the Jacobian at the wrist. Use Jacobian to get the Jacobian at a general location.')
        jntarr = self.pr2_to_kdl(q)
        kdl_jac = self.Jac_kdl(arm, jntarr)
        pr2_jac = kdl_jac
        return pr2_jac

    ## compute Jacobian at point pos.
    # p is in the torso_lift_link coord frame.
    def Jacobian(self, arm, q, pos):
        if arm != 0:
            rospy.logerr('Arm %d is not supported.'%(arm))
            return None

        tooltip = self.right_tooltip
        self.right_tooltip = np.matrix([0.,0.,0.]).T
        v_list = []
        w_list = []
        for i in range(7):
            p, rot = self.FK_all(arm, q, i)
            r = pos - p
            z_idx = self.right_chain.getSegment(i).getJoint().getType() - 1
            z = rot[:, z_idx]
            v_list.append(np.matrix(np.cross(z.A1, r.A1)).T)
            w_list.append(z)

        J = np.row_stack((np.column_stack(v_list), np.column_stack(w_list)))
        self.right_tooltip = tooltip
        return J

    def within_joint_limits(self, arm, q, delta_list=[0., 0., 0., 0., 0., 0., 0.]):
        if arm == 0: # right arm
            min_arr = np.radians(np.array([-109., -24, -220, -132, -np.inf, -120, -np.inf]))
            #max_arr = np.radians(np.array([26., 68, 41, 0, np.inf, 0, np.inf]))
            max_arr = np.radians(np.array([26., 68, 41, 5, np.inf, 5, np.inf])) # 5 to prevent singularity. Need to come up with a better solution.
        else:
            raise RuntimeError('within_joint_limits unimplemented for left arm')

        q_arr = np.array(q)
        d_arr = np.array(delta_list)
        return np.all((q_arr <= max_arr+d_arr, q_arr >= min_arr-d_arr))


def main():
    from visualization_msgs.msg import Marker
    import hrl_lib.viz as hv

    rospy.init_node('pr2_arm_kinematics_test')

    kinematics = PR2ArmKinematics()
    print kinematics.FK([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    return


    pr2_arms = PR2Arms()
    pr2_kdl = PR2Arms_kdl()
    r_arm, l_arm = 0, 1
    arm = r_arm

    if False:
        np.set_printoptions(precision=2, suppress=True)
        while not rospy.is_shutdown():
            q = pr2_arms.get_joint_angles(arm)
            print 'q in degrees:', np.degrees(q)
            rospy.sleep(0.1)

    if False:
        jep = [0.] * 7
        rospy.loginfo('Going to home location.')
        raw_input('Hit ENTER to go')
        pr2_arms.set_jep(arm, jep, duration=2.)

    if False:
        # testing FK by publishing a frame marker.
        marker_pub = rospy.Publisher('/pr2_kdl/ee_marker', Marker)
        pr2_kdl.set_tooltip(arm, np.matrix([0.15, 0., 0.]).T)
        rt = rospy.Rate(100)
        rospy.loginfo('Starting the maker publishing loop.')
        while not rospy.is_shutdown():
            q = pr2_arms.get_joint_angles(arm)
            p, rot = pr2_kdl.FK_all(arm, q)
            m = hv.create_frame_marker(p, rot, 0.15, '/torso_lift_link')
            m.header.stamp = rospy.Time.now()
            marker_pub.publish(m)
            rt.sleep()

    if False:
        # testing Jacobian by printing KDL and my own Jacobian at the
        # current configuration.
        while not rospy.is_shutdown():
            q = pr2_arms.get_joint_angles(arm)
            J_kdl = pr2_kdl.Jac(arm , q)
            p, rot = pr2_kdl.FK_all(arm, q)
            J_adv = pr2_kdl.Jacobian(arm, q, p)
            print J_adv.shape
            diff_J = J_kdl - J_adv
            print 'difference between KDL and adv is:'
            print diff_J
            print 'Norm of difference matrix:', np.linalg.norm(diff_J)
            raw_input('Move arm into some configuration and hit enter to get the Jacobian.')


    if True:
        while not rospy.is_shutdown():
            q = pr2_arms.wrap_angles(pr2_arms.get_joint_angles(arm))
            print "actual", q
            p_ros, rot_ros = pr2_arms.FK(arm, q)
            p_kdl, rot_kdl = pr2_kdl.FK_all(arm, q)
            ik_ros = pr2_arms.IK(r_arm, p_ros, rot_ros, q)
            ik_kdl = pr2_arms.IK(r_arm, p_kdl, rot_kdl, q)
            diff = np.array(ik_ros) - np.array(ik_kdl)
            print "IK ros", ik_ros
            print "IK kdl", ik_kdl
            if len(ik_ros) == 7:
                err_ros = np.array(q) - np.array(ik_ros)
                err_kdl = np.array(q) - np.array(ik_kdl)
                print "err ros", sum(err_ros**2), "err kdl", sum(err_kdl**2), "diff", sum(diff**2)
            
if __name__ == '__main__':
    main()

