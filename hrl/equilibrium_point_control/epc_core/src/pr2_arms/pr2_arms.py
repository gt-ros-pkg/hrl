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


import roslib; roslib.load_manifest('epc_core')
roslib.load_manifest('force_torque') # hack by Advait
import force_torque.FTClient as ftc

import tf
import hrl_lib.transforms as tr
import hrl_lib.viz as hv

import rospy
import PyKDL as kdl

import actionlib
from actionlib_msgs.msg import GoalStatus

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, JointTrajectoryControllerState
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from teleop_controllers.msg import JTTeleopControllerState

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import hrl_lib.transforms as tr
import hrl_lib.kdl_utils as ku
import time

from visualization_msgs.msg import Marker


node_name = "pr2_arms" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

class PR2Arms():
    def __init__(self, primary_ft_sensor=None):
        log("Loading PR2Arms")

        self.arms = PR2Arms_kdl() # KDL chain.

        self.joint_names_list = [['r_shoulder_pan_joint',
                           'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                           'r_elbow_flex_joint', 'r_forearm_roll_joint',
                           'r_wrist_flex_joint', 'r_wrist_roll_joint'],
                           ['l_shoulder_pan_joint',
                           'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                           'l_elbow_flex_joint', 'l_forearm_roll_joint',
                           'l_wrist_flex_joint', 'l_wrist_roll_joint']]

        self.arm_state_lock = [RLock(), RLock()]
        self.jep = [None, None]

        self.arm_angles = [None, None]
        self.torso_position = None
        self.arm_efforts = [None, None]

        self.r_arm_cart_pub = rospy.Publisher('/r_cart/command_pose', PoseStamped)
        self.l_arm_cart_pub = rospy.Publisher('/l_cart/command_pose', PoseStamped)

        rospy.Subscriber('/r_cart/state', JTTeleopControllerState, self.r_cart_state_cb)
        rospy.Subscriber('/l_cart/state', JTTeleopControllerState, self.l_cart_state_cb)

        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        self.marker_pub = rospy.Publisher('/pr2_arms/viz_markers', Marker)
        self.cep_marker_id = 1

        self.r_arm_ftc = ftc.FTClient('force_torque_ft2')
        self.r_arm_ftc_estimate = ftc.FTClient('force_torque_ft2_estimate')
        self.tf_lstnr = tf.TransformListener()

        if primary_ft_sensor == 'ati':
            self.get_wrist_force = self.get_wrist_force_ati
        if primary_ft_sensor == 'estimate':
            self.get_wrist_force = self.get_wrist_force_estimate

        r_action_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action',
                                                       JointTrajectoryAction)
        l_action_client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action',
                                                       JointTrajectoryAction)
        self.joint_action_client = [r_action_client, l_action_client]

        r_gripper_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',
                                                        Pr2GripperCommandAction)
        l_gripper_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action',
                                                        Pr2GripperCommandAction)
        self.gripper_action_client = [r_gripper_client, l_gripper_client]

        r_ik_srv = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik', GetPositionIK)
        l_ik_srv = rospy.ServiceProxy('pr2_left_arm_kinematics/get_ik', GetPositionIK)
        self.ik_srv = [r_ik_srv, l_ik_srv]
        r_fk_srv = rospy.ServiceProxy('pr2_right_arm_kinematics/get_fk', GetPositionFK)
        l_fk_srv = rospy.ServiceProxy('pr2_left_arm_kinematics/get_fk', GetPositionFK)
        self.fk_srv = [r_fk_srv, l_fk_srv]

        rospy.sleep(2.)

#        self.joint_action_client[0].wait_for_server()
#        self.joint_action_client[1].wait_for_server()
        self.gripper_action_client[0].wait_for_server()
        self.gripper_action_client[1].wait_for_server()

        log("Finished loading SimpleArmManger")

    ##
    # Callback for /joint_states topic. Updates current joint
    # angles and efforts for the arms constantly
    # @param data JointState message recieved from the /joint_states topic
    def joint_states_cb(self, data):
        arm_angles = [[], []]
        arm_efforts = [[], []]
        r_jt_idx_list = [0]*7
        l_jt_idx_list = [0]*7
        for i, jt_nm in enumerate(self.joint_names_list[0]):
            r_jt_idx_list[i] = data.name.index(jt_nm)
        for i, jt_nm in enumerate(self.joint_names_list[1]):
            l_jt_idx_list[i] = data.name.index(jt_nm)


        for i in range(7):
            idx = r_jt_idx_list[i]
            if data.name[idx] != self.joint_names_list[0][i]:
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            arm_angles[0].append(data.position[idx])
            arm_efforts[0].append(data.effort[idx])

            idx = l_jt_idx_list[i]
            if data.name[idx] != self.joint_names_list[1][i]:
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            #ang = tr.angle_within_mod180(data.position[idx])
            ang = data.position[idx]
            arm_angles[1] += [ang]
            arm_efforts[1] += [data.effort[idx]]

        self.arm_state_lock[0].acquire()
        self.arm_angles[0] = arm_angles[0]
        self.arm_efforts[0] = arm_efforts[0]

        torso_idx = data.name.index('torso_lift_joint')
        self.torso_position = data.position[torso_idx]

        self.arm_state_lock[0].release()

        self.arm_state_lock[1].acquire()
        self.arm_angles[1] = arm_angles[1]
        self.arm_efforts[1] = arm_efforts[1]
        self.arm_state_lock[1].release()

    def r_cart_state_cb(self, msg):
        try:
            trans, quat = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                     'r_gripper_tool_frame', rospy.Time(0))
            rot = tr.quaternion_to_matrix(quat)
            tip = np.matrix([0.12, 0., 0.]).T
            self.r_ee_pos = rot*tip + np.matrix(trans).T
            self.r_ee_rot = rot


            marker = Marker()
            marker.header.frame_id = 'torso_lift_link'
            time_stamp = rospy.Time.now()
            marker.header.stamp = time_stamp
            marker.ns = 'aloha land'
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.r_ee_pos[0,0]
            marker.pose.position.y = self.r_ee_pos[1,0]
            marker.pose.position.z = self.r_ee_pos[2,0]
            size = 0.02
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            marker.lifetime = rospy.Duration()

            marker.id = 71
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1

            color = (0.5, 0., 0.0)
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.
            self.marker_pub.publish(marker)
            
            ros_pt = msg.x_desi_filtered.pose.position
            x, y, z = ros_pt.x, ros_pt.y, ros_pt.z
            self.r_cep_pos = np.matrix([x, y, z]).T
            pt = rot.T * (np.matrix([x,y,z]).T - np.matrix(trans).T)
            pt = pt + tip
            self.r_cep_pos_hooktip = rot*pt + np.matrix(trans).T
            ros_quat = msg.x_desi_filtered.pose.orientation
            quat = (ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w)
            self.r_cep_rot = tr.quaternion_to_matrix(quat)
        except:
            return

    def l_cart_state_cb(self, msg):
        ros_pt = msg.x_desi_filtered.pose.position
        self.l_cep_pos = np.matrix([ros_pt.x, ros_pt.y, ros_pt.z]).T
        ros_quat = msg.x_desi_filtered.pose.orientation
        quat = (ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w)
        self.l_cep_rot = tr.quaternion_to_matrix(quat)

    def IK(self, arm, pos, rot, q_guess=[0]*7):
        ik_req = GetPositionIKRequest()
        ik_req.timeout = rospy.Duration(5.)
        if arm == 0:
            ik_req.ik_request.ik_link_name = 'r_wrist_roll_link'
        else:
            ik_req.ik_request.ik_link_name = 'l_wrist_roll_link'
        ik_req.ik_request.pose_stamped.header.frame_id = 'torso_lift_link'
        quat = tr.matrix_to_quaternion(rot)
        ik_req.ik_request.pose_stamped.pose = Pose(Point(*pos), Quaternion(*quat))
        ik_req.ik_request.ik_seed_state.joint_state.position = q_guess
        ik_req.ik_request.ik_seed_state.joint_state.name = self.joint_names_list[arm]

        ik_resp = self.ik_srv[arm].call(ik_req)
        return ik_resp.solution.joint_state.position

    def FK(self, arm, q):
        fk_req = GetPositionFKRequest()
        fk_req.header.frame_id = 'torso_lift_link'
        if arm == 0:
            fk_req.fk_link_names.append('r_wrist_roll_link') 
        else:
            fk_req.fk_link_names.append('l_wrist_roll_link')
        fk_req.robot_state.joint_state.name = self.joint_names_list[arm]
        fk_req.robot_state.joint_state.position = q

        fk_resp = self.fk_srv[arm].call(fk_req)
        if fk_resp.error_code.val == fk_resp.error_code.SUCCESS:
            x = fk_resp.pose_stamped[0].pose.position.x
            y = fk_resp.pose_stamped[0].pose.position.y
            z = fk_resp.pose_stamped[0].pose.position.z
            pos = [x, y, z]
            q1 = fk_resp.pose_stamped[0].pose.orientation.x
            q2 = fk_resp.pose_stamped[0].pose.orientation.y
            q3 = fk_resp.pose_stamped[0].pose.orientation.z
            q4 = fk_resp.pose_stamped[0].pose.orientation.w
            quat = [q1,q2,q3,q4]
            rot = tr.quaternion_to_matrix(quat)
        else:
            rospy.logerr('Forward kinematics failed')
            return None, None

        return pos, rot


    ## Returns the current position, rotation of the arm.
    # @param arm 0 for right, 1 for left
    # @return position, rotation
    def end_effector_pos(self, arm):
        q = self.get_joint_angles(arm)
        return self.arms.FK_all(arm, q)

    ## Returns the list of 7 joint angles
    # @param arm 0 for right, 1 for left
    # @return list of 7 joint angles
    def get_joint_angles(self, arm):
        if arm != 1:
            arm = 0
        self.arm_state_lock[arm].acquire()
        q = self.wrap_angles(self.arm_angles[arm])
        self.arm_state_lock[arm].release()
        return q

    def set_jep(self, arm, q, duration=0.15):
        if q is None or len(q) != 7:
            raise RuntimeError("set_jep value is " + str(q))
        self.arm_state_lock[arm].acquire()

        jtg = JointTrajectoryGoal()
        jtg.trajectory.joint_names = self.joint_names_list[arm]
        jtp = JointTrajectoryPoint()
        jtp.positions = q
        #jtp.velocities = [0 for i in range(len(q))]
        #jtp.accelerations = [0 for i in range(len(q))]
        jtp.time_from_start = rospy.Duration(duration)
        jtg.trajectory.points.append(jtp)
        self.joint_action_client[arm].send_goal(jtg)

        self.jep[arm] = q
        cep, r = self.arms.FK_all(arm, q)
        self.arm_state_lock[arm].release()

        o = np.matrix([0.,0.,0.,1.]).T
        cep_marker = hv.single_marker(cep, o, 'sphere',
                        '/torso_lift_link', color=(0., 0., 1., 1.),
                            scale = (0.02, 0.02, 0.02),
                            m_id = self.cep_marker_id)

        cep_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(cep_marker)

    def get_jep(self, arm):
        self.arm_state_lock[arm].acquire()
        jep = copy.copy(self.jep[arm])
        self.arm_state_lock[arm].release()
        return jep

    def get_ee_jtt(self, arm):
        if arm == 0:
            return self.r_ee_pos, self.r_ee_rot
        else:
            return self.l_ee_pos, self.l_ee_rot

    def get_cep_jtt(self, arm, hook_tip = False):
        if arm == 0:
            if hook_tip:
                return self.r_cep_pos_hooktip, self.r_cep_rot
            else:
                return self.r_cep_pos, self.r_cep_rot
        else:
            return self.l_cep_pos, self.l_cep_rot

    # set a cep using the Jacobian Transpose controller.
    def set_cep_jtt(self, arm, p, rot=None):
        if arm != 1:
            arm = 0
        ps = PoseStamped()
        ps.header.stamp = rospy.rostime.get_rostime()
        ps.header.frame_id = 'torso_lift_link'
 
        ps.pose.position.x = p[0,0]
        ps.pose.position.y = p[1,0]
        ps.pose.position.z = p[2,0]
 
        if rot == None:
            if arm == 0:
                rot = self.r_cep_rot
            else:
                rot = self.l_cep_rot

        quat = tr.matrix_to_quaternion(rot)
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        if arm == 0:
            self.r_arm_cart_pub.publish(ps)
        else:
            self.l_arm_cart_pub.publish(ps)

    # rotational interpolation unimplemented.
    def go_cep_jtt(self, arm, p):
        step_size = 0.01
        sleep_time = 0.1
        cep_p, cep_rot = self.get_cep_jtt(arm)
        unit_vec = (p-cep_p)
        unit_vec = unit_vec / np.linalg.norm(unit_vec)
        while np.linalg.norm(p-cep_p) > step_size:
            cep_p += unit_vec * step_size
            self.set_cep_jtt(arm, cep_p)
            rospy.sleep(sleep_time)
        self.set_cep_jtt(arm, p)
        rospy.sleep(sleep_time)


    #----------- forces ------------

    # force that is being applied on the wrist. (estimate as returned
    # by the cartesian controller)
    def get_wrist_force_estimate(self, arm, bias = True, base_frame = False):
        if arm != 0:
            rospy.logerr('Unsupported arm: %d'%arm)
            raise RuntimeError('Unimplemented function')
 
        f = self.r_arm_ftc_estimate.read(without_bias = not bias)
        f = f[0:3, :]
        if base_frame:
            trans, quat = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                                '/ft2_estimate', rospy.Time(0))
            rot = tr.quaternion_to_matrix(quat)
            f = rot * f
        return -f # the negative is intentional (Advait, Nov 24. 2010.)

    # force that is being applied on the wrist.
    def get_wrist_force_ati(self, arm, bias = True, base_frame = False):
        if arm != 0:
            rospy.logerr('Unsupported arm: %d'%arm)
            raise RuntimeError('Unimplemented function')
 
        f = self.r_arm_ftc.read(without_bias = not bias)
        f = f[0:3, :]
        if base_frame:
            trans, quat = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                                '/ft2', rospy.Time(0))
            rot = tr.quaternion_to_matrix(quat)
            f = rot * f
        return -f # the negative is intentional (Advait, Nov 24. 2010.)

    ## Returns the list of 7 joint angles
    # @param arm 0 for right, 1 for left
    # @return list of 7 joint angles
    def get_force_from_torques(self, arm):
        if arm != 1:
            arm = 0
        self.arm_state_lock[arm].acquire()
        q = self.arm_angles[arm]
        tau = self.arm_efforts[arm]
        self.arm_state_lock[arm].release()
        p, _ = self.arms.FK_all(arm, q)
        J = self.arms.Jacobian(arm, q, p)
        f = np.linalg.pinv(J.T) * np.matrix(tau).T
        f = f[0:3,:]
        return -f


    def bias_wrist_ft(self, arm):
        if arm != 0:
            rospy.logerr('Unsupported arm: %d'%arm)
            raise RuntimeError('Unimplemented function')
        self.r_arm_ftc.bias()
        self.r_arm_ftc_estimate.bias()

    #-------- gripper functions ------------
    def move_gripper(self, arm, amount=0.08, effort = 15):
        self.gripper_action_client[arm].send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=amount,
                                                                                    max_effort = effort)))

    ## Open the gripper
    # @param arm 0 for right, 1 for left
    def open_gripper(self, arm):
        self.move_gripper(arm, 0.08, -1)

    ## Close the gripper
    # @param arm 0 for right, 1 for left
    def close_gripper(self, arm, effort = 15):
        self.move_gripper(arm, 0.0, effort)

    def wrap_angles(self, q):
        for ind in [4, 6]:
            while q[ind] < -np.pi:
                q[ind] += 2*np.pi
            while q[ind] > np.pi:
                q[ind] -= 2*np.pi
        return q


##
# using KDL for pr2 arm kinematics.
class PR2Arms_kdl():
    def __init__(self):
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

    ## define tooltip as a 3x1 np matrix in the wrist coord frame.
    def set_tooltip(self, arm, p):
        if arm == 0:
            self.right_tooltip = p
        else:
            rospy.logerr('Arm %d is not supported.'%(arm))

    def FK_kdl(self, arm, q, link_number):
        if arm == 0:
            fk = self.right_fk
            endeffec_frame = kdl.Frame()
            kinematics_status = fk.JntToCart(q, endeffec_frame,
                                             link_number)
            if kinematics_status >= 0:
                return endeffec_frame
            else:
                rospy.loginfo('Could not compute forward kinematics.')
                return None
        else:
            msg = '%s arm not supported.'%(arm)
            rospy.logerr(msg)
            raise RuntimeError(msg)

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

        q_pr2 = [0] * 7
        q_pr2[0] = q[0]
        q_pr2[1] = q[1]
        q_pr2[2] = q[2]
        q_pr2[3] = q[3]
        q_pr2[4] = q[4]
        q_pr2[5] = q[5]
        q_pr2[6] = q[6]
        return q_pr2

    def pr2_to_kdl(self, q):
        if q == None:
            return None
        n = len(q)
        q_kdl = kdl.JntArray(n)
        for i in range(n):
            q_kdl[i] = q[i]
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

    def close_to_singularity(self, arm, q):
        pass

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


if __name__ == '__main__':
    from visualization_msgs.msg import Marker
    import hrl_lib.viz as hv

    rospy.init_node('pr2_arms_test')

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

            











