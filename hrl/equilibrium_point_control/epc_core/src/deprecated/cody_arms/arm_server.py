#!/usr/bin/python
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


import m3.rt_proxy as m3p
import m3.arm
import m3.toolbox as m3t
import m3.pwr as m3w
import m3.loadx6

import m3.component_factory as m3f

import arm_client as ac

import math
import numpy as np
import sys, time, os
import copy
from threading import RLock

import arms as ar
import roslib; roslib.load_manifest('epc_core')
import rospy

import hrl_lib.transforms as tr
from hrl_msgs.msg import FloatArray
from roslib.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse
from sensor_msgs.msg import JointState

THETA_GC = 5
TORQUE_GC = 4
THETA = 3
OFF = 0 # in OFF mode also, the behavior is strange. Not the same as hitting the estop (Advait, Jan 1 2010)


## 1D kalman filter update.
def kalman_update(xhat, P, Q, R, z):
    #time update
    xhatminus = xhat
    Pminus = P + Q
    #measurement update
    K = Pminus / (Pminus + R)
    xhat = xhatminus + K * (z-xhatminus)
    P = (1-K) * Pminus
    return xhat, P

class MekaArmSettings():
    def __init__(self, stiffness_list=[0.7,0.7,0.8,0.8,0.3],
                 control_mode='theta_gc'):
        ''' stiffness_list: list of 5 stiffness values for joints 0-4.
            control_mode: 'theta_gc' or 'torque_gc'
        '''
        self.set_stiffness_scale(stiffness_list)
        self.control_mode = control_mode

    def set_stiffness_scale(self, l):
        # for safety of wrist roll. Advait Jun 18, 2010.
        # changed to 0.2 from 0.3 (Advait, Sept 19, 2010)
        l[4] = min(l[4], 0.2)
        self.stiffness_list = l


class MekaArmServer():
    def __init__(self, right_arm_settings=None, left_arm_settings=None):
        self.arm_settings = {}  # dict is set in set_arm_settings
        self.initialize_joints(right_arm_settings, left_arm_settings)

        #self.initialize_gripper()

        self.left_arm_ft = {'force': np.matrix(np.zeros((3,1),dtype='float32')),
                            'torque': np.matrix(np.zeros((3,1),dtype='float32'))}
        self.right_arm_ft = {'force': np.matrix(np.zeros((3,1),dtype='float32')),
                             'torque': np.matrix(np.zeros((3,1),dtype='float32'))}
        self.fts_bias = {'left_arm': self.left_arm_ft, 'right_arm': self.right_arm_ft}

        # kalman filtering force vector. (self.step and bias_wrist_ft)
        self.Q_force, self.R_force = {}, {}
        self.xhat_force, self.P_force = {}, {}

        self.Q_force['right_arm'] = [1e-3, 1e-3, 1e-3]
        self.R_force['right_arm'] = [0.2**2, 0.2**2, 0.2**2]
        self.xhat_force['right_arm'] = [0., 0., 0.]
        self.P_force['right_arm'] = [1.0, 1.0, 1.0]

        self.Q_force['left_arm'] = [1e-3, 1e-3, 1e-3]
        self.R_force['left_arm'] = [0.2**2, 0.2**2, 0.2**2]
        self.xhat_force['left_arm'] = [0., 0., 0.]
        self.P_force['left_arm'] = [1.0, 1.0, 1.0]

        #----- ROS interface ---------
        rospy.init_node('arm_server', anonymous=False)

        rospy.Service('toggle_floating_arms', Empty_srv, self.floating_arms_cb)
        self.q_r_pub = rospy.Publisher('/r_arm/q', FloatArray)
        self.q_l_pub = rospy.Publisher('/l_arm/q', FloatArray)

        self.force_raw_r_pub = rospy.Publisher('/r_arm/force_raw', FloatArray)
        self.force_raw_l_pub = rospy.Publisher('/l_arm/force_raw', FloatArray)
        self.force_r_pub = rospy.Publisher('/r_arm/force', FloatArray)
        self.force_l_pub = rospy.Publisher('/l_arm/force', FloatArray)

        self.jep_r_pub = rospy.Publisher('/r_arm/jep', FloatArray)
        self.jep_l_pub = rospy.Publisher('/l_arm/jep', FloatArray)
        self.alph_r_pub = rospy.Publisher('/r_arm/joint_impedance_scale', FloatArray)
        self.alph_l_pub = rospy.Publisher('/l_arm/joint_impedance_scale', FloatArray)

        self.pwr_state_pub = rospy.Publisher('/arms/pwr_state', Bool)
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState)
        
        rospy.Subscriber('/r_arm/command/jep', FloatArray, self.r_jep_cb)
        rospy.Subscriber('/l_arm/command/jep', FloatArray, self.l_jep_cb)
        rospy.Subscriber('/r_arm/command/joint_impedance_scale', FloatArray, self.r_alpha_cb)
        rospy.Subscriber('/l_arm/command/joint_impedance_scale', FloatArray, self.l_alpha_cb)

        # publishing to this message will stop the arms but also crash
        # the server (since meka server crashes.) Advait Nov 14, 2010
        rospy.Subscriber('/arms/stop', Empty, self.stop)
        rospy.Subscriber('/arms/command/motors_off', Empty,
                         self.motors_off)

        self.cb_lock = RLock()
        self.r_jep = None # see set_jep
        self.l_jep = None # see set_jep
        self.qr_prev = None # see step_ros
        self.ql_prev = None # see step_ros

        self.joint_names_list = ac.get_joint_name_dict()

        self.floating_arms = False
        self.floating_arms_counter = 0

    def floating_arms_cb(self, req):
        self.floating_arms_counter = 0
        self.floating_arms = not self.floating_arms
        #rospy.logout('floating_arms_cb called')
        return EmptyResponse()


    def set_arm_settings(self,right_arm_settings,left_arm_settings):
        self.arm_settings['right_arm'] = right_arm_settings
        self.arm_settings['left_arm'] = left_arm_settings

        for arm,arm_settings in zip(['right_arm','left_arm'],[right_arm_settings,left_arm_settings]):
            joint_component_list = self.joint_list_dict[arm]

# OFF mode doesn't seem to work. (Advait, Jan 1 2010)
            if arm_settings == None:
                for c in joint_component_list:
                    c.set_control_mode(OFF)
                continue

            stiffness_list = arm_settings.stiffness_list

            if arm_settings.control_mode == 'torque_gc':
                print 'setting control mode to torque_gc'
                for c in joint_component_list:
                    c.set_control_mode(TORQUE_GC)
                    c.set_torque_mNm(0.0)
            elif arm_settings.control_mode == 'theta_gc':
                print 'setting control mode to theta_gc'
                for i in range(5):
                    joint_component_list[i].set_control_mode(THETA_GC)
                    joint_component_list[i].set_stiffness(stiffness_list[i])
                    joint_component_list[i].set_slew_rate_proportion(1.)

                joint_component_list[5].set_control_mode(THETA)
                joint_component_list[5].set_slew_rate_proportion(1.)
                joint_component_list[6].set_control_mode(THETA)
                joint_component_list[6].set_slew_rate_proportion(1.)

            elif arm_settings.control_mode == 'wrist_theta_gc':
                print 'setting control mode to theta_gc include wrist joints'
                for i in range(7):
                    joint_component_list[i].set_control_mode(THETA_GC)
                    joint_component_list[i].set_stiffness(stiffness_list[i])
                    joint_component_list[i].set_slew_rate_proportion(1.)

            else:
                print 'hrl_robot.initialize_joints. unknown control mode for ', arm,':', arm_settings.control_mode

    # put a few things into safeop so that individual joint
    # components work.
    def safeop_things(self):
        robot_name = 'm3humanoid_bimanual_mr1'
        chain_names = ['m3arm_ma1', 'm3arm_ma2']
        dynamatics_nms = ['m3dynamatics_ma1', 'm3dynamatics_ma2']

        self.proxy.make_safe_operational(robot_name)
        for c in chain_names:
            self.proxy.make_safe_operational(c)
        for d in dynamatics_nms:
            self.proxy.make_safe_operational(d)

    def initialize_joints(self, right_arm_settings, left_arm_settings):
        self.proxy = m3p.M3RtProxy()
        self.proxy.start()
        for c in ['m3pwr_pwr003','m3loadx6_ma1_l0','m3arm_ma1','m3loadx6_ma2_l0','m3arm_ma2']:
            if not self.proxy.is_component_available(c):
                raise m3t.M3Exception('Component '+c+' is not available.')
        
        self.joint_list_dict = {}

        right_l = []
        for c in ['m3joint_ma1_j0','m3joint_ma1_j1','m3joint_ma1_j2',
                  'm3joint_ma1_j3','m3joint_ma1_j4','m3joint_ma1_j5',
                  'm3joint_ma1_j6']:
            if not self.proxy.is_component_available(c):
                raise m3t.M3Exception('Component '+c+' is not available.')
            right_l.append(m3f.create_component(c))
        self.joint_list_dict['right_arm'] = right_l

        left_l = []
        for c in ['m3joint_ma2_j0','m3joint_ma2_j1','m3joint_ma2_j2',
                  'm3joint_ma2_j3','m3joint_ma2_j4','m3joint_ma2_j5',
                  'm3joint_ma2_j6']:
            if not self.proxy.is_component_available(c):
                raise m3t.M3Exception('Component '+c+' is not available.')
            left_l.append(m3f.create_component(c))
        self.joint_list_dict['left_arm'] = left_l


        for arm,arm_settings in zip(['right_arm','left_arm'],[right_arm_settings,left_arm_settings]):
            if arm_settings == None:
                continue

            for comp in self.joint_list_dict[arm]:
                self.proxy.subscribe_status(comp)
                self.proxy.publish_command(comp)

        self.set_arm_settings(right_arm_settings,left_arm_settings)

        right_fts=m3.loadx6.M3LoadX6('m3loadx6_ma1_l0')
        self.proxy.subscribe_status(right_fts)
        left_fts=m3.loadx6.M3LoadX6('m3loadx6_ma2_l0')
        self.proxy.subscribe_status(left_fts)

        self.fts = {'right_arm':right_fts,'left_arm':left_fts}

        #self.pwr=m3w.M3Pwr('m3pwr_pwr003')
        self.pwr=m3f.create_component('m3pwr_pwr003')
        self.proxy.subscribe_status(self.pwr)
        self.proxy.publish_command(self.pwr)

        self.arms = {}
        self.arms['right_arm']=m3.arm.M3Arm('m3arm_ma1')
        self.proxy.subscribe_status(self.arms['right_arm'])

        self.arms['left_arm']=m3.arm.M3Arm('m3arm_ma2')
        self.proxy.subscribe_status(self.arms['left_arm'])

        self.proxy.step()
        self.proxy.step()

    def initialize_gripper(self):
        #self.right_gripper = m3h.M3Gripper('m3gripper_mg0')
        self.right_gripper = m3h.M3Gripper('m3gripper_mg1')
        self.proxy.publish_command(self.right_gripper)
        self.proxy.subscribe_status(self.right_gripper)

    def step(self):
        self.proxy.step()
        for arm in ['left_arm', 'right_arm']:
            z = self.get_wrist_force(arm).A1 # Force vector
            #if arm == 'right_arm':
            #    z = self.get_wrist_force_nano().A1

            for i in range(3):
                xhat, p = kalman_update(self.xhat_force[arm][i],
                                        self.P_force[arm][i],
                                        self.Q_force[arm][i],
                                        self.R_force[arm][i], z[i])
                if abs(z[i] - self.xhat_force[arm][i]) > 3.:
                    xhat = z[i] # not filtering step changes.
                self.xhat_force[arm][i] = xhat
                self.P_force[arm][i] = p

    def step_ros(self):
        r_arm = 'right_arm'
        l_arm = 'left_arm'

        self.cb_lock.acquire()
        r_jep = copy.copy(self.r_jep)
        l_jep = copy.copy(self.l_jep)
        r_alpha = copy.copy(self.arm_settings['right_arm'].stiffness_list)
        l_alpha = copy.copy(self.arm_settings['left_arm'].stiffness_list)
        self.cb_lock.release()

        self.set_jep(r_arm, r_jep)
        self.set_jep(l_arm, l_jep)
        self.set_alpha(r_arm, r_alpha)
        self.set_alpha(l_arm, l_alpha)

        self.step()

        motor_pwr_state = self.is_motor_power_on()

        if not motor_pwr_state:
            self.maintain_configuration()

        q_r = self.get_joint_angles(r_arm)
        q_l = self.get_joint_angles(l_arm)

        if self.floating_arms:
            if self.qr_prev == None or self.floating_arms_counter < 100:
                self.qr_prev = q_r
                self.ql_prev = q_l
                self.floating_arms_counter += 1
            else:
                tol = np.radians([5., 2., 10., 2., 10., 0.03, 0.6])
                r_arr = np.array(q_r)
                l_arr = np.array(q_l)
                prev_r_arr = np.array(self.qr_prev)
                prev_l_arr = np.array(self.ql_prev)

                dqr = np.array(q_r) - np.array(prev_r_arr)
                dql = np.array(q_l) - np.array(prev_l_arr)
                dqr = dqr * (np.abs(dqr) > tol)
                dql = dql * (np.abs(dql) > tol)
                
                r_jep = (np.array(r_jep) + dqr).tolist()
                l_jep = (np.array(l_jep) + dql).tolist()

                self.cb_lock.acquire()
                self.r_jep = copy.copy(r_jep)
                self.l_jep = copy.copy(l_jep)
                self.cb_lock.release()

                change_idxs = np.where(dqr != 0)
                prev_r_arr[change_idxs] = r_arr[change_idxs]
                change_idxs = np.where(dql != 0)
                prev_l_arr[change_idxs] = l_arr[change_idxs]
                self.qr_prev = prev_r_arr.tolist()
                self.ql_prev = prev_l_arr.tolist()

        f_raw_r = self.get_wrist_force(r_arm).A1.tolist()
        f_raw_l = self.get_wrist_force(l_arm).A1.tolist()
        f_r = self.xhat_force[r_arm]
        f_l = self.xhat_force[l_arm]

        # publish stuff over ROS.
        time_stamp = rospy.Time.now()
        h = Header()
        h.stamp = time_stamp

        self.q_r_pub.publish(FloatArray(h, q_r))
        self.q_l_pub.publish(FloatArray(h, q_l))

        self.jep_r_pub.publish(FloatArray(h, r_jep))
        self.jep_l_pub.publish(FloatArray(h, l_jep))

        self.alph_r_pub.publish(FloatArray(h, r_alpha))
        self.alph_l_pub.publish(FloatArray(h, l_alpha))

        h.frame_id = '/torso_lift_link'
        nms = self.joint_names_list['right_arm'] + self.joint_names_list['left_arm']
        pos = q_r + q_l
        self.joint_state_pub.publish(JointState(h, nms, pos,
                                    [0.]*len(pos), [0.]*len(pos)))

        h.frame_id = ar.link_tf_name(r_arm, 7)
        self.force_raw_r_pub.publish(FloatArray(h, f_raw_r))
        self.force_r_pub.publish(FloatArray(h, f_r))
        
        h.frame_id = ar.link_tf_name(l_arm, 7)
        self.force_raw_l_pub.publish(FloatArray(h, f_raw_l))
        self.force_l_pub.publish(FloatArray(h, f_l))

        self.pwr_state_pub.publish(Bool(motor_pwr_state))

    def is_motor_power_on(self):
        return self.pwr.is_motor_power_on(None)

    # Advait, Aug 8, 2009
    # two steps in motors_on and off because with simply one step
    # pwr.is_motor_on does not get the correct value. (I think this is
    # because at the clock edge when motor on command is sent, the power
    # is still off and thus the status is not affected.)
    def motors_off(self, msg=None):
        self.pwr.set_motor_power_off()

    def motors_on(self):
        self.maintain_configuration()
        self.pwr.set_motor_power_on()
        self.step()
        self.step()

    def maintain_configuration(self):
        for arm in ['right_arm','left_arm']:
            q = self.get_joint_angles(arm)
            if self.arm_settings[arm] == None:
                continue
            if 'theta_gc' not in self.arm_settings[arm].control_mode:
                raise RuntimeError('bad control mode: %s', self.arm_settings[arm].control_mode)
            self.set_jep(arm, q)
            self.cb_lock.acquire()
            if arm == 'right_arm':
                self.r_jep = q
            else:
                self.l_jep = q
            self.cb_lock.release()

    def power_on(self):
        self.maintain_configuration()
        self.proxy.make_operational_all()
        self.safeop_things()
        self.pwr.set_motor_power_on()
        self.step()
        self.step()

    def stop(self, msg=None):
        self.pwr.set_motor_power_off()
        self.step()
        self.proxy.stop()

    ##3X1 numpy matrix of forces measured by the wrist FT sensor.
    #(This is the force that the environment is applying on the wrist)
    # @param arm - 'left_arm' or 'right_arm'
    # @return in SI units
    #coord frame - tool tip coord frame (parallel to the base frame in the home position)
    # 2010/2/5 Advait, Aaron King, Tiffany verified that coordinate frame 
    # from Meka is the left-hand coordinate frame.
    def get_wrist_force(self, arm):
        m = []
        lc = self.fts[arm]
        m.append(lc.get_Fx_mN()/1000.)
        m.append(lc.get_Fy_mN()/1000.)
        m.append(-lc.get_Fz_mN()/1000.)        
        m = tr.Rz(math.radians(-30.0))*np.matrix(m).T
        m[1,0] = -m[1,0]
        m[2,0] = -m[2,0]
        return m

    def get_wrist_force_nano(self):
        f = self.r_arm_ftc.read()[0:3, :]
        f = tr.Rz(math.radians(-60.)) * f
        f[1,0] = f[1,0] * -1
        return f


    #-------------------- getting and setting joint angles ------------

    ##
    # @param arm - 'left_arm' or 'right_arm'
    # @return list of 7 joint accelerations in RADIANS/s^2.
    #         according to meka's coordinate frames.
    def get_joint_accelerations(self, arm):
        return self.arms[arm].get_thetadotdot_rad().tolist()

    ##
    # @param arm - 'left_arm' or 'right_arm'
    # @return list of 7 joint velocities in RADIANS/s.
    #         according to meka's coordinate frames.
    def get_joint_velocities(self, arm):
        return self.arms[arm].get_thetadot_rad().tolist()

    def get_joint_angles(self, arm):
        ''' returns list of 7 joint angles in RADIANS.
            according to meka's coordinate frames.
        '''
        return self.arms[arm].get_theta_rad().tolist()

    def l_jep_cb(self, msg):
        self.cb_lock.acquire()
        self.l_jep = msg.data
        self.cb_lock.release()

    def r_jep_cb(self, msg):
        self.cb_lock.acquire()
        self.r_jep = msg.data
        self.cb_lock.release()

    ##
    # @param q - list of 7 joint angles in RADIANS. according to meka's coordinate frames.
    def set_jep(self, arm, q):
        if self.arm_settings[arm] == None:
            return
        if self.arm_settings[arm].control_mode != 'theta_gc' and \
            self.arm_settings[arm].control_mode != 'wrist_theta_gc':
            raise RuntimeError('Bad control mode: %s'%(self.arm_settings[arm].control_mode))

        for i,qi in enumerate(q):
            ## NOTE - meka's set_theta_deg takes angle in radians.
            #self.joint_list_dict[arm][i].set_theta_deg(qi)
            # Not anymore. (Advait Aug 27, 2009)
            self.joint_list_dict[arm][i].set_theta_rad(qi)

        self.cb_lock.acquire()
        if arm == 'right_arm':
            self.r_jep = q
        else:
            self.l_jep = q
        self.cb_lock.release()

    def r_alpha_cb(self, msg):
        self.cb_lock.acquire()
        self.arm_settings['right_arm'].set_stiffness_scale(list(msg.data))
        self.cb_lock.release()

    def l_alpha_cb(self, msg):
        self.cb_lock.acquire()
        self.arm_settings['left_arm'].set_stiffness_scale(list(msg.data))
        self.cb_lock.release()

    def set_alpha(self, arm, alpha):
        jcl = self.joint_list_dict[arm]
        for i,a in enumerate(alpha):
            jcl[i].set_stiffness(a)



if __name__ == '__main__':
    try:
        settings_r = MekaArmSettings(stiffness_list=[0.1939,0.6713,0.748,0.7272,0.75])
        #settings_r = None
        settings_l = MekaArmSettings(stiffness_list=[0.1939,0.6713,0.748,0.7272,0.75])
        #settings_l = None
        cody_arms = MekaArmServer(settings_r, settings_l)

#        print 'hit a key to power up the arms.'
#        k=m3t.get_keystroke()
        cody_arms.power_on()

        while not rospy.is_shutdown():
            cody_arms.step_ros()
            rospy.sleep(0.005)
        cody_arms.stop()

    except m3t.M3Exception:
        print '############################################################'
        print 'In all likelihood the Meka server is not running.'
        print '############################################################'
        raise
    except:
        # only use cody_arms if it was successfully created.
        if 'cody_arms' in locals():
            cody_arms.stop()
        raise





