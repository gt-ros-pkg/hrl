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

import math
import numpy as np
import copy
import sys, time, os
from threading import RLock

import roslib; roslib.load_manifest('hrl_cody_arms')
import rospy

import hrl_lib.viz as hv

from hrl_msgs.msg import FloatArray
from std_msgs.msg import Header, Bool, Empty

from std_srvs.srv import Empty as Empty_srv
from visualization_msgs.msg import Marker


# used in client and server.
def get_joint_name_dict():
    joint_names_list = {}
    joint_names_list['right_arm'] = ['m3joint_ma1_j%d'%i for i in range(7)]
    joint_names_list['left_arm'] = ['m3joint_ma2_j%d'%i for i in range(7)]
    return joint_names_list


class MekaArmClient():
    ##
    # @param arms: object of the ArmKinematics class.
    def __init__(self, arms):
        self.cb_lock = RLock()
        self.r_arm_jep = None
        self.l_arm_jep = None
        self.r_arm_alpha = None
        self.l_arm_alpha = None
        self.r_arm_q = None
        self.l_arm_q = None
        self.r_arm_force = None
        self.r_arm_raw_force = None
        self.l_arm_force = None
        self.l_arm_raw_force = None
        self.pwr_state = False

        self.left_arm_ft = {'force': np.matrix(np.zeros((3,1),dtype='float32')),
                             'torque': np.matrix(np.zeros((3,1),dtype='float32'))}
        self.right_arm_ft = {'force': np.matrix(np.zeros((3,1),dtype='float32')),
                            'torque': np.matrix(np.zeros((3,1),dtype='float32'))}
        self.fts_bias = {'left_arm': self.left_arm_ft, 'right_arm': self.right_arm_ft}
        self.arms = arms

        self.joint_names_list = get_joint_name_dict()

        self.r_jep_cmd_pub = rospy.Publisher('/r_arm/command/jep', FloatArray)
        self.l_jep_cmd_pub = rospy.Publisher('/l_arm/command/jep', FloatArray)
        self.r_alph_cmd_pub = rospy.Publisher('/r_arm/command/joint_impedance_scale', FloatArray)
        self.l_alph_cmd_pub = rospy.Publisher('/l_arm/command/joint_impedance_scale', FloatArray)
        self.stop_pub = rospy.Publisher('/arms/stop', Empty)
        self.motors_off_pub = rospy.Publisher('/arms/command/motors_off', Empty)

        self.cep_marker_pub = rospy.Publisher('/arms/viz/cep', Marker)

        rospy.Subscriber('/r_arm/jep', FloatArray, self.r_arm_jep_cb)
        rospy.Subscriber('/l_arm/jep', FloatArray, self.l_arm_jep_cb)
        rospy.Subscriber('/r_arm/joint_impedance_scale', FloatArray, self.r_arm_alpha_cb)
        rospy.Subscriber('/l_arm/joint_impedance_scale', FloatArray, self.l_arm_alpha_cb)

        rospy.Subscriber('/r_arm/q', FloatArray, self.r_arm_q_cb)
        rospy.Subscriber('/l_arm/q', FloatArray, self.l_arm_q_cb)

        rospy.Subscriber('/r_arm/force', FloatArray, self.r_arm_force_cb)
        rospy.Subscriber('/l_arm/force', FloatArray, self.l_arm_force_cb)
        rospy.Subscriber('/r_arm/force_raw', FloatArray, self.r_arm_raw_force_cb)
        rospy.Subscriber('/l_arm/force_raw', FloatArray, self.l_arm_raw_force_cb)

        rospy.Subscriber('/arms/pwr_state', Bool, self.pwr_state_cb)

        rospy.wait_for_service('toggle_floating_arms')
        self.toggle_floating_arms = rospy.ServiceProxy('toggle_floating_arms', Empty_srv)


        try:
            rospy.init_node('cody_arm_client', anonymous=True)
        except rospy.ROSException:
            pass

    #---------- ROS callbacks -----------------
    def pwr_state_cb(self, msg):
        self.cb_lock.acquire()
        self.pwr_state = msg.data
        self.cb_lock.release()

    def r_arm_jep_cb(self, msg):
        self.cb_lock.acquire()
        self.r_arm_jep = list(msg.data)
        self.cb_lock.release()

        # publish the CEP marker.
        cep, r = self.arms.FK_all('right_arm', self.r_arm_jep)
        o = np.matrix([0.,0.,0.,1.]).T
        cep_marker = hv.single_marker(cep, o, 'sphere',
                        '/torso_lift_link', color=(0., 0., 1., 1.),
                        scale = (0.02, 0.02, 0.02), duration=0.)

        cep_marker.header.stamp = msg.header.stamp
        self.cep_marker_pub.publish(cep_marker)

    def l_arm_jep_cb(self, msg):
        self.cb_lock.acquire()
        self.l_arm_jep = list(msg.data)
        self.cb_lock.release()

    def r_arm_alpha_cb(self, msg):
        self.cb_lock.acquire()
        self.r_arm_alpha = list(msg.data)
        self.cb_lock.release()

    def l_arm_alpha_cb(self, msg):
        self.cb_lock.acquire()
        self.l_arm_alpha = list(msg.data)
        self.cb_lock.release()

    def r_arm_q_cb(self, msg):
        self.cb_lock.acquire()
        self.r_arm_q = list(msg.data)
        self.cb_lock.release()

    def l_arm_q_cb(self, msg):
        self.cb_lock.acquire()
        self.l_arm_q = list(msg.data)
        self.cb_lock.release()

    def r_arm_force_cb(self, msg):
        self.cb_lock.acquire()
        self.r_arm_force = msg.data
        self.cb_lock.release()

    def l_arm_force_cb(self, msg):
        self.cb_lock.acquire()
        self.l_arm_force = msg.data
        self.cb_lock.release()

    def r_arm_raw_force_cb(self, msg):
        self.cb_lock.acquire()
        self.r_arm_raw_force = msg.data
        self.cb_lock.release()

    def l_arm_raw_force_cb(self, msg):
        self.cb_lock.acquire()
        self.l_arm_raw_force = msg.data
        self.cb_lock.release()

    #--------- functions to use -----------------

    ## Returns the current position, rotation of the arm.
    # @param arm 0 for right, 1 for left
    # @return position, rotation
    def end_effector_pos(self, arm):
        q = self.get_joint_angles(arm)
        return self.arms.FK_all(arm, q)

    ##
    # @return list of 7 joint angles.
    def get_joint_angles(self, arm):
        self.cb_lock.acquire()
        if arm == 'right_arm':
            q = copy.copy(self.r_arm_q)
        elif arm == 'left_arm':
            q = copy.copy(self.l_arm_q)
        else:
            self.cb_lock.release()
            raise RuntimeError('Undefined arm: %s'%(arm))
        self.cb_lock.release()
        return q

    def get_wrist_force(self, arm, bias=True, base_frame=False,
                        filtered = True):
        self.cb_lock.acquire()
        if arm == 'right_arm':
            if filtered:
                f = copy.copy(self.r_arm_force)
            else:
                f = copy.copy(self.r_arm_raw_force)
        elif arm == 'left_arm':
            if filtered:
                f = copy.copy(self.l_arm_force)
            else:
                f = copy.copy(self.l_arm_raw_force)
        else:
            self.cb_lock.release()
            raise RuntimeError('Undefined arm: %s'%(arm))
        self.cb_lock.release()

        f_mat = np.matrix(f).T
        if bias:
            f_mat = f_mat - self.fts_bias[arm]['force']
        
        if base_frame:
            q = self.get_joint_angles(arm)
            rot = self.arms.FK_rot(arm, q)
            f_mat = rot*f_mat
        return f_mat
            
    def bias_wrist_ft(self, arm):
        f_list = []
        t_list = []
        print 'Starting biasing...'
        for i in range(20):
            f_list.append(self.get_wrist_force(arm, bias=False))
            rospy.sleep(0.02)

        f_b = np.mean(np.column_stack(f_list), 1)
        # torque is unimplemented.
        t_b = self.get_wrist_torque(arm, bias=False)
        self.fts_bias[arm]['force'] = f_b
        self.fts_bias[arm]['torque'] = t_b
        print 'self.fts_bias[arm][\'force\']', self.fts_bias[arm]['force']
        print 'arm:', arm
        print '...done'

    ##
    # @return list of floats b/w 0 and 1.
    def get_impedance_scale(self, arm):
        self.cb_lock.acquire()
        if arm == 'right_arm':
            sc = copy.copy(self.r_arm_alpha)
        elif arm == 'left_arm':
            sc = copy.copy(self.l_arm_alpha)
        else:
            self.cb_lock.release()
            raise RuntimeError('Undefined arm: %s'%(arm))
        self.cb_lock.release()
        return sc

    ##
    # @param s - list of floats b/w 0 and 1.
    def set_impedance_scale(self, arm, s):
        if arm == 'right_arm': 
            pub = self.r_alph_cmd_pub
        elif arm == 'left_arm':
            pub = self.l_alph_cmd_pub
        else:
            raise RuntimeError('Undefined arm: %s'%(arm))
        time_stamp = rospy.Time.now()
        h = Header()
        h.stamp = time_stamp
        pub.publish(FloatArray(h, s))

    def get_jep(self, arm):
        self.cb_lock.acquire()
        if arm == 'right_arm':
            jep = copy.copy(self.r_arm_jep)
        elif arm == 'left_arm':
            jep = copy.copy(self.l_arm_jep)
        else:
            self.cb_lock.release()
            raise RuntimeError('Undefined arm: %s'%(arm))
        self.cb_lock.release()
        return jep

    ##
    # @param q - list of 7 joint angles in RADIANS. according to meka's coordinate frames.
    # @param duration - for compatibility with the PR2 class.
    def set_jep(self, arm, q, duration=None):
        if arm == 'right_arm': 
            pub = self.r_jep_cmd_pub
        elif arm == 'left_arm':
            pub = self.l_jep_cmd_pub
        else:
            raise RuntimeError('Undefined arm: %s'%(arm))
        time_stamp = rospy.Time.now()
        h = Header()
        h.stamp = time_stamp
        pub.publish(FloatArray(h, q))


    ##
    #Function that commands the arm(s) to incrementally move to a jep
    #@param speed the max angular speed (in radians per second)
    #@return 'reach'
    def go_jep(self, arm, q, stopping_function=None, speed=math.radians(30)):
        if speed>math.radians(90.):
            speed = math.radians(90.)

        qs_list,qe_list,ns_list,qstep_list = [],[],[],[]
        done_list = []
        time_between_cmds = 0.025
        
        #qs = np.matrix(self.get_joint_angles(arm))
        qs = np.matrix(self.get_jep(arm))
        qe = np.matrix(q)
        max_change = np.max(np.abs(qe-qs))

        total_time = max_change/speed
        n_steps = int(total_time/time_between_cmds+0.5)

        qstep = (qe-qs)/n_steps

        if stopping_function != None:
            done = stopping_function()
        else:
            done = False

        step_number = 0
        t0 = rospy.Time.now().to_time()
        t_end = t0
        while done==False:
            t_end += time_between_cmds
            t1 = rospy.Time.now().to_time()

            if stopping_function != None:
                done = stopping_function()
            if step_number < n_steps and done == False:
                q = (qs + step_number*qstep).A1.tolist()
                self.set_jep(arm, q)
            else:
                done = True

            while t1 < t_end:
                if stopping_function != None:
                    done = done or stopping_function()
                rospy.sleep(time_between_cmds/5)
                t1 = rospy.Time.now().to_time()
            step_number += 1

        rospy.sleep(time_between_cmds)
        return 'reach'

    # Expect this to crash the program because sending a stop crashes
    # the meka server
    def stop(self):
        self.stop_pub.publish()

    def is_motor_power_on(self):
        return self.pwr_state

    def go_cep(self, arm, p, rot, speed = 0.10,
                     stopping_function = None, q_guess = None):
        q = self.arms.IK(arm, p, rot, q_guess)
        if q == None:
            print 'IK soln NOT found.'
            print 'trying to reach p= ', p
            return 'fail'
        else:
            q_start = np.matrix(self.get_joint_angles(arm))
            p_start = self.arms.FK(arm, q_start.A1.tolist())
            q_end = np.matrix(q)
    
            dist = np.linalg.norm(p-p_start)
            total_time = dist/speed
            max_change = np.max(np.abs(q_end-q_start))
            ang_speed = max_change/total_time
            return self.go_jep(arm, q, stopping_function, speed=ang_speed)

    ##
    # linearly interpolates the commanded cep.
    # @param arm - 'left_arm' or 'right_arm'
    # @param p - 3x1 np matrix
    # @param rot - rotation matrix
    # @param speed - linear speed (m/s)
    # @param stopping_function - returns True or False
    # @return string (reason for stopping)
    def go_cep_interpolate(self, arm, p, rot=None, speed=0.10,
                                 stopping_function=None):
        rot = None # Rotational interpolation not implemented right now.
        time_between_cmds = 0.025

        q_guess = self.get_jep(arm)
        cep = self.arms.FK(arm, q_guess)
        if rot == None:
            rot = self.arms.FK_rot(arm, q_guess)

        vec = p-cep
        dist = np.linalg.norm(vec)
        total_time = dist/speed
        n_steps = int(total_time/time_between_cmds + 0.5)
        vec = vec/dist
        vec = vec * speed * time_between_cmds
        
        pt = cep
        all_done = False
        i = 0 
        t0 = rospy.Time.now().to_time()
        t_end = t0
        while all_done==False:
            t_end += time_between_cmds
            t1 = rospy.Time.now().to_time()
            pt = pt + vec
            q = self.arms.IK(arm, pt, rot, q_guess)

            if q == None:
                all_done = True
                stop = 'IK fail'
                continue
            self.set_jep(arm, q)
            q_guess = q
            while t1<t_end:
                if stopping_function != None:
                    all_done = stopping_function()
                if all_done:
                    stop = 'Stopping Condition'
                    break
                rospy.sleep(time_between_cmds/5)
                t1 = rospy.Time.now().to_time()

            i+=1
            if i == n_steps:
                all_done = True
                stop = ''
        return stop

    ##  
    # @param vec - displacement vector (base frame)
    # @param q_guess - previous JEP?
    # @return string
    def move_till_hit(self, arm, vec=np.matrix([0.3,0.,0.]).T, force_threshold=3.0,
                      speed=0.1, bias_FT=True):
        unit_vec =  vec/np.linalg.norm(vec)
        def stopping_function():
            force = self.get_wrist_force(arm, base_frame = True)
            force_projection = force.T*unit_vec *-1 # projection in direction opposite to motion.
            if force_projection>force_threshold:
                return True
            elif np.linalg.norm(force)>45.:
                return True
            else:
                return False

        jep = self.get_jep(arm)
        cep, rot = self.arms.FK_all(arm, jep)

        if bias_FT:
            self.bias_wrist_ft(arm)
        time.sleep(0.5)

        p = cep + vec
        return self.go_cep_interpolate(arm, p, rot, speed,
                                       stopping_function)

    def motors_off(self):
        self.motors_off_pub.publish()

#    def step(self):
#        rospy.sleep(0.01)

    #-------- unimplemented functions -----------------

    # leaving this unimplemented for now. Advait Nov 14, 2010.
    def get_joint_velocities(self, arm):
        pass

    # leaving this unimplemented for now. Advait Nov 14, 2010.
    def get_joint_accelerations(self, arm):
        pass

    # leaving this unimplemented for now. Advait Nov 14, 2010.
    def get_joint_torques(self, arm):
        pass

    # leaving this unimplemented for now. Advait Nov 14, 2010.
    def get_wrist_torque(self, arm, bias=True):
        pass

    # leaving this unimplemented for now. Advait Nov 14, 2010.
    def power_on(self):
        pass

    # leaving this unimplemented for now. Advait Nov 14, 2010.
    # something to change and get arm_settings.


if __name__ == '__main__':
    import arms as ar
    import m3.toolbox as m3t
    import hrl_lib.transforms as tr

    r_arm = 'right_arm'
    l_arm = 'left_arm'

    arms = ar.M3HrlRobot(0.16)
    ac = MekaArmClient(arms)

    # print FT sensor readings.
    if False:
        ac.bias_wrist_ft(r_arm)
        while not rospy.is_shutdown():
            f = ac.get_wrist_force(r_arm)
            print 'f:', f.A1
            rospy.sleep(0.05)

    # move the arms.
    if False:
        print 'hit a key to move the arms.'
        k=m3t.get_keystroke()

        rot_mat = tr.rotY(math.radians(-90))
        p = np.matrix([0.3, -0.24, -0.3]).T
    #    q = arms.IK(l_arm, p, rot_mat)
    #    ac.go_jep(l_arm, q)
    #    ac.go_cep(l_arm, p, rot_mat)
        ac.go_cep(r_arm, p, rot_mat)

    #    jep = ac.get_jep(r_arm)
    #    ac.go_jep(r_arm, jep)

        rospy.sleep(0.5)
        raw_input('Hit ENTER to print ee position')
        q = ac.get_joint_angles(r_arm)
        ee = ac.arms.FK(r_arm, q)
        print 'ee:', ee.A1
        print 'desired ee:', p.A1

    if False:
        print 'hit a key to float arms.'
        k=m3t.get_keystroke()
        ac.toggle_floating_arms()

        print 'hit a key to UNfloat arms.'
        k=m3t.get_keystroke()
        ac.toggle_floating_arms()

        #ac.move_till_hit(l_arm)
        #ac.motors_off()
    #    ac.stop()

    if False:
        while not rospy.is_shutdown():
            jep = ac.get_jep(r_arm)
            print 'jep:', jep
            rospy.sleep(0.05)

    if True:
        rospy.sleep(1.)
        isc =  ac.get_impedance_scale(r_arm)
        print isc
        isc[1] = 0.3
        ac.set_impedance_scale(r_arm, isc)
        rospy.sleep(1.)
        isc =  ac.get_impedance_scale(r_arm)
        print isc




