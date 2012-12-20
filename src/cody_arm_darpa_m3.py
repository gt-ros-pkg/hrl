#
# subscribe to thw joint angles and raw forces topics,  and provide FK
# etc.
#
# Copyright (c) 2012, Georgia Tech Research Corporation
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

# Author: Daehyung Park (Based on old code from Advait Jain)

import math
import numpy as np
import copy, sys

import roslib; roslib.load_manifest('hrl_tactile_controller')
import rospy

from hrl_cody_arms.cody_arm_kinematics import CodyArmKinematics
from equilibrium_point_control.hrl_arm import HRLArm
import hrl_lib.viz as hv

from hrl_msgs.msg import FloatArrayBare
from std_msgs.msg import Header, Bool, Empty

from visualization_msgs.msg import Marker
from hrl_haptic_manipulation_in_clutter_msgs.msg import MechanicalImpedanceParams

class CodyArmClient(HRLArm):
    # arm - 'r' or 'l'
    def __init__(self, arm):
        kinematics = CodyArmKinematics(arm)
        HRLArm.__init__(self, kinematics)

        self.jep_pub = rospy.Publisher('/sim_arm/command/jep', FloatArrayBare)
        self.cep_marker_pub = rospy.Publisher('/sim_arm/viz/cep', Marker)

        self.impedance_pub = rospy.Publisher('/sim_arm/command/joint_impedance',
                                             MechanicalImpedanceParams)

        rospy.Subscriber('/sim_arm/joint_angles', FloatArrayBare,
                         self.joint_states_cb)
        rospy.Subscriber('/sim_arm/joint_angle_rates', FloatArrayBare,
                         self.joint_rates_cb)
        rospy.Subscriber('/sim_arm/jep', FloatArrayBare, self.jep_cb)
        rospy.Subscriber('/sim_arm/joint_impedance', MechanicalImpedanceParams,
                         self.impedance_params_cb)

        rospy.sleep(1.)
        rospy.loginfo("Finished loading SimpleArmManger")

    def jep_cb(self, msg):
        with self.lock:
            self.ep = copy.copy(msg.data)

    def joint_states_cb(self, data):
        with self.lock:
            self.q = copy.copy(data.data)

    def joint_rates_cb(self, data):
        with self.lock:
            self.qdot = copy.copy(data.data)

    def impedance_params_cb(self, data):
        with self.lock:
            self.kp = copy.copy(data.k_p.data)
            self.kd = copy.copy(data.k_d.data)

    #--------- functions to use -----------------

    def publish_rviz_markers(self):
        # publish the CEP marker.
        o = np.matrix([0.,0.,0.,1.]).T
        jep = self.get_ep()
        cep, r = self.kinematics.FK(jep)
        cep_marker = hv.single_marker(cep, o, 'sphere',
                        '/torso_lift_link', color=(0., 0., 1., 1.),
                        scale = (0.02, 0.02, 0.02), duration=0.,
                        m_id=1)
        cep_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(cep_marker)

        q = self.get_joint_angles()
        ee, r = self.kinematics.FK(q)
        ee_marker = hv.single_marker(ee, o, 'sphere',
                        '/torso_lift_link', color=(0., 1., 0., 1.),
                        scale = (0.02, 0.02, 0.02), duration=0.,
                        m_id=2)
        ee_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(ee_marker)


    def get_wrist_force(self, bias=True, base_frame=True,
                        filtered = True):
        with self.lock:
            if filtered:
                f = copy.copy(self.ft_val)
            else:
                f = copy.copy(self.raw_force)

        f_mat = np.matrix(f).T
        f_mat = f_mat[0:3,:] # ignoring the torques.

        if bias:
            f_mat = f_mat - self.fts_bias['force']  #Tiffany added an 's' to 'ft'
        
        if base_frame:
            q = self.get_joint_angles()
            rot = self.kinematics.FK(q)[1]
            f_mat = rot*f_mat
        return f_mat
            
    def bias_wrist_ft(self):
        f_list = []
        t_list = []
        rospy.loginfo('Starting biasing...')
        for i in range(20):
            f_list.append(self.get_wrist_force(bias=False,
                                        base_frame=False))
            rospy.sleep(0.02)

        f_b = np.mean(np.column_stack(f_list), 1)
        # torque is unimplemented.
        t_b = self.get_wrist_torque(bias=False) #Tiffany Nov 8 2011 removed 'arm' arg
        self.fts_bias['force'] = f_b
        self.fts_bias['torque'] = t_b                
        rospy.loginfo('...done')

    # @return array-like of floats b/w 0 and 1.
    def get_impedance_scale(self):
        with self.lock:
            sc = copy.copy(self.alpha)
        return sc

    # @param s - list of floats b/w 0 and 1.
    def set_impedance_scale(self, s):
        time_stamp = rospy.Time.now()
        h = Header()
        h.stamp = time_stamp
        self.alph_cmd_pub.publish(FloatArray(h, s))

    # @param duration - for compatibility with the PR2 class.
    def set_ep(self, jep, duration=None):
        time_stamp = rospy.Time.now()
        h = Header()
        h.stamp = time_stamp
        self.jep_cmd_pub.publish(FloatArray(h, jep))
        self.publish_rviz_markers()

    def is_motor_power_on(self):
        return self.pwr_state

    def motors_off(self):
        self.motors_off_pub.publish()


    #-------- unimplemented functions -----------------

    # leaving this unimplemented for now. Advait Nov 14, 2010.
    def get_joint_accelerations(self, arm):
        pass

    # now implemented - marc Jul 2012
    def get_joint_torques(self):
        return self.torque

    # added this as a safety measure since fabrice skin is not
    # measuring real forces - marc Jul 2012
    def get_joint_motor_temps(self):
        return self.motor_temps

    # leaving this unimplemented for now. Advait Nov 14, 2010.
    def get_wrist_torque(self, bias=True):  #Tiffany Nov 8 2011 removed 'arm' arg
        pass




# these two classes are specifically for the model predictive
# controller that we developed for reaching in clutter with multiple
# contacts.
# We override the get_joint_impedance function to return values that
# the model predictive controller will use.
class CodyArmClient_7DOF(CodyArmClient):
    def __init__(self, arm):
        CodyArmClient.__init__(self, arm)

    # dummy kp values for the wrist, kd still of length five
    def get_joint_impedance(self):
        with self.lock:
            kp_t = [k for k in self.kp]
            kp_t[-1] = 30.
            kp_t.append(30.)
            kp_t.append(30.)
            return kp_t, copy.copy(self.kd)

class CodyArmClient_4DOF(CodyArmClient):
    def __init__(self, arm):
        CodyArmClient.__init__(self, arm)

    def get_joint_impedance(self):
        with self.lock:
            return copy.copy(self.kp[0:4]), copy.copy(self.kd[0:4])



if __name__ == '__main__':
    import m3.toolbox as m3t
    import hrl_lib.transforms as tr
    import optparse

    p = optparse.OptionParser()
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)
    opt, args = p.parse_args()

    if opt.arm == None:
        print 'Please specify an arm_to_use'
        print 'Exiting...'
        sys.exit()

    ac = CodyArmClient(opt.arm)

    # print FT sensor readings.
    if False:
        ac.bias_wrist_ft()
        while not rospy.is_shutdown():
            f = ac.get_wrist_force()
            print 'f:', f.A1
            rospy.sleep(0.05)

    # print joint angles.
    if False:
        while not rospy.is_shutdown():
            #jep = ac.get_ep()
            #print 'jep:', jep
            q = ac.get_joint_angles()
            print 'q:', np.degrees(q).tolist()[1]
            rospy.sleep(0.05)

    # publish end effector marker. useful to verify set_tooltip.
    if False:
        ac.kinematics.set_tooltip(np.matrix([0.,0.,-0.04]).T) # stub with mini45
        while not rospy.is_shutdown():
            ac.publish_rviz_markers()
            rospy.sleep(0.05)

    # print joint velocities
    if False:
        while not rospy.is_shutdown():
            qdot = ac.get_joint_velocities()
            print 'qdot:', np.degrees(qdot)
            rospy.sleep(0.05)

    # testing changing the impedance
    if False:
        rospy.sleep(1.)
        isc =  ac.get_impedance_scale()
        print isc
        isc[1] = 0.3
        ac.set_impedance_scale(isc)
        rospy.sleep(1.)
        isc =  ac.get_impedance_scale()
        print isc







