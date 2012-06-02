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


import roslib; roslib.load_manifest('hrl_pr2_arms')

import tf
import hrl_lib.viz as hv

import rospy

import actionlib

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker

class PR2Arm(HRLArm):
    def __init__(self, arm):
        kinematics = PR2ArmKinematics(arm)
        HRLArm.__init__(self, kinematics)

        self.arms = PR2Arms_kdl() # KDL chain.

        self.joint_names_list = [arm + s for s in ['_shoulder_pan_joint', '_shoulder_lift_joint',
                                                   '_upper_arm_roll_joint', '_elbow_flex_joint',
                                                   '_forearm_roll_joint', '_wrist_flex_joint',
                                                   '_wrist_roll_joint']]

        self.torso_position = None
        self.arm_efforts = None

        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        self.marker_pub = rospy.Publisher(arm+'_arm/viz_markers', Marker)
        self.cep_marker_id = 1

        self.tf_lstnr = tf.TransformListener()

        self.joint_action_client = actionlib.SimpleActionClient(arm+'_arm_controller/joint_trajectory_action',
                                                                JointTrajectoryAction)

#        self.joint_action_client.wait_for_server()


    ##
    # Callback for /joint_states topic. Updates current joint
    # angles and efforts for the arms constantly
    # @param data JointState message recieved from the /joint_states topic
    def joint_states_cb(self, data):
        arm_angles = []
        arm_efforts = []
        jt_idx_list = [0]*7
        for i, jt_nm in enumerate(self.joint_names_list):
            jt_idx_list[i] = data.name.index(jt_nm)

        for i in range(7):
            idx = r_jt_idx_list[i]
            if data.name[idx] != self.joint_names_list[i]:
                raise RuntimeError('joint angle name does not match.')
            arm_angles.append(data.position[idx])
            arm_efforts.append(data.effort[idx])

        with self.lock:
            self.q = arm_angles
            self.arm_efforts = arm_efforts

            torso_idx = data.name.index('torso_lift_joint')
            self.torso_position = data.position[torso_idx]

    def set_ep(self, jep, duration=0.15):
        jep = copy.copy(jep)
        if jep is None or len(jep) != 7:
            raise RuntimeError("set_jep value is " + str(jep))

        with self.lock:
            jtg = JointTrajectoryGoal()
            jtg.trajectory.joint_names = self.joint_names_list
            jtp = JointTrajectoryPoint()
            jtp.positions = jep
            #jtp.velocities = [0 for i in range(len(q))]
            #jtp.accelerations = [0 for i in range(len(q))]
            jtp.time_from_start = rospy.Duration(duration)
            jtg.trajectory.points.append(jtp)
            self.joint_action_client[arm].send_goal(jtg)
            self.ep = jep

    def wrap_angles(self, q):
        for ind in [4, 6]:
            while q[ind] < -np.pi:
                q[ind] += 2*np.pi
            while q[ind] > np.pi:
                q[ind] -= 2*np.pi
        return q


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














