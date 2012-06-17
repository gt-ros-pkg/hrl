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
import rospy

from equilibrium_point_control.hrl_arm import HRLArm
from pr2_arm_kinematics_darpa_m3 import PR2ArmKinematics

import tf
import hrl_lib.viz as hv

import actionlib

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand

from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker

class PR2Arm(HRLArm):
    def __init__(self, arm):
        kinematics = PR2ArmKinematics(arm)
        HRLArm.__init__(self, kinematics)

        self.joint_names_list = [arm + s for s in ['_shoulder_pan_joint', '_shoulder_lift_joint',
                                                   '_upper_arm_roll_joint', '_elbow_flex_joint',
                                                   '_forearm_roll_joint', '_wrist_flex_joint',
                                                   '_wrist_roll_joint']]

        self.torso_position = None
        self.arm_efforts = None

        self.kp = [rospy.get_param(arm+'_arm_controller/gains/'+nm+'/p') for nm in self.joint_names_list]
        self.kp[-1] = 50.
        self.kp[-2] = 50.
        self.kp[-3] = 50.

        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        self.marker_pub = rospy.Publisher(arm+'_arm/viz/markers', Marker)
        self.cep_marker_id = 1

        self.tf_lstnr = tf.TransformListener()

        self.joint_action_client = actionlib.SimpleActionClient(arm+'_arm_controller/joint_trajectory_action',
                                                                JointTrajectoryAction)

        self.gripper_action_client = actionlib.SimpleActionClient(arm+'_gripper_controller/gripper_action',
                                                                  Pr2GripperCommandAction)
#        self.joint_action_client.wait_for_server()
#        self.gripper_action_client.wait_for_server()


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
            idx = jt_idx_list[i]
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
            self.joint_action_client.send_goal(jtg)
            self.ep = jep

    def wrap_angles(self, q):
        for ind in [4, 6]:
            while q[ind] < -np.pi:
                q[ind] += 2*np.pi
            while q[ind] > np.pi:
                q[ind] -= 2*np.pi
        return q


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


    #-------- gripper functions ------------
    def move_gripper(self, amount=0.08, effort = 15):
        self.gripper_action_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=amount,
                                                                                    max_effort = effort)))

    def open_gripper(self):
        self.move_gripper(0.08, -1)

    def close_gripper(self, effort = 15):
        self.move_gripper(0.0, effort)


if __name__ == '__main__':
    rospy.init_node('pr2_arms_test')
    robot = PR2Arm('r')

    if False:
        jep = [0.] * 7
        jep = np.radians([-30, 0, -90, -60, 0, 0, 0])
        rospy.loginfo('Going to home location.')
        raw_input('Hit ENTER to go')
        robot.set_ep(jep, duration=2.)

    if False:
        # simple go_jep example
        roslib.load_manifest('equilibrium_point_control')
        import equilibrium_point_control.epc as epc
        epcon = epc.EPC(robot)

        while robot.get_joint_angles() == None:
            rospy.sleep(0.1)

        q = robot.get_joint_angles()
        robot.set_ep(q)

        jep = [0.] * 7
#        jep = np.radians([-30, 0, -90, -60, 0, 0, 0])
        epcon.go_jep(jep, speed=math.radians(30.))
    
    if True:
        import hrl_lib.util as ut
        ut.get_keystroke('Hit ENTER to open gripper')
        robot.open_gripper()
        ut.get_keystroke('Hit ENTER to close gripper')
        robot.close_gripper()






