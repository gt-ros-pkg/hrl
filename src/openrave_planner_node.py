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

# Author: Marc Killpack

import numpy as np, math
from threading import RLock, Timer
import sys, copy

import roslib; roslib.load_manifest('hrl_tactile_controller')
import rospy

from hrl_arm import HRLArm
from pr2_arm_kinematics_darpa_m3 import PR2ArmKinematics
from hrl_msgs.msg import FloatArrayBare

import tf
import hrl_lib.viz as hv

import actionlib

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand

from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import Marker

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
from openravepy import *
import time


class OpenRavePlanner():
    def __init__(self, arm, chain_name):
        self.kinematics = PR2ArmKinematics(arm)

        self.joint_names_list = [arm + s for s in ['_shoulder_pan_joint', '_shoulder_lift_joint',
                                                   '_upper_arm_roll_joint', '_elbow_flex_joint',
                                                   '_forearm_roll_joint', '_wrist_flex_joint',
                                                   '_wrist_roll_joint']]

        rospy.Subscriber('/haptic_mpc/robot_state', haptic_msgs.RobotHapticState, self.update_joint_angles)
        rospy.Subscriber('/haptic_mpc/goal_pose', PoseStamped, self.update_T_des)

        self.plan_pub = rospy.Publisher('/haptic_mpc/openrave/joint_trajectory_plan', JointTrajectory)
        self.q = None
        self.lock = RLock()
        self.traj = None

        self.env=Environment()
        #self.env.SetViewer('qtcoin')
        self.env.Load('./empty.env.xml')

        self.robot = self.env.GetRobots()[0]
        self.manip = self.robot.SetActiveManipulator(chain_name)
        self.torso_lift_link = self.robot.GetLink('torso_lift_link')
        
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()


        with self.env:
            self.robot.SetActiveDOFs(self.ikmodel.manip.GetArmIndices())
            self.basemanip=interfaces.BaseManipulation(self.robot)
            self.robot.WaitForController(0)

        self.T_des = None

        rospy.logwarn("waiting for a desired pose ...")
        while self.T_des == None:
            rospy.sleep(0.1)
        rospy.logwarn("got it! Moving on so to speak.")

        # self.T_des = np.array([[ 1. , 0. , 0. ,  0.8 ],
        #                        [ 0. , 1. , 0. ,  -0.6],
        #                        [ 0. , 0. , 1. ,  0.1],
        #                        [ 0. , 0. , 0. ,  1.]])


    ##
    # Callback for robot haptic state message. Updates current joint
    # angles for use with openrave birrt planner
    # @param data RobotHapticState message
    def update_joint_angles(self, msg):
        with self.lock:
            self.q = copy.copy(msg.joint_angles)


    def update_T_des(self, msg):
        with self.lock:
            if msg.header.frame_id != "/torso_lift_link":
                rospy.logerr("pose isn't in torso_lift_link frame, give me a good pose ...")
            else:
                self.T_des = tf.transformations.quaternion_matrix([msg.pose.orientation.x, 
                                                                   msg.pose.orientation.y,
                                                                   msg.pose.orientation.z,
                                                                   msg.pose.orientation.w])
                self.T_des[0,3] = msg.pose.position.x
                self.T_des[1,3] = msg.pose.position.y
                self.T_des[2,3] = msg.pose.position.z

    def get_plan(self):
        with self.lock:
            self.robot.SetActiveDOFValues(self.q)
          


            ee, r = self.kinematics.FK(self.q)
            start = time.time()

            #h=misc.DrawAxes(env, robot.GetLinks()[16].GetTransform())

            with self.env:
                T_torso_world = self.torso_lift_link.GetTransform()

                T_des_torso = copy.copy(self.T_des)

                R_offset = np.array([[0., 0., 1.],
                                     [0., 1., 0.],
                                     [-1., 0., 0.]])

                T_des_torso[0:3,0:3] = np.dot(T_des_torso[0:3,0:3], R_offset)

                T = np.dot(T_torso_world, T_des_torso)

                # check if ik solutions exist
                sol0=self.manip.FindIKSolution(T,IkFilterOptions.CheckEnvCollisions)
                if sol0 == None:
                    print "joint angles are now:", self.robot.GetActiveDOFValues()                    
                    rospy.logerr('There is no IK solution for the desired Pose')
                    return 

            #when done debugging, execute should be set to "False"
            #seedik almost doubles time to solve
            traj = self.basemanip.MoveToHandPosition(matrices=[T], execute=False, outputtrajobj=True, seedik=True)  
            planningutils.RetimeAffineTrajectory(traj,maxvelocities=1.*np.ones(7),maxaccelerations=1.*np.ones(7))  # these should probably be updated for our actual desired velocities and accel.

            end = time.time()

            print "approx. calculation time is ", end-start

            joint_traj = JointTrajectory()

            for i in xrange(traj.GetNumWaypoints()):
                joint_traj_pt = JointTrajectoryPoint()
                joint_traj_pt.positions = (traj.GetWaypoint(i))[0:7] 
                joint_traj_pt.time_from_start = rospy.Duration(.15)
                joint_traj.points.append(joint_traj_pt)

            joint_traj.header.stamp = rospy.Time.now()
            joint_traj.header.frame_id = '/torso_lift_link'
            joint_traj.joint_names = self.joint_names_list

            self.plan_pub.publish(joint_traj)


if __name__ == '__main__':

    rospy.loginfo('starting openrave node ...')
    rospy.init_node('pr2_openrave_node_test')
    planner = OpenRavePlanner('r', 'rightarm')
    rospy.loginfo('now spinning openrave node ...')

    while planner.q == None:
        rospy.sleep(0.05)

    while not rospy.is_shutdown():
        planner.get_plan()
        rospy.spin()
        rospy.sleep(0.1)


    #def __init__(self, arm, chain_name):

