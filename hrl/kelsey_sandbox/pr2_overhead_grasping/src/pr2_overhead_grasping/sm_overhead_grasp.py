#! /usr/bin/python
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

#  \author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)

import roslib
roslib.load_manifest('pr2_overhead_grasping')
roslib.load_manifest('tf')
roslib.load_manifest('hrl_table_detect')
roslib.load_manifest('rfid_behaviors')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
import tf.transformations as tft

from rfid_behaviors.srv import HandoffSrv
from hrl_table_detect.srv import DetectTableInst, DetectTableInstRequest
from pr2_overhead_grasping.msg import OverheadGraspAction, OverheadGraspGoal
from pr2_overhead_grasping.msg import OverheadGraspSetupAction, OverheadGraspSetupGoal
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal

# Overhead grasping requres:
#   run: hrl_pr2_gains/change_gains_grasp.py
#   roslaunch pr2_overhead_grasping overhead_grasping_server.launch

class NTries(smach.State):
    def __init__(self, n):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.counter = 0
        self.n = n

    def execute(self, userdata):
        self.counter += 1

        if self.counter <= self.n:
            rospy.logout( 'Executing NTries: On #%d of %d' % (self.counter, self.n))
            return 'succeeded'
        else:
            return 'aborted'


def sm_grasp():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm:
        # Setup arm pose (out of way for perception)
        tgoal = SingleJointPositionGoal()
        tgoal.position = 0.190  # all the way up is 0.200
        tgoal.min_duration = rospy.Duration( 2.0 )
        tgoal.max_velocity = 1.0
        smach.StateMachine.add(
            'TORSO_SETUP',
            SimpleActionState( 'torso_controller/position_joint_action',
                               SingleJointPositionAction,
                               goal = tgoal),
            transitions = { 'succeeded': 'THREE_TRIES' })

        # We will run the grasper at most 3 times.
        smach.StateMachine.add(
            'THREE_TRIES',
            NTries( 3 ),
            transitions = {'succeeded':'PERCEIVE_SETUP',
                           'aborted':'aborted'})

        # get hand out of face
        smach.StateMachine.add(
            'PERCEIVE_SETUP',
            ServiceState( '/rfid_handoff/grasp', HandoffSrv ),
            transitions = { 'succeeded' : 'PERCEIVE_OBJECT' })
            

        # Setment objects
        smach.StateMachine.add(
            'PERCEIVE_OBJECT',
            ServiceState( '/obj_segment_inst',
                          DetectTableInst,
                          request = DetectTableInstRequest( 1.0 ),
                          response_slots = ['grasp_points']), # PoseArray
            transitions = {'succeeded':'GRASP_SETUP'},
            remapping = {'grasp_points':'object_poses'}) #output

        # Setup arm pose (out of way for perception)
        smach.StateMachine.add(
            'GRASP_SETUP',
            SimpleActionState( 'overhead_grasp_setup',
                               OverheadGraspSetupAction,
                               goal = OverheadGraspSetupGoal( True )), # disable new look
            transitions = { 'succeeded': 'GRASP' })

        # Actually perform grasp of some object in front of robot on table
        def grasp_goal_cb( userdata, goal ):
            # grasp_poses is PoseArray in base_link
            mgp = userdata.grasp_poses.poses[0]
            
            ggoal = OverheadGraspGoal()
            ggoal.is_grasp = True
            ggoal.grasp_type = OverheadGraspGoal.MANUAL_GRASP
            
            ggoal.x = mgp.position.x + 0.05 # Converts base_link -> torso_lift_link (only for x,y)
            ggoal.y = mgp.position.y

            o = mgp.orientation
            r,p,y = tft.euler_from_quaternion(( o.x, o.y, o.z, o.w ))
            ggoal.rot = y
            
            return ggoal

        smach.StateMachine.add(
            'GRASP',
            SimpleActionState( 'overhead_grasp',
                               OverheadGraspAction,
                               goal_cb = grasp_goal_cb,
                               input_keys = ['grasp_poses']),
            remapping = {'grasp_poses':'object_poses'},
            transitions = { 'succeeded': 'succeeded',
                            'aborted':'THREE_TRIES' })
            
    return sm



if __name__ == '__main__':
    rospy.init_node('smach_sm_grasp')

    sm = sm_grasp()

    sis = IntrospectionServer('Grasping', sm, '/SM_GRASPING')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()

    

