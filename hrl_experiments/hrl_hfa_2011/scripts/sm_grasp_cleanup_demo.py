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

#  \author Travis Deyle and Kelsey Hawkins (Healthcare Robotics Lab, Georgia Tech.)

GRASP_LOCATION  =  [ 0.50, -0.30,  0.00]
PLACE_LOCATIONS = [[ 0.58,  0.13,  0.00],
                   [ 0.58,  0.21,  0.00],
                   [ 0.58,  0.29,  0.00]]

import sys

import roslib
roslib.load_manifest('hrl_pr2_experiments')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
import tf.transformations as tft

from pr2_grasp_behaviors.msg import OverheadGraspAction, OverheadGraspSetupAction
from pr2_grasp_behaviors.msg import OverheadGraspGoal, OverheadGraspSetupGoal
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest

# Overhead grasping requres:
#   run: hrl_pr2_gains/change_gains_grasp.sh
#   roslaunch pr2_grasping_behaviors overhead_grasping_server_trained.launch

class NTries(smach.State):
    def __init__(self, n):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                   output_keys=['ntries_counter'])
        self.counter = 0
        self.n = n

    def execute(self, userdata):
        self.counter += 1
        userdata.ntries_counter = self.counter

        if self.counter <= self.n:
            rospy.logout( 'Executing NTries: On #%d of %d' % (self.counter, self.n))
            return 'succeeded'
        else:
            return 'aborted'

def sm_grasp():
    if len(sys.argv) < 2 or sys.argv[1] not in ['r', 'l']:
        print "First arg should be 'r' or 'l'"
        return None
    arm = sys.argv[1]
    if arm == 'r':
        arm_mult = 1
    else:
        arm_mult = -1

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm:
        # Setup arm pose (out of way for perception)
        tgoal = SingleJointPositionGoal()
        #tgoal.position = 0.190  # all the way up is 0.200
        tgoal.position = 0.210  # all the way up is 0.200
        tgoal.min_duration = rospy.Duration( 2.0 )
        tgoal.max_velocity = 1.0
        smach.StateMachine.add(
            'TORSO_SETUP',
            SimpleActionState( 'torso_controller/position_joint_action',
                               SingleJointPositionAction,
                               goal = tgoal),
            transitions = { 'succeeded': 'ARM_UNTUCK' })

        smach.StateMachine.add(
            'ARM_UNTUCK',
            ServiceState('traj_playback/' + arm + '_arm_untuck', TrajPlaybackSrv),
            transitions = { 'succeeded': 'GRASP_BEGIN_SETUP' })

        # Setup arm pose (out of way for perception)
        smach.StateMachine.add(
            'GRASP_BEGIN_SETUP',
            SimpleActionState( arm + '_overhead_grasp_setup',
                               OverheadGraspSetupAction,
                               goal = OverheadGraspSetupGoal()), 
            transitions = { 'succeeded': 'DEMO_START' })

        @smach.cb_interface(outcomes=['succeeded'])
        def wait_for_enter(ud):
            raw_input("Press enter to begin cleanup demo.")
            return 'succeeded'
        smach.StateMachine.add(
            'DEMO_START',
            smach.CBState(wait_for_enter),
            transitions = {'succeeded': 'THREE_OBJECTS'})

        # We will pick up 3 objects.
        smach.StateMachine.add(
            'THREE_OBJECTS',
            NTries( 3 ),
            transitions = {'succeeded':'THREE_TRIES',
                           'aborted':'RESET_ARMS'},
            remapping={'ntries_counter':'object_number'})

        # We will run the grasper at most 3 times.
        grasp_tries = NTries( 3 )
        smach.StateMachine.add(
            'THREE_TRIES',
            grasp_tries,
            transitions = {'succeeded':'GRASP_SETUP',
                           'aborted':'aborted'})

        # Setup arm pose (out of way for perception)
        smach.StateMachine.add(
            'GRASP_SETUP',
            SimpleActionState( arm + '_overhead_grasp_setup',
                               OverheadGraspSetupAction,
                               goal = OverheadGraspSetupGoal()), 
            transitions = { 'succeeded': 'GRASP' })

        def grasp_goal_cb(userdata, goal):
            ############################################################
            # Creating grasp goal
            grasp_goal = OverheadGraspGoal()
            grasp_goal.is_grasp = True
            grasp_goal.disable_head = False
            grasp_goal.disable_coll = False
            grasp_goal.grasp_type = OverheadGraspGoal.VISION_GRASP
            grasp_goal.x = GRASP_LOCATION[0]
            grasp_goal.y = arm_mult * GRASP_LOCATION[1]
            grasp_goal.behavior_name = "overhead_grasp"
            grasp_goal.sig_level = 0.999
            ############################################################
            return grasp_goal

        smach.StateMachine.add(
            'GRASP',
            SimpleActionState( arm + '_overhead_grasp',
                               OverheadGraspAction,
                               goal_cb = grasp_goal_cb),
            transitions = { 'succeeded': 'PLACE',
                            'aborted':'THREE_TRIES' })

        def place_goal_cb(userdata, goal):
            print "object Number", userdata.object_number
            ############################################################
            # Creating place place_goal
            place_goal = OverheadGraspGoal()
            place_goal.is_grasp = False
            place_goal.disable_head = False
            place_goal.disable_coll = False
            place_goal.grasp_type = OverheadGraspGoal.MANUAL_GRASP
            place_goal.x = PLACE_LOCATIONS[userdata.object_number-1][0]
            place_goal.y = arm_mult * PLACE_LOCATIONS[userdata.object_number-1][1]
            place_goal.roll = PLACE_LOCATIONS[userdata.object_number-1][2]
            place_goal.behavior_name = "overhead_grasp"
            place_goal.sig_level = 0.999
            ############################################################
            return place_goal

        def clear_grasp_tries(userdata, status, result):
            grasp_tries.counter = 0

        smach.StateMachine.add(
            'PLACE',
            SimpleActionState( arm + '_overhead_grasp',
                               OverheadGraspAction,
                               goal_cb = place_goal_cb,
                               result_cb = clear_grasp_tries,
                               input_keys = ['object_number']),
            transitions = { 'succeeded': 'THREE_OBJECTS',
                            'aborted':'THREE_OBJECTS' })

        # Setup arm pose (out of way for perception)
        smach.StateMachine.add(
            'RESET_ARMS',
            SimpleActionState( arm + '_overhead_grasp_setup',
                               OverheadGraspSetupAction,
                               goal = OverheadGraspSetupGoal()), 
            transitions = { 'succeeded': 'ARM_TUCK' })

        smach.StateMachine.add(
            'ARM_TUCK',
            ServiceState('traj_playback/' + arm + '_arm_untuck', TrajPlaybackSrv,
                         request=TrajPlaybackSrvRequest(True)),
            transitions = { 'succeeded': 'succeeded' })
            
    return sm



if __name__ == '__main__':
    rospy.init_node('smach_sm_grasp')

    sm = sm_grasp()

    sis = IntrospectionServer('Grasp Cleanup', sm, '/SM_GRASP_CLEANUP')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()

    

