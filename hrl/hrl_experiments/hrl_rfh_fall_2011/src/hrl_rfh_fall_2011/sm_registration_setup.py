#! /usr/bin/python

import sys

import roslib
roslib.load_manifest('hrl_rfh_summer_2011')
roslib.load_manifest('hrl_rfh_fall_2011')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer

from hrl_rfh_summer_2011.sm_approach_only import SMNavApproach
from hrl_rfh_fall_2011.sm_register_head_ellipse import SMEllipsoidRegistration
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal

class SetupTaskController(smach.State):
        def __init__(self):
            smach.State.__init__(self, output_keys=['nav_dist'],
                                 outcomes=['success'])
            self.arm = create_pr2_arm('l', PR2ArmJTransposeTask, end_link="%s_gripper_shaver45_frame")

        def execute(self, userdata):
            # TODO SETUP ARM TODO
            # posture, gains, 
            return 'success'

class SMRegistrationSetup(object):
    def __init__(self):
        self.sm_nav_approach = SMNavApproach()
        self.sm_ell_reg = SMEllipsoidRegistration()

    def get_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
        
        with sm:
            smach.StateMachine.add('NAV_APPROACH',
                    self.sm_nav_approach.get_sm(),
                    transitions={'succeeded' : 'TORSO_SETUP',
                                 'shutdown' : 'aborted'})

            # move torso up
            tgoal = SingleJointPositionGoal()
            tgoal.position = 0.300  # all the way up is 0.300
            tgoal.min_duration = rospy.Duration( 2.0 )
            tgoal.max_velocity = 1.0
            smach.StateMachine.add(
                'TORSO_SETUP',
                SimpleActionState( 'torso_controller/position_joint_action',
                                   SingleJointPositionAction,
                                   goal = tgoal),
                transitions = { 'succeeded': 'UNTUCK' })

            # Untucks the arm
            smach.StateMachine.add(
                'UNTUCK',
                ServiceState( 'traj_playback/untuck_l_arm',
                              TrajPlaybackSrv,
                              request = TrajPlaybackSrvRequest( False )), # if True, reverse trajectory
                transitions = { 'succeeded' : 'HEAD_REG_ALL' })

            smach.StateMachine.add('HEAD_REG_ALL',
                    self.sm_ell_reg.get_sm(),
                    transitions = { 'succeeded' : 'SETUP_TASK_CONTROLLER' })

            smach.StateMachine.add(
                'SETUP_TASK_CONTROLLER',
                SetupTaskController())

        return sm

def main():
    rospy.init_node("sm_registration_setup")
    smrs = SMRegistrationSetup()
    sm = smrs.get_sm()
    rospy.sleep(1)

    sis = IntrospectionServer('registration_setup', sm, '/NAV_APPROACH')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()


if __name__ == "__main__":
    main()
