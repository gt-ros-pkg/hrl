#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('hrl_rfh_summer_2011')
roslib.load_manifest('hrl_rfh_fall_2011')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer

from hrl_rfh_fall_2011.sm_register_head_ellipse import SMEllipsoidRegistration
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from hrl_rfh_fall_2011.sm_topic_monitor import TopicMonitor
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

class SetupTaskController(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded'])
            self.ctrl_switcher = ControllerSwitcher()

        def execute(self, userdata):
            self.ctrl_switcher.carefree_switch('l', '%s_cart_jt_task', 
                                               "$(find hrl_rfh_fall_2011)/params/l_jt_task_shaver45.yaml") 
            rospy.sleep(0.3)
            self.arm = create_pr2_arm('l', PR2ArmJTransposeTask, 
                                      controller_name='%s_cart_jt_task', 
                                      end_link="%s_gripper_shaver45_frame")
            setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
            self.arm.set_posture(setup_angles)
            self.arm.set_gains([200, 800, 800, 80, 80, 80], [15, 15, 15, 1.2, 1.2, 1.2])
            return 'succeeded'

class SMRegistrationSetup(object):
    def __init__(self):
        self.sm_ell_reg = SMEllipsoidRegistration()

    def get_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
        
        with sm:

            smach.StateMachine.add('HEAD_REG_ALL',
                    self.sm_ell_reg.get_sm(),
                    transitions = { 'succeeded' : 'L_UNTUCK' })

            # Untucks the left arm
            smach.StateMachine.add(
                'L_UNTUCK',
                ServiceState( 'traj_playback/untuck_l_arm',
                              TrajPlaybackSrv,
                              request = TrajPlaybackSrvRequest( False )), # if True, reverse trajectory
                transitions = { 'succeeded' : 'SETUP_TASK_CONTROLLER' })

            smach.StateMachine.add(
                'SETUP_TASK_CONTROLLER',
                SetupTaskController(),
                transitions = { 'succeeded' : 'HEAD_REG_ALL'})

        return sm

def main():
    rospy.init_node("sm_registration_setup")
    smrs = SMRegistrationSetup()
    sm = smrs.get_sm()
    if False:
        sm.set_initial_state(['SETUP_TASK_CONTROLLER'])
    rospy.sleep(1)

    sis = IntrospectionServer('registration_setup', sm, '/HEAD_REG_ALL')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()


if __name__ == "__main__":
    main()
