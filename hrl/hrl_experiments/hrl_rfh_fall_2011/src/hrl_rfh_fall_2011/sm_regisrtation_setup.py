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

class SMRegistrationSetup(object):
    def __init__(self):
        self.sm_nav_approach = SMNavApproach()
        self.sm_ell_reg = SMEllipsoidRegistration()

    def get_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded','preempted','shutdown'])
        
        with sm:
            smach.StateMachine.add('NAV_APPROACH',
                    self.sm_nav_approach.get_sm(),
                    transitions={'succeeded' : 'HEAD_REG_ALL'})

            smach.StateMachine.add('HEAD_REG_ALL',
                    self.sm_ell_reg.get_sm())

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
