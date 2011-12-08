#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_demos')
roslib.load_manifest('robotis')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib

from rfid_servoing.msg import ServoAction, ServoGoal
from rfid_artoolkit.msg import UpCloseAction, UpCloseGoal
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from explore_hrl.msg import ExploreAction, ExploreActionGoal
from rfid_behaviors.srv import HandoffSrv
from rfid_behaviors.srv import FlapEarsSrv
from rfid_behaviors.srv import String_Int32 as OrientSrv
from rfid_behaviors.srv import String_Int32Request as OrientSrvRequest
from robotis.srv import MoveAng, MoveAngRequest

#import rfid_search
import yaml

def sm_rfid_servo_approach():
    # Create a SMACH state machine
    sm = smach.StateMachine( outcomes = ['succeeded','aborted','preempted'],
                             input_keys = ['tagid'])

    # Open the container
    with sm:
        # Initial RFID Ear Flapping
        smach.StateMachine.add(
            'FLAP_EARS',
            ServiceState( '/rfid_orient/flap', FlapEarsSrv ),
            transitions = { 'succeeded' : 'ORIENT' })

        # Orient towards tag
        smach.StateMachine.add(
            'ORIENT',
            ServiceState( '/rfid_orient/orient',
                          OrientSrv,
                          request_slots = ['data']), #request = OrientSrvRequest( 'person      ' )
            transitions = { 'succeeded' : 'SERVO' },
            remapping = {'data':'tagid'}) # input

        # Servoing is a basic state machine.  Success means servoing finished @ obs.
        smach.StateMachine.add(
            'SERVO',
            SimpleActionState( '/rfid_servo/servo_act',
                               ServoAction,
                               goal_slots = ['tagid']), #goal = ServoGoal( 'person      ' ),
            transitions = { 'succeeded': 'TUCK_LEFT' },
            remapping = {'tagid':'tagid'}) # input

        # Tuck Left (non-blocking)
        smach.StateMachine.add(
            'TUCK_LEFT',
            ServiceState( 'robotis/servo_left_pan_moveangle',
                          MoveAng,
                          request = MoveAngRequest( 1.350, 0.2, 0 )), # ang (float), angvel (float), blocking (bool)
            transitions = {'succeeded':'TUCK_RIGHT'})

        # Tuck Right (non-blocking)
        smach.StateMachine.add(
            'TUCK_RIGHT',
            ServiceState( 'robotis/servo_right_pan_moveangle',
                          MoveAng,
                          request = MoveAngRequest( -1.350, 0.2, 0 )), # ang (float), angvel (float), blocking (bool)
            transitions = {'succeeded':'succeeded'})

    return sm


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--tag', action='store', type='string', dest='tagid',
                 help='Tagid to approach', default='person      ')
    opt, args = p.parse_args()

    print 'SERVO APPROACH to ID: \'%s\'' % (opt.tagid)

    rospy.init_node('smach_example_state_machine')

    sm = sm_rfid_servo_approach()

    sis = IntrospectionServer('RFID_servo_approach', sm, '/SM_RFID_SERVO_APPROACH')
    sis.start()

    sm.userdata.tagid = opt.tagid
    outcome = sm.execute()
    
    sis.stop()

    

