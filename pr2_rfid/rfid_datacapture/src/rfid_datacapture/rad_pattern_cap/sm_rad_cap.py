#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('robotis')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('move_base_msgs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('robotis')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib

from rfid_servoing.msg import ServoAction, ServoGoal
from robotis.srv import MoveAng, MoveAngRequest
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import String
from rfid_datacapture.srv import BagCapture, BagCaptureRequest
from rfid_behaviors.srv import FlapEarsSrv

import rfid_datacapture.utils as rdut

import numpy as np, math


def sm_rfid_servo_approach( yaml_fname ):
    # Create a SMACH state machine
    sm = smach.StateMachine( outcomes = ['succeeded','aborted','preempted'])

    # Open the container
    with sm:

        smach.StateMachine.add(
            'CAPTURE_POSITIONS',
            rdut.YAMLprocPoses( yaml_fname ),
            remapping = {'next_move_pose':'next_move_pose'}, # output
            transitions = {'aborted':'succeeded',
                           'succeeded':'READY_MOVE'})

        smach.StateMachine.add(
            'READY_MOVE',
            rdut.MoveNotify(),
            transitions = {'succeeded':'MOVE_POSITION'})

        smach.StateMachine.add(
            'MOVE_POSITION',
            SimpleActionState( '/move_base',
                               MoveBaseAction,
                               goal_slots = [ 'target_pose' ]),
            remapping = { 'target_pose' : 'next_move_pose' }, # input
            transitions = {'aborted':'MANUAL_SKIP',
                           'preempted':'aborted',
                           'succeeded':'CAPTURE_TAGS'})

        smach.StateMachine.add(
            'MANUAL_SKIP',
            rdut.ManualSkip(),
            transitions = {'succeeded':'CAPTURE_TAGS', # We already manually positioned the robot
                           'aborted':'CAPTURE_POSITIONS'}) # skip this position and go to next

        smach.StateMachine.add(
            'CAPTURE_TAGS',
            rdut.YAMLprocMultitag( yaml_fname ),
            remapping = {'bagfile_name':'bagfile_name',  # output
                         'bagfile_topics':'bagfile_topics', # output
                         'panrate':'panrate',
                         'tagid':'tagid',
                         'tilt_left':'tilt_left',
                         'tilt_right':'tilt_right',
                         'tilt_rate':'tilt_rate',
                         'tilt_block':'tilt_block'}, # output
            transitions = {'aborted':'CAPTURE_POSITIONS', # move to next location
                           'succeeded':'TILT_LEFT'}) # capture bag

        smach.StateMachine.add(
            'TILT_LEFT',
            ServiceState( '/robotis/servo_left_tilt_moveangle',
                          MoveAng,
                          request_slots = ['angle','angvel','blocking']),
            transitions = {'succeeded':'TILT_RIGHT'},
            remapping = {'angle':'tilt_left',
                         'angvel':'tilt_rate',
                         'blocking':'tilt_block'})

        smach.StateMachine.add(
            'TILT_RIGHT',
            ServiceState( '/robotis/servo_right_tilt_moveangle',
                          MoveAng,
                          request_slots = ['angle','angvel','blocking']),
            transitions = {'succeeded':'START_BAG_CAPTURE'},
            remapping = {'angle':'tilt_right',
                         'angvel':'tilt_rate',
                         'blocking':'tilt_block'})


        smach.StateMachine.add(
            'START_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request_slots = ['topics','dest'] ),
            remapping = {'topics':'bagfile_topics',
                         'dest':'bagfile_name'},
            transitions = {'succeeded':'FLAP'})
                          

        # Servoing is a basic state machine.  Success means servoing finished @ obs.
        smach.StateMachine.add(
            'FLAP',
            ServiceState( '/rfid_orient/flap',
                          FlapEarsSrv,
                          request_slots = ['data','panrate']),
            transitions = { 'succeeded': 'STOP_BAG_CAPTURE' },
            remapping = {'data':'tagid',  # input
                         'panrate':'panrate'}) # input

        smach.StateMachine.add(
            'STOP_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request = BagCaptureRequest('','') ),
            transitions = {'succeeded':'TUCK_LEFT'})

        # Tuck Left (non-blocking)
        smach.StateMachine.add(
            'TUCK_LEFT',
            ServiceState( 'robotis/servo_left_pan_moveangle',
                          MoveAng,
                          request = MoveAngRequest( 1.250, 0.2, 0 )), # ang (float), angvel (float), blocking (bool)
            transitions = {'succeeded':'TUCK_RIGHT'})

        # Tuck Right (non-blocking)
        smach.StateMachine.add(
            'TUCK_RIGHT',
            ServiceState( 'robotis/servo_right_pan_moveangle',
                          MoveAng,
                          request = MoveAngRequest( -1.250, 0.2, 0 )), # ang (float), angvel (float), blocking (bool)
            transitions = {'succeeded':'CAPTURE_TAGS'})

    return sm


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--yaml', action='store', type='string', dest='yaml',
                 help='Capture description yaml file', default='')
    opt, args = p.parse_args()

    if opt.yaml == '':
        print 'ERROR: Must specify YAML file.'
        exit()

    rospy.init_node('rfid_servo_capture')

    sm = sm_rfid_servo_approach( opt.yaml )

    sis = IntrospectionServer('RFID_servo_approach', sm, '/SM_RFID_SERVO_APPROACH')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()

    

# python sm_servo_capture_simple.py --yaml datacap_vert.yaml
