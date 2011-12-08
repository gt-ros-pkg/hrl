#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('move_base_msgs')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('rfid_hardware')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib

from move_base_msgs.msg import MoveBaseAction
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from rfid_datacapture.srv import BagCapture, BagCaptureRequest
from hrl_rfid.srv import RfidSrv

import rfid_datacapture.utils as rdut

import numpy as np, math


class RfidStart(smach.State):
    def __init__(self, srv_path ):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             input_keys = ['tagid'])
        self.srv_path = srv_path
        self.init = False

    def execute(self, userdata):
        if not self.init:
            rospy.wait_for_service( self.srv_path )
            self.srv = rospy.ServiceProxy( self.srv_path, RfidSrv )
            self.init = True
            
        rv = self.srv([ 'track', userdata.tagid ])
        if rv:
            return 'succeeded'
        else:
            return 'aborted' # done!

class RfidStop(smach.State):
    def __init__(self, srv_path ):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.srv_path = srv_path
        self.init = False

    def execute(self, userdata):
        if not self.init:
            rospy.wait_for_service( self.srv_path )
            self.srv = rospy.ServiceProxy( self.srv_path, RfidSrv )
            self.init = True
            
        rv = self.srv([ 'stop' ])
        if rv:
            return 'succeeded'
        else:
            return 'aborted' # done!


def sm_cap_360( yaml_fname ):
    # Create a SMACH state machine
    sm = smach.StateMachine( outcomes = ['succeeded','aborted','preempted'],
                             input_keys = ['track_mode'])

    # Open the container
    with sm:

        # Lots of head moving -- abstract into function
        def PointAdd( x, y, z, dur, state, res ):
            pgoal = PointHeadGoal()
            pgoal.target.header.frame_id = '/torso_lift_link'
            pgoal.target.point.x = x
            pgoal.target.point.y = y
            pgoal.target.point.z = z
            pgoal.min_duration = rospy.Duration( dur )
            pgoal.max_velocity = 1.0
            smach.StateMachine.add(
                state,
                SimpleActionState( '/head_traj_controller/point_head_action',
                                   PointHeadAction,
                                   goal = pgoal ),
                transitions = { 'succeeded' : res })
            return

        PointAdd(  1.0,   0.0, 0.35,  5.0, 'INIT_HEAD', 'CAP_START' ) # Go to safe initial conditions
        PointAdd( -1.0, -0.25, 0.35,  7.0, 'CAP_START', 'CAPTURE_POSITIONS' ) # Prepare for lots of "neck craning"

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

        # This isn't realy necessary, but it provides a nice way to reuse code.
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
                           'succeeded':'START_BAG_CAPTURE'}) # capture bag

        smach.StateMachine.add(
            'START_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request_slots = ['topics','dest'] ),
            remapping = {'topics':'bagfile_topics',
                         'dest':'bagfile_name'},
            transitions = {'succeeded':'RFID_START'})

        # Initialize RFID reader!
        #   rosservice call /rfid/head_mode -- ['track','OrangeMedBot']

        # Am having issues with service states with request_cb, so just making my own....
        smach.StateMachine.add(
            'RFID_START',
            RfidStart( '/rfid/head_mode' ),
            remapping = {'tagid':'tagid'},
            transitions = {'succeeded':'LOOK_LEFT'})
                          
        PointAdd( -1.0,  0.25, 0.35, 27.0, 'LOOK_LEFT', 'RFID_STOP' )

        smach.StateMachine.add(
            'RFID_STOP',
            RfidStop( '/rfid/head_mode' ),
            transitions = {'succeeded':'STOP_BAG_CAPTURE'})
        
        smach.StateMachine.add(
            'STOP_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request = BagCaptureRequest('','') ),
            transitions = {'succeeded':'LOOK_RIGHT'})

        PointAdd( -1.0, -0.25, 0.35,  8.0, 'LOOK_RIGHT', 'CAPTURE_TAGS' )

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

    rospy.init_node('rfid_head_capture')

    sm = sm_cap_360( opt.yaml )

    sm.userdata.track_mode = 'track'
    outcome = sm.execute()
    

    

