#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_demos')
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
from rfid_behaviors.srv import OrientSrv, OrientSrvRequest
from robotis.srv import MoveAng, MoveAngRequest

# import rfid_search
import yaml



def sm_delivery():
    # Create a SMACH state machine
    sm = smach.StateMachine( outcomes = ['succeeded','aborted','preempted'],
                             input_keys = ['tagid'])

    # Open the container
    with sm:

        # smach.StateMachine.add(
        #     'EXPLORE',
        #     Explore(),
        #     transitions = { 'succeeded' : 'FLAP_EARS',
        #                     'aborted' : 'RECOVER_ONCE' })

        # smach.StateMachine.add(
        #     'RECOVER_ONCE',
        #     RecoverOnce(),
        #     transitions = { 'succeeded' : 'EXPLORE',
        #                     'aborted' : 'aborted' })

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
                          request_slots = ['data'], 
                          input_keys=['data']), # tagid (string)
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


        # # ARTag scanner.  Result is actually stored in result message "status"
        # # Remap based on upclose_action result
        # def ar_detect_cb( userdata, status, result ):
        #     if result.status == 'SUCCEEDED':  # Actionlib Succeeded: Found the tag!
        #         userdata.tag_pos = result.ps
        #         return 'succeeded'
        #     elif result.status == 'FAILED':   # Actionlib Succeed, but no tag found.
        #         return 'recover'
        #     elif result.status == 'PREEMPTED': # Actionlib was preempted (higher level)
        #         return 'preempted'
        #     else: # result.status == 'RESERVO' # Obstacle from servo cleared.  Self-preempt to reservo.
        #         return 'reservo'
            
        # smach.StateMachine.add(
        #     'ARTAG',
        #     SimpleActionState( '/rfid_artoolkit/upclose_act',
        #                        UpCloseAction,
        #                        goal_slots = ['tagid'], #goal = UpCloseGoal( 'person      ' )
        #                        output_keys = ['tag_pos'],
        #                        outcomes = ['succeeded', 'recover', 'preempted', 'reservo'],
        #                        result_cb = ar_detect_cb ),
        #     transitions = { 'recover' : 'aborted',
        #                     'reservo' : 'HEAD_REPOS',
        #                     'succeeded': 'TUCK_LEFT' },
        #     remapping = {'tagid':'tagid'}) # input


        # # Reposition the head:
        # goal = PointHeadGoal()
        # goal.target.header.frame_id = '/torso_lift_link'
        # goal.target.point.x = 0.54
        # goal.target.point.z = 0.35
        # goal.min_duration = rospy.Duration(0.6)
        # smach.StateMachine.add(
        #     'HEAD_REPOS',
        #     SimpleActionState( '/head_traj_controller/point_head_action',
        #                        PointHeadAction,
        #                        goal = goal ),
        #     transitions = { 'succeeded' : 'SERVO' })


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
            transitions = {'succeeded':'HANDOFF'})

        # Handoff if tag found
        smach.StateMachine.add(
            'HANDOFF',
            ServiceState( '/rfid_handoff/handoff', HandoffSrv ),
            transitions = { 'succeeded' : 'succeeded' })
                

    # Execute SMACH plan
    return sm


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    sm = sm_delivery()

    sis = IntrospectionServer('RFID_delivery', sm, '/SM_ROOT')
    sis.start()

    sm.userdata.tagid = 'person      '
    outcome = sm.execute()
    
    rospy.spin()
    sis.stop()

    

