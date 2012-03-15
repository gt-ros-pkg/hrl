#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('hrl_object_fetching')
roslib.load_manifest('hrl_trajectory_playback')
roslib.load_manifest('pr2_overhead_grasping')
roslib.load_manifest('pr2_controllers_msgs')
import rospy

import smach
import actionlib

from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from rfid_behaviors.srv import NextBestVantage
from rfid_behaviors.srv import HandoffSrv, HandoffSrvRequest
from rfid_behaviors.srv import FloatFloat_Int32Request as RotateBackupSrvRequest
from rfid_behaviors.srv import FloatFloat_Int32 as RotateBackupSrv
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

import sm_rfid_delivery
#from sm_next_best_vantage import BestVantage
import sm_fetch
import sm_rfid_explore

class PrintStr(smach.State):
    def __init__(self, ins = 'Hello'):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.ins = ins

    def execute(self, userdata):
        rospy.logout( 'Executing PrintStr: %s' % self.ins )
        rospy.sleep(4.0)
        return 'succeeded'



def cousins_demo():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'explore_id',  # should be ''
                                           'obj_id',
                                           'person_id',
                                           'explore_radius' ])

    with sm:
        sm_search = sm_rfid_explore.sm_search()
        smach.StateMachine.add(
            'RFID_SEARCH',  # outcomes: succeded, aborted, preempted
            sm_search,
            remapping = { 'tagid' : 'explore_id',  # input
                          'explore_radius' : 'explore_radius' },
            transitions={'succeeded':'BEST_VANTAGE_OBJ'})

        # Get best vantage for obj.
        # The NextBestVantage was initialized in the search.
        smach.StateMachine.add(
            'BEST_VANTAGE_OBJ',
            ServiceState( '/rfid_recorder/best_vantage',
                          NextBestVantage,
                          request_slots = ['tagid'],
                          response_slots = ['best_vantage']),
            remapping = {'best_vantage':'best_vantage_obj', # output
                         'tagid':'obj_id'}, # input
            transitions = {'succeeded':'MOVE_VANTAGE_OBJ'})


        smach.StateMachine.add(
            'MOVE_VANTAGE_OBJ',
            SimpleActionState( '/move_base',
                               MoveBaseAction,
                               goal_slots = [ 'target_pose' ]),
            remapping = { 'target_pose' : 'best_vantage_obj' }, # input
            transitions = {'aborted':'BEST_VANTAGE_OBJ',
                           'preempted':'aborted',
                           'succeeded':'FETCH'})
        # # FETCH
        # smach.StateMachine.add('FETCH',PrintStr( 'Fetching object' ),
        #                        transitions = { 'succeeded' : 'BEST_VANTAGE_PERSON' })

        # Fetch
        smach.StateMachine.add(
            'FETCH',
            sm_fetch.sm_fetch_object(),
            remapping = {'tagid':'obj_id'},
            transitions = {'succeeded':'POINT_HEAD',
                           'aborted':'BACKUP'})

        # Backup (60cm)
        smach.StateMachine.add(
            'BACKUP',
            ServiceState( '/rotate_backup',
                          RotateBackupSrv,
                          request = RotateBackupSrvRequest(0.0, -0.50)), 
            transitions = { 'succeeded':'PRE_STOW' })

        smach.StateMachine.add(
            'PRE_STOW',
            ServiceState( 'rfid_handoff/stow_grasp',
                          HandoffSrv,
                          request = HandoffSrvRequest()), 
            transitions = { 'succeeded':'POINT_HEAD' })

        # Point Head Down (eventaully roll this and perceive table into own sm?)
        pgoal = PointHeadGoal()
        pgoal.target.header.frame_id = '/torso_lift_link'
        pgoal.target.point.x = 0.50
        pgoal.target.point.z = 0.30
        pgoal.min_duration = rospy.Duration(0.6)
        pgoal.max_velocity = 1.0
        smach.StateMachine.add(
            'POINT_HEAD',
            SimpleActionState( '/head_traj_controller/point_head_action',
                               PointHeadAction,
                               goal = pgoal ),
            transitions = { 'succeeded' : 'BEST_VANTAGE_PERSON' })
        

        # Get best vantage for obj.
        # The NextBestVantage was initialized in the search.
        smach.StateMachine.add(
            'BEST_VANTAGE_PERSON',
            ServiceState( '/rfid_recorder/best_vantage',
                          NextBestVantage,
                          request_slots = ['tagid'],
                          response_slots = ['best_vantage']),
            remapping = {'best_vantage':'best_vantage_person', # output
                         'tagid':'person_id'}, # input
            transitions = {'succeeded':'MOVE_VANTAGE_PERSON'})

        smach.StateMachine.add(
            'MOVE_VANTAGE_PERSON',
            SimpleActionState( '/move_base',
                               MoveBaseAction,
                               goal_slots = [ 'target_pose' ]),
            remapping = { 'target_pose' : 'best_vantage_person' }, # input
            transitions = {'aborted':'BEST_VANTAGE_PERSON',
                           'preempted':'aborted',
                           'succeeded':'DELIVERY'})

        sm_delivery = sm_rfid_delivery.sm_delivery()
        smach.StateMachine.add(
            'DELIVERY', # outcomes: succeeded, aborted, preempted
            sm_delivery,
            remapping = { 'tagid' : 'person_id'}, #input
            transitions = { 'succeeded': 'succeeded' })
            
    return sm



if __name__ == '__main__':
    rospy.init_node('smach_rfid_explore')

    sm = cousins_demo()

    sis = IntrospectionServer('sm_cousins_demo', sm, '/SM_COUSINS_DEMO')
    sis.start()

    rospy.logout( 'READY TO RUN COUSINS DEMO' )

    sm.userdata.explore_id = ''
    sm.userdata.obj_id = 'SpectrMedBot'
    sm.userdata.person_id = 'person      '
    sm.userdata.explore_radius = 1.8
    # sm.userdata.explore_rfid_reads = []
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

    

