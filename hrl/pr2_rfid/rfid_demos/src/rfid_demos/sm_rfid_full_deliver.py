#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_demos')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib

import sm_rfid_explore
import sm_next_best_vantage
import sm_rfid_delivery
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction


def full_delivery():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'tagid', 'explore_radius' ])

    # Open the container
    with sm:
        sm_search = sm_rfid_explore.sm_search()
        smach.StateMachine.add(
            'RFID_SEARCH',  # outcomes: succeded, aborted, preempted
            sm_search,
            remapping = { 'tagid' : 'tagid',  # input
                          'explore_radius' : 'explore_radius',   # input
                          'explore_rfid_reads' : 'explore_rfid_reads' }, # output
            transitions={'succeeded':'BEST_VANTAGE'})

        sm_vantage = sm_next_best_vantage.sm_best_vantage()
        smach.StateMachine.add(
            'BEST_VANTAGE', # outcomes: succeeded, aborted, preempted
            sm_vantage,
            remapping = { 'tagid' : 'tagid', # input
                          'rfid_reads' : 'explore_rfid_reads' }, # input
            transitions = {'succeeded':'DELIVERY'})

        sm_delivery = sm_rfid_delivery.sm_delivery()
        smach.StateMachine.add(
            'DELIVERY', # outcomes: succeeded, aborted, preempted
            sm_delivery,
            remapping = { 'tagid' : 'tagid'}, #input
            transitions = { 'succeeded': 'succeeded' })
            

    # Execute SMACH plan
    return sm


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    sm = full_delivery()

    sis = IntrospectionServer('RFID_full_delivery', sm, '/SM_ROOT_FULL_DELIVERY')
    sis.start()
    rospy.sleep(3.0)

    sm.userdata.tagid = 'person      '
    sm.userdata.explore_radius = 2.7
    outcome = sm.execute()
    
    rospy.spin()
    sis.stop()

    

