#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_demos')
import rospy

import smach
import tf
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from rfid_behaviors.srv import RecorderSrv
from rfid_behaviors.msg import RecorderReads

import numpy as np, math


class BestVantage(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             input_keys = ['tagid', 'rfid_reads'],
                             output_keys = ['best_vantage', 'filtered_reads']) 
        self.initialized = False
        self.orig_reads = [] # holds the original data
        self.curr_reads = [] # holds data after processing
        self.last_pose = None

    def execute(self, userdata):
        rospy.logout('BestVantage: Calculating' )

        if not self.initialized:
            self.initialized = True
            # [ RFIDread, PoseStamped_Antenna(map frame), PoseStamped_BaseLink(map frame)
            self.orig_reads = userdata.rfid_reads # (RecorderReads[])
            self.curr_reads = [ r for r in self.orig_reads
                                if r != None and r.read.rssi != -1 and
                                r.read.tagID == userdata.tagid ]
        else:
            rospy.logout( 'BestVantage: First try did not work, eh? Filtering and retrying a new vantage.' )
            return 'aborted'

        if not self.curr_reads:
            rospy.logout( 'BestVantage: No more positive-read poses to consider.' )
            return 'aborted'

        rssi = [ r.read.rssi for r in self.curr_reads ]
        ind = np.argmax( rssi )
        best = self.curr_reads[ ind ] # RecorderRead
        best_read = best.read
        best_ant = best.ps_ant_map
        best_base = best.ps_base_map

        #print best_read, best_ant, best_base

        # We're going to keep the <x,y> location from the baselink (mapframe),
        # but keep <ang> (mapframe) from the antenna.

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = best_base.header.frame_id
        ps.pose.position = best_base.pose.position
        ps.pose.orientation = best_ant.pose.orientation

        userdata.best_vantage = ps
        userdata.filtered_reads = self.curr_reads
        
        return 'succeeded'

def sm_best_vantage():
    sm = smach.StateMachine( outcomes = ['succeeded', 'aborted'],
                             input_keys = ['rfid_reads', 'tagid'])
    with sm:
        smach.StateMachine.add(
            'SELECT_BEST_VANTAGE',
            BestVantage(),
            remapping = { 'tagid' : 'tagid', # input (string)
                          'rfid_reads' : 'rfid_reads', # input (RecorderReads)
                          'best_vantage' : 'best_vantage', # output (PoseStamped)
                          'filtered_reads' : 'filtered_reads' }, # output (RecorderReads)
            transitions = {'succeeded':'MOVE_BEST_VANTAGE'})

        smach.StateMachine.add(
            'MOVE_BEST_VANTAGE',
            SimpleActionState( '/move_base',
                               MoveBaseAction,
                               goal_slots = [ 'target_pose' ]),
            remapping = { 'target_pose' : 'best_vantage' }, # input (PoseStamped)
            transitions = {'aborted':'SELECT_BEST_VANTAGE',
                           'preempted':'aborted',
                           'succeeded':'succeeded'})

    return sm

        
    
if __name__ == '__main__':
    import sm_rfid_explore
    
    rospy.init_node('smach_rfid_explore')

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = ['tagid','explore_radius'])

    # Open the container
    with sm:
        sm_search = sm_rfid_explore.sm_search()
        smach.StateMachine.add(
            'RFID_SEARCH',  # outcomes: succeded, aborted, preempted
            sm_search,
            remapping = { 'tagid' : 'tagid',  # input
                          'explore_radius' : 'explore_radius',   # input
                          'explore_rfid_reads' : 'explore_rfid_reads' }, # output
#            transitions={'succeeded':'succeeded'})
            transitions={'succeeded':'BEST_VANTAGE'})

        sm_vantage = sm_best_vantage()
        smach.StateMachine.add(
            'BEST_VANTAGE', # outcomes: succeeded, aborted, preempted
            sm_vantage,
            remapping = { 'tagid' : 'tagid', # input
                          'rfid_reads' : 'explore_rfid_reads' }, # input
            transitions = {'succeeded':'succeeded'})

    sis = IntrospectionServer('sm_rfid_explore', sm, '/SM_ROOT_RFID_EXPLORE')
    sis.start()
    rospy.sleep(3.0)

    sm.userdata.tagid = 'person      '
    sm.userdata.explore_radius = 2.7
    # sm.userdata.explore_rfid_reads = []
    outcome = sm.execute()
    # print sm.userdata.explore_rfid_reads
    
    rospy.spin()
    sis.stop()

    

    
