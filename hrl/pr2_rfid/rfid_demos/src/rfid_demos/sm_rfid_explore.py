#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_explore_room')
import rospy

import smach
import tf
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib

from rfid_behaviors.srv import FlapperSrv, FlapperSrvRequest
from rfid_behaviors.srv import RecorderSrv, RecorderSrvRequest, NextBestVantage
from rfid_behaviors.msg import RecorderReads
from rfid_explore_room.srv import ExploreRoomSrv, ExploreRoomSrvResponse
from explore_hrl.msg import ExploreAction, ExploreGoal
from geometry_msgs.msg import PoseStamped


class Sleeper( smach.State ):
    def __init__( self, time ):
        smach.State.__init__( self, outcomes = ['succeeded'] )
        self.time = time
        
    def execute( self, userdata ):
        rospy.sleep( self.time )
        return 'succeeded'

def sm_search():
    # Create a SMACH state machine
    sm_search = smach.StateMachine( outcomes=['succeeded', 'aborted', 'preempted'],
                                    input_keys = ['tagid', 'explore_radius'])

    # Open the container
    with sm_search:

        # Start the RFID recorder
        smach.StateMachine.add(
            'RECORDER_START',
            ServiceState( '/rfid_recorder/record',
                          RecorderSrv),
            transitions = {'succeeded':'FLAPPER_START'})


        # Start the ears flapping.
        smach.StateMachine.add(
            'FLAPPER_START',
            ServiceState( '/flapper/flap',
                          FlapperSrv,
                          request_slots = ['tagid']),
            transitions = {'succeeded':'EXPLORE_ROOM'},
            # transitions = {'succeeded':'SLEEPER'},
            remapping = {'tagid':'tagid'})

        # smach.StateMachine.add('SLEEPER', Sleeper( 5.0 ), transitions = {'succeeded':'FLAPPER_STOP'})
                          

        # EXPLORE
        def explore_response_cb( userdata, response ):
            # result is of ExploreRoomSrvResponse
            return response.result
        
        smach.StateMachine.add(
            'EXPLORE_ROOM',
            ServiceState( '/explore/explore', # Default outcomes
                          ExploreRoomSrv,
                          request_slots = [ 'radius' ],
                          response_cb = explore_response_cb),
            remapping = { 'radius':'explore_radius' },
            transitions = {'succeeded':'FLAPPER_STOP'})  # input
        

        # Stop the ears flapping.
        smach.StateMachine.add(
            'FLAPPER_STOP',
            ServiceState( '/flapper/flap',
                          FlapperSrv,
                          request_slots = ['tagid']),
            transitions = {'succeeded':'RECORDER_STOP'},
            remapping = {'tagid':'tagid'})

        # Start the RFID recorder
        smach.StateMachine.add(
            'RECORDER_STOP',
            ServiceState( '/rfid_recorder/record',
                          RecorderSrv),
            transitions = {'succeeded':'succeeded'})


    return sm_search
    



if __name__ == '__main__':
    rospy.init_node('smach_rfid_explore')

    sm = sm_search()

    sis = IntrospectionServer('sm_rfid_explore', sm, '/SM_ROOT_RFID_EXPLORE')
    sis.start()
    rospy.sleep(3.0)

    sm.userdata.tagid = ''
    #sm.userdata.tagid = 'person      '
    sm.userdata.explore_radius = 2.7
    outcome = sm.execute()

    sis.stop()

    



# This old concurrency one didn't work.

# def sm_search():
#     # Create a SMACH state machine
#     sm_search = smach.Concurrence( outcomes=['succeeded', 'aborted', 'preempted'],
#                                    default_outcome = 'aborted',
#                                    outcome_map = {'succeeded': {'EXPLORE_ROOM':'succeeded'},
#                                                   'aborted': {'EXPLORE_ROOM':'aborted'},
#                                                   'preempted': {'EXPLORE_ROOM':'preempted'}},
#                                    output_keys = ['explore_rfid_reads'],
#                                    input_keys = ['tagid', 'explore_radius'],
#                                    child_termination_cb = lambda arg: True )
#     # Note: child_termination_cb: Terminate all other states' execution upon first child complete (Always EXPLORE_ROOM).

#     # Open the container
#     with sm_search:
#         smach.Concurrence.add(
#             'EXPLORE_ROOM',
#             SimpleActionState( '/explore', # Default outcomes
#                                ExploreAction,
#                                goal_slots = [ 'radius' ]),
#             remapping = { 'radius':'explore_radius' })  # input
        
#         smach.Concurrence.add(
#             'RFID_FLAPPER', 
#             Flapper(), # outcomes: preempted
#             remapping = { 'tagid' : 'tagid' }) # input
        
#         smach.Concurrence.add(
#             'RFID_RECORDER',
#             Recorder(), # outcomes: preempted
#             remapping = { 'rfid_reads' : 'explore_rfid_reads' }) # output


#     return sm_search
    



# import hrl_rfid.ros_M5e_client as rmc
# class Recorder(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes = ['preempted'],
#                              output_keys = ['rfid_reads']) # [(RFIDread, ps_ant_map, ps_base_map ), ...]
#         self.data = []

#     def execute(self, userdata):
#         rospy.logout( 'RFID Recorder: Initiated.' )
#         self.data = []

#         listener = tf.TransformListener()
#         rospy.logout( 'RFID Recorder: Waiting on transforms' )
#         listener.waitForTransform('/ear_antenna_left', '/map',
#                                   rospy.Time(0), timeout = rospy.Duration(100) )
#         listener.waitForTransform('/ear_antenna_right', '/map',
#                                   rospy.Time(0), timeout = rospy.Duration(100) )
#         rospy.logout( 'RFID Recorder: Ready' )

#         def process_datum( datum ):
#             # Hooray for lexical scope (listener)!
#             ant_lookup = { 'EleLeftEar': '/ear_antenna_left',
#                            'EleRightEar': '/ear_antenna_right' }
            
#             ps_ant = PoseStamped()
#             ps_ant.header.stamp = rospy.Time( 0 )
#             ps_ant.header.frame_id = ant_lookup[ datum.antenna_name ]
            
#             ps_base = PoseStamped()
#             ps_base.header.stamp = rospy.Time( 0 )
#             ps_base.header.frame_id = '/base_link'

#             try:
#                 ps_ant_map = listener.transformPose( '/map', ps_ant )
#                 ps_base_map = listener.transformPose( '/map', ps_base )
#                 rv = ( datum, ps_ant_map, ps_base_map )
#             except:
#                 rospy.logout( 'RFID Recorder: TF failed. Ignoring read.' )
#                 rv = None
#             return rv

#         def add_datum( datum ):
#             # Hooray for lexical scope (data)!
#             self.data.append( process_datum( datum ))

#         # Notes: In this case, the flapper will initiate RFID reads
#         #   for the proper tagid.  Not ideal, but how it is for now.
#         rec = rmc.ROS_M5e_Client('ears', callbacks = [add_datum])
        
#         rospy.logout( 'RFID Recorder: Logging Reads.' )
#         while not smach.State.preempt_requested( self ):
#             rospy.sleep( 0.1 )

#         rospy.logout( 'RFID Recorder: Preempt requested. Saving reads.' )
#         rec.unregister() # Stop processing new reads
#         rospy.sleep( 0.5 ) # Give it some time to settle
#         userdata.rfid_reads = list(self.data) # Save the data.
#         self.data = []
#         return 'preempted'

