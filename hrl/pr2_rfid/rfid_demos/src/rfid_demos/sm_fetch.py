#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_demos')
roslib.load_manifest('hrl_object_fetching')
roslib.load_manifest('hrl_table_detect')
roslib.load_manifest('hrl_trajectory_playback')
roslib.load_manifest('pr2_overhead_grasping')
import rospy

import smach
import actionlib

from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
# from hrl_object_fetching.srv import DetectTable
from hrl_table_detect.srv import DetectTableInst, DetectTableInstRequest
from hrl_table_detect.srv import DetectTableStart, DetectTableStop
from rfid_behaviors.srv import FloatFloat_Int32 as RotateBackupSrv
from rfid_behaviors.srv import FloatFloat_Int32Request as RotateBackupSrvRequest
from rfid_behaviors.srv import HandoffSrv, HandoffSrvRequest
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest
import pr2_overhead_grasping.sm_overhead_grasp as sm_overhead_grasp
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped

import sm_rfid_servo_approach
import approach_table.sm_approach as sm_approach

# Overhead grasping requres:
#   run: hrl_pr2_gains/change_gains_grasp.py
#   roslaunch pr2_overhead_grasping overhead_grasping_server.launch

# Perception requires:
#   roslaunch hrl_pr2_lib openni_kinect.launch
#   roslaunch hrl_object_fetching tabletop_detect.launch


def sm_fetch_object():
        # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'tagid' ])

    with sm:
        # Servo in towards table
        approach = sm_rfid_servo_approach.sm_rfid_servo_approach()
        smach.StateMachine.add(
            'SERVO_APPROACH', # outcomes: succeeded, aborted, preempted
            approach,
            remapping = { 'tagid' : 'tagid'}, #input
            transitions = { 'succeeded': 'POINT_HEAD' })
            

        # Point Head Down (eventaully roll this and perceive table into own sm?)
        pgoal = PointHeadGoal()
        pgoal.target.header.frame_id = '/torso_lift_link'
        pgoal.target.point.x = 0.50
        pgoal.min_duration = rospy.Duration(0.6)
        pgoal.max_velocity = 1.0
        smach.StateMachine.add(
            'POINT_HEAD',
            SimpleActionState( '/head_traj_controller/point_head_action',
                               PointHeadAction,
                               goal = pgoal ),
            transitions = { 'succeeded' : 'PERCEIVE_TABLE' })

        # Detect table and determine possible approach directions!
        # requires:
        #   roslaunch hrl_pr2_lib  openni_kinect.launch
        #   roslaunch hrl_object_fetching tabletop_detect.launch
        smach.StateMachine.add(
            'PERCEIVE_TABLE',
            ServiceState( '/table_detect_inst',
                          DetectTableInst,
                          request = DetectTableInstRequest( 1.0 ),
                          response_slots = ['grasp_points']), # PoseArray
            transitions = {'succeeded':'PERCEIVE_OBJECT'},
            remapping = {'grasp_points':'approach_poses'})

        # Setment objects
        smach.StateMachine.add(
            'PERCEIVE_OBJECT',
            ServiceState( '/obj_segment_inst',
                          DetectTableInst,
                          request = DetectTableInstRequest( 1.0 ),
                          response_slots = ['grasp_points']), # PoseArray
            transitions = {'succeeded':'APPROACH_TABLE'},
            remapping = {'grasp_points':'object_poses'}) #output
                          

        # Move to the desired approach vector
        sm_table = sm_approach.sm_approach_table()
        smach.StateMachine.add(
            'APPROACH_TABLE',
            sm_table,
            remapping = {'table_edge_poses':'approach_poses', # input (PoseArray)
                         'movebase_pose_global':'approach_movebase_pose', # output (PoseStamped)
                         'table_edge_global':'table_edge_global'}, # output (PoseStamped)
            transitions = {'succeeded':'REPOINT_HEAD'})

        # Re-Point Head to table's edge.
        def repoint_goal_cb(userdata, goal):
            # Convert PoseStamped in userdata to PointHeadGoal
            # mobj = userdata.look_points.poses[0] # We'll have head point to first object.

            # pgoal = PointHeadGoal()
            # pgoal.target.header.frame_id = userdata.look_points.header.frame_id
            # pgoal.target.point.x = mobj.pose.position.x
            # pgoal.target.point.y = mobj.pose.position.y
            # pgoal.target.point.z = mobj.pose.position.z
            # pgoal.min_duration = rospy.Duration(0.6)
            # pgoal.max_velocity = 1.0

            pgoal = PointHeadGoal()
            pgoal.target.header.frame_id = '/torso_lift_link'
            pgoal.target.point.x = 0.50
            pgoal.target.point.y = 0.0
            pgoal.target.point.z = -0.35
            pgoal.min_duration = rospy.Duration(0.6)
            pgoal.max_velocity = 1.0
            return pgoal
                                
        smach.StateMachine.add(
            'REPOINT_HEAD',
            SimpleActionState( '/head_traj_controller/point_head_action',
                               PointHeadAction,
                               goal_cb = repoint_goal_cb,
                               input_keys = ['look_points']),
            remapping = {'look_points':'object_poses'}, # input (PoseStamped)
            transitions = { 'succeeded' : 'PREP_UNFOLD' })

        # Prep unfold
        smach.StateMachine.add(
            'PREP_UNFOLD',
            ServiceState( 'rfid_handoff/stow',
                          HandoffSrv,
                          request = HandoffSrvRequest()), 
            transitions = { 'succeeded':'UNFOLD' })

        # Unfold
        smach.StateMachine.add(
            'UNFOLD',
            ServiceState( 'traj_playback/unfold',
                          TrajPlaybackSrv,
                          request = TrajPlaybackSrvRequest( 0 )), 
            transitions = { 'succeeded':'MANIPULATE' })

        # Manipulate
        sm_grasp = sm_overhead_grasp.sm_grasp()
        smach.StateMachine.add(
            'MANIPULATE',
            sm_grasp,
            transitions = {'succeeded':'BACKUP'})

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
            transitions = { 'succeeded':'succeeded' })
        
        # # Setup robot for further navigation:
        # #   fold arms, position head, position ear antennas.
        # smach.StateMachine.add(
        #     'PRE_STOW',
        #     ServiceState( 'rfid_handoff/pre_stow',
        #                   HandoffSrv,
        #                   request = HandoffSrvRequest()), 
        #     transitions = { 'succeeded':'STOW' })
        # smach.StateMachine.add(
        #     'STOW',
        #     ServiceState( 'rfid_handoff/stow',
        #                   HandoffSrv,
        #                   request = HandoffSrvRequest()), 
        #     transitions = { 'succeeded':'succeeded' })
        

    return sm



if __name__ == '__main__':
    rospy.init_node('smach_sm_fetch')

    sm = sm_fetch_object()

    sis = IntrospectionServer('RFID_fetching', sm, '/SM_FETCHING')
    sis.start()

    #sm.userdata.tagid = 'person      '
    #sm.userdata.tagid = 'OrangeMedBot'
    sm.userdata.tagid = 'SpectrMedBot'
    outcome = sm.execute()
    
    sis.stop()

    

