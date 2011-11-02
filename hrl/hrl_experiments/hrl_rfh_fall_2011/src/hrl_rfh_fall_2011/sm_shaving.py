#!/usr/bin/python

import numpy as np

import roslib
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('web_teleop_trunk')
roslib.load_manifest('smach_ros')
import rospy
import smach
import smach_ros

from hrl_rfh_fall_2011.sm_topic_monitor import TopicMonitor
from hrl_rfh_fall_2011.sm_input_parser import InputParser
 
from hrl_rfh_fall_2011.msg import EllipsoidMoveAction, EllipsoidMoveGoal
from web_teleop_trunk.msg import FtMoveAction, FtHoldAction, FtHoldGoal
from geometry_msgs.msg import Point, WrenchStamped
from std_msgs.msg import Int8

                
APPROACH_DURATION = 8
FORCE_THRESH = 3.0
GLOBAL_DURATION = 5
GRIPPER_ROT = np.pi
HEIGHT_STEP = 0.17
HOLD_ACTIVITY_THRESH = 1.0
HOLD_MAG_THRESH = 10.0
LATITUDE_STEP = 0.12
LOCAL_DURATION = 5
LONGITUDE_STEP = 0.2
RETREAT_HEIGHT = 1.45
SAFETY_RETREAT_HEIGHT = 1.9
SHAVE_HEIGHT = 0.8

outcomes_spa = ['succeeded','preempted','aborted']

class ForceCollisionMonitor(smach.State):
    def __init__(self, thresh):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'aborted'])
        self.thresh = thresh
        self.collided = False
        rospy.Subscriber('/netft_gravity_zeroing/wrench_zeroed', WrenchStamped, self.force_cb)

    def force_cb(self, msg):
        force_mag = np.linalg.norm([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.collided = force_mag > self.thresh

    def execute(self, userdata):
        self.collided = False
        while not rospy.is_shutdown():
            if self.collided:
                return 'collision'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.01)
        return 'aborted'
############# Begin Concurrent Listeners (listening to interface for publications)

def build_cc_listeners(prefix):
    print "Building a concurrent listener"    
    global_name = prefix + '_GLOBAL_LISTENER'
    local_name = prefix + '_LOCAL_LISTENER'
    def cc_listeners_child_term_cb(outcome_map):
        if outcome_map[global_name] == 'succeeded' or outcome_map[local_name] == 'succeeded':
            return True
        else:
            return False

    def cc_listeners_out_cb(outcome_map):
        if outcome_map[global_name] == 'succeeded':
            return 'global' 
        if outcome_map[local_name] == 'succeeded':
            return 'local'
        if outcome_map[global_name] == 'aborted' or outcome_map[local_name] == 'aborted':
            return 'aborted'
        if outcome_map[global_name] == 'preempted' or outcome_map[local_name] == 'preempted':
            return 'preempted'
    print "Creating a concurrent listener"                            
    cc_listeners = smach.Concurrence(outcomes = ['global', 'local','aborted','preempted'],
                               default_outcome = 'global',
                               output_keys = ['goal_location'],
                               child_termination_cb = cc_listeners_child_term_cb,
                               outcome_cb = cc_listeners_out_cb)

    with cc_listeners:
        print "Adding GLOBAL_LISTENER to a concurrent listener"
        smach.Concurrence.add(global_name, TopicMonitor('wt_shave_location', Int8),
                        remapping={'output':'goal_location'})

        print "Adding LOCAL_LISTENER to a concurrent listener"
        smach.Concurrence.add(local_name, TopicMonitor('wt_shave_step', Point),
                        remapping={'output':'goal_location'})

        return cc_listeners
############# End Concurrent Listeners (listening to interface for publications)

def build_sm():
    print "Creating SM_TOP"
    sm_top = smach.StateMachine(outcomes=outcomes_spa)
    sm_top.userdata.goal_pose = [(0,0,0),(0,0,0)] #Initialize Ellipse Pose in [(Lat,Lon,height),(r,p,y)]

    with sm_top:
        print "Building outer CC_LISTENER"
        cc_outer_listener = build_cc_listeners('OUTER')
        print "Created CC_OUTER_LISTENER", cc_outer_listener
        smach.StateMachine.add('CC_OUTER_LISTENER', cc_outer_listener, transitions={'global':'CC_LISTENER_BEHAVIOR',
                                                                                    'local':'CC_LISTENER_BEHAVIOR'})
       
        def cc_list_beh_child_term_cb(outcome_map):
            return True

        def cc_list_beh_out_cb(outcome_map):
            if outcome_map['CC_INNER_LISTENER'] == 'global':
                return 'new_global_goal'
            if outcome_map['CC_INNER_LISTENER'] == 'local':
                return 'new_local_goal'
            if outcome_map['SM_BEHAVIOR'] == 'succeeded':
                return 'completed'

        cc_listener_behavior = smach.Concurrence(outcomes=['new_global_goal','new_local_goal','completed'],
                                                 default_outcome='completed',
                                                 input_keys = ['goal_location'],
                                                 output_keys = ['goal_location'],
                                                 child_termination_cb = cc_list_beh_child_term_cb,
                                                 outcome_cb = cc_list_beh_out_cb)
        print "Created CC_LISTENER_BEHAVIOR", cc_listener_behavior

        with cc_listener_behavior:

            print "Building Inner_cc_listener"
            cc_inner_listener = build_cc_listeners('INNER')
            print "Created Inner_CC_LISTENER", cc_inner_listener
            smach.Concurrence.add('CC_INNER_LISTENER', cc_inner_listener)

            print "Creating SM_BEHAVIOR"
            sm_behavior = smach.StateMachine(outcomes=['succeeded','preempted','aborted'],
                                             input_keys=['goal_location'])            
            
            with sm_behavior:
                
                print "Adding Input parser to SM_BEHAVIOR"
                smach.StateMachine.add('INPUT_PARSER', InputParser(), 
                                       transitions = {'ell_right':'ELL_MOVE_RIGHT',
                                                      'ell_left':'ELL_MOVE_LEFT',
                                                      'ell_up':'ELL_MOVE_UP',
                                                      'ell_down':'ELL_MOVE_DOWN',
                                                      'ell_in':'ELL_MOVE_IN',
                                                      'ell_out':'ELL_MOVE_OUT',
                                                      'shave':'ELL_APPROACH',
                                                      'global_move':'ELL_MOVE_GLOBAL_FULL'},
                                        remapping = {'input':'goal_location',
                                                     'output':'goal_pose'})

#  print "Adding Glocal Input Parser State to SM_BEHAVIOR"
#                smach.StateMachine.add('GLOBAL_INPUT_PARSER', GlobalInputParser(),
#                                       transitions = {'shave':'ELL_APPROACH',
#                                                      'move':'ELL_MOVE_GLOBAL_FULL'},
#                                        remapping = {'input':'goal_location',
#                                                     'goal_pose':'goal_pose'})
                
#                print "Adding Local Input parser to SM_BEHAVIOR"
#                smach.StateMachine.add('LOCAL_INPUT_PARSER', LocalInputParser(), 
#                                       transitions = {'ell_right':'ELL_MOVE_RIGHT',
#                                                      'ell_left':'ELL_MOVE_LEFT',
#                                                      'ell_up':'ELL_MOVE_UP',
#                                                      'ell_down':'ELL_MOVE_DOWN',
#                                                      'ell_in':'ELL_MOVE_IN',
#                                                      'ell_out':'ELL_MOVE_OUT'},
#                                                      'shave':'LILNEAR_APPROACH',
#                                                      'global_move':'ELL_MOVE_GLOBAL_FULL'
#                                        remapping = {'input':'goal_location'})
                
                def wrap_state_force_detection(state_name, state, force_thresh, outcomes, input_keys=[]):
                    collision_outcomes = ['collision']
                    total_outcomes = outcomes + collision_outcomes

                    def child_term_cb(outcome_map):
                        # stop execution under all circumstances
                        return True

                    def out_cb(outcome_map):
                        if outcome_map['FORCE_MONITOR_'+state_name] == 'collision':
                            return 'collision'
                        return outcome_map[state_name]

                    sm_coll_detect_state = smach.Concurrence(
                        outcomes=total_outcomes,
                        input_keys=input_keys,
                        default_outcome='aborted',
                        child_termination_cb=child_term_cb,
                        outcome_cb=out_cb)

                    with sm_coll_detect_state:
                        smach.Concurrence.add(
                            'FORCE_MONITOR_' + state_name,
                            ForceCollisionMonitor(force_thresh))
                        smach.Concurrence.add(state_name, state)

                    return sm_coll_detect_state

                def get_ell_move_local(delta_lat, delta_lon, delta_hei, gripper_rot):
                    goal = EllipsoidMoveGoal()
                    goal.change_latitude = delta_lat
                    goal.change_longitude = delta_lon
                    goal.change_height = delta_hei
                    goal.gripper_rot = gripper_rot
                    goal.duration = LOCAL_DURATION
                    return smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, goal=goal)
                
                
                def get_ell_move_height(height, duration, gripper_rot):
                    goal = EllipsoidMoveGoal()
                    goal.change_height = height
                    goal.absolute_height = True
                    goal.gripper_rot = gripper_rot
                    goal.duration = duration
                    return smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, goal=goal)


                smach.StateMachine.add(
                   'ELL_MOVE_RESET',
                    wrap_state_force_detection(
                            'ELL_MOVE_RESET_ACT',
                            get_ell_move_local(0, 0, 0, GRIPPER_ROT),
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'succeeded',
                                 'collision' : 'aborted'})

                smach.StateMachine.add(
                    'ELL_MOVE_UP',
                    wrap_state_force_detection(
                        'ELL_MOVE_UP',
                        get_ell_move_local(-LATITUDE_STEP, 0, 0, GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'succeeded',
                                 'collision' : 'FT_HOLD'})

                smach.StateMachine.add(
                    'ELL_MOVE_LEFT',
                    wrap_state_force_detection(
                        'ELL_MOVE_LEFT',
                        get_ell_move_local(0, LONGITUDE_STEP, 0, GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'succeeded',
                                 'collision' : 'FT_HOLD'})

                smach.StateMachine.add(
                    'ELL_MOVE_DOWN',
                    wrap_state_force_detection(
                        'ELL_MOVE_DOWN',
                        get_ell_move_local(LATITUDE_STEP, 0, 0, GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'succeeded',
                                 'collision' : 'FT_HOLD'})

                smach.StateMachine.add(
                    'ELL_MOVE_RIGHT',
                    wrap_state_force_detection(
                        'ELL_MOVE_RIGHT',
                        get_ell_move_local(0, -LONGITUDE_STEP, 0, GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'succeeded',
                                 'collision' : 'FT_HOLD'})

                smach.StateMachine.add(
                    'ELL_MOVE_OUT',
                    wrap_state_force_detection(
                        'ELL_MOVE_OUT',
                        get_ell_move_local(0, 0, HEIGHT_STEP, GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'succeeded',
                                 'collision' : 'FT_HOLD'})

                smach.StateMachine.add(
                    'ELL_MOVE_IN',
                    wrap_state_force_detection(
                        'ELL_MOVE_IN',
                        get_ell_move_local(0, 0, -HEIGHT_STEP, GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'succeeded',
                                 'collision' : 'FT_HOLD'})


                smach.StateMachine.add(
                    'ELL_RETREAT_FAST',
                    get_ell_move_height(SAFETY_RETREAT_HEIGHT, 1, GRIPPER_ROT),
                    transitions={'succeeded' : 'succeeded'})

                smach.StateMachine.add(
                    'ELL_RETREAT_SETUP',
                    get_ell_move_local(0, 0, -0.5, GRIPPER_ROT),
                    transitions={'succeeded' : 'succeeded'})
                                 

                def get_ell_move_spec_height(duration, gripper_rot):
                    def goal_cb(userdata, goal_in):
                        goal = EllipsoidMoveGoal()
                        goal.change_height = userdata.goal_pose[0][2]
                        goal.absolute_height = True
                        goal.gripper_rot = gripper_rot
                        goal.duration = duration
                        return goal

                    return smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                                       goal_cb=goal_cb,
                                                       input_keys=['goal_pose'])
                
                def get_ell_move_global(retreat_height, duration, gripper_rot):
                    def goal_cb(userdata, goal_in):
                        goal = EllipsoidMoveGoal()
                        lat, lon, height = userdata.goal_pose[0]
                        goal.change_latitude = lat
                        goal.change_longitude = lon
                        goal.change_height = retreat_height
                        goal.gripper_rot = gripper_rot
                        goal.absolute_latitude = True
                        goal.absolute_longitude = True
                        goal.absolute_height = True
                        goal.duration = duration
                        return goal

                    return smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                       goal_cb=goal_cb,
                                       input_keys=['goal_pose'])

                def get_ell_move_global_full(gripper_rot):
                    sm = smach.StateMachine(outcomes=outcomes_spa, input_keys=['goal_pose'])

                    with sm:
                        smach.StateMachine.add(
                            'ELL_RETREAT_GLOBAL',
                            get_ell_move_height(RETREAT_HEIGHT, APPROACH_DURATION, GRIPPER_ROT),
                            transitions={'succeeded' : 'ELL_MOVE_GLOBAL'})

                        smach.StateMachine.add(
                            'ELL_MOVE_GLOBAL',
                            get_ell_move_global(RETREAT_HEIGHT, GLOBAL_DURATION, GRIPPER_ROT),
                            transitions={'succeeded' : 'ELL_APPROACH_GLOBAL'})

                        smach.StateMachine.add(
                            'ELL_APPROACH_GLOBAL',
                            get_ell_move_spec_height(APPROACH_DURATION, GRIPPER_ROT))
                    return sm


                smach.StateMachine.add(
                    'ELL_RETREAT_SLOW',
                    wrap_state_force_detection(
                        'ELL_RETREAT_SLOW',
                        get_ell_move_height(RETREAT_HEIGHT, 8, GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'succeeded',
                                 'collision' : 'FT_HOLD'})

                smach.StateMachine.add(
                    'ELL_MOVE_GLOBAL_FULL',
                    wrap_state_force_detection(
                        'ELL_MOVE_GLOBAL_FULL_ACT',
                        get_ell_move_global_full(GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=outcomes_spa,
                        input_keys=['goal_pose']),
                        transitions={'succeeded' : 'ELL_APPROACH',
                                     'collision' : 'FT_HOLD'})

                smach.StateMachine.add(
                    'ELL_APPROACH',
                    wrap_state_force_detection(
                        'ELL_APPROACH',
                        get_ell_move_height(SHAVE_HEIGHT, APPROACH_DURATION, GRIPPER_ROT),
                        FORCE_THRESH,
                        outcomes=['succeeded','preempted','aborted']),
                    transitions={'succeeded' : 'FT_HOLD',
                                 'preempted':'ELL_RETREAT_SLOW',
                                 'aborted':'ELL_RETREAT_SLOW',
                                 'collision' : 'FT_HOLD'})

#     print "Addling Linear Approach State to SM_BEHAVIOR"
#                smach.StateMachine.add('ELL_APPROACH',
#                        smach_ros.SimpleActionState('ft_move_action', FtMoveAction,
#                                                    goal_slots=['poses','force_thresh',
#                                                                'ignore_ft']),
#                        transitions={'succeeded':'FT_HOLD', 
#                                     'preempted':'ELL_RETREAT_SLOW',
#                                     'aborted':'ELL_RETREAT_SLOW'},
#                        remapping={'poses':'linear_approach_poses',
#                                   'force_thresh':'linear_approach_force_thresh',
#                                   'ignore_ft':'linear_approach_ignore_ft'})

                print "Adding FtHold State to SM_BEHAVIOR"
                hold_goal = FtHoldGoal()
                hold_goal.activity_thresh = HOLD_ACTIVITY_THRESH
                hold_goal.z_thresh = 8.0
                hold_goal.mag_thresh = HOLD_MAG_THRESH
                hold_goal.timeout = rospy.Duration(30.0)

                smach.StateMachine.add('FT_HOLD',
                                        smach_ros.SimpleActionState('ft_hold_action', FtHoldAction,
                                                                    goal=hold_goal),
                                        transitions={'succeeded':'ELL_RETREAT_SLOW',
                                                     'preempted':'ELL_RETREAT_SLOW',
                                                     'aborted':'ELL_RETREAT_FAST'},
                                        remapping={'activity_thresh':'hold_activity_thresh',
                                                   'z_thresh':'hold_z_thresh',
                                                   'mag_thresh':'hold_mag_thresh',
                                                   'timeout':'hold_timeout'}
                                        )

            print "Adding SM_BEHAVIOR to CC_LISTENER_BEHAVIOR"
            smach.Concurrence.add('SM_BEHAVIOR', sm_behavior)

        print "Adding CC_LISTENER_BEHAVIOR to SM_TOP"
        smach.StateMachine.add('CC_LISTENER_BEHAVIOR', cc_listener_behavior, transitions={
                                                                    'new_local_goal':'CC_LISTENER_BEHAVIOR',
                                                                    'new_global_goal':'CC_LISTENER_BEHAVIOR',
                                                                    'completed':'CC_OUTER_LISTENER'})

 #   sis = smach_ros.IntrospectionServer('shaving_sis', sm_top, 'SM_SHAVING')
#  sis.start() 

    outcome = sm_top.execute()
#   sis.stop()

if __name__=='__main__':
    rospy.init_node('shaving_behavior_sm_node')
    build_sm()

