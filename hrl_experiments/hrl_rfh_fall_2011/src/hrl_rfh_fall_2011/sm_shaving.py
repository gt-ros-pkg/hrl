#!/usr/bin/python

import numpy as np
import copy
from functools import partial
import sys

import roslib
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('assistive_teleop')
roslib.load_manifest('smach_ros')
import rospy
import smach
import smach_ros

from hrl_rfh_fall_2011.sm_topic_monitor import TopicMonitor
from hrl_rfh_fall_2011.sm_input_parser import InputParser
 
from hrl_rfh_fall_2011.msg import EllipsoidMoveAction, EllipsoidMoveGoal
from assistive_teleop.msg import FtHoldAction, FtHoldGoal
from geometry_msgs.msg import Point, WrenchStamped
from std_msgs.msg import Int8

from hrl_rfh_fall_2011.shaving_parameters import *
from kelsey_sandbox.sm_pr2_servoing import PublishState

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

class WaitState(smach.State):
    def __init__(self, duration):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.duration = duration

    def execute(self, userdata):
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if rospy.get_time() - start_time >= self.duration:
                return 'succeeded'
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
        if (global_name in outcome_map and outcome_map[global_name] == 'succeeded' or 
                local_name in outcome_map and outcome_map[local_name] == 'succeeded'):
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

class ShavingTopGenerator(object):
    def __init__(self):
        self.trans_pub = rospy.Publisher('shaving_state', Int8, latch=True)

    def build_sm(self):
        print "Creating SM_TOP"
        sm_top = smach.StateMachine(outcomes=outcomes_spa)
        sm_top.userdata.goal_pose = [(0,0,0),(0,0,0)] #Initialize Ellipse Pose in [(Lat,Lon,height),(r,p,y)]

        with sm_top:
            print "Building outer CC_LISTENER"
            cc_outer_listener = build_cc_listeners('OUTER')
            print "Created CC_OUTER_LISTENER", cc_outer_listener
            smach.StateMachine.add('CC_OUTER_LISTENER', cc_outer_listener, 
                                   transitions={'global':'CC_LISTENER_BEHAVIOR',
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
                    
                    def wrap_state_force_detection(state_name, state, force_thresh, outcomes, input_keys=[]):
                        collision_outcomes = ['collision']
                        total_outcomes = outcomes + collision_outcomes

                        def child_term_cb(outcome_map):
                            # stop execution under all circumstances
                            return True

                        def out_cb(outcome_map):
                            if outcome_map['FORCE_MONITOR_'+state_name] == 'collision':
                                self.trans_pub.publish(TransitionIDs.MOVE_COLLISION)
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

                    def get_ell_move_local(delta_lat, delta_lon, delta_hei, userdata, default_goal):
                        goal = EllipsoidMoveGoal()
                        goal.change_latitude = delta_lat
                        goal.change_longitude = delta_lon
                        goal.change_height = delta_hei
                        goal.velocity = LOCAL_VELOCITY
                        if rospy.get_param("/shaving_side") == 'r':
                            goal.gripper_rot = np.pi
                        else:
                            goal.gripper_rot = 0.
                        return goal
                    
                    
                    def get_ell_move_height(height, velocity, userdata, default_goal):
                        goal = EllipsoidMoveGoal()
                        goal.change_height = height
                        goal.absolute_height = True
                        goal.velocity = velocity
                        if rospy.get_param("/shaving_side") == 'r':
                            goal.gripper_rot = np.pi
                        else:
                            goal.gripper_rot = 0.
                        return goal 

                    smach.StateMachine.add(
                       'ELL_MOVE_RESET',
                        wrap_state_force_detection(
                                'ELL_MOVE_RESET_ACT',
                                smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                     goal_cb=partial(get_ell_move_local, 0, 0, 0)),
                                FORCE_THRESH,
                                outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'succeeded',
                                     'collision' : 'aborted'})

                    smach.StateMachine.add(
                        'ELL_MOVE_UP',
                        wrap_state_force_detection(
                            'ELL_MOVE_UP',
                            smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                 goal_cb=partial(get_ell_move_local, -LATITUDE_STEP, 0, 0)),
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'FT_HOLD',
                                     'collision' : 'FT_HOLD'})

                    smach.StateMachine.add(
                        'ELL_MOVE_LEFT',
                        wrap_state_force_detection(
                            'ELL_MOVE_LEFT',
                            smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                 goal_cb=partial(get_ell_move_local, 0, LONGITUDE_STEP, 0)),
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'FT_HOLD',
                                     'collision' : 'FT_HOLD'})

                    smach.StateMachine.add(
                        'ELL_MOVE_DOWN',
                        wrap_state_force_detection(
                            'ELL_MOVE_DOWN',
                            smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                 goal_cb=partial(get_ell_move_local, LATITUDE_STEP, 0, 0)), 
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'FT_HOLD',
                                     'collision' : 'FT_HOLD'})

                    smach.StateMachine.add(
                        'ELL_MOVE_RIGHT',
                        wrap_state_force_detection(
                            'ELL_MOVE_RIGHT',
                            smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                 goal_cb=partial(get_ell_move_local, 0, -LONGITUDE_STEP, 0)), 
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'FT_HOLD',
                                     'collision' : 'FT_HOLD'})

                    smach.StateMachine.add(
                        'ELL_MOVE_OUT',
                        wrap_state_force_detection(
                            'ELL_MOVE_OUT',
                            smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                 goal_cb=partial(get_ell_move_local, 0, 0, HEIGHT_STEP)), 
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'FT_HOLD',
                                     'collision' : 'FT_HOLD'})

                    smach.StateMachine.add(
                        'ELL_MOVE_IN',
                        wrap_state_force_detection(
                            'ELL_MOVE_IN',
                            smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                 goal_cb=partial(get_ell_move_local, 0, 0, -HEIGHT_STEP)), 
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'FT_HOLD',
                                     'collision' : 'FT_HOLD'})


                    smach.StateMachine.add(
                        'ELL_RETREAT_FAST',
                        smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                            goal_cb=partial(get_ell_move_height, SAFETY_RETREAT_HEIGHT, SAFETY_RETREAT_VELOCITY)),
                        transitions={'succeeded' : 'succeeded'})

                    smach.StateMachine.add(
                        'ELL_RETREAT_SETUP',
                        smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                             goal_cb=partial(get_ell_move_local, 0, 0, -0.5)), 
                        transitions={'succeeded' : 'succeeded'})
                                     

                    def ell_move_spec_height_goal_cb(userdata, default_goal):
                        goal = EllipsoidMoveGoal()
                        goal.change_height = userdata.goal_pose[0][2]
                        goal.absolute_height = True
                        goal.velocity = APPROACH_VELOCITY
                        if rospy.get_param("/shaving_side") == 'r':
                            goal.gripper_rot = np.pi
                        else:
                            goal.gripper_rot = 0.
                        return goal
                    
                    def ell_move_global_goal_cb(userdata, default_goal):
                        goal = EllipsoidMoveGoal()
                        lat, lon, height = userdata.goal_pose[0]
                        goal.change_latitude = lat
                        goal.change_longitude = lon
                        goal.change_height = RETREAT_HEIGHT
                        goal.absolute_latitude = True
                        goal.absolute_longitude = True
                        goal.absolute_height = True
                        goal.velocity = GLOBAL_VELOCITY
                        if rospy.get_param("/shaving_side") == 'r':
                            goal.gripper_rot = np.pi
                        else:
                            goal.gripper_rot = 0.
                        return goal

                    def get_ell_move_global_full():
                        sm = smach.StateMachine(outcomes=outcomes_spa, input_keys=['goal_pose'])

                        with sm:

                            smach.StateMachine.add('PUB_ELL_RETREAT_GLOBAL',
                                PublishState("shaving_state",
                                Int8, Int8(ServoStates.ELL_RETREAT_GLOBAL)),
                                transitions={'succeeded' : 'ELL_RETREAT_GLOBAL'})

                            smach.StateMachine.add(
                                'ELL_RETREAT_GLOBAL',
                                smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                    goal_cb=partial(get_ell_move_height, RETREAT_HEIGHT, APPROACH_VELOCITY)),
                                transitions={'succeeded' : 'ELL_MOVE_GLOBAL_WAIT'})

                            smach.StateMachine.add(
                                'ELL_MOVE_GLOBAL_WAIT', 
                                WaitState(0.3),
                                transitions={'succeeded' : 'PUB_ELL_MOVE_GLOBAL'})

                            smach.StateMachine.add('PUB_ELL_MOVE_GLOBAL',
                                PublishState("shaving_state",
                                Int8, Int8(ServoStates.ELL_MOVE_GLOBAL)),
                                transitions={'succeeded' : 'ELL_MOVE_GLOBAL'})

                            smach.StateMachine.add(
                                'ELL_MOVE_GLOBAL',
                                smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                    goal_cb=ell_move_global_goal_cb,
                                    input_keys=['goal_pose']),
                                transitions={'succeeded' : 'ELL_APPROACH_GLOBAL_WAIT'})

                            smach.StateMachine.add(
                                'ELL_APPROACH_GLOBAL_WAIT', 
                                WaitState(0.3),
                                transitions={'succeeded' : 'ELL_APPROACH_GLOBAL'})

                            smach.StateMachine.add(
                                'ELL_APPROACH_GLOBAL',
                                smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                    goal_cb=ell_move_spec_height_goal_cb,
                                    input_keys=['goal_pose']),
                                transitions={'succeeded' : 'PUB_ELL_APPROACH_GLOBAL'})

                            smach.StateMachine.add('PUB_ELL_APPROACH_GLOBAL',
                                PublishState("shaving_state",
                                Int8, Int8(ServoStates.ELL_APPROACH_GLOBAL)))
                        return sm


                    smach.StateMachine.add(
                        'ELL_RETREAT_SLOW',
                        wrap_state_force_detection(
                            'ELL_RETREAT_SLOW',
                            smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                goal_cb=partial(get_ell_move_height, RETREAT_HEIGHT, SAFETY_RETREAT_VELOCITY)),
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'succeeded',
                                     'collision' : 'FT_HOLD'})

                    smach.StateMachine.add(
                        'ELL_MOVE_GLOBAL_FULL',
                        wrap_state_force_detection(
                            'ELL_MOVE_GLOBAL_FULL_ACT',
                            get_ell_move_global_full(),
                            FORCE_THRESH,
                            outcomes=outcomes_spa,
                            input_keys=['goal_pose']),
                            transitions={'succeeded' : 'ELL_APPROACH',
                                         'collision' : 'FT_HOLD'})

                    smach.StateMachine.add(
                        'ELL_APPROACH',
                        wrap_state_force_detection(
                            'ELL_APPROACH',
                            smach_ros.SimpleActionState('ellipsoid_move', EllipsoidMoveAction, 
                                goal_cb=partial(get_ell_move_height, SHAVE_HEIGHT, APPROACH_VELOCITY)),
                            FORCE_THRESH,
                            outcomes=['succeeded','preempted','aborted']),
                        transitions={'succeeded' : 'FT_HOLD',
                                     'preempted':'ELL_RETREAT_SLOW',
                                     'aborted':'ELL_RETREAT_SLOW',
                                     'collision' : 'FT_HOLD'})

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
                                                         'preempted':'succeeded',
                                                         'aborted':'ELL_RETREAT_FAST'},
                                            remapping={'activity_thresh':'hold_activity_thresh',
                                                       'z_thresh':'hold_z_thresh',
                                                       'mag_thresh':'hold_mag_thresh',
                                                       'timeout':'hold_timeout'}
                                            )

                print "Adding SM_BEHAVIOR to CC_LISTENER_BEHAVIOR"
                smach.Concurrence.add('SM_BEHAVIOR', sm_behavior)

            print "Adding CC_LISTENER_BEHAVIOR to SM_TOP"
            smach.StateMachine.add('CC_LISTENER_BEHAVIOR', cc_listener_behavior, 
                    transitions={
                        'new_local_goal':'CC_LISTENER_BEHAVIOR',
                        'new_global_goal':'CC_LISTENER_BEHAVIOR',
                        'completed':'CC_OUTER_LISTENER'})

        return sm_top

def main():
    rospy.init_node('shaving_behavior_sm_node')

    if len(sys.argv) >= 2 and sys.argv[1] == '-d':
        sm_top_gen = ShavingTopGenerator()
        sm_top = sm_top_gen.build_sm()
        sis = smach_ros.IntrospectionServer('shaving_sis', sm_top, 'SM_SHAVING')
        sis.start() 
        rospy.spin()
        sis.stop()
        return
    else:
        sm_top = build_sm()
        outcome = sm_top.execute()
        return

if __name__=='__main__':
    main()
