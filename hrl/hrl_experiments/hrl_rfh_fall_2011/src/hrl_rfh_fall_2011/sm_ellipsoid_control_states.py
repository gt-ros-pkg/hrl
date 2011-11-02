#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest('smach_ros')
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('hrl_pr2_arms')

import rospy
import smach
import smach_ros
from geometry_msgs.msg import WrenchStamped

from hrl_rfh_fall_2011.msg import EllipsoidMoveAction, EllipsoidMoveGoal

LATITUDE_STEP = 0.1
LONGITUDE_STEP = 0.1
HEIGHT_STEP = 0.17
LOCAL_DURATION = 5
APPROACH_DURATION = 8
GLOBAL_DURATION = 8
GRIPPER_ROT = np.pi
RETREAT_HEIGHT = 1.5
FORCE_THRESH = 2.0
GLOBAL_GOAL = [(4 * np.pi/8,   -3 * np.pi/8,     1),      (0,     0,      0)]

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
    
class GlobalMoveLoadGoal(smach.State):
    def __init__(self, ell_goal):
        smach.State.__init__(self, output_keys=['goal_pose'], outcomes=['succeeded'])
        self.ell_goal = ell_goal

    def execute(self, userdata):
        userdata.goal_pose = self.ell_goal
        return 'succeeded'

def get_test_global_full():
    sm = smach.StateMachine(outcomes=outcomes_spa)
    with sm:
        smach.StateMachine.add(
            'ELL_MOVE_GLOBAL_LOAD',
            GlobalMoveLoadGoal(GLOBAL_GOAL),
            transitions={'succeeded' : 'ELL_MOVE_GLOBAL_FULL'})

        smach.StateMachine.add(
            'ELL_MOVE_GLOBAL_FULL',
            wrap_state_force_detection(
                'ELL_MOVE_GLOBAL_FULL_ACT',
                get_ell_move_global_full(GRIPPER_ROT),
                FORCE_THRESH,
                outcomes=outcomes_spa,
                input_keys=['goal_pose']),
                transitions={'collision' : 'aborted'})

    return sm

def main():
    rospy.init_node("sm_registration_setup")

    if True:
        sm = get_test_global_full()
        sm.execute()
        return
        

    sm = smach.StateMachine(outcomes=outcomes_spa)
    with sm:
        smach.StateMachine.add(
            'ELL_MOVE_RESET',
            wrap_state_force_detection(
                'ELL_MOVE_RESET_ACT',
                get_ell_move_local(0, 0, 0, GRIPPER_ROT),
                FORCE_THRESH,
                outcomes=outcomes_spa),
            transitions={'succeeded' : 'ELL_MOVE_UP',
                         'collision' : 'aborted'})

        smach.StateMachine.add(
            'ELL_MOVE_UP',
            get_ell_move_local(-LATITUDE_STEP, 0, 0, GRIPPER_ROT),
            transitions={'succeeded' : 'ELL_MOVE_LEFT'})

        smach.StateMachine.add(
            'ELL_MOVE_LEFT',
            get_ell_move_local(0, LONGITUDE_STEP, 0, GRIPPER_ROT),
            transitions={'succeeded' : 'ELL_MOVE_DOWN'})
                                
        smach.StateMachine.add(
            'ELL_MOVE_DOWN',
            get_ell_move_local(LATITUDE_STEP, 0, 0, GRIPPER_ROT),
            transitions={'succeeded' : 'ELL_MOVE_RIGHT'})

        smach.StateMachine.add(
            'ELL_MOVE_RIGHT',
            get_ell_move_local(0, -LONGITUDE_STEP, 0, GRIPPER_ROT),
            transitions={'succeeded' : 'ELL_MOVE_OUT'})

        smach.StateMachine.add(
            'ELL_MOVE_OUT',
            get_ell_move_local(0, 0, HEIGHT_STEP, GRIPPER_ROT),
            transitions={'succeeded' : 'ELL_MOVE_IN'})

        smach.StateMachine.add(
            'ELL_MOVE_IN',
            get_ell_move_local(0, 0, -HEIGHT_STEP, GRIPPER_ROT),
            transitions={'succeeded' : 'ELL_RETREAT_FAST'})

        smach.StateMachine.add(
            'ELL_RETREAT_FAST',
            get_ell_move_height(RETREAT_HEIGHT, 1, GRIPPER_ROT),
            transitions={'succeeded' : 'ELL_RETREAT_SETUP'})
        
        smach.StateMachine.add(
            'ELL_RETREAT_SETUP',
            get_ell_move_local(0, 0, -0.5, GRIPPER_ROT),
            transitions={'succeeded' : 'ELL_RETREAT_SLOW'})

        smach.StateMachine.add(
            'ELL_RETREAT_SLOW',
            get_ell_move_height(RETREAT_HEIGHT, 8, GRIPPER_ROT),
            transitions={'succeeded' : 'succeeded'})

    sm.set_initial_state(['ELL_MOVE_RESET'])
    sm.execute()


if __name__ == "__main__":
    main()
