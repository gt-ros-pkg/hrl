#!/usr/bin/python

import numpy as np

import roslib
roslib.load_manifest('smach_ros')
roslib.load_manifest('costmap_services')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory
from costmap_services.python_client import CostmapServices

outcomes_spa = ['succeeded','preempted','aborted']

class ArmCollisionDetection(smach.State):
    def __init__(self, min_l_torques=[-3.]*7, min_r_torques=[-3.]*7,
                       max_l_torques=[3.]*7, max_r_torques=[3.]*7):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'aborted'])
        self.min_l_tor = np.array(min_l_torques)
        self.min_r_tor = np.array(min_r_torques)
        self.max_l_tor = np.array(max_l_torques)
        self.max_r_tor = np.array(max_r_torques)
        l_arm = create_pr2_arm('l', PR2ArmJointTrajectory)
        r_arm = create_pr2_arm('r', PR2ArmJointTrajectory)

    def execute(self, userdata):
        r = Rate(20)
        while not rospy.is_shutdown():
            l_tor = l_arm.get_joint_efforts()
            r_tor = r_arm.get_joint_efforts()
            if np.any(l_tor < self.min_l_tor):
                print "Collision detected on left arm with torque:", l_tor
                print "Minimum torques:", self.min_l_tor
                return 'collision'
            if np.any(l_tor > self.max_l_tor):
                print "Collision detected on left arm with torque:", l_tor
                print "Maximum torques:", self.max_l_tor
                return 'collision'
            if np.any(r_tor < self.min_r_tor):
                print "Collision detected on right arm with torque:", r_tor
                print "Minimum torques:", self.min_r_tor
                return 'collision'
            if np.any(r_tor > self.max_r_tor):
                print "Collision detected on right arm with torque:", r_tor
                print "Minimum torques:", self.max_r_tor
                return 'collision'
                
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            r.sleep()
        return 'aborted'

class LaserCollisionDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'aborted'])
        self.cs = CostmapServices(accum=3)

    def execute(self, userdata):
        r = Rate(20)
        while not rospy.is_shutdown():
            score = self.cs.scoreTraj_PosHyst(vx, vy, vtheta) # TODO
            if score < 0:
                print "Base laser detected a collision."
                return 'collision'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            r.sleep()
        return 'aborted'

def build_cc_servoing():
    def term_cb(outcome_map):
        return True
    def out_cb(outcome_map):
        if outcome_map['ARM_COLLISION_DETECTION'] == 'collision' or
           outcome_map['LASER_COLLISION_DETECTION'] == 'collision':
            return 'collision'
        return outcome_map['SERVOING']

    cc_servoing = smach.Concurrence(
                            outcomes=outcomes_spa+['collision', 'lost_tag'],
                            default_outcome='aborted',
                            child_termination_cb=term_cb,
                            outcome_cb=out_cb)
    
    with cc_servoing:
        Concurrence.add('SERVOING',
                        ,)# TODO

        Concurrence.add('ARM_COLLISION_DETECTION',
                        ArmCollisionDetection())

        Concurrence.add('LASER_COLLISION_DETECTION',
                        LaserCollisionDetection())
                              
    return cc_servoing

class UIState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=outcomes_spa)
        self.ui_list = rospy.Subscriber("servo_continue", Bool, self.ui_cb)
        self.got_msg = False
        self.should_continue = None
    
    def ui_cb(self, msg):
        self.should_continue = msg.data
        self.got_msg = True

    def execute(self, userdata):
        r = Rate(20)
        while not rospy.is_shutdown():
            if self.got_msg:
                if self.should_continue:
                    return "succeeded"
                else:
                    return "aborted"
                self.got_msg = False
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            r.sleep()
        return 'aborted'

def build_sm():
    sm_pr2_servoing = smach.StateMachine(outcomes=outcomes_spa)
    with sm_pr2_servoing:
        smach.StateMachine.add('VALIDATE_AR',
                               ,# TODO
                               transitions={'found_tag' : 'CC_SERVOING',
                                            'tag_missing' : 'USER_INPUT_WAIT'})

        smach.StateMachine.add('USER_INPUT_WAIT',
                               UIState(),
                               transitions={'succeeded' : 'VALIDATE_AR'}

        smach.StateMachine.add('CC_SERVOING', 
                               build_cc_servoing(),
                               transitions={'collision' : 'USER_INPUT_WAIT',
                                            'lost_tag' : 'VALIDATE_AR'})
