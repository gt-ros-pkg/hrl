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
from kelsey_sandbox.pr2_viz_servo import PR2VisualServoAR

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

def build_cc_servoing(viz_servo):
    def term_cb(outcome_map):
        return True
    def out_cb(outcome_map):
        if outcome_map['ARM_COLLISION_DETECTION'] == 'collision':
            return 'arm_collision'
        if outcome_map['LASER_COLLISION_DETECTION'] == 'collision':
            return 'laser_collision'
        if outcome_map['USER_PREEMPT_DETECTION'] == 'true':
            return 'user_preempted'
        if outcome_map['USER_PREEMPT_DETECTION'] == 'false':
            return 'aborted'
        return outcome_map['SERVOING']

    cc_servoing = smach.Concurrence(
                            outcomes=outcomes_spa+
                            ['arm_collision', 'laser_collision', 'lost_tag', 'user_preempted'],
                            default_outcome='aborted',
                            child_termination_cb=term_cb,
                            outcome_cb=out_cb)
    
    with cc_servoing:
        Concurrence.add('SERVOING',
                        ServoARTagState(viz_servo))

        Concurrence.add('ARM_COLLISION_DETECTION',
                        ArmCollisionDetection())

        Concurrence.add('LASER_COLLISION_DETECTION',
                        LaserCollisionDetection())

        Concurrence.add('USER_PREEMPT_DETECTION',
                        BoolTopicState("servo_preempt"))
                              
    return cc_servoing

class BoolTopicState(smach.State):
    def __init__(self, topic, rate=20):
        smach.State.__init__(self, outcomes=['true', 'false', 'preempted', 'aborted'])
        self.rate = rate
        self.ui_list = rospy.Subscriber(topic, Bool, self.ui_cb)
    
    def ui_cb(self, msg):
        self.bool_msg = msg.data
        self.got_msg = True

    def execute(self, userdata):
        self.got_msg = False
        r = Rate(self.rate)
        while not rospy.is_shutdown():
            if self.bool_msg:
                return 'true'
            else:
                return 'false'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            r.sleep()
        return 'aborted'

class FindARTagState(smach.State):
    def __init__(self, viz_servo, timeout=6.):
        smach.State.__init__(self, 
                             outcomes=['found_tag', 'tag_missing', 'preempted', 'aborted'],
                             output_keys=['initial_ar_pose'])
        self.viz_servo = viz_servo
        self.timeout = timeout

    def execute(self, userdata):
        def check_preempt(event):
            if self.preempt_requested():
                self.service_preempt()
                self.viz_servo.request_preempt()
                
        preempt_timer = rospy.Timer(rospy.Duration(0.1), check_preempt)
        mean_ar, outcome = self.viz_servo.find_ar_tag(self.timeout)
        preempt_timer.shutdown()
        userdata['initial_ar_pose'] = mean_ar
        return outcome

class ServoARTagState(smach.State):
    def __init__(self, viz_servo):
        smach.State.__init__(self, 
                             outcomes=['lost_tag'] + outcomes_spa,
                             input_keys=['goal_ar_pose', 'initial_ar_pose'])
        self.viz_servo = viz_servo

    def execute(self, userdata):
        if 'initial_ar_pose' in userdata:
            initial_ar_pose = userdata.initial_ar_pose
        else:
            rospy.logerr("Initial AR pose should be in userdata")
            initial_ar_pose = None

        def check_preempt(event):
            if self.preempt_requested():
                self.service_preempt()
                self.viz_servo.request_preempt()
                
        preempt_timer = rospy.Timer(rospy.Duration(0.1), check_preempt)
        outcome = self.viz_servo.servo_to_tag(pose_goal=userdata.goal_ar_pose,
                                              initial_ar_pose=initial_ar_pose)
        preempt_timer.shutdown()
        return outcome

def build_full_sm():

    viz_servo = PR2VisualServoAR()

    sm_pr2_servoing = smach.StateMachine(outcomes=outcomes_spa)
    with sm_pr2_servoing:

        smach.StateMachine.add('VALIDATE_AR',
                               FindARTagState(),
                               transitions={'found_tag' : 'CC_SERVOING',
                                            'tag_missing' : 'USER_INPUT_WAIT'})

        smach.StateMachine.add('USER_INPUT_WAIT',
                               BoolTopicState("servo_continue"),
                               transitions={'true' : 'VALIDATE_AR',
                                            'false' : 'aborted'})

        smach.StateMachine.add('CC_SERVOING', 
                               build_cc_servoing(viz_servo),
                               transitions={'collision' : 'USER_INPUT_WAIT',
                                            'lost_tag' : 'VALIDATE_AR'})

    return sm_pr2_servoing

def build_test_sm():
    viz_servo = PR2VisualServoAR()
    
    sm_only_servo = smach.StateMachine(outcomes=['lost_tag'] + outcomes_spa)
    with sm_only_servo:
        smach.StateMachine.add('ONLY_SERVO',
                               ServoARTagState(viz_servo))
    return sm_only_servo

def main():
    rospy.init_node("sm_pr2_servoing")
    userdata = smach.user_data.UserData()
    userdata['goal_ar_pose'] = [0.55, -0.29, 1.57]

    if False:
        sm_pr2_servoing = build_sm()
        sm_pr2_servoing.execute(userdata)
    else:
        sm_test = build_test_sm()
        sm_test.execute(userdata)

if __name__ == "__main__":
    main()
