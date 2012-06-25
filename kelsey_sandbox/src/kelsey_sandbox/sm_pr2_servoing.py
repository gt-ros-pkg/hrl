#!/usr/bin/python

import numpy as np
import functools

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('smach_ros')
roslib.load_manifest('costmap_services')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Int8

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory
from costmap_services.python_client import CostmapServices
from pr2_viz_servo import PR2VisualServoAR

outcomes_spa = ['succeeded','preempted','aborted']

#viz_servo_dict = {
#    "r_pr2_ar_pose_marker" : [0.57226345,  0.32838129, -1.15480113],
#    "l_pr2_ar_pose_marker" : [0.57903398,  0.44215034, -0.76362254]}
#"l_pr2_ar_pose_marker" : [0.59739709,  0.39469539, -0.7088098]}

class ServoStates:
    BEGIN_FIND_TAG = 1
    FOUND_TAG = 2
    TIMEOUT_FIND_TAG = 3
    BEGIN_SERVO = 4
    SUCCESS_SERVO = 5
    ARM_COLLISION = 6
    LASER_COLLISION = 7
    LOST_TAG = 8
    USER_PREEMPT = 9

class ArmCollisionDetection(smach.State):
    def __init__(self, min_l_torques=[-5.]*7, min_r_torques=[-5.]*7,
                       max_l_torques=[5.]*7, max_r_torques=[5.]*7):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'aborted'])
        self.min_l_tor = np.array(min_l_torques)
        self.min_r_tor = np.array(min_r_torques)
        self.max_l_tor = np.array(max_l_torques)
        self.max_r_tor = np.array(max_r_torques)
        self.l_arm = create_pr2_arm('l', PR2ArmJointTrajectory, timeout=0)
        self.r_arm = create_pr2_arm('r', PR2ArmJointTrajectory, timeout=0)

    def execute(self, userdata):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            l_tor = self.l_arm.get_joint_efforts()
            r_tor = self.r_arm.get_joint_efforts()
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
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            vx, vy, vtheta = 0., 0., 0. #TODO REMOVE
            score = self.cs.scoreTraj_PosHyst(vx, vy, vtheta) # TODO
            if score < 0:
                print "Base laser detected a collision."
                return 'collision'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            r.sleep()
        return 'aborted'

def build_cc_servoing(viz_servos):
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
                            input_keys=['goal_ar_pose', 'initial_ar_pose', 'servo_topic'],
                            default_outcome='aborted',
                            child_termination_cb=term_cb,
                            outcome_cb=out_cb)
    
    with cc_servoing:
        smach.Concurrence.add('SERVOING',
                              ServoARTagState(viz_servos))

        smach.Concurrence.add('ARM_COLLISION_DETECTION',
                              ArmCollisionDetection())

        smach.Concurrence.add('LASER_COLLISION_DETECTION',
                              LaserCollisionDetection())

        smach.Concurrence.add('USER_PREEMPT_DETECTION',
                              BoolTopicState("/pr2_ar_servo/preempt"))
                              
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
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.got_msg:
                if self.bool_msg:
                    return 'true'
                else:
                    return 'false'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            r.sleep()
        return 'aborted'

class PublishState(smach.State):
    def __init__(self, topic, msg_type, msg):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub = rospy.Publisher(topic, msg_type)
        self.msg = msg

    def execute(self, userdata):
        self.pub.publish(self.msg)
        return 'succeeded'

class FindARTagState(smach.State):
    def __init__(self, viz_servos, timeout=6.):
        smach.State.__init__(self, 
                             outcomes=['found_tag', 'timeout', 'preempted', 'aborted'],
                             output_keys=['initial_ar_pose', 'goal_ar_pose', 'servo_topic'])
        self.viz_servos = viz_servos
        self.timeout = timeout

    def execute(self, userdata):
        # This is just a mess.
        # Figure this out and rewrite it...
        def check_preempt(event):
            if self.preempt_requested():
                self.service_preempt()
                for topic in self.viz_servos:
                    self.viz_servos[topic].request_preempt()
                
        preempt_timer = rospy.Timer(rospy.Duration(0.1), check_preempt)
        self.outcome_dict = {}
        def call_find_ar_tag(te, viz_servo, fart_state, topic):
            mean_ar, outcome = viz_servo.find_ar_tag(self.timeout)
            self.outcome_dict[topic] = (mean_ar, outcome)
            if outcome == "found_tag":
                self.request_preempt()
        for topic in self.viz_servos:
            call_find_ar_tag_filled = functools.partial(call_find_ar_tag, 
                                                        viz_servo=self.viz_servos[topic],
                                                        fart_state=self,
                                                        topic=topic)
            rospy.Timer(rospy.Duration(0.01), call_find_ar_tag_filled, oneshot=True)
        while not rospy.is_shutdown():
            if self.preempt_requested() or len(self.outcome_dict) == len(self.viz_servos):
                break
            rospy.sleep(0.05)
        outcome = "timeout"
        for topic in self.outcome_dict:
            if self.outcome_dict[topic][1] == "found_tag":
                userdata['initial_ar_pose'] = self.outcome_dict[topic][0]
                userdata['goal_ar_pose'] = viz_servo_dict[topic]
                userdata['servo_topic'] = topic
                outcome = "found_tag"
                # FIXME I should be shot for these lines: FIXME
                if topic == "r_pr2_ar_pose_marker":
                    rospy.set_param("/shaving_side", 'r')
                else:
                    rospy.set_param("/shaving_side", 'l')
                ##################################################
                break
            
        preempt_timer.shutdown()
        return outcome

class ServoARTagState(smach.State):
    def __init__(self, viz_servos):
        smach.State.__init__(self, 
                             outcomes=['lost_tag'] + outcomes_spa,
                             input_keys=['goal_ar_pose', 'initial_ar_pose', 'servo_topic'])
        self.viz_servos = viz_servos

    def execute(self, userdata):
        if 'initial_ar_pose' in userdata:
            initial_ar_pose = userdata.initial_ar_pose
        else:
            rospy.logerr("Initial AR pose should be in userdata")
            initial_ar_pose = None

        servo_topic = userdata['servo_topic']
        def check_preempt(event):
            if self.preempt_requested():
                self.service_preempt()
                self.viz_servos[servo_topic].request_preempt()
                
        preempt_timer = rospy.Timer(rospy.Duration(0.1), check_preempt)
        self.viz_servos[servo_topic].preempt_requested = False
        outcome = self.viz_servos[servo_topic].servo_to_tag(pose_goal=userdata.goal_ar_pose,
                                                            initial_ar_pose=initial_ar_pose)
        preempt_timer.shutdown()
        return outcome

def build_full_sm():
    find_tag_timeout = 6.

    viz_servos = {}
    for viz_servo_topic in viz_servo_dict:
        viz_servos[viz_servo_topic] = PR2VisualServoAR(viz_servo_topic)

    sm_pr2_servoing = smach.StateMachine(outcomes=outcomes_spa,
                                         input_keys=['goal_ar_pose', 'initial_ar_pose'])
    with sm_pr2_servoing:

        smach.StateMachine.add('UI_FIND_TAG_WAIT',
                               BoolTopicState("/pr2_ar_servo/find_tag"),
                               transitions={'true' : 'BEGIN_FIND_TAG',
                                            'false' : 'aborted'})

        smach.StateMachine.add('BEGIN_FIND_TAG',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.BEGIN_FIND_TAG)),
                               transitions={'succeeded' : 'FIND_AR_TAG'})

        smach.StateMachine.add('FIND_AR_TAG',
                               FindARTagState(viz_servos, timeout=find_tag_timeout),
                               transitions={'found_tag' : 'FOUND_TAG',
                                            'timeout' : 'TIMEOUT_FIND_TAG'})

        smach.StateMachine.add('FOUND_TAG',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.FOUND_TAG)),
                               transitions={'succeeded' : 'UI_SERVO_WAIT'})

        smach.StateMachine.add('TIMEOUT_FIND_TAG',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.TIMEOUT_FIND_TAG)),
                               transitions={'succeeded' : 'UI_FIND_TAG_WAIT'})

        smach.StateMachine.add('UI_SERVO_WAIT',
                               BoolTopicState("/pr2_ar_servo/tag_confirm"),
                               transitions={'true' : 'BEGIN_SERVO',
                                            'false' : 'UI_FIND_TAG_WAIT'})

        smach.StateMachine.add('BEGIN_SERVO',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.BEGIN_SERVO)),
                               transitions={'succeeded' : 'CC_SERVOING'})

        smach.StateMachine.add('CC_SERVOING', 
                               build_cc_servoing(viz_servos),
                               transitions={'arm_collision' : 'ARM_COLLISION',
                                            'laser_collision' : 'LASER_COLLISION',
                                            'user_preempted' : 'USER_PREEMPT',
                                            'lost_tag' : 'LOST_TAG',
                                            'succeeded' : 'SUCCESS_SERVO'})

        smach.StateMachine.add('SUCCESS_SERVO',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.SUCCESS_SERVO)),
                               transitions={'succeeded' : 'UI_FIND_TAG_WAIT'})

        smach.StateMachine.add('ARM_COLLISION',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.ARM_COLLISION)),
                               transitions={'succeeded' : 'UI_FIND_TAG_WAIT'})

        smach.StateMachine.add('LASER_COLLISION',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.LASER_COLLISION)),
                               transitions={'succeeded' : 'UI_FIND_TAG_WAIT'})

        smach.StateMachine.add('LOST_TAG',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.LOST_TAG)),
                               transitions={'succeeded' : 'UI_FIND_TAG_WAIT'})

        smach.StateMachine.add('USER_PREEMPT',
                               PublishState("/pr2_ar_servo/state_feedback",
                                            Int8, Int8(ServoStates.USER_PREEMPT)),
                               transitions={'succeeded' : 'UI_FIND_TAG_WAIT'})

    return sm_pr2_servoing

def build_test_sm():
    viz_servo = PR2VisualServoAR("r_pr2_ar_pose_marker")
    
    sm_only_servo = smach.StateMachine(outcomes=['lost_tag'] + outcomes_spa,
                                       input_keys=['goal_ar_pose', 'initial_ar_pose'])
    with sm_only_servo:
        smach.StateMachine.add('ONLY_SERVO',
                               ServoARTagState(viz_servo))
    return sm_only_servo

def main():
    rospy.init_node("sm_pr2_servoing")
    userdata = smach.user_data.UserData()

    # old testing
    #userdata['goal_ar_pose'] = [ 0.57160106, -0.4300153 , -1.70840111]

    # both sides spot?
    userdata['goal_ar_pose'] = [ 0.57226345,  0.32838129, -1.15480113]
    viz_servo_dict = rospy.get_param("~ar_servo_poses", {})
    print viz_servo_dict

    if True:
        sm_pr2_servoing = build_full_sm()
        sis = smach_ros.IntrospectionServer('pr2_servo', sm_pr2_servoing, 'UI_FIND_TAG_WAIT')
        sis.start()
        sm_pr2_servoing.execute(userdata)
        sis.stop()
    else:
        sm_test = build_test_sm()
        rospy.sleep(4)
        sm_test.execute(userdata)

if __name__ == "__main__":
    main()
