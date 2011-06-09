#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_arm_move_behaviors')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped, Vector3
from actionlib_msgs.msg import GoalStatus

from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest

from hrl_arm_move_behaviors.msg import EPCDirectMoveAction, EPCLinearMoveAction
from hrl_arm_move_behaviors.msg import EPCDirectMoveGoal, EPCLinearMoveGoal
from hrl_arm_move_behaviors.pr2_arm_base import PR2ArmBase
import hrl_arm_move_behaviors.util as util

class NTries(smach.State):
    def __init__(self, n):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                   output_keys=['ntries_counter'])
        self.counter = 0
        self.n = n

    def execute(self, userdata):
        self.counter += 1
        userdata.ntries_counter = self.counter

        if self.counter <= self.n:
            rospy.logout( 'Executing NTries: On #%d of %d' % (self.counter, self.n))
            return 'succeeded'
        else:
            return 'aborted'

class ForceCollisionMonitor(smach.State):
    def __init__(self, thresh=0.0005):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'shutdown'])
        self.thresh = thresh
        self.collided = False
        rospy.Subscriber('/force_torque_monitor/force_signal', Float32, self.force_cb)

    def force_cb(self, msg):
        self.collided = msg.data > self.thresh

    def execute(self, userdata):
        while not rospy.is_shutdown():
            if self.collided:
                return 'collision'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.01)
        return 'shutdown'
        
class ClickMonitor(smach.State):
    def __init__(self, appr_B_tool=np.eye(4)):
        smach.State.__init__(self, outcomes=['click', 'shutdown'],
                                   output_keys=['click_pose'])
        self.appr_B_tool = appr_B_tool
        self.cur_msg = None
        rospy.Subscriber('/pixel3d', PoseStamped, self.click_cb)

    def click_cb(self, msg):
        self.cur_msg = msg

    def execute(self, userdata):
        self.cur_msg = None
        while not rospy.is_shutdown():
            if self.cur_msg is not None:
                userdata.click_pose = self.cur_msg
                return 'click'
            rospy.sleep(0.01)
        return 'shutdown'
        
        
class MoveCoarsePose(smach.State):
    def __init__(self, arm, duration=5.0, q=None):
        smach.State.__init__(self, outcomes=['succeeded', 'ik_failure', 'shutdown'],
                                   input_keys=['wrist_mat'])
        self.JOINTS_BIAS = [0.0, 0.012, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.INIT_ANGS = [-0.417, -0.280, -1.565, -2.078, -2.785, -1.195, -0.369]
        self.duration = duration
        self.q = q
        self.pr2_arm = PR2ArmBase(arm)

    def execute(self, ud):
        if self.q is None:
            q = self.pr2_arm.biased_IK(np.mat(ud.wrist_mat), self.INIT_ANGS, self.JOINTS_BIAS)
            if q is None:
                return 'ik_failure'
        else:
            q = self.q
        self.pr2_arm.command_joint_angles(q, duration=self.duration, delay=1.0)
        rospy.sleep(0.1)
        if not self.pr2_arm.joint_action_client.wait_for_result(rospy.Duration(15.0)):
            rospy.logerr('[sm_touch_face] Timeout commanding joint angles.')
            return 'shutdown'
        rospy.loginfo('[sm_touch_face] MoveCoarsePose joint command returned with state: %d.' % 
                      self.pr2_arm.joint_action_client.get_state())
        if self.pr2_arm.joint_action_client.get_state() == GoalStatus.SUCCEEDED:
            return 'succeeded'
        return 'shutdown'

class SMTouchFace(object):
    def __init__(self):
        self.arm = rospy.get_param("~arm", default="r")
        self.tool_frame = rospy.get_param("~tool_frame", default="r_gripper_tool_frame")
        self.tool_approach_frame = rospy.get_param("~tool_approach_frame", default="")

        self.tf_listener = tf.TransformListener()

        self.wrist_pub = rospy.Publisher("~wrist_setup", PoseStamped)
        self.appr_pub = rospy.Publisher("~approach_location", PoseStamped)
        self.touch_pub = rospy.Publisher("~touch_location", PoseStamped)

    def get_transform(self, from_frame, to_frame, time=None):
        if time is None:
            time = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(from_frame, to_frame, time, rospy.Duration(5))
            pos, quat = self.tf_listener.lookupTransform(from_frame, to_frame, time)
            return util.pose_pq_to_mat(pos, quat)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return None

    def get_fine_pos_setup(self):
        def child_term_cb(outcome_map):
            return True

        def out_cb(outcome_map):
            if outcome_map['FORCE_COLL_MONITOR'] == 'collision':
                return 'force_collision'
            return outcome_map['FINE_POSITION_MOVE']

        sm_fine_pos_setup = smach.Concurrence(
            outcomes=['succeeded', 'error_high', 'ik_failure', 'shutdown', 
                      'force_collision', 'aborted', 'preempted'],
            input_keys=['target_pose'],
            default_outcome='shutdown',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb)

        with sm_fine_pos_setup:
            def fine_goal_cb(ud, goal):
                dm_goal = EPCDirectMoveGoal(ud.target_pose, self.tool_frame, True, 
                                            0.02, 0.35, 0.2, 1.0, 5)
                return dm_goal
            @smach.cb_interface(outcomes=['succeeded', 'error_high', 'ik_failure', 'shutdown', 
                                          'force_collision', 'aborted', 'preempted'])
            def res_cb(us, status, result):
                rospy.sleep(1)
                return result.result
            smach.Concurrence.add(
                'FINE_POSITION_MOVE',
                SimpleActionState(self.arm + '_epc_move/direct_move',
                                  EPCDirectMoveAction,
                                  goal_cb=fine_goal_cb,
                                  result_cb=res_cb,
                                  input_keys=['target_pose']))
            smach.Concurrence.add(
                'FORCE_COLL_MONITOR',
                ForceCollisionMonitor(0.0006))

        return sm_fine_pos_setup

    def get_fine_approach(self):
        def child_term_cb(outcome_map):
            return True

        def out_cb(outcome_map):
            if outcome_map['FORCE_COLL_MONITOR'] == 'collision':
                return 'force_collision'
            return outcome_map['FINE_APPROACH_MOVE']

        sm_fine_approach = smach.Concurrence(
            outcomes=['succeeded', 'error_high', 'ik_failure', 'shutdown', 
                      'force_collision', 'aborted', 'preempted'],
            input_keys=['start_pose', 'end_pose'],
            default_outcome='shutdown',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb)

        with sm_fine_approach:
            def fine_appr_cb(ud, goal):
                lm_goal = EPCLinearMoveGoal(ud.start_pose, ud.end_pose, self.tool_frame, 
                                            False, 0.003, 0.1, 1.0, 3)
                return lm_goal
            @smach.cb_interface(outcomes=['succeeded', 'error_high', 'ik_failure', 'shutdown', 
                                          'force_collision', 'aborted', 'preempted'])
            def res_cb(us, status, result):
                rospy.sleep(1)
                return result.result
            smach.Concurrence.add(
                'FINE_APPROACH_MOVE',
                SimpleActionState(self.arm + '_epc_move/linear_move',
                                  EPCLinearMoveAction,
                                  goal_cb=fine_appr_cb,
                                  result_cb=res_cb,
                                  input_keys=['start_pose', 'end_pose']))
            smach.Concurrence.add(
                'FORCE_COLL_MONITOR',
                ForceCollisionMonitor(0.0003))

        return sm_fine_approach

    def get_fine_retreat(self):
        def child_term_cb(outcome_map):
            if outcome_map['FORCE_COLL_MONITOR'] == 'collision':
                return False
            return True

        def out_cb(outcome_map):
            if outcome_map['FORCE_COLL_MONITOR'] == 'collision':
                return 'force_collision'
            return outcome_map['FINE_RETREAT_MOVE']

        sm_fine_retreat = smach.Concurrence(
            outcomes=['succeeded', 'error_high', 'ik_failure', 'shutdown', 
                      'force_collision', 'aborted', 'preempted'],
            input_keys=['start_pose', 'end_pose'],
            default_outcome='shutdown',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb)

        with sm_fine_retreat:
            def retreat_cb(ud, goal):
                lm_goal = EPCLinearMoveGoal(ud.start_pose, ud.end_pose, self.tool_frame, 
                                            False, 0.03, 0.1, 1.0, 3)
                return lm_goal
            @smach.cb_interface(outcomes=['succeeded', 'error_high', 'ik_failure', 'shutdown', 
                                          'force_collision', 'aborted'])
            def res_cb(us, status, result):
                rospy.sleep(1)
                return result.result
            smach.Concurrence.add(
                'FINE_RETREAT_MOVE',
                SimpleActionState(self.arm + '_epc_move/linear_move',
                                  EPCLinearMoveAction,
                                  goal_cb=retreat_cb,
                                  result_cb=res_cb,
                                  input_keys=['start_pose', 'end_pose']))
            smach.Concurrence.add(
                'FORCE_COLL_MONITOR',
                ForceCollisionMonitor(0.01))

        return sm_fine_retreat

    def get_sm(self):

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded','preempted','shutdown'])

        with sm:
            # move to general pose where manipulation begins
            q_setup = [-1.324,  0.083, -0.689, -2.102,  3.127, -0.861, -1.584]
            smach.StateMachine.add(
                'MOVE_PREP_POSE',
                MoveCoarsePose(self.arm, duration=10.0, q=q_setup),
                transitions={'succeeded' : 'WAIT_TOUCH_CLICK',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown'})

            smach.StateMachine.add(
                'WAIT_TOUCH_CLICK',
                ClickMonitor(),
                transitions={'click' : 'PROCESS_CLICK',
                             'shutdown' : 'shutdown'},
                remapping={'click_pose' : 'touch_click_pose'})

            @smach.cb_interface(input_keys=['touch_click_pose'],
                                output_keys=['appr_wrist_mat', 'appr_tool_ps', 'touch_tool_ps'],
                                outcomes=['succeeded', 'tf_failure'])
            def process_touch_pose(ud):
                ######################################################################### 
                # tranformation logic for manipulation
                # put touch pose in torso frame
                frame_B_touch = util.pose_msg_to_mat(ud.touch_click_pose)
                torso_B_frame = self.get_transform("torso_lift_link", 
                                                   ud.touch_click_pose.header.frame_id)
                if torso_B_frame is None:
                    return 'tf_failure'
                torso_B_touch_bad = torso_B_frame * frame_B_touch

                # rotate pixel23d the right way
                t_pos, t_quat = util.pose_mat_to_pq(torso_B_touch_bad)
                # rotate so x axis pointing out
                quat_flip_rot = tf_trans.quaternion_from_euler(0.0, np.pi/2.0, 0.0)
                quat_flipped = tf_trans.quaternion_multiply(t_quat, quat_flip_rot)
                # rotate around x axis so the y axis is flat
                mat_flipped = tf_trans.quaternion_matrix(quat_flipped)
                rot_angle = np.arctan(-mat_flipped[2,1] / mat_flipped[2,2])
                quat_ortho_rot = tf_trans.quaternion_from_euler(rot_angle + np.pi, 0.0, 0.0)
                quat_flipped_ortho = tf_trans.quaternion_multiply(quat_flipped, quat_ortho_rot)

                torso_B_touch = util.pose_pq_to_mat(t_pos, quat_flipped_ortho)

                # offset the touch location by the approach tranform
                appr_B_tool = self.get_transform(self.tool_approach_frame, self.tool_frame) 
                if appr_B_tool is None:
                    return 'tf_failure'
                torso_B_touch_appr = torso_B_touch * appr_B_tool

                # project the approach back into the wrist
                appr_B_wrist = self.get_transform(self.tool_approach_frame, 
                                                  self.arm + "_wrist_roll_link") 
                if appr_B_wrist is None:
                    return 'tf_failure'

                torso_B_wrist = torso_B_touch_appr * appr_B_wrist
                ######################################################################### 
                # create PoseStamped
                appr_wrist_ps = util.pose_mat_to_stamped_msg('torso_lift_link',
                                                             torso_B_wrist)
                appr_tool_ps = util.pose_mat_to_stamped_msg('torso_lift_link', 
                                                            torso_B_touch_appr)
                touch_tool_ps = util.pose_mat_to_stamped_msg('torso_lift_link', 
                                                             torso_B_touch)

                # visualization
                self.wrist_pub.publish(appr_wrist_ps)
                self.appr_pub.publish(appr_tool_ps)
                self.touch_pub.publish(touch_tool_ps)

                # set return values
                ud.appr_wrist_mat = torso_B_wrist
                ud.appr_tool_ps = appr_tool_ps
                ud.touch_tool_ps = touch_tool_ps

                
                return 'succeeded'

            smach.StateMachine.add(
                'PROCESS_CLICK',
                smach.CBState(process_touch_pose),
                transitions = {'succeeded' : 'MOVE_COARSE_IK',
                               'tf_failure' : 'WAIT_TOUCH_CLICK'},
                remapping={'touch_click_pose' : 'touch_click_pose',
                           'appr_wrist_mat' : 'appr_wrist_mat',
                           'appr_tool_ps' : 'appr_tool_ps',
                           'touch_tool_ps' : 'touch_tool_ps'})

            # TODO ADD FORCE DETECTION
            smach.StateMachine.add(
                'MOVE_COARSE_IK',
                MoveCoarsePose(self.arm, duration=10.0),
                transitions={'succeeded' : 'FINE_POSITION_SETUP',
                             'ik_failure' : 'WAIT_TOUCH_CLICK',
                             'shutdown' : 'shutdown'},
                remapping={'wrist_mat' : 'appr_wrist_mat'})

            smach.StateMachine.add(
                'FINE_POSITION_SETUP',
                self.get_fine_pos_setup(),
                transitions={'succeeded' : 'FINE_APPROACH',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'shutdown',
                             'aborted' : 'shutdown',
                             'preempted' : 'FINE_POSITION_SETUP'},
                remapping={'target_pose' : 'appr_tool_ps'})

            smach.StateMachine.add(
                'FINE_APPROACH',
                self.get_fine_approach(),
                transitions={'succeeded' : 'WAIT_RETREAT_CLICK',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'WAIT_RETREAT_CLICK',
                             'aborted' : 'shutdown',
                             'preempted' : 'FINE_APPROACH'},
                remapping={'start_pose' : 'appr_tool_ps',
                           'end_pose' : 'touch_tool_ps'})

            smach.StateMachine.add(
                'WAIT_RETREAT_CLICK',
                ClickMonitor(),
                transitions={'click' : 'FINE_RETREAT',
                             'shutdown' : 'shutdown'})

            smach.StateMachine.add(
                'FINE_RETREAT',
                self.get_fine_retreat(),
                transitions={'succeeded' : 'MOVE_PREP_POSE',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'shutdown',
                             'aborted' : 'shutdown',
                             'preempted' : 'shutdown'},
                remapping={'end_pose' : 'appr_tool_ps',
                           'start_pose' : 'touch_tool_ps'})
                
        return sm


def main():
    rospy.init_node('smach_sm_touch_face')

    smtf = SMTouchFace()
    sm = smtf.get_sm()
    rospy.sleep(1)

    sis = IntrospectionServer('touch_face', sm, '/SM_TOUCH_FACE')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()


if __name__ == '__main__':
    main()

