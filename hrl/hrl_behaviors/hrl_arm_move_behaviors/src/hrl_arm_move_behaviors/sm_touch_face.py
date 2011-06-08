#! /usr/bin/python

import sys

import roslib
roslib.load_manifest('hrl_arm_move_behaviors')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
import tf.transformations as tft
from geometry_msgs import PoseStamped, Vector3
from actionlib_msgs import GoalStatus

from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest

from hrl_arm_move_behaviors.msg import EPCDirectMoveAction, EPCLinearMoveAction

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
    def __init__(self, thresh=4.0):
        smach.State.__init__(self, outcomes=['collision', 'shutdown'])
        self.thresh = thresh
        self.collided = False
#rospy.Subscriber('', Vector3, self.force_cb)
# TODO FIX THIS

    def force_cb(self, msg):
        f = [msg.x, msg.y, msg.z]
        mag = np.linalg.norm(f)
        self.collided = mag > self.thresh

    def execute(self, userdata):
        while not rospy.is_shutdown():
            if self.collided:
                return 'collision'
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
                userdata.click_pose = self.cur_msg.pose
                return 'click'
            rospy.sleep(0.01)
        return 'shutdown'
        
        
class MoveCoarsePose(smach.State):
    def __init__(self, arm, duration=5.0, q=None):
        smach.State.__init__(self, outcomes=['succeeded', 'shutdown'],
                                   input_keys=['wrist_mat'])
        self.JOINTS_BIAS = [0.0, 0.012, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.INIT_ANGS = [-0.417, -0.280, -1.565, -2.078, -2.785, -1.195, -0.369]
        self.q = q
        self.pr2_arm = PR2ArmBase(arm)

    def execute(self, ud):
        if self.q is None:
            q = self.pr2_arm.biased_IK(ud.wrist_mat, self.INIT_ANGS, self.JOINTS_BIAS)
        else:
            q = self.q
        self.pr2_arm.command_joint_angles(q, duration=self.DURATION, delay=1.0)
        rospy.sleep(0.1)
        if not self.pr2_arm.joint_action_client.wait_for_result(rospy.Duration(10.0)):
            return 'shutdown'
        if self.pr2_arm.joint_action_client.get_state() == GoalStatus.SUCCEEDED:
            return 'succeeded'
        return 'shutdown'

class SMTouchFace(object):
    def __init__(self, arm):
        self.arm = rospy.get_param("~arm", default="r")
        self.tool_frame = rospy.get_param("~tool_frame", default="r_gripper_tool_frame")
        self.tool_approach_frame = rospy.get_param("~tool_approach_frame", default="")

        self.tf_listener = tf.TransformListener()

    def get_transform(self, from_frame, to_frame, time=None):
        if time is None:
            time = ropsy.Time.now()
        try:
            self.tf_listener.waitForTransform(from_frame, to_frame, time)
            pos, quat = self.tf_listener.lookupTransform(from_frame, to_frame)
            return pose_pq_to_mat(pos, quat)
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
            outcomes=['success', 'error_high', 'ik_failure', 'shutdown', 'force_collision'],
            default_outcome='aborted',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb)

        with sm_fine_pos_setup:
            def fine_goal_cb(ud):
                dm_goal = EPCDirectMoveGoal()
                dm_goal.target_pose = ud.target_pose
                dm_gool.tool_frame = self.tool_frame
                return dm_goal
            smach.Concurrence.add(
                'FINE_POSITION_MOVE',
                SimpleActionState('_epc_move/direct_move',
                                  EPCDirectMoveAction,
                                  goal_cb=fine_goal_cb))
            smach.Concurrence.add(
                'FORCE_COLL_MONITOR',
                ForceCollisionMonitor(4.0))

        return sm_fine_pos_setup

    def get_fine_approach(self):
        def child_term_cb(outcome_map):
            return True

        def out_cb(outcome_map):
            if outcome_map['FORCE_COLL_MONITOR'] == 'collision':
                return 'force_collision'
            return outcome_map['FINE_APPROACH_MOVE']

        sm_fine_approach = smach.Concurrence(
            outcomes=['success', 'error_high', 'ik_failure', 'shutdown', 'force_collision'],
            default_outcome='shutdown',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb)

        with sm_fine_approach:
            def fine_appr_cb(ud):
                lm_goal = EPCLinearMoveGoal()
                lm_goal.start_pose = ud.start_pose
                lm_goal.end_pose = ud.end_pose
                lm_gool.tool_frame = self.tool_frame
                return lm_goal
            smach.Concurrence.add(
                'FINE_APPROACH_MOVE',
                SimpleActionState('_epc_move/linear_move',
                                  EPCLinearMoveAction,
                                  goal_cb=fine_appr_cb))
            smach.Concurrence.add(
                'FORCE_COLL_MONITOR',
                ForceCollisionMonitor(4.0))

        return sm_fine_approach

    def get_fine_retreat(self):
        def child_term_cb(outcome_map):
            return True

        def out_cb(outcome_map):
            if outcome_map['FORCE_COLL_MONITOR'] == 'collision':
                return 'force_collision'
            return outcome_map['FINE_POSITION_MOVE']

        sm_fine_retreat = smach.Concurrence(
            outcomes=['success', 'error_high', 'ik_failure', 'shutdown', 'force_collision'],
            default_outcome='shutdown',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb)

        with sm_fine_retreat:
            def retreat_cb(ud):
                lm_goal = EPCLinearMoveGoal()
                lm_goal.start_pose = ud.start_pose
                lm_goal.end_pose = ud.end_pose
                lm_gool.tool_frame = self.tool_frame
                return lm_goal
            smach.Concurrence.add(
                'FINE_POSITION_MOVE',
                SimpleActionState('_epc_move/linear_move',
                                  EPCLinearMoveAction,
                                  goal_cb=retreat_cb))
            smach.Concurrence.add(
                'FORCE_COLL_MONITOR',
                ForceCollisionMonitor(4.0))

        return sm_fine_retreat

    def get_sm(self):

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded','aborted','shutdown'])

        with sm:
            # move to general pose where manipulation begins
            q_setup = [-1.324,  0.083, -1.689, -2.102,  3.127, -0.861, -1.584]
            smach.StateMachine.add(
                'MOVE_PREP_POSE',
                MoveCoarsePose(self.arm, duration=5.0, q=q_setup),
                transitions={'success' : 'WAIT_TOUCH_CLICK',
                             'shutdown' ; 'shutdown'})

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

                torso_B_wrist = torso_B_tool_appr * appr_B_wrist
                ######################################################################### 
                ud.appr_wrist_mat = torso_B_wrist
                ud.appr_tool_ps = util.pose_mat_to_stamped_msg('torso_lift_link', 
                                                               torso_B_touch_appr)
                ud.touch_tool_ps = util.pose_mat_to_stamped_msg('torso_lift_link', 
                                                                torso_B_touch)
                return 'succeeded'

            smach.StateMachine.add(
                'PROCESS_CLICK',
                smach.CBState(process_touch_pose),
                transitions = {'succeeded': 'MOVE_COARSE_IK'},
                remapping={'touch_click_pose' : 'touch_click_pose',
                           'appr_wrist_mat' : 'appr_wrist_mat',
                           'appr_tool_ps' : 'appr_tool_ps',
                           'touch_tool_ps' : 'touch_tool_ps'})

            smach.StateMachine.add(
                'MOVE_COARSE_IK',
                MoveCoarsePose(self.arm, duration=5.0),
                transitions={'success' : 'FINE_POSITION_SETUP',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'WAIT_TOUCH_CLICK',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'shutdown'},
                remapping={'appr_wrist_mat' : 'wrist_mat'})

            smach.StateMachine.add(
                'FINE_POSITION_SETUP',
                self.get_fine_pos_setup(),
                transitions={'success' : 'FINE_APPROACH',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'shutdown'},
                remapping={'target_pose' : 'appr_tool_ps'})

            smach.StateMachine.add(
                'FINE_APPROACH',
                self.get_fine_approach(),
                transitions={'success' : 'WAIT_RETREAT_CLICK',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'WAIT_RETREAT_CLICK'},
                remapping={'start_pose' : 'appr_tool_ps',
                           'end_pose' : 'touch_tool_ps'})

            smach.StateMachine.add(
                'WAIT_RETREAT_CLICK',
                ClickMonitor(),
                transitions={'click' : '',
                             'shutdown' ; 'shutdown'})

            smach.StateMachine.add(
                'FINE_RETREAT',
                self.get_fine_retreat(),
                transitions={'success' : 'MOVE_PREP_POSE',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'shutdown'},
                remapping={'end_pose' : 'appr_tool_ps',
                           'start_pose' : 'touch_tool_ps'})
                
        return sm


def main():
    rospy.init_node('smach_sm_touch_face')

    smtf = SMTouchFace()
    sm = smtf.get_sm()

    sis = IntrospectionServer('Touch Face', sm, '/SM_TOUCH_FACE')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()


if __name__ == '__main__':
    main()

