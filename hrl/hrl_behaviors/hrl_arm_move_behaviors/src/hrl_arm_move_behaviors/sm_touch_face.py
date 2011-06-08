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
    def __init__(self):
        smach.State.__init__(self, outcomes=['click', 'shutdown'],
                                   output_keys=['click_pose'])
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
        smach.State.__init__(self, outcomes=['succeeded', 'shutdown'],
                                   input_keys=['touch_pose'])
        self.JOINTS_BIAS = [0.0, 0.012, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.INIT_ANGS = [-0.417, -0.280, -1.565, -2.078, -2.785, -1.195, -0.369]
        self.q = q
        self.pr2_arm = PR2ArmBase(arm)

    def execute(self, userdata):
        if self.q is None:
            # TODO FILL pose
            q = self.pr2_arm.biased_IK(pose, self.INIT_ANGS, self.JOINTS_BIAS)
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
        self.arm = arm

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
            dm_goal = EPCDirectMoveGoal()
            # TODO FILL THIS OUT
            smach.Concurrence.add(
                'FINE_POSITION_MOVE',
                SimpleActionState('_epc_move/direct_move',
                                  EPCDirectMoveAction,
                                  goal = dm_goal))
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
            return outcome_map['FINE_POSITION_MOVE']

        sm_fine_approach = smach.Concurrence(
            outcomes=['success', 'error_high', 'ik_failure', 'shutdown', 'force_collision'],
            default_outcome='shutdown',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb)

        with sm_fine_approach:
            lm_goal = EPCLinearMoveGoal()
            # TODO FILL THIS OUT
            smach.Concurrence.add(
                'FINE_POSITION_MOVE',
                SimpleActionState('_epc_move/linear_move',
                                  EPCLinearMoveAction,
                                  goal = lm_goal))
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
            lm_goal = EPCLinearMoveGoal()
            # TODO FILL THIS OUT
            smach.Concurrence.add(
                'FINE_POSITION_MOVE',
                SimpleActionState('_epc_move/linear_move',
                                  EPCLinearMoveAction,
                                  goal = lm_goal))
            smach.Concurrence.add(
                'FORCE_COLL_MONITOR',
                ForceCollisionMonitor(4.0))

        return sm_fine_retreat

    def get_sm(self):

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded','aborted','shutdown'])

        with sm:
            # move to general pose where manipulation begins
            # TODO FILL q_setup
            smach.StateMachine.add(
                'MOVE_PREP_POSE',
                MoveCoarsePose(self.arm, duration=5.0, q=q_setup),
                transitions={'success' : 'WAIT_TOUCH_CLICK',
                             'shutdown' ; 'shutdown'})

            smach.StateMachine.add(
                'WAIT_TOUCH_CLICK',
                ClickMonitor(),
                transitions={'click' : 'MOVE_COARSE_IK',
                             'shutdown' : 'shutdown'},
                remapping={'click_pose' : 'touch_pose'})

            smach.StateMachine.add(
                'MOVE_COARSE_IK',
                MoveCoarsePose(self.arm, duration=5.0),
                transitions={'success' : 'FINE_POSITION_SETUP',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'WAIT_TOUCH_CLICK',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'shutdown'})

            smach.StateMachine.add(
                'FINE_POSITION_SETUP',
                self.get_fine_pos_setup(),
                transitions={'success' : 'FINE_APPROACH',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'shutdown'},
                remapping={'touch_pose' : 'touch_pose'})

            smach.StateMachine.add(
                'FINE_APPROACH',
                self.get_fine_approach(),
                transitions={'success' : 'WAIT_RETREAT_CLICK',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'WAIT_RETREAT_CLICK'},
                remapping={'touch_pose' : 'touch_pose'})

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
                remapping={'touch_pose' : 'touch_pose'})
                
        return sm


def main():
    rospy.init_node('smach_sm_touch_face')
    if len(sys.argv) < 2 or sys.argv[1] not in ['r', 'l']:
        print "First arg should be 'r' or 'l'"
        return
    arm = sys.argv[1]

    smtf = SMTouchFace(arm)
    sm = smtf.get_sm()

    sis = IntrospectionServer('Touch Face', sm, '/SM_TOUCH_FACE')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()


if __name__ == '__main__':
    main()

