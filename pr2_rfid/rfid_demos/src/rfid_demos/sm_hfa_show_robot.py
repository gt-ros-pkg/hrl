#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('pr2_grasp_behaviors')
roslib.load_manifest('hrl_trajectory_playback')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('robotis')
import rospy

import smach
import actionlib

from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rfid_behaviors.srv import NextBestVantage
from rfid_behaviors.srv import HandoffSrv, HandoffSrvRequest
from rfid_behaviors.srv import FloatFloat_Int32Request as RotateBackupSrvRequest
from rfid_behaviors.srv import FloatFloat_Int32 as RotateBackupSrv
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from robotis.srv import MoveAng, MoveAngRequest
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest

import sm_rfid_delivery
#from sm_next_best_vantage import BestVantage
# import sm_fetch
# import sm_rfid_explore

class PrintStr(smach.State):
    def __init__(self, ins = 'Hand me an object [ENTER]'):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.ins = ins

    def execute(self, userdata):
        rospy.logout( 'Executing PrintStr: %s' % self.ins )
        raw_input()
        return 'succeeded'

class InitLocalization(smach.State):
    def __init__(self, init_pose = None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.init_pose = init_pose
                            
    def execute(self, userdata):
        rospy.logout( 'Initializing Localization' )

        pub = rospy.Publisher( '/initialpose', PoseWithCovarianceStamped )
        rospy.sleep( 0.5 )

        if not self.init_pose:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = '\map'

            msg.pose.pose.position.x = -1.565
            msg.pose.pose.position.y =  0.807
            msg.pose.pose.position.z =  0.000

            msg.pose.pose.orientation.w = 1.000

            msg.pose.covariance = [0.25, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.25,
                                              0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.068, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0]

            print 'RUNNING: ', msg

        raw_input( 'Drive the robot to the initial location (in kitchen by default).  Hit [ENTER] when done.' )
        msg.header.stamp = rospy.Time(0)
        pub.publish( msg )

        # for i in xrange( 5 ):
        #     msg.header.stamp = rospy.Time(0)
        #     pub.publish( msg )
        #     rospy.sleep( 0.05 )
            
        return 'succeeded'


def init_local_test():
    sm = smach.StateMachine( outcomes=['succeeded','aborted'] )
    with sm:
        smach.StateMachine.add(
            'INIT_LOCALIZATION',
            InitLocalization(),
            transitions = { 'succeeded':'succeeded' })
    return sm


def cousins_demo():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'person_id' ])

    with sm:

        smach.StateMachine.add(
            'ROTATE_BEFORE_MOVE',
            ServiceState( '/rotate_backup',
                          RotateBackupSrv,
                          request = RotateBackupSrvRequest( 3.14, 0.0)),  # Full 180-deg spin.
            transitions = { 'succeeded':'MOVE_SHOW_OFF_POSE' })

        gd = MoveBaseGoal()
        gd.target_pose.header.frame_id = '/map'
        gd.target_pose.header.stamp = rospy.Time(0)
        gd.target_pose.pose.position.x = 2.956
        gd.target_pose.pose.position.y = 3.047
        gd.target_pose.pose.orientation.z = 0.349
        gd.target_pose.pose.orientation.w = 0.937

        smach.StateMachine.add(
            'MOVE_SHOW_OFF_POSE',
            SimpleActionState( '/move_base',
                               MoveBaseAction,
                               goal = gd ),
            # transitions = {'succeeded':'READY_RETURN_HALLWAY'})
            transitions = {'succeeded':'succeeded'})


        # smach.StateMachine.add(
        #     'READY_RETURN_HALLWAY',
        #     PrintStr('Ready to return to hallway [ENTER]'), 
        #     transitions = { 'succeeded':'MOVE_HALLWAY' })

        # go = MoveBaseGoal()
        # go.target_pose.header.frame_id = '/map'
        # go.target_pose.header.stamp = rospy.Time(0)
        # go.target_pose.pose.position.x = -5.07
        # go.target_pose.pose.position.y = 8.725
        # go.target_pose.pose.orientation.z = 0.926
        # go.target_pose.pose.orientation.w = 0.377

        # smach.StateMachine.add(
        #     'MOVE_HALLWAY',
        #     SimpleActionState( '/move_base',
        #                        MoveBaseAction,
        #                        goal = go ),  # Back down the hallway
        #     transitions = {'succeeded':'succeeded'})

        

            
    return sm

if False:
    rospy.init_node('localization_trial')

    sm = smach.StateMachine( outcomes=['succeeded','aborted','preempted'] )
    with sm:
        # Just a precation
        tgoal = SingleJointPositionGoal()
        tgoal.position = 0.040  # all the way up is 0.200
        tgoal.min_duration = rospy.Duration( 2.0 )
        tgoal.max_velocity = 1.0
        smach.StateMachine.add(
            'TORSO_SETUP',
            SimpleActionState( 'torso_controller/position_joint_action',
                               SingleJointPositionAction,
                               goal = tgoal),
            transitions = { 'succeeded': 'succeeded' })

    sm.execute()
    

if __name__ == '__main__':
# if False:
    rospy.init_node('smach_aware_home')

    sm = cousins_demo()

    sis = IntrospectionServer('sm_aware_home', sm, '/SM_AWARE_HOME')
    sis.start()

    rospy.logout( 'READY TO RUN AWARE_HOME DEMO' )

    sm.userdata.person_id = 'person      '
    # sm.userdata.explore_radius = 1.5
    # sm.userdata.explore_rfid_reads = []
    outcome = sm.execute()

    # rospy.spin()
    sis.stop()

    

