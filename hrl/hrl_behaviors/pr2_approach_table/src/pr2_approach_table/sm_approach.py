#! /usr/bin/python
import roslib
roslib.load_manifest('pr2_approach_table')
roslib.load_manifest('rfid_behaviors')
import rospy

import tf
import tf.transformations as tft
import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction

from pr2_approach_table.srv import ApproachSrv
from pr2_approach_table.msg import ApproachAction, ApproachResult, ApproachGoal
from rfid_behaviors.srv import FloatFloat_Int32 as RotateBackupSrv

import numpy as np, math

class CheckHeading(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'preempted'],
                             input_keys = ['target_pose'],  # PoseStamped
                             output_keys = ['angular_error']) # float
        self.listener = listener
        self.initialized = False

    def execute(self, userdata):
        ps_desired = userdata.target_pose
        self.GLOBAL_FRAME = ps_desired.header.frame_id
        
        if not self.initialized:
            self.initialized = True
            # self.listener = tf.TransformListener() # this is now passed in.
            rospy.logout( 'CheckHeading (smach): Waiting on transforms from (%s -> %s)'
                          % ( self.GLOBAL_FRAME, '/base_link' ))
            self.listener.waitForTransform( '/base_link',
                                            self.GLOBAL_FRAME,
                                            rospy.Time(0), timeout = rospy.Duration(30) )
            rospy.logout( 'CheckHeading (smach): Ready.' )

        try:
            ps = PoseStamped()
            ps.header.stamp = rospy.Time(0)
            ps.header.frame_id = '/base_link'
            ps.pose.orientation.w = 1.0

            ps_global = self.listener.transformPose( self.GLOBAL_FRAME, ps )
            efq = tft.euler_from_quaternion
            r,p,yaw_curr = efq(( ps_global.pose.orientation.x,
                                 ps_global.pose.orientation.y,
                                 ps_global.pose.orientation.z,
                                 ps_global.pose.orientation.w ))
            r,p,yaw_des = efq(( ps_desired.pose.orientation.x,
                                ps_desired.pose.orientation.y,
                                ps_desired.pose.orientation.z,
                                ps_desired.pose.orientation.w ))

            rospy.logout( 'CheckHeading (smach): Error was %3.2f (deg)' % math.degrees(yaw_des - yaw_curr))
            userdata.angular_error = yaw_des - yaw_curr
        except:
            rospy.logout( 'CheckHeading (smach): TF failed.  Returning ang error of 0.0' )
            userdata.angular_error = 0.0
        return 'succeeded'


class PoseSelection(smach.State):
    def __init__(self, listener):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             input_keys=['candidate_poses'], # of type PoseArray!
                             output_keys=['selected_pose_global',   # of type PoseStamped!
                                          'movebase_pose_global' ]) # of type PoseStamped!
        self.listener = listener
        self.poses = []
        self.initialized = False
        self.ind = 0
        #self.GLOBAL_FRAME = '/odom_combined'
        self.GLOBAL_FRAME = '/map'

    def execute(self, userdata):
        rospy.logout( 'Executing POSE_SELECTION' )

        poses_frame = userdata.candidate_poses.header.frame_id
        #dist = userdata.approach_dist
        dist = 0.80

        if not self.initialized:
            self.initialized = True
            self.ind = 0
            # self.listener = tf.TransformListener()  # This is now passed in
            rospy.logout( 'PoseSelector (smach): Waiting on transforms from (%s -> %s)'
                          % ( self.GLOBAL_FRAME, poses_frame ))
            self.listener.waitForTransform( poses_frame,
                                            self.GLOBAL_FRAME,
                                            rospy.Time(0), timeout = rospy.Duration(30) )
            rospy.logout( 'PoseSelector (smach): Ready.' )

            
            # convert PoseArray to list of PoseStamped in global frame
            frame_adjusted = [ self.frame_adjust(i,poses_frame,dist) for i in userdata.candidate_poses.poses ]
            self.poses = [ i for i in frame_adjusted if i != None ] # in case the transforms fail.

        if len( self.poses ) == 0 or self.ind >= len( self.poses ): # we've tried everything...
            return 'aborted'

        # print 'POSES:\n', self.poses
        userdata.selected_pose_global = self.poses[ self.ind ][0]
        userdata.movebase_pose_global = self.poses[ self.ind ][1]
        # print 'SELECTED_POSE:\n', self.poses[ self.ind ]
        self.ind += 1
        return 'succeeded'

    def frame_adjust( self, pose, poses_frame, dist, arm_offset = 0.0 ):
        # Adjust all incoming candidate poses into a "Global" frame.
        # Positive arm offset to move the approach point "right"
        ps = PoseStamped()
        ps.header.frame_id = poses_frame
        ps.header.stamp = rospy.Time(0)
        ps.pose = pose

        # In some cases, we may want to move the pose right or left
        #   depending on which arm will be used for grasping.
        efq = tft.euler_from_quaternion
        r,p,y = efq(( pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w ))
        ps.pose.position.x = pose.position.x + arm_offset * np.cos( y + np.pi/2 )
        ps.pose.position.y = pose.position.y + arm_offset * np.sin( y + np.pi/2 )

        # print ps, '\nto\n', self.GLOBAL_FRAME, '\ntime:', rospy.Time.now()
        
        try:
            ps_global = self.listener.transformPose( self.GLOBAL_FRAME, ps )
            ps_global.pose.position.z = 0.0
            # print 'PS:\n', ps, '\nPS_GLOBAL:\n', ps_global

            r,p,y = efq(( ps_global.pose.orientation.x,
                          ps_global.pose.orientation.y,
                          ps_global.pose.orientation.z,
                          ps_global.pose.orientation.w ))

            # We want to move to a position that is 40-cm back from the tabletop using navstack
            mps = PoseStamped() 
            mps.header.frame_id = ps_global.header.frame_id
            mps.header.stamp = rospy.Time.now()
            mps.pose.position.x = ps_global.pose.position.x + dist * np.cos( y )
            mps.pose.position.y = ps_global.pose.position.y + dist * np.sin( y )
            qfe = tft.quaternion_from_euler
            mps.pose.orientation = Quaternion( *qfe( 0.0, 0.0, y - np.pi ))
            # print 'MPS:\n', mps

            # Return value: selected_pose_global, movebase_pose_global.
            rv = [ ps_global, mps ]

        except:
            rospy.logout( 'PoseSelector (smach): TF failed. Ignoring pose.' )
            rv = None

        return rv # Return value: selected_pose_global (PoseStamped), movebase_pose_global (PoseStamped)
        
            

def sm_approach_table( listener = None ):
    
    # for various states, you need a tf listener, but only one per thread supported.
    if listener == None:
        try:
            rospy.init_node( 'sm_approach' )
        except:
            rospy.logout( 'sm_approach_table: Node already initialized elsewhere' )

        listener = tf.TransformListener()
    
    # Python only allows one listener per thread.  This function has two (or more)
    #   classes that require the listener.  You can pass in one from higher in the
    #   hierarchy if you prefer.
    
    # Create a SMACH state machine
    sm = smach.StateMachine( outcomes = ['succeeded','aborted','preempted'],
                             input_keys = ['table_edge_poses'],  # type PoseArray
                             output_keys = ['movebase_pose_global', # type PoseStamped
                                            'table_edge_global']) # type PoseStamped

    # Open the container
    with sm:
        smach.StateMachine.add(
            'POSE_SELECTION',
            PoseSelection( listener = listener ),
            transitions = { 'succeeded' : 'MOVE_BASE' },
            remapping = {'candidate_poses':'table_edge_poses', # input (PoseArray)
                         'selected_pose_global':'table_edge_global', # output (PoseStamped)
                         'movebase_pose_global':'movebase_pose_global'}) # output (PoseStamped)


        smach.StateMachine.add(
            'MOVE_BASE',
            SimpleActionState( '/move_base',
                               MoveBaseAction,
                               goal_slots = ['target_pose'], # PoseStamped
                               outcomes = ['succeeded','aborted','preempted']),
            transitions = { 'succeeded' : 'CHECK_HEADING',
                            'aborted' : 'POSE_SELECTION' },
            remapping = {'target_pose':'movebase_pose_global'}) # input (PoseStamped)

        smach.StateMachine.add(
            'CHECK_HEADING',
            CheckHeading( listener = listener ),
            transitions = { 'succeeded':'ADJUST_HEADING' },
            remapping = { 'target_pose':'movebase_pose_global', # input (PoseStamped)
                          'angular_error':'angular_error' }) # output (float)

        smach.StateMachine.add(
            'ADJUST_HEADING',
            ServiceState( '/rotate_backup',
                          RotateBackupSrv,
                          request_slots = ['rotate']), # float (displace = 0.0)
            transitions = { 'succeeded':'MOVE_FORWARD' },
            remapping = {'rotate':'angular_error'})
        

        approach_goal = ApproachGoal()
        approach_goal.forward_vel = 0.05
        approach_goal.forward_mult = 0.50
        smach.StateMachine.add(
            'MOVE_FORWARD',
            SimpleActionState( '/approach_table/move_forward_act',
                               ApproachAction,
                               goal = approach_goal ),
            transitions = { 'succeeded' : 'succeeded' })

    return sm


            
            
if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    # sm = sm_approach_table()

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'approach_poses' ])

    p = Pose()
    p.position.x = 0.3879
    p.position.y = 0.79838
    p.position.z = 0.0

    p.orientation.z = -0.704
    p.orientation.w = 0.709

    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.poses = [ p ]

    sm.userdata.table_edge_poses = pa

    with sm:
        sm_table = sm_approach_table()

        smach.StateMachine.add(
            'APPROACH_TABLE',
            sm_table,
            remapping = {'table_edge_poses':'table_edge_poses', # input
                         'movebase_pose_global':'movebase_pose_global', # output
                         'table_edge_global':'table_edge_global'}, # output
            #transitions = {'succeeded':'MOVE_BACK'})
            transitions = {'succeeded':'succeeded'})

        # GRASP!
        
        # smach.StateMachine.add(
        #     'MOVE_BACK',
        #     SimpleActionState( '/move_base',
        #                        MoveBaseAction,
        #                        goal_slots = ['target_pose'], # PoseStamped
        #                        outcomes = ['succeeded','aborted','preempted']),
        #     transitions = { 'succeeded' : 'succeeded'},
        #     remapping = {'target_pose':'intermediate_pose'}) # input

        # GO DELIVER!


    sis = IntrospectionServer('Approach_Table', sm, '/SM_APPROACH_TABLE')
    sis.start()

    
    rospy.sleep( 3.0 )
    outcome = sm.execute()
    
    sis.stop()

    

            
                               
                               

    
