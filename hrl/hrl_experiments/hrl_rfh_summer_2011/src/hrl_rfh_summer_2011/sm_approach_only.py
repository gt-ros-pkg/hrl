#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_rfh_summer_2011')
roslib.load_manifest('hrl_rfh_fall_2011')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Vector3
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal


from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest
from pr2_collision_monitor.srv import JointDetectionStart
from pr2_approach_table.srv import ApproachSrv
from pr2_approach_table.msg import ApproachAction, ApproachResult, ApproachGoal
from rfid_behaviors.srv import FloatFloat_Int32 as RotateBackupSrv
import hrl_rfh_summer_2011.util as util

from hrl_rfh_fall_2011.sm_topic_monitor import TopicMonitor

class DetectForwardDistance(smach.State):
    def __init__(self, get_transform, distance):
        smach.State.__init__(self, output_keys=['nav_dist'],
                             outcomes=['reached', 'preempted', 'shutdown'])
        self.get_transform = get_transform
        self.distance = distance

    def execute(self, userdata):
        start_x = self.get_transform("base_link", "map")[0,3]
        nav_dist = 0
        while not rospy.is_shutdown():
            nav_dist = start_x - self.get_transform("base_link", "map")[0,3]
            print "nav_dist", nav_dist
            if nav_dist >= self.distance:
                userdata.nav_dist = nav_dist
                return 'reached'
            if self.preempt_requested():
                self.service_preempt()
                userdata.nav_dist = nav_dist
                return 'preempted'
            rospy.sleep(0.01)
        userdata.nav_dist = nav_dist
        return 'shutdown'

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

        
        
##
# SMACH state which moves the arm to a desired pose.
# Two modes are defined based on whether the q parameter is defined
# if it is defined, it will go directly to that position when called
# If not, it will perform biased_IK on the 'wrist_mat' homogeneous
# matrix defining the desired pose of the wrist in the torso frame

##
# SMACH state which listens to the force_signal topic and only returns 'collided'
# if the signal exceeds the threshold

class SMNavApproach(object):
    def __init__(self):
        self.nav_approach_dist = rospy.get_param("~nav_approach_dist", default=1.0)
        self.base_offset_frame = rospy.get_param("~base_offset_frame")
        self.tf_listener = tf.TransformListener()
        self.nav_pub = rospy.Publisher("~nav_location", PoseStamped)

    def get_transform(self, from_frame, to_frame, time=None):
        if time is None:
            time = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(from_frame, to_frame, time, rospy.Duration(5))
            pos, quat = self.tf_listener.lookupTransform(from_frame, to_frame, time)
            return util.pose_pq_to_mat(pos, quat)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return None

    def get_nav_approach_pose(self):
        @smach.cb_interface(input_keys=['head_click_pose'],
                            output_keys=['nav_pose_ps'],
                            outcomes=['succeeded', 'tf_failure'])
        def make_approach_pose(ud):
            frame_B_head = util.pose_msg_to_mat(ud.head_click_pose)
            base_B_frame = self.get_transform("base_link", ud.head_click_pose.header.frame_id)
            if base_B_frame is None:
                return 'tf_failure'
            base_B_head = base_B_frame * frame_B_head
            norm_z = np.array([base_B_head[0,2], base_B_head[1,2], 0])
            norm_z /= np.linalg.norm(norm_z)
            base_B_head[:3,:3] = np.mat([[-norm_z[0], norm_z[1], 0],
                                          [-norm_z[1],-norm_z[0], 0],
                                          [         0,         0, 1]])
            # offset in the x-direction by the given parameter
            offset_B_base = self.get_transform(self.base_offset_frame, "base_link")
            if offset_B_base is None:
                return 'tf_failure'
            nav_pose = base_B_head * offset_B_base  
            print nav_pose
            nav_pose[:4,3] = nav_pose * np.mat([[-self.nav_approach_dist, 0, 0, 1]]).T
            print nav_pose
            now = rospy.Time.now()
            nav_pose_ps = util.pose_mat_to_stamped_msg('base_link', nav_pose, now) 
            self.tf_listener.waitForTransform("/map", "base_link",
                                              now, timeout=rospy.Duration(20)) 
            nav_pose_map_ps = self.tf_listener.transformPose("/map", nav_pose_ps)

            #self.nav_pub.publish(util.pose_mat_to_stamped_msg('base_link', base_B_head, rospy.Time.now()))
            print base_B_head, base_B_head.T * base_B_head
            self.nav_pub.publish(nav_pose_map_ps)
            ud.nav_pose_ps = nav_pose_map_ps
            return 'succeeded'

        return smach.CBState(make_approach_pose)

    def get_nav_approach(self):
        def child_term_cb(outcome_map):
            return True

        def out_cb(outcome_map):
            if outcome_map['DETECT_FORWARD_DISTANCE'] == 'reached':
                return 'succeeded'
            return 'shutdown'

        sm_nav_approach_state = smach.Concurrence(
            outcomes=['succeeded', 'shutdown'],
            default_outcome='shutdown',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb,
            output_keys=['nav_dist'])

        with sm_nav_approach_state:
            approach_goal = ApproachGoal()
            approach_goal.forward_vel = 0.05
            approach_goal.forward_mult = 0.50
            smach.Concurrence.add(
                'MOVE_FORWARD',
                SimpleActionState( '/approach_table/move_forward_act',
                                   ApproachAction,
                                   goal = approach_goal ))
            smach.Concurrence.add(
                'DETECT_FORWARD_DISTANCE',
                DetectForwardDistance(self.get_transform, self.nav_approach_dist))

        return sm_nav_approach_state

    def get_sm(self):
        nav_prep_sm = smach.StateMachine(outcomes=['succeeded','preempted','shutdown', 'aborted'])
        with nav_prep_sm:

            # make sure the robot is clear of obstacles
            # make sure the arms are tucked with
            # rosrun pr2_tuckarm tuck_arms.py r t l t
            # wait for the user to click on the head so the robot can approach
            smach.StateMachine.add(
                'WAIT_FOR_HEAD_CLICK',
                TopicMonitor('/head_nav_goal', PoseStamped),
                transitions={'succeeded' : 'PROCESS_NAV_POSE'},
                remapping={'output' : 'head_click_pose_global'})

            # prepare the navigation pose for move_base
            # gets a point aligned with the normal and a distance away (nav_approach_dist)
            smach.StateMachine.add(
                'PROCESS_NAV_POSE',
                self.get_nav_approach_pose(),
                transitions={'succeeded' : 'MOVE_BASE', 
                             'tf_failure' : 'WAIT_FOR_HEAD_CLICK'},
                remapping={'head_click_pose' : 'head_click_pose_global', # input (PoseStamped)
                           'nav_pose_ps' : 'nav_pose_ps_global'}) # output (PoseStamped)

            # moves the base using nav stack to the appropirate location for moving forward
            smach.StateMachine.add(
                'MOVE_BASE',
                SimpleActionState('/move_base',
                                  MoveBaseAction,
                                  goal_slots=['target_pose'], # PoseStamped
                                  outcomes=['succeeded','aborted','preempted']),
                transitions = { 'succeeded' : 'CHECK_HEADING',
                                'preempted' : 'WAIT_FOR_HEAD_CLICK',
                                'aborted' : 'WAIT_FOR_HEAD_CLICK' },
                remapping = {'target_pose':'nav_pose_ps_global'}) # input (PoseStamped)

            # checks the current angle and returns the error
            smach.StateMachine.add(
                'CHECK_HEADING',
                CheckHeading( listener = self.tf_listener ),
                transitions = { 'succeeded':'ADJUST_HEADING' },
                remapping = { 'target_pose':'nav_pose_ps_global', # input (PoseStamped)
                              'angular_error':'angular_error' }) # output (float)

            # corrects for the error in angle
            smach.StateMachine.add(
                'ADJUST_HEADING',
                ServiceState( '/rotate_backup',
                              RotateBackupSrv,
                              request_slots = ['rotate']), # float (displace = 0.0)
                transitions = { 'succeeded':'MOVE_FORWARD_DIST' },
                remapping = {'rotate':'angular_error'})
            
            # approaches the touching position by moving forward nav_approach_dist meters
            # checks for collisions and will return shutdown if it detects a collsion
            smach.StateMachine.add(
                'MOVE_FORWARD_DIST',
                self.get_nav_approach(),
                transitions = {'succeeded' : 'succeeded',
                               'shutdown' : 'succeeded'},
                remapping={'nav_dist' : 'nav_dist_global'})

        return nav_prep_sm

def main():
    rospy.init_node('smach_sm_touch_face')

    smna = SMNavApproach()
    sm = smna.get_sm()
    rospy.sleep(1)

    sis = IntrospectionServer('nav_prep', sm, '/SM_NAV_PREP')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()

if __name__ == '__main__':
    main()
