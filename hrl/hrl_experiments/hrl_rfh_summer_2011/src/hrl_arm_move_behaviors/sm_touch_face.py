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
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Vector3
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from pr2_controllers_msgs.msg import JointTrajectoryControllerState


from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest
from pr2_collision_monitor.srv import JointDetectionStart
from pr2_approach_table.srv import ApproachSrv
from pr2_approach_table.msg import ApproachAction, ApproachResult, ApproachGoal
from rfid_behaviors.srv import FloatFloat_Int32 as RotateBackupSrv
from hrl_arm_move_behaviors.msg import EPCDirectMoveAction, EPCLinearMoveAction
from hrl_arm_move_behaviors.msg import EPCDirectMoveGoal, EPCLinearMoveGoal
from hrl_arm_move_behaviors.pr2_arm_base import PR2ArmBase
import hrl_arm_move_behaviors.util as util

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
# SMACH state which listens to the force_signal topic and only returns 'collided'
# if the signal exceeds the threshold

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

##
# SMACH state which listens to the finger_signal topic and only returns 'collided'
# if the signal exceeds the threshold

class FingerCollisionMonitor(smach.State):
    def __init__(self, arm, thresh=1):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'shutdown'])
        self.thresh = thresh
        self.collided = False
        rospy.Subscriber('/'+arm+'_fingertip_monitor/finger_signal', Float32, self.finger_cb)

    def finger_cb(self, msg):
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

##
# SMACH state which listens to the joint collision detector and returns 'collision'
# if the detector signals a collision

class JointCollisionMonitor(smach.State):
    ##
    # behavior_name needs to be the name stored when training the detector or empty
    # otherwise.  sig_level should be a number 0.99 <= sig_level < 1.0
    def __init__(self, arm, behavior_name='', sig_level=0.99):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'shutdown'])
        self.behavior_name = behavior_name
        self.sig_level = sig_level
        self.collided = False
        prefix = arm + '_joint_coll_detect/'
        rospy.loginfo("[sm_touch_face] JointCollisionMonitor: Waiting for start_detection")
        rospy.wait_for_service(prefix + 'start_detection')
        self.start_detection = rospy.ServiceProxy(prefix + 'start_detection', 
                                                  JointDetectionStart, persistent=False)
        rospy.loginfo("[sm_touch_face] JointCollisionMonitor: Waiting for stop_detection")
        rospy.wait_for_service(prefix + 'stop_detection')
        self.stop_detection = rospy.ServiceProxy(prefix + 'stop_detection', 
                                                 Empty, persistent=False)
        self.stop_detection()
        rospy.Subscriber(prefix + 'arm_collision_detected', Bool, self.joint_cb)

    def joint_cb(self, msg):
        self.collided = msg.data

    def execute(self, userdata):
        self.start_detection(self.behavior_name, self.sig_level)
        rospy.sleep(0.3)
        while not rospy.is_shutdown():
            if self.collided:
                self.stop_detection()
                return 'collision'
            if self.preempt_requested():
                self.service_preempt()
                self.stop_detection()
                return 'preempted'
            rospy.sleep(0.01)
        self.stop_detection()
        return 'shutdown'

class JointCollisionMonitor2(smach.State):
    def __init__(self, arm, min_errors, max_errors):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'shutdown'])
        rospy.Subscriber('/r_arm_controller/state', JointTrajectoryControllerState,
                         self.error_cb)

    def error_cb(self, msg):
        error = np.array(msg.error.positions)
        if np.any(error < min_errors) or np.any(error > max_errors):
            self.collided = True

    def execute(self, ud):
        self.collided = False
        while not rospy.is_shutdown():
            if self.collided:
                self.stop_detection()
                return 'collision'
            if self.preempt_requested():
                self.service_preempt()
                self.stop_detection()
                return 'preempted'
            rospy.sleep(0.01)
        self.stop_detection()
        return 'shutdown'

        
##
# SMACH state which listens to /pixel3d until a message is received.
# The result is stored in key 'click_pose' and returns outcome 'click'
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
        
        
##
# SMACH state which moves the arm to a desired pose.
# Two modes are defined based on whether the q parameter is defined
# if it is defined, it will go directly to that position when called
# If not, it will perform biased_IK on the 'wrist_mat' homogeneous
# matrix defining the desired pose of the wrist in the torso frame
class MovePoseIK(smach.State):
    def __init__(self, pr2_arm, duration=5.0, q=None):
        smach.State.__init__(self, outcomes=['succeeded', 'ik_failure', 'shutdown', 'preempted'],
                                   input_keys=['wrist_mat'])
        self.JOINTS_BIAS = [0.0, 0.012, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.INIT_ANGS = [-0.417, -0.280, -1.565, -2.078, -2.785, -1.195, -0.369]
        self.ANGS_SETTLED = np.array([0.15, 0.15, 0.15, 0.18, 0.20, 0.15, 0.15])
        self.duration = duration
        self.q = q
        self.pr2_arm = pr2_arm

    def execute(self, ud):
        if self.q is None:
            q = self.pr2_arm.biased_IK(np.mat(ud.wrist_mat), self.INIT_ANGS, self.JOINTS_BIAS)
            if q is None:
                return 'ik_failure'
        else:
            q = self.q
        self.pr2_arm.command_joint_angles(q, duration=self.duration, delay=1.0)
        rospy.sleep(0.1)
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            ang_diff = self.pr2_arm.angle_difference(q, self.pr2_arm.get_joint_angles())
            if np.all(np.fabs(ang_diff) < self.ANGS_SETTLED):
                return 'succeeded'
            if rospy.Time.now().to_sec() - start_time > 15:
                rospy.logerr('[sm_touch_face] Timeout commanding joint angles.')
                return 'shutdown'
            if self.preempt_requested():
                self.pr2_arm.freeze_arm()
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.01)
        return 'shutdown'

class MoveAnglesBase(smach.State):
    def __init__(self, pr2_arm):
        self.ANGS_SETTLED = np.array([0.15]*7)
        self.pr2_arm = pr2_arm

    def move_angles(self, q, duration):
        self.pr2_arm.command_joint_angles(q, duration=duration, delay=1.0)
        rospy.sleep(0.1)
        start_time = rospy.Time.now().to_sec()
	settled_count = 0
        while not rospy.is_shutdown():
            ang_diff = self.pr2_arm.angle_difference(q, self.pr2_arm.get_joint_angles())
            print "ang_diff", ang_diff
            if np.all(np.fabs(ang_diff) < self.ANGS_SETTLED):
                settled_count += 1
                if settled_count > 5:
                    return 'succeeded'
            if rospy.Time.now().to_sec() - start_time > 25:
                rospy.logerr('[sm_touch_face] Timeout commanding joint angles.')
                return 'shutdown'
            if self.preempt_requested():
                self.pr2_arm.freeze_arm()
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.01)
        return 'shutdown'

class MoveCoarsePose(MoveAnglesBase):
    def __init__(self, pr2_arm):
        super(MoveCoarsePose, self).__init__(pr2_arm)
        smach.State.__init__(self, outcomes=['succeeded', 'ik_failure', 'shutdown', 'preempted'],
                                   input_keys=['wrist_mat'])
        self.JOINTS_BIAS = [0.0, 0.012, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.INIT_ANGS = [-0.417, -0.280, -1.565, -2.078, -2.785, -1.195, -0.369]

    def execute(self, ud):
        q_end = self.pr2_arm.biased_IK(np.mat(ud.wrist_mat), self.INIT_ANGS, self.JOINTS_BIAS)
        if q_end is None:
            return 'ik_failure'
        q_end = np.array(q_end)

        # move wrist to correct position but tucked
        q_tuck = q_end.copy()
        q_tuck[[0,1,3,4]] = [-1.324, -0.35, -2.2, 3.127]
        result = self.move_angles(q_tuck, 5)
        if result != 'succeeded':
            return result

        # move the elbow to its correct position but keep the wrist in
        q_move_elbow = q_tuck.copy()
        q_move_elbow[[0,1]] = q_end[[0,1]]
        result = self.move_angles(q_move_elbow, 5)
        if result != 'succeeded':
            return result

        # bring the upper arm back down
        q_move_upr_arm = q_move_elbow.copy()
        q_move_upr_arm[4] = q_end[4]
        result = self.move_angles(q_move_upr_arm, 5)
        if result != 'succeeded':
            return result

        # flex the elbow out and go to the final position
        result = self.move_angles(q_end, 5)
        return result

class MoveReturnSetup(MoveAnglesBase):
    def __init__(self, pr2_arm):
        super(MoveReturnSetup, self).__init__(pr2_arm)
        smach.State.__init__(self, outcomes=['succeeded', 'ik_failure', 'shutdown', 'preempted'])
        self.q_setup = np.array([-1.324,  -0.35, -0.689, -2.2,  3.127, -0.861, -1.584])

    def execute(self, ud):
        q_cur = self.pr2_arm.get_joint_angles(wrapped=True)
        # flex the elbow back in
        q_flex_elbow = q_cur.copy()
        q_flex_elbow[3] = self.q_setup[3]
        result = self.move_angles(q_flex_elbow, 5)
        if result != 'succeeded':
            return result

        # pull the gripper up
        q_move_upr_arm = q_flex_elbow.copy()
        q_move_upr_arm[4] = self.q_setup[4]
        result = self.move_angles(q_move_upr_arm, 5)
        if result != 'succeeded':
            return result

        # move the elbow up and to the side
        q_move_elbow = q_move_upr_arm.copy()
        q_move_elbow[[0,1]] = self.q_setup[[0,1]]
        result = self.move_angles(q_move_elbow, 5)
        if result != 'succeeded':
            return result

        # fully return to setup
        result = self.move_angles(self.q_setup, 5)
        return result
        

class SMTouchFace(object):
    def __init__(self):
        self.arm = rospy.get_param("~arm", default="r")
        self.base_offset_frame = rospy.get_param("~base_offset_frame")
        self.tool_frame = rospy.get_param("~tool_frame", default="r_gripper_tool_frame")
        self.tool_approach_frame = rospy.get_param("~tool_approach_frame", default="")

        self.tf_listener = tf.TransformListener()
        self.pr2_arm = PR2ArmBase(self.arm)

        self.nav_approach_dist = rospy.get_param("~nav_approach_dist", default=1.0)

        self.wrist_pub = rospy.Publisher("~wrist_setup", PoseStamped)
        self.appr_pub = rospy.Publisher("~approach_location", PoseStamped)
        self.touch_pub = rospy.Publisher("~touch_location", PoseStamped)
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

    def get_nav_prep_sm(self):
        nav_prep_sm = smach.StateMachine(outcomes=['succeeded','preempted','shutdown', 'aborted'])
        with nav_prep_sm:

            # make sure the robot is clear of obstacles
            # make sure the arms are tucked with
            # rosrun pr2_tuckarm tuck_arms.py r t l t
            # wait for the user to click on the head so the robot can approach
            smach.StateMachine.add(
                'WAIT_FOR_HEAD_CLICK',
                ClickMonitor(),
                transitions={'click' : 'PROCESS_NAV_POSE',
                             'shutdown' : 'shutdown'},
                remapping={'click_pose' : 'head_click_pose_global'}) # output (PoseStamped)

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
                transitions = {'succeeded' : 'TORSO_SETUP',
                               'shutdown' : 'shutdown'},
                remapping={'nav_dist' : 'nav_dist_global'})

            # move torso up
            tgoal = SingleJointPositionGoal()
            tgoal.position = 0.300  # all the way up is 0.300
            tgoal.min_duration = rospy.Duration( 2.0 )
            tgoal.max_velocity = 1.0
            smach.StateMachine.add(
                'TORSO_SETUP',
                SimpleActionState( 'torso_controller/position_joint_action',
                                   SingleJointPositionAction,
                                   goal = tgoal),
                transitions = { 'succeeded': 'UNFOLD' })

            # Unfolds the arms
            smach.StateMachine.add(
                'UNFOLD',
                ServiceState( 'traj_playback/unfold',
                              TrajPlaybackSrv,
                              request = TrajPlaybackSrvRequest( False )), # if True, reverse trajectory
                transitions = { 'succeeded':'succeeded' })

        return nav_prep_sm
        

    ##
    # returns a SMACH state which processes the output of the WAIT_TOUCH_CLICK state
    def get_process_click(self):
        # process the 'touch_click_pose' output of WAIT_TOUCH_CLICK for use in the
        # prep, approach, and move
        # output:
        # appr_wrist_mat - homogeneous matrix pose of the wrist used in MOVE_COARSE_IK
        # appr_tool_ps - PoseStamped pose of the desired tool location to start approach;
        #                used in FINE_POSITION_SETUP and FINE_APPROACH
        # touch_tool_ps - PoseStamped pose of the desired tool location to touch/end aproach;
        #                 used in FINE_APPROACH
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
            appr_B_wrist = self.get_transform(self.tool_frame, 
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
        return smach.CBState(process_touch_pose)

    def get_coll_detect_action_state(self, input_keys,
                                     action_name, action_state, action_outcomes, 
                                     force_thresh, finger_thresh, behavior_name, sig_level):

        collision_outcomes = ['force_collision', 'finger_collision', 'joint_collision']
        total_outcomes = action_outcomes + collision_outcomes

        def child_term_cb(outcome_map):
            # stop execution under all circumstances
            return True

        def out_cb(outcome_map):
            if outcome_map['FORCE_COLL_MONITOR'] == 'collision':
                return 'force_collision'
            if outcome_map['FINGER_COLL_MONITOR'] == 'collision':
                return 'finger_collision'
            if outcome_map['JOINT_COLL_MONITOR'] == 'collision':
                return 'joint_collision'
            return outcome_map[action_name]

        sm_coll_detect_state = smach.Concurrence(
            outcomes=total_outcomes,
            input_keys=input_keys,
            default_outcome='shutdown',
            child_termination_cb=child_term_cb,
            outcome_cb=out_cb)

        with sm_coll_detect_state:
            smach.Concurrence.add(
                'FORCE_COLL_MONITOR',
                ForceCollisionMonitor(force_thresh))
            smach.Concurrence.add(
                'FINGER_COLL_MONITOR',
                FingerCollisionMonitor(self.arm, finger_thresh))
            smach.Concurrence.add(
                'JOINT_COLL_MONITOR',
                JointCollisionMonitor(self.arm, behavior_name, sig_level))
            smach.Concurrence.add(
                action_name,
                action_state)

        return sm_coll_detect_state

    def get_fine_pos_setup(self):

        action_outcomes = ['succeeded', 'error_high', 'ik_failure', 'shutdown',
                           'aborted', 'preempted']
        def fine_goal_cb(ud, goal):
            dm_goal = EPCDirectMoveGoal(ud.target_pose, self.tool_frame, True, 
                                        0.02, 0.35, 0.16, 1.0, 5)
            return dm_goal
        @smach.cb_interface(outcomes=action_outcomes)
        def res_cb(us, status, result):
            rospy.sleep(1)
            return result.result
        fine_pos_move = SimpleActionState(self.arm + '_epc_move/direct_move',
                                          EPCDirectMoveAction,
                                          goal_cb=fine_goal_cb,
                                          result_cb=res_cb,
                                          input_keys=['target_pose'])
        return self.get_coll_detect_action_state(
                    ['target_pose'],
                    'FINE_POSITION_MOVE', fine_pos_move, action_outcomes,
                    0.0006, 3.0, '', 0.995)

    def get_fine_approach(self):

        action_outcomes = ['succeeded', 'error_high', 'ik_failure', 'shutdown',
                           'aborted', 'preempted']

        def fine_appr_cb(ud, goal):
            lm_goal = EPCLinearMoveGoal(ud.start_pose, ud.end_pose, self.tool_frame, 
                                        False, 0.003, 0.1, 1.0, 3)
            return lm_goal
        @smach.cb_interface(outcomes=action_outcomes)
        def res_cb(us, status, result):
            rospy.sleep(1)
            return result.result
        fine_appr_move = SimpleActionState(self.arm + '_epc_move/linear_move',
                                           EPCLinearMoveAction,
                                           goal_cb=fine_appr_cb,
                                           result_cb=res_cb,
                                           input_keys=['start_pose', 'end_pose'])

        return self.get_coll_detect_action_state(
                    ['start_pose', 'end_pose'],
                    'FINE_APPROACH_MOVE', fine_appr_move, action_outcomes,
                    0.0003, 3.0, '', 0.995)

    def get_fine_retreat(self):

        action_outcomes = ['succeeded', 'error_high', 'ik_failure', 'shutdown',
                           'aborted', 'preempted']

        def retreat_cb(ud, goal):
            # get the current location of the tool
            start_pose_mat = self.get_transform("torso_lift_link", self.tool_frame)
            start_pose_ps = util.pose_mat_to_stamped_msg("torso_lift_link", start_pose_mat)
            # move from the current location to the approach location
            lm_goal = EPCLinearMoveGoal(start_pose_ps, ud.end_pose, self.tool_frame, 
                                        False, 0.03, 0.1, 1.0, 3)
            return lm_goal
        @smach.cb_interface(outcomes=action_outcomes)
        def res_cb(us, status, result):
            rospy.sleep(1)
            return result.result
        find_retreat_move = SimpleActionState(self.arm + '_epc_move/linear_move',
                                              EPCLinearMoveAction,
                                              goal_cb=retreat_cb,
                                              result_cb=res_cb,
                                              input_keys=['start_pose', 'end_pose'])

        return self.get_coll_detect_action_state(
                    ['start_pose', 'end_pose'],
                    'FINE_RETREAT_MOVE', find_retreat_move, action_outcomes,
                    0.0006, 3.0, '', 0.995)

    def get_touch_sm(self):

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded','preempted','shutdown'])

        with sm:
            # move to general pose where manipulation begins
            #q_setup = [-1.324,  -0.35, -0.689, -2.2,  3.127, -0.861, -1.584]
            #prep_pose_move  = MovePoseIK(self.pr2_arm, duration=10.0, q=q_setup)
            #prep_pose_outcomes = ['succeeded', 'ik_failure', 'shutdown', 'preempted']
            #smach.StateMachine.add(
            #    'PREP_POSE',
            #    self.get_coll_detect_action_state(
            #        [],
            #        'PREP_POSE_MOVE', prep_pose_move, prep_pose_outcomes,
            #        0.0006, 3.0, '', 0.995),
            #    transitions={'succeeded' : 'WAIT_TOUCH_CLICK',
            #                 'ik_failure' : 'shutdown',
            #                 'force_collision' : 'shutdown',
            #                 'finger_collision' : 'shutdown',
            #                 'joint_collision' : 'shutdown',
            #                 'preempted' : 'shutdown',
            #                 'shutdown' : 'shutdown'})

            move_return_setup = MoveReturnSetup(self.pr2_arm)
            move_return_setup_outcomes = ['succeeded', 'ik_failure', 'shutdown', 'preempted']
            smach.StateMachine.add(
                'MOVE_RETURN_SETUP',
                self.get_coll_detect_action_state(
                    [],
                    'MOVE_RETURN_SETUP_MOVE', move_return_setup, move_return_setup_outcomes,
                    0.0010, 3.0, '', 0.995),
                transitions={'succeeded' : 'WAIT_TOUCH_CLICK',
                             'ik_failure' : 'shutdown',
                             'force_collision' : 'WAIT_RETURN_SETUP_CLICK',
                             'finger_collision' : 'WAIT_RETURN_SETUP_CLICK',
                             'joint_collision' : 'WAIT_RETURN_SETUP_CLICK',
                             'shutdown' : 'shutdown'},
                remapping={'wrist_mat' : 'appr_wrist_mat'})

            # wait for the user to click anywhere on the screen, signifying a desire to continue
            smach.StateMachine.add(
                'WAIT_RETURN_SETUP_CLICK',
                ClickMonitor(),
                transitions={'click' : 'MOVE_RETURN_SETUP',
                             'shutdown' : 'shutdown'})

            # wait for the user to select a position to scratch
            smach.StateMachine.add(
                'WAIT_TOUCH_CLICK',
                ClickMonitor(),
                transitions={'click' : 'PROCESS_CLICK',
                             'shutdown' : 'shutdown'},
                remapping={'click_pose' : 'touch_click_pose'})

            # process the point to get desired poses for the arm
            smach.StateMachine.add(
                'PROCESS_CLICK',
                self.get_process_click(),
                transitions = {'succeeded' : 'COARSE_POSE',
                               'tf_failure' : 'WAIT_TOUCH_CLICK'},
                remapping={'touch_click_pose' : 'touch_click_pose',
                           'appr_wrist_mat' : 'appr_wrist_mat',
                           'appr_tool_ps' : 'appr_tool_ps',
                           'touch_tool_ps' : 'touch_tool_ps'})
            
            # move to the expected wrist position without EPC
            coarse_pose_move  = MoveCoarsePose(self.pr2_arm)
            coarse_pose_outcomes = ['succeeded', 'ik_failure', 'shutdown', 'preempted']
            smach.StateMachine.add(
                'COARSE_POSE',
                self.get_coll_detect_action_state(
                    ['wrist_mat'],
                    'COARSE_POSE_MOVE', coarse_pose_move, coarse_pose_outcomes,
                    0.0008, 3.0, '', 0.995),
                transitions={'succeeded' : 'FINE_POSITION_SETUP',
                             'ik_failure' : 'MOVE_RETURN_SETUP',
                             'force_collision' : 'shutdown',
                             'finger_collision' : 'shutdown',
                             'joint_collision' : 'shutdown',
                             'preempted' : 'shutdown',
                             'shutdown' : 'shutdown'},
                remapping={'wrist_mat' : 'appr_wrist_mat'})

            # do the fine positioning to put the tool at the precise
            # approach start position
            smach.StateMachine.add(
                'FINE_POSITION_SETUP',
                self.get_fine_pos_setup(),
                transitions={'succeeded' : 'FINE_APPROACH',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'shutdown',
                             'finger_collision' : 'shutdown',
                             'joint_collision' : 'shutdown',
                             'aborted' : 'shutdown',
                             'preempted' : 'FINE_POSITION_SETUP'},
                remapping={'target_pose' : 'appr_tool_ps'})

            # approach the touch position linearly
            smach.StateMachine.add(
                'FINE_APPROACH',
                self.get_fine_approach(),
                transitions={'succeeded' : 'WAIT_RETREAT_CLICK',
                             'error_high' : 'shutdown',
                             'ik_failure' : 'shutdown',
                             'shutdown' : 'shutdown',
                             'force_collision' : 'WAIT_RETREAT_CLICK',
                             'finger_collision' : 'WAIT_RETREAT_CLICK',
                             'joint_collision' : 'shutdown',
                             'aborted' : 'shutdown',
                             'preempted' : 'FINE_APPROACH'},
                remapping={'start_pose' : 'appr_tool_ps',
                           'end_pose' : 'touch_tool_ps'})

            # wait for the user to click anywhere on the screen, signifying a desire to retreat
            smach.StateMachine.add(
                'WAIT_RETREAT_CLICK',
                ClickMonitor(),
                transitions={'click' : 'COARSE_RETREAT',
                             'shutdown' : 'shutdown'})

            # move to the expected wrist position without EPC
            coarse_retreat = MovePoseIK(self.pr2_arm, duration=10.0)
            coarse_retreat_outcomes = ['succeeded', 'ik_failure', 'shutdown', 'preempted']
            smach.StateMachine.add(
                'COARSE_RETREAT',
                self.get_coll_detect_action_state(
                    ['wrist_mat'],
                    'COARSE_RETREAT_MOVE', coarse_retreat, coarse_retreat_outcomes,
                    0.0006, 3.0, '', 0.995),
                transitions={'succeeded' : 'MOVE_RETURN_SETUP',
                             'ik_failure' : 'shutdown',
                             'force_collision' : 'WAIT_RETREAT_CLICK',
                             'finger_collision' : 'WAIT_RETREAT_CLICK',
                             'joint_collision' : 'WAIT_RETREAT_CLICK',
                             'shutdown' : 'shutdown'},
                remapping={'wrist_mat' : 'appr_wrist_mat'})
                
        return sm


def main():
    rospy.init_node('smach_sm_touch_face')
    if sys.argv[1] == 'nav_only':

        smtf = SMTouchFace()
        sm = smtf.get_nav_prep_sm()
        rospy.sleep(1)

        sis = IntrospectionServer('nav_prep', sm, '/SM_NAV_PREP')
        sis.start()

        outcome = sm.execute()
        
        sis.stop()

    elif sys.argv[1] == 'full_demo':

        smtf = SMTouchFace()
        sm_nav = smtf.get_nav_prep_sm()
        sm_touch = smtf.get_touch_sm()
        sm_full = smach.StateMachine(outcomes=['succeeded','preempted','shutdown', 'aborted'])
        with sm_full:
            smach.StateMachine.add(
                'SM_NAV_PREP',
                sm_nav,
                transitions={'succeeded' : 'SM_TOUCH_FACE'})
            smach.StateMachine.add(
                'SM_TOUCH_FACE',
                sm_touch)
        rospy.sleep(1)


        sis = IntrospectionServer('nav_prep', sm_full, '/SM_TOUCH_FACE_FULL')
        sis.start()

        outcome = sm_full.execute()
        
        sis.stop()

    elif sys.argv[1] == 'touch_only':

        smtf = SMTouchFace()
        sm = smtf.get_touch_sm()
        rospy.sleep(1)

        sis = IntrospectionServer('touch_face', sm, '/SM_TOUCH_FACE')
        sis.start()

        outcome = sm.execute()
        
        sis.stop()


if __name__ == '__main__':
    main()

