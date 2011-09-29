#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
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

#from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest
from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import create_pr2_arm
from hrl_pr2_arms.pr2_arm_hybrid import PR2ArmHybridForce

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

class TFPubLoop(object):
    def __init__(self, parent_frame_name, child_frame_name, rate=100):
        self.child_frame_name = child_frame_name
        self.parent_frame_name = parent_frame_name
        self.tf_broad = tf.TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(1. / rate), self.pub_tf)
        self.tf_pose = None

    def pub_tf(self, timer_info):
        if self.tf_pose is not None:
            self.tf_broad.sendTransform(self.tf_pose[0], self.tf_pose[1], rospy.Time.now(), 
                                        self.child_frame_name, self.parent_frame_name)

    def update_pose(self, pose):
        self.tf_pose = PoseConverter.to_pos_quat(pose)
        self.pub_tf(None)
        
class SMTouchSimple(object):
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.start_frame_pub = rospy.Publisher("~start_frame", PoseStamped)
        self.end_frame_pub = rospy.Publisher("~end_frame", PoseStamped)

        self.arm = create_pr2_arm('l', PR2ArmHybridForce)
        self.tf_pub = TFPubLoop("/torso_lift_link", "/contact_control_frame")

    def get_transform(self, from_frame, to_frame, time=None):
        if time is None:
            time = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(from_frame, to_frame, time, rospy.Duration(5))
            pos, quat = self.tf_listener.lookupTransform(from_frame, to_frame, time)
            return util.pose_pq_to_mat(pos, quat)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return None

    def get_trajectory_generator(self):
        
        @smach.cb_interface(input_keys=['start_click', 'end_click'],
                            output_keys=['start_traj_frame', 'end_traj_frame'],
                            outcomes=['succeeded'])
        def generate_trajectory(ud):
            b_B_s = PoseConverter.to_homo_mat(ud.start_click)
            b_B_e = PoseConverter.to_homo_mat(ud.end_click)
            s_B_e = (b_B_s ** -1) * b_B_e
            b_normal = b_B_s[:3, 2] / np.linalg.norm(b_B_s[:3, 2])
            s_vel = np.mat([s_B_e[0, 3], s_B_e[1, 3], 0]).T
            s_vel = s_vel / np.linalg.norm(s_vel)
            b_vel = b_B_s[:3, :3].T * s_vel
            b_ortho = np.mat(np.cross(b_normal.T, b_vel.T)).T
            b_ortho = b_ortho /  np.linalg.norm(b_ortho)
            b_R_traj = np.vstack([b_vel.T, b_ortho.T, b_normal.T])

            b_p_start = b_B_s[:3, 3]
            b_p_end = b_B_e[:3, 3]
            b_p_end = 3 #TODO TODO
            
            self.start_frame_pub.publish(PoseConverter.to_pose_stamped_msg(ud.start_click.header.frame_id, 
                                                                           (b_p_start, b_R_traj)))
            self.end_frame_pub.publish(PoseConverter.to_pose_stamped_msg(ud.start_click.header.frame_id, 
                                                                         (b_p_end, b_R_traj)))
            

            ud.start_traj_frame = (b_p_start, b_R_traj)
            ud.end_traj_frame = (b_p_end, b_R_traj)
            return 'succeeded'

        return smach.CBState(generate_trajectory)
            

    def get_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded','preempted','shutdown'])
        
        with sm:
            smach.StateMachine.add(
                'INPUT_START_CLICK',
                ClickMonitor(),
                transitions={'click' : 'INPUT_END_CLICK',
                             'shutdown' : 'shutdown'},
                remapping={'click_pose' : 'start_click'}) # output (PoseStamped)

            smach.StateMachine.add(
                'INPUT_END_CLICK',
                ClickMonitor(),
                transitions={'click' : 'GENERATE_TRAJECTORY',
                             'shutdown' : 'shutdown'},
                remapping={'click_pose' : 'end_click'}) # output (PoseStamped)

            smach.StateMachine.add(
                'GENERATE_TRAJECTORY',
                self.get_trajectory_generator(),
                transitions={'succeeded' : 'INPUT_START_CLICK'})

        return sm

    def get_sm_basic(self):
        sm = smach.StateMachine(outcomes=['succeeded','preempted','shutdown'])
        
        with sm:
            smach.StateMachine.add(
                'INPUT_START_CLICK',
                ClickMonitor(),
                transitions={'click' : 'PUBLISH_CONTROL_FRAME',
                             'shutdown' : 'shutdown'},
                remapping={'click_pose' : 'start_click'}) # output (PoseStamped)

            @smach.cb_interface(input_keys=['start_click'],
                                outcomes=['succeeded', 'failed'])
            def publish_control_frame(ud):
                if ud.start_click.pose.position.x == -10000.0:
                    return 'failed'
                pose = ud.start_click
                pose.header.stamp = rospy.Time(0)
                click_torso = self.tf_listener.transformPose("/torso_lift_link", pose)
                self.tf_pub.update_pose(click_torso)
                rospy.sleep(1)
                self.arm.set_tip_frame("/contact_control_frame")
#self.arm.set_motion_gains(p_trans=[300, 300, 100])
                self.arm.update_gains()
                return 'succeeded'
            smach.StateMachine.add(
                'PUBLISH_CONTROL_FRAME',
                smach.CBState(publish_control_frame),
                transitions={'succeeded' : 'INPUT_START_CLICK',
                             'failed' : 'INPUT_START_CLICK'})

        return sm


def main():
    rospy.init_node('pr2_touch_simple')
    smts = SMTouchSimple()
    sm = smts.get_sm_basic()
    rospy.sleep(1)

    sis = IntrospectionServer('touch_simple', sm, '/INPUT_START_CLICK')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()

if __name__ == '__main__':
    main()
