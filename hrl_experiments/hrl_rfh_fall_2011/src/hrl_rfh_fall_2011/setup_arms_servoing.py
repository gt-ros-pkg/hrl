#! /usr/bin/python

import sys
import numpy as np
import copy
from threading import Lock

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('hrl_rfh_summer_2011')
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('pr2_traj_playback')
import rospy

import smach
import actionlib
from std_srvs.srv import Empty, EmptyResponse

from hrl_rfh_fall_2011.sm_register_head_ellipse import SMEllipsoidRegistration
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from hrl_rfh_fall_2011.sm_topic_monitor import TopicMonitor
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from pr2_traj_playback.msg import TrajectoryPlayAction, TrajectoryPlayGoal

def get_goal_template():
    traj = TrajectoryPlayGoal()
    traj.mode = traj.SETUP_AND_TRAJ
    traj.reverse = True
    traj.setup_velocity = 0.3
    traj.traj_rate_mult = 0.8
    return traj

class SetupArmsServoing():
    def __init__(self):
        self.lock = Lock()
        self.ctrl_switcher = ControllerSwitcher()
        self.torso_sac = actionlib.SimpleActionClient('torso_controller/position_joint_action',
                                                      SingleJointPositionAction)
        self.traj_playback_l = actionlib.SimpleActionClient('/trajectory_playback_l', TrajectoryPlayAction)
        self.traj_playback_r = actionlib.SimpleActionClient('/trajectory_playback_r', TrajectoryPlayAction)
        self.torso_sac.wait_for_server()
        self.traj_playback_l.wait_for_server()
        self.traj_playback_r.wait_for_server()
        rospy.loginfo("[setup_arms_servoing] SetupArmsServoing ready.")

    def adjust_torso(self):
        # move torso up
        tgoal = SingleJointPositionGoal()
        tgoal.position = 0.01  # all the way up is 0.300, all the way down is 0.01
        tgoal.min_duration = rospy.Duration( 2.0 )
        tgoal.max_velocity = 1.0
        self.torso_sac.send_goal_and_wait(tgoal)

    def run(self, req):
        ############################################################
        self.lock.acquire(False)

        print "A"
        self.adjust_torso()
        print "B"

        l_traj = get_goal_template()
        l_traj.filepath = "$(find hrl_rfh_fall_2011)/data/l_arm_servo_setup.pkl"

        r_traj = get_goal_template()
        r_traj.filepath = "$(find hrl_rfh_fall_2011)/data/r_arm_servo_setup.pkl"

        self.traj_playback_l.send_goal(l_traj)
        self.traj_playback_r.send_goal(r_traj)
        self.traj_playback_l.wait_for_result()
        self.traj_playback_r.wait_for_result()
    
        self.ctrl_switcher.carefree_switch('l', '%s_arm_controller')
        self.ctrl_switcher.carefree_switch('r', '%s_arm_controller')

        self.lock.release()
        ############################################################

        return EmptyResponse()

def main():
    rospy.init_node("setup_arms_servoing")
    assert(len(sys.argv) > 1)
    sas = SetupArmsServoing()
    if sys.argv[1] == "-s":
        rospy.Service("/pr2_ar_servo/arms_setup", Empty, sas.run)
        rospy.spin()
    elif sys.argv[1] == "-p":
        sas.run(None)
        
if __name__ == "__main__":
    main()
